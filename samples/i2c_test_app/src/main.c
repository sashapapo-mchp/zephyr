#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/util.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_I2C_TARGET_DUAL_ADDRESS),
	     "i2c_test_app requires CONFIG_I2C_TARGET_DUAL_ADDRESS=y");

LOG_MODULE_REGISTER(i2c_test_app, LOG_LEVEL_INF);

/* LED using the standard led0 alias on the board */
#define LED_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS(LED_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* I2C bus: use SERCOM3 node directly on SAM E54 Xplained Pro */
#define I2C_BUS_NODE DT_NODELABEL(sercom3)
#if !DT_NODE_HAS_STATUS(I2C_BUS_NODE, okay)
#error "SERCOM7 I2C node not available in devicetree"
#endif

static const struct device *const i2c_dev = DEVICE_DT_GET(I2C_BUS_NODE);

static bool g_is_target_mode = false;

/* Simple I2C target behavior:
 *  - Always ACKs both programmed addresses (primary 0x33, secondary 0x66)
 *  - When the host reads from 0x33 it returns 0x11/0x22/0x33/0x44
 *  - When the host reads from 0x66 it returns 0xA1/0xB2/0xC3/0xD4
 *  - Writes still update i2c_last_written/i2c_next_read so shell demos work
 */
static uint8_t i2c_last_written;
static uint8_t i2c_next_read;
static uint8_t tx_buf[16];
static uint32_t tx_len = 0;
static uint8_t read_byte_index = 0; /* Track which byte to send in sequence */

struct reply_profile {
	const uint8_t *bytes;
	size_t len;
	const char *label;
};

static const uint8_t primary_reply_bytes[] = { 0x11, 0x22, 0x33, 0x44 };
static const uint8_t secondary_reply_bytes[] = { 0xA1, 0xB2, 0xC3, 0xD4 };

static const struct reply_profile reply_primary = {
	.bytes = primary_reply_bytes,
	.len = ARRAY_SIZE(primary_reply_bytes),
	.label = "primary",
};

static const struct reply_profile reply_secondary = {
	.bytes = secondary_reply_bytes,
	.len = ARRAY_SIZE(secondary_reply_bytes),
	.label = "secondary",
};

static const struct reply_profile *active_reply = &reply_primary;
static uint16_t active_reply_addr;

static const struct reply_profile *select_reply_profile(struct i2c_target_config *config,
							uint16_t *addr_out);
static void reset_reply_sequence(struct i2c_target_config *config);


#if defined(CONFIG_I2C_TARGET_BUFFER_MODE)

#define I2C_RX_LOG_BYTES 32U

static void log_hex_buffer(const char *prefix, const uint8_t *buf, uint32_t len)
{
	if ((buf == NULL) || (len == 0U)) {
		LOG_INF("%s (0 bytes)", prefix);
		return;
	}

	char line[(I2C_RX_LOG_BYTES * 3U) + 1U];
	uint32_t to_log = MIN(len, I2C_RX_LOG_BYTES);
	size_t pos = 0U;

	for (uint32_t i = 0U; i < to_log && pos < sizeof(line); i++) {
		pos += snprintk(line + pos, sizeof(line) - pos, "%02x ", buf[i]);
	}

	if (pos > 0U && pos <= sizeof(line)) {
		line[pos - 1U] = '\0';
	} else {
		line[sizeof(line) - 1U] = '\0';
	}

	if (len > to_log) {
		LOG_INF("%s (%u/%u bytes): %s ...", prefix, to_log, len, line);
	} else {
		LOG_INF("%s (%u bytes): %s", prefix, len, line);
	}
}

struct i2c_dma_stats {
	atomic_t rx_blocks;
	atomic_t rx_bytes;
	atomic_t rx_last;
	atomic_t tx_blocks;
	atomic_t tx_bytes;
	atomic_t tx_last;
};

static struct i2c_dma_stats dma_stats;
#endif

static int cb_write_requested(struct i2c_target_config *config)
{
	ARG_UNUSED(config);
	/* Prepare to receive data from controller */
	read_byte_index = 0; /* Reset read index on new transaction */
	return 0;
}

static int cb_write_received(struct i2c_target_config *config, uint8_t val)
{
	ARG_UNUSED(config);
	i2c_last_written = val;
	/* Also update next read to mirror last written value */
	i2c_next_read = i2c_last_written;
	LOG_INF("I2C write: 0x%02x", i2c_last_written);
	return 0;
}

static int cb_read_requested(struct i2c_target_config *config, uint8_t *val)
{
	reset_reply_sequence(config);

	if ((active_reply == NULL) || (active_reply->len == 0U)) {
		*val = 0x00;
		LOG_WRN("I2C read first: no reply profile, sending 0x00");
		return 0;
	}

	*val = active_reply->bytes[read_byte_index];
	LOG_INF("I2C read first (%s @0x%02x): 0x%02x (byte %u/%zu)", active_reply->label,
		active_reply_addr & 0x7FU, *val, read_byte_index + 1U, active_reply->len);
	read_byte_index++;
	return 0;
}

static int cb_read_processed(struct i2c_target_config *config, uint8_t *val)
{
	ARG_UNUSED(config);
	const struct reply_profile *profile = (active_reply != NULL) ? active_reply : &reply_primary;
	size_t last_index = (profile->len > 0U) ? (profile->len - 1U) : 0U;

	if (profile->len == 0U) {
		*val = 0x00;
		LOG_WRN("I2C read next: empty profile, sending 0x00");
		return 0;
	}

	if (read_byte_index < profile->len) {
		*val = profile->bytes[read_byte_index];
		LOG_INF("I2C read next (%s @0x%02x): 0x%02x (byte %u/%zu)", profile->label,
			active_reply_addr & 0x7FU, *val, read_byte_index + 1U, profile->len);
		read_byte_index++;
	} else {
		*val = profile->bytes[last_index];
		LOG_INF("I2C read next (%s @0x%02x): 0x%02x (repeat last)", profile->label,
			active_reply_addr & 0x7FU, *val);
	}
	return 0;
}

static int cb_stop(struct i2c_target_config *config)
{
	ARG_UNUSED(config);
	LOG_INF("I2C STOP");
	read_byte_index = 0; /* Reset for next transaction */
	return 0;
}

#if defined(CONFIG_I2C_TARGET_BUFFER_MODE)
__attribute__((noinline))
static void cb_buf_write_received(struct i2c_target_config *config, uint8_t *ptr, uint32_t len)
{
    ARG_UNUSED(config);
    /* For demo: remember last byte and prep next read buffer */
    if (len > 0) {
        i2c_last_written = ptr[len - 1];
        i2c_next_read = i2c_last_written;
    }
	atomic_inc(&dma_stats.rx_blocks);
	atomic_add(&dma_stats.rx_bytes, (atomic_val_t)len);
	atomic_set(&dma_stats.rx_last, (atomic_val_t)len);
    LOG_INF("I2C buf write: %u bytes", len);
	log_hex_buffer("I2C buf write data", ptr, len);
}

__attribute__((noinline))
static int cb_buf_read_requested(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len)
{
	reset_reply_sequence(config);

	const struct reply_profile *profile = (active_reply != NULL) ? active_reply : &reply_primary;

	tx_len = MIN(profile->len, ARRAY_SIZE(tx_buf));
	memcpy(tx_buf, profile->bytes, tx_len);
	*ptr = tx_buf;
	*len = tx_len;
	atomic_inc(&dma_stats.tx_blocks);
	atomic_add(&dma_stats.tx_bytes, (atomic_val_t)tx_len);
	atomic_set(&dma_stats.tx_last, (atomic_val_t)tx_len);
	LOG_INF("I2C buf read (%s @0x%02x): offering %u bytes", profile->label,
		active_reply_addr & 0x7FU, tx_len);
    return 0;
}
#endif

static const struct i2c_target_callbacks i2c_cb = {
    .write_requested = cb_write_requested,
    .read_requested = cb_read_requested,
    .write_received = cb_write_received,
    .read_processed = cb_read_processed,
#if defined(CONFIG_I2C_TARGET_BUFFER_MODE)
    .buf_write_received = cb_buf_write_received,
    .buf_read_requested = cb_buf_read_requested,
#endif
    .stop = cb_stop,
};

/* Target configurations: primary 0x33 (51) and optional secondary 0x66 (102) */
static struct i2c_target_config tgt_cfg_primary = {
	.address = 0x33,
	.flags = 0,
	.callbacks = &i2c_cb,
};

static struct i2c_target_config tgt_cfg_secondary = {
	.address = 0x66,
    .flags = 0,
    .callbacks = &i2c_cb,
};

static bool g_primary_registered;
static bool g_secondary_registered;

static int register_primary_target(uint16_t addr)
{
	tgt_cfg_primary.address = addr & 0x7FU;

	int ret = i2c_target_register(i2c_dev, &tgt_cfg_primary);

	if (ret == 0) {
		g_primary_registered = true;
		g_is_target_mode = true;
	}

	return ret;
}

static int register_secondary_target(uint16_t addr)
{
	tgt_cfg_secondary.address = addr & 0x7FU;

	int ret = i2c_target_register(i2c_dev, &tgt_cfg_secondary);

	if (ret == 0) {
		g_secondary_registered = true;
	}

	return ret;
}

static int unregister_secondary_target(void)
{
	if (!g_secondary_registered) {
		return 0;
	}

	int ret = i2c_target_unregister(i2c_dev, &tgt_cfg_secondary);

	if (ret == 0) {
		g_secondary_registered = false;
	}

	return ret;
}

static int unregister_primary_target(void)
{
	if (!g_primary_registered) {
		return 0;
	}

	int ret = i2c_target_unregister(i2c_dev, &tgt_cfg_primary);

	if (ret == 0) {
		g_primary_registered = false;
		g_is_target_mode = false;
	}

	return ret;
}

static int configure_target_mode(uint16_t primary, uint16_t secondary)
{
	int ret;

	ret = unregister_secondary_target();
	if (ret) {
		return ret;
	}

	ret = unregister_primary_target();
	if (ret) {
		return ret;
	}

	ret = register_primary_target(primary);
	if (ret) {
		return ret;
	}

	if (secondary != 0U) {
		ret = register_secondary_target(secondary);
		if (ret) {
			(void)unregister_primary_target();
			return ret;
		}
	}

	return 0;
}

static const struct reply_profile *select_reply_profile(struct i2c_target_config *config,
							uint16_t *addr_out)
{
	uint16_t addr = config->address & 0x3FFU;

	if (addr_out != NULL) {
		*addr_out = addr;
	}

	if (config == &tgt_cfg_secondary ||
	    addr == (tgt_cfg_secondary.address & 0x3FFU)) {
		return &reply_secondary;
	}

	return &reply_primary;
}

static void reset_reply_sequence(struct i2c_target_config *config)
{
	active_reply = select_reply_profile(config, &active_reply_addr);
	read_byte_index = 0;
	LOG_INF("I2C target addr 0x%02x using %s reply", active_reply_addr & 0x7FU,
		active_reply->label);
}

/* --- Shell helpers --- */
static int parse_u8(const char *s, uint8_t *out)
{
    unsigned long v = 0;
    char *endp = NULL;
    if (s == NULL || out == NULL) {
        return -EINVAL;
    }
    v = strtoul(s, &endp, 0);
    if (endp == s || v > 0xFF) {
        return -EINVAL;
    }
    *out = (uint8_t)v;
    return 0;
}

static int parse_u16(const char *s, uint16_t *out)
{
    unsigned long v = 0;
    char *endp = NULL;
    if (s == NULL || out == NULL) {
        return -EINVAL;
    }
    v = strtoul(s, &endp, 0);
    if (endp == s || v > 0x3FF) { /* 10-bit max if used */
        return -EINVAL;
    }
    *out = (uint16_t)v;
    return 0;
}

static int cmd_i2c_mode(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    if (!device_is_ready(i2c_dev)) {
        shell_print(shell, "I2C device not ready");
        return -ENODEV;
    }

    if (strcmp(argv[1], "target") == 0) {
        bool was_target = g_is_target_mode;
        uint16_t addr = g_primary_registered ? tgt_cfg_primary.address : 0x33;
        uint16_t secondary = g_secondary_registered ? tgt_cfg_secondary.address : 0;
        if (argc >= 3) {
            uint16_t tmp;
            if (parse_u16(argv[2], &tmp) != 0 || tmp == 0U) {
                shell_print(shell, "Invalid addr");
                return -EINVAL;
            }
            addr = tmp & 0x7FU;
        }
        if (argc >= 4) {
            uint16_t tmp;
            if (parse_u16(argv[3], &tmp) != 0) {
                shell_print(shell, "Invalid secondary addr");
                return -EINVAL;
            }
            secondary = tmp == 0U ? 0U : (tmp & 0x7FU);
            if (secondary == addr && secondary != 0U) {
                shell_print(shell, "Secondary must differ from primary");
                return -EINVAL;
            }
        }

        if (!g_is_target_mode) {
            memset(tx_buf, 0, sizeof(tx_buf));
            i2c_last_written = 0;
            i2c_next_read = 0;
        }

        int ret = configure_target_mode(addr, secondary);
            if (ret) {
            shell_print(shell, "target configure failed: %d", ret);
                return ret;
            }

        const char *suffix = was_target ? " (updated)" : "";

            if (secondary) {
            shell_print(shell, "Mode: TARGET @0x%02x/0x%02x%s", addr, secondary, suffix);
        } else {
            shell_print(shell, "Mode: TARGET @0x%02x%s", addr, suffix);
            }

        return 0;
    }

    if (strcmp(argv[1], "ctrl") == 0 || strcmp(argv[1], "controller") == 0) {
        if (g_is_target_mode) {
            int ret = unregister_secondary_target();
            if (ret) {
                shell_print(shell, "secondary unregister failed: %d", ret);
                return ret;
            }
            ret = unregister_primary_target();
            if (ret) {
                shell_print(shell, "primary unregister failed: %d", ret);
                return ret;
            }
            g_is_target_mode = false;
        }
        /* Ensure controller config applied */
        int ret = i2c_configure(i2c_dev, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_FAST));
        if (ret) {
            shell_print(shell, "i2c_configure failed: %d", ret);
            return ret;
        }
        shell_print(shell, "Mode: CONTROLLER");
        return 0;
    }

    shell_print(shell, "Usage: i2c mode <target [addr [secondary]]|ctrl>");
    return -EINVAL;
}

static int cmd_i2c_scan(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    if (!device_is_ready(i2c_dev)) {
        shell_print(shell, "I2C not ready");
        return -ENODEV;
    }
    if (g_is_target_mode) {
        shell_print(shell, "Currently in TARGET. Switch to controller first.");
        return -EBUSY;
    }
    shell_print(shell, "Scanning...");
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        uint8_t dummy = 0;
        struct i2c_msg msg = { .buf = &dummy, .len = 0, .flags = I2C_MSG_WRITE | I2C_MSG_STOP };
        int ret = i2c_transfer(i2c_dev, &msg, 1, addr);
        if (ret == 0) {
            shell_print(shell, "Found 0x%02x", addr);
        }
    }
    return 0;
}

static int cmd_i2c_write(const struct shell *shell, size_t argc, char **argv)
{
    if (argc < 3) {
        shell_print(shell, "Usage: i2c write <addr> <b0> [b1 ...]");
        return -EINVAL;
    }
    if (g_is_target_mode) {
        shell_print(shell, "Currently in TARGET. Switch to controller first.");
        return -EBUSY;
    }
    uint16_t addr;
    if (parse_u16(argv[1], &addr) != 0) {
        shell_print(shell, "Invalid addr");
        return -EINVAL;
    }
    uint8_t buf[64];
    size_t n = argc - 2;
    if (n > sizeof(buf)) n = sizeof(buf);
    for (size_t i = 0; i < n; i++) {
        if (parse_u8(argv[2 + i], &buf[i]) != 0) {
            shell_print(shell, "Invalid byte at %d", (int)i);
            return -EINVAL;
        }
    }
    struct i2c_msg msg = { .buf = buf, .len = n, .flags = I2C_MSG_WRITE | I2C_MSG_STOP };
    int ret = i2c_transfer(i2c_dev, &msg, 1, addr);
    shell_print(shell, "write ret=%d", ret);
    return ret;
}

static int cmd_i2c_read(const struct shell *shell, size_t argc, char **argv)
{
    if (argc < 3) {
        shell_print(shell, "Usage: i2c read <addr> <len>");
        return -EINVAL;
    }
    if (g_is_target_mode) {
        shell_print(shell, "Currently in TARGET. Switch to controller first.");
        return -EBUSY;
    }
    uint16_t addr; unsigned long len_ul;
    if (parse_u16(argv[1], &addr) != 0) {
        shell_print(shell, "Invalid addr");
        return -EINVAL;
    }
    char *endp = NULL;
    len_ul = strtoul(argv[2], &endp, 0);
    if (endp == argv[2] || len_ul == 0 || len_ul > 64) {
        shell_print(shell, "Invalid len (1..64)");
        return -EINVAL;
    }
    uint8_t buf[64];
    size_t n = (size_t)len_ul;
    struct i2c_msg msg = { .buf = buf, .len = n, .flags = I2C_MSG_READ | I2C_MSG_STOP };
    int ret = i2c_transfer(i2c_dev, &msg, 1, addr);
    if (ret == 0) {
        shell_print(shell, "read:");
        for (size_t i = 0; i < n; i++) {
            shell_fprintf(shell, SHELL_NORMAL, "%02x ", buf[i]);
        }
        shell_print(shell, "");
    } else {
        shell_print(shell, "read ret=%d", ret);
    }
    return ret;
}

static int cmd_i2c_stats(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if defined(CONFIG_I2C_TARGET_BUFFER_MODE)
	uint32_t rx_blocks = (uint32_t)atomic_get(&dma_stats.rx_blocks);
	uint32_t rx_bytes = (uint32_t)atomic_get(&dma_stats.rx_bytes);
	uint32_t rx_last = (uint32_t)atomic_get(&dma_stats.rx_last);
	uint32_t tx_blocks = (uint32_t)atomic_get(&dma_stats.tx_blocks);
	uint32_t tx_bytes = (uint32_t)atomic_get(&dma_stats.tx_bytes);
	uint32_t tx_last = (uint32_t)atomic_get(&dma_stats.tx_last);

	shell_print(shell, "Target DMA stats:");
	shell_print(shell, "  RX blocks=%u bytes=%u last_block=%u", rx_blocks, rx_bytes, rx_last);
	shell_print(shell, "  TX blocks=%u bytes=%u last_block=%u", tx_blocks, tx_bytes, tx_last);
#else
	shell_print(shell, "Target buffer mode disabled; no DMA stats collected.");
#endif

	shell_print(shell, "Current mode: %s", g_is_target_mode ? "TARGET" : "CONTROLLER");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_i2c,
    SHELL_CMD(mode, NULL, "i2c mode <target [addr]|ctrl>", cmd_i2c_mode),
    SHELL_CMD(scan, NULL, "i2c scan", cmd_i2c_scan),
    SHELL_CMD(write, NULL, "i2c write <addr> <b0> [b1 ...]", cmd_i2c_write),
    SHELL_CMD(read, NULL, "i2c read <addr> <len>", cmd_i2c_read),
	SHELL_CMD(stats, NULL, "Show DMA statistics", cmd_i2c_stats),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(i2c, &sub_i2c, "I2C control/target commands", NULL);

int main(void)
{
	int ret;

	LOG_INF("i2c_test_app start");

	/* Initialize LED */
	if (!device_is_ready(led.port)) {
		LOG_ERR("LED GPIO device not ready");
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("Failed to configure LED: %d", ret);
		return -EINVAL;
	}

	/* Ensure I2C device is ready */
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

    /* Register as I2C target (slave) by default on addresses 0x33 and 0x66 */
	ret = configure_target_mode(0x33, 0x66);
    if (ret) {
		LOG_ERR("configure_target_mode failed: %d", ret);
        return -EINVAL;
    }
    LOG_INF("I2C target registered on addresses 0x33 (51) and 0x66 (102)");

	/* Blink loop like blinky */
	while (1) {
		gpio_pin_toggle_dt(&led);
		k_msleep(500);
	}

    return 0;
}
