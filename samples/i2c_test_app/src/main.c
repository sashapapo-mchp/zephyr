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
#include <zephyr/sys/util.h>

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

/* Simple I2C target behavior: echo last written byte, increment on reads */
static uint8_t i2c_last_written;
static uint8_t i2c_next_read;
static uint8_t tx_buf[16];
static uint32_t tx_len = 0;
static uint8_t read_byte_index = 0; /* Track which byte to send in sequence */
static const uint8_t reply_bytes[4] = {0x11, 0x22, 0x33, 0x44}; /* Fixed reply sequence */

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
	ARG_UNUSED(config);
	/* Reset index and send first byte of fixed sequence */
	read_byte_index = 0;
	*val = reply_bytes[read_byte_index];
	LOG_INF("I2C read first: 0x%02x (byte %d/4)", *val, read_byte_index + 1);
	read_byte_index++;
	return 0;
}

static int cb_read_processed(struct i2c_target_config *config, uint8_t *val)
{
	ARG_UNUSED(config);
	/* Send next byte in fixed sequence, wrap around after 4 bytes */
	if (read_byte_index < 4) {
		*val = reply_bytes[read_byte_index];
		LOG_INF("I2C read next: 0x%02x (byte %d/4)", *val, read_byte_index + 1);
		read_byte_index++;
	} else {
		/* After 4 bytes, keep sending the last byte */
		*val = reply_bytes[3];
		LOG_INF("I2C read next: 0x%02x (repeat last)", *val);
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
    ARG_UNUSED(config);
    /* Prepare fixed 4-byte response */
    tx_len = 4;
    tx_buf[0] = 0x11;
    tx_buf[1] = 0x22;
    tx_buf[2] = 0x33;
    tx_buf[3] = 0x44;
    *ptr = tx_buf;
    *len = tx_len;
	atomic_inc(&dma_stats.tx_blocks);
	atomic_add(&dma_stats.tx_bytes, (atomic_val_t)tx_len);
	atomic_set(&dma_stats.tx_last, (atomic_val_t)tx_len);
    LOG_INF("I2C buf read: offering %u bytes (0x11 0x22 0x33 0x44)", tx_len);
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

/* Target configuration: respond on addresses 0x28 (28) and 0x37 (55) */
static struct i2c_target_config tgt_cfg = {
    .address = 0x33,
    .flags = 0,
    .callbacks = &i2c_cb,
    .address_mask = 0,
    .secondary_address = 0x66, 
};

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
        uint16_t addr = tgt_cfg.address;
        uint16_t secondary = tgt_cfg.secondary_address;
        if (argc >= 3) {
            uint16_t tmp;
            if (parse_u16(argv[2], &tmp) != 0 || tmp == 0) {
                shell_print(shell, "Invalid addr");
                return -EINVAL;
            }
            addr = tmp & 0x7F; /* 7-bit by default */
        }
        if (argc >= 4) {
            uint16_t tmp;
            if (parse_u16(argv[3], &tmp) != 0) {
                shell_print(shell, "Invalid secondary addr");
                return -EINVAL;
            }
            secondary = tmp == 0 ? 0 : (tmp & 0x7F);
            if (secondary == addr && secondary != 0) {
                shell_print(shell, "Secondary must differ from primary");
                return -EINVAL;
            }
        }

        if (!g_is_target_mode) {
            /* Clear buffers before enabling target mode */
            memset(tx_buf, 0, sizeof(tx_buf));
            i2c_last_written = 0;
            i2c_next_read = 0;

            /* Update address and register */
            tgt_cfg.address = addr;
            tgt_cfg.secondary_address = secondary;
            int ret = i2c_target_register(i2c_dev, &tgt_cfg);
            if (ret) {
                shell_print(shell, "target_register failed: %d", ret);
                return ret;
            }
            g_is_target_mode = true;
            if (secondary) {
                shell_print(shell, "Mode: TARGET @0x%02x/0x%02x", addr, secondary);
            } else {
                shell_print(shell, "Mode: TARGET @0x%02x", addr);
            }
        } else {
            /* Re-register with new address: unregister then register */
            (void)i2c_target_unregister(i2c_dev, &tgt_cfg);
            tgt_cfg.address = addr;
            tgt_cfg.secondary_address = secondary;
            int ret = i2c_target_register(i2c_dev, &tgt_cfg);
            if (ret) {
                shell_print(shell, "target_register failed: %d", ret);
                return ret;
            }
            if (secondary) {
                shell_print(shell, "Mode: TARGET @0x%02x/0x%02x (updated)", addr, secondary);
            } else {
                shell_print(shell, "Mode: TARGET @0x%02x (updated)", addr);
            }
        }
        return 0;
    }

    if (strcmp(argv[1], "ctrl") == 0 || strcmp(argv[1], "controller") == 0) {
        if (g_is_target_mode) {
            int ret = i2c_target_unregister(i2c_dev, &tgt_cfg);
            if (ret) {
                shell_print(shell, "target_unregister failed: %d", ret);
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

	LOG_INF("i2c_test_app start test");

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

    /* Register as I2C target (slave) by default on addresses 0x28 and 0x37 */
    ret = i2c_target_register(i2c_dev, &tgt_cfg);
    if (ret) {
        LOG_ERR("i2c_target_register failed: %d", ret);
        return -EINVAL;
    }
    g_is_target_mode = true;
    LOG_INF("I2C target registered on addresses 0x28 (28) and 0x37 (55)");

	/* Blink loop like blinky */
	while (1) {
		gpio_pin_toggle_dt(&led);
		k_msleep(500);
	}

    return 0;
}
