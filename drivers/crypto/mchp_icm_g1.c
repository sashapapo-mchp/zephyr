/*
 * Copyright (c) 2024 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * ICM (Integrity Check Monitor) Crypto Driver for Microchip PIC32CX/SAM E5x
 *
 * This driver implements the Zephyr crypto hash API using the ICM hardware
 * accelerator for SHA-256 and SHA-224 hash computation.
 */

#define DT_DRV_COMPAT microchip_icm_g1

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

/* Include Microchip HAL headers */
#include <component/icm.h>
#include <component/mclk.h>

LOG_MODULE_REGISTER(crypto_mchp_icm, CONFIG_CRYPTO_LOG_LEVEL);

/* Driver capabilities */
#define ICM_CAPS_SUPPORT (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

/* SHA block and digest sizes */
#define SHA256_BLOCK_SIZE   64
#define SHA256_DIGEST_SIZE  32
#define SHA224_DIGEST_SIZE  28
#define SHA256_DIGEST_WORDS 8

/* Maximum sessions from Kconfig */
#define ICM_MAX_SESSION CONFIG_CRYPTO_MCHP_ICM_MAX_SESSION

/*
 * ICM Region Descriptor (16 bytes each, must be contiguous)
 * The descriptor array base must be 64-byte aligned.
 */
struct icm_descriptor {
	uint32_t raddr;  /* Region start address */
	uint32_t rcfg;   /* Region configuration */
	uint32_t rctrl;  /* Region control (TRSIZE) */
	uint32_t rnext;  /* Next descriptor address */
};

/*
 * Per-session context
 */
struct icm_session {
	struct icm_descriptor desc __aligned(64);
	uint32_t hash_output[SHA256_DIGEST_WORDS] __aligned(128);
	enum hash_algo algo;
	bool in_use;
	bool started;
	/* For multi-part hashing, we'd need intermediate state storage */
};

/*
 * Driver configuration (from device tree)
 */
struct icm_config {
	icm_registers_t *regs;
	uint32_t irq_num;
	void (*irq_config_func)(const struct device *dev);
};

/*
 * Driver runtime data
 */
struct icm_data {
	struct k_sem lock;
	struct k_sem sync;
	struct icm_session sessions[ICM_MAX_SESSION];
	bool clocks_enabled;
};

/*
 * MCLK register access for clock control
 * Note: Using direct register access as MCLK_REGS may not be properly defined
 */
#define MCLK_AHBMASK_REG    (*(volatile uint32_t *)(0x40000800 + 0x10))
#define MCLK_APBCMASK_REG   (*(volatile uint32_t *)(0x40000800 + 0x1C))

static int icm_enable_clocks(struct icm_data *data)
{
	if (data->clocks_enabled) {
		return 0;
	}

	/* Enable ICM AHB clock (bit 19) and APBC clock (bit 11) */
	MCLK_AHBMASK_REG |= MCLK_AHBMASK_ICM_Msk;
	MCLK_APBCMASK_REG |= MCLK_APBCMASK_ICM_Msk;

	/* Verify clocks are enabled */
	if (!(MCLK_AHBMASK_REG & MCLK_AHBMASK_ICM_Msk) ||
	    !(MCLK_APBCMASK_REG & MCLK_APBCMASK_ICM_Msk)) {
		LOG_ERR("Failed to enable ICM clocks");
		return -EIO;
	}

	data->clocks_enabled = true;
	LOG_DBG("ICM clocks enabled");
	return 0;
}

static void icm_reset(const struct icm_config *cfg)
{
	icm_registers_t *regs = cfg->regs;

	/* Disable ICM */
	regs->ICM_CTRL = ICM_CTRL_DISABLE_Msk;
	k_busy_wait(100);

	/* Software reset */
	regs->ICM_CTRL = ICM_CTRL_SWRST_Msk;
	k_busy_wait(1000);

	/* Wait for reset to complete */
	int timeout = 100;
	while ((regs->ICM_SR != 0) && (timeout-- > 0)) {
		k_busy_wait(100);
	}

	/* Clear pending interrupts */
	(void)regs->ICM_ISR;
}

static int icm_get_session(struct icm_data *data)
{
	for (int i = 0; i < ICM_MAX_SESSION; i++) {
		if (!data->sessions[i].in_use) {
			data->sessions[i].in_use = true;
			data->sessions[i].started = false;
			return i;
		}
	}
	return -1;
}

static void icm_free_session_idx(struct icm_data *data, int idx)
{
	if (idx >= 0 && idx < ICM_MAX_SESSION) {
		memset(&data->sessions[idx], 0, sizeof(struct icm_session));
		data->sessions[idx].in_use = false;
	}
}

/*
 * Hash computation implementation
 *
 * Note: The ICM hardware is designed for memory integrity monitoring and
 * processes data in 64-byte blocks. For proper SHA-256 computation, the
 * input data must include proper SHA padding.
 *
 * This implementation handles single-shot hash operations. Multi-part
 * hashing would require additional state management.
 */
static int icm_hash_compute(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
	const struct device *dev = ctx->device;
	const struct icm_config *cfg = dev->config;
	struct icm_data *data = dev->data;
	struct icm_session *session = ctx->drv_sessn_state;
	icm_registers_t *regs = cfg->regs;
	int ret = 0;

	if (!session || !session->in_use) {
		LOG_ERR("Invalid session");
		return -EINVAL;
	}

	/* For now, we only support single-shot operations with finish=true */
	if (!finish) {
		LOG_ERR("Multi-part hashing not yet supported");
		return -ENOTSUP;
	}

	/* Validate input */
	if (!pkt->in_buf || pkt->in_len == 0) {
		LOG_ERR("Invalid input buffer");
		return -EINVAL;
	}

	/* Data must be 64-byte aligned and length must be multiple of 64 */
	if (((uint32_t)pkt->in_buf & 0x3F) != 0) {
		LOG_ERR("Input buffer not 64-byte aligned");
		return -EINVAL;
	}
	if ((pkt->in_len % SHA256_BLOCK_SIZE) != 0) {
		LOG_ERR("Input length must be multiple of 64 bytes (with SHA padding)");
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);

	ret = icm_enable_clocks(data);
	if (ret != 0) {
		goto out;
	}

	icm_reset(cfg);

	/* Configure ICM_CFG:
	 *   - WBDIS = 0: Write back enabled (hash written to output)
	 *   - SLBDIS = 1: Secondary list branch disable
	 *   - UALGO = algorithm selection
	 */
	uint32_t algo_cfg;
	switch (session->algo) {
	case CRYPTO_HASH_ALGO_SHA256:
		algo_cfg = ICM_CFG_UALGO_SHA256;
		break;
	case CRYPTO_HASH_ALGO_SHA224:
		algo_cfg = ICM_CFG_UALGO_SHA224;
		break;
	default:
		LOG_ERR("Unsupported algorithm: %d", session->algo);
		ret = -EINVAL;
		goto out;
	}

	regs->ICM_CFG = ICM_CFG_SLBDIS_Msk | algo_cfg;

	/* Verify CFG write */
	if (regs->ICM_CFG != (ICM_CFG_SLBDIS_Msk | algo_cfg)) {
		LOG_ERR("ICM_CFG write failed");
		ret = -EIO;
		goto out;
	}

	/* Setup region descriptor */
	memset(&session->desc, 0, sizeof(session->desc));
	session->desc.raddr = (uint32_t)pkt->in_buf;
	session->desc.rcfg = ICM_RCFG_ALGO(
		(session->algo == CRYPTO_HASH_ALGO_SHA256) ?
		ICM_CFG_UALGO_SHA256_Val : ICM_CFG_UALGO_SHA224_Val) |
		ICM_RCFG_EOM_Msk;
	session->desc.rctrl = ICM_RCTRL_TRSIZE((pkt->in_len / SHA256_BLOCK_SIZE) - 1);
	session->desc.rnext = 0;

	/* Clear hash output */
	memset(session->hash_output, 0, sizeof(session->hash_output));

	/* Set descriptor and hash area addresses */
	regs->ICM_DSCR = (uint32_t)&session->desc;
	regs->ICM_HASH = (uint32_t)session->hash_output;

	/* Start hash computation */
	regs->ICM_CTRL = ICM_CTRL_ENABLE_Msk;

	/* Wait for completion */
	int timeout = 10000;
	uint32_t isr = 0;
	while (timeout > 0) {
		isr = regs->ICM_ISR;
		if (isr & ICM_ISR_RHC(1)) {
			/* Region 0 hash completed */
			break;
		}
		if (isr & ICM_ISR_URAD_Msk) {
			LOG_ERR("ICM URAD error: UASR=0x%08x", regs->ICM_UASR);
			ret = -EIO;
			goto disable;
		}
		k_busy_wait(10);
		timeout--;
	}

	if (timeout <= 0) {
		LOG_ERR("ICM hash timeout");
		ret = -ETIMEDOUT;
		goto disable;
	}

	/* Copy hash result to output buffer
	 * ICM stores each word in little-endian; extract bytes LSB first
	 */
	int digest_words = (session->algo == CRYPTO_HASH_ALGO_SHA256) ?
			   8 : 7; /* SHA-256: 8 words, SHA-224: 7 words */
	for (int i = 0; i < digest_words; i++) {
		uint32_t word = session->hash_output[i];
		pkt->out_buf[i * 4 + 0] = (word >> 0) & 0xFF;
		pkt->out_buf[i * 4 + 1] = (word >> 8) & 0xFF;
		pkt->out_buf[i * 4 + 2] = (word >> 16) & 0xFF;
		pkt->out_buf[i * 4 + 3] = (word >> 24) & 0xFF;
	}

	ret = 0;

disable:
	regs->ICM_CTRL = ICM_CTRL_DISABLE_Msk;

out:
	k_sem_give(&data->lock);
	return ret;
}

/*
 * Session management
 */
static int icm_hash_begin_session(const struct device *dev, struct hash_ctx *ctx,
				  enum hash_algo algo)
{
	struct icm_data *data = dev->data;
	int idx;

	/* Validate algorithm */
	if (algo != CRYPTO_HASH_ALGO_SHA256 && algo != CRYPTO_HASH_ALGO_SHA224) {
		LOG_ERR("Unsupported algorithm: %d", algo);
		return -EINVAL;
	}

	/* Check capabilities */
	if (ctx->flags & ~ICM_CAPS_SUPPORT) {
		LOG_ERR("Unsupported flags: 0x%x", ctx->flags);
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);

	idx = icm_get_session(data);
	if (idx < 0) {
		LOG_ERR("No free sessions");
		k_sem_give(&data->lock);
		return -ENOSPC;
	}

	data->sessions[idx].algo = algo;
	ctx->drv_sessn_state = &data->sessions[idx];
	ctx->device = dev;
	ctx->hash_hndlr = icm_hash_compute;
	ctx->started = false;

	k_sem_give(&data->lock);

	LOG_DBG("Hash session started (algo=%d, idx=%d)", algo, idx);
	return 0;
}

static int icm_hash_free_session(const struct device *dev, struct hash_ctx *ctx)
{
	struct icm_data *data = dev->data;
	struct icm_session *session = ctx->drv_sessn_state;

	if (!session) {
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);

	/* Find session index and free it */
	for (int i = 0; i < ICM_MAX_SESSION; i++) {
		if (&data->sessions[i] == session) {
			icm_free_session_idx(data, i);
			break;
		}
	}

	ctx->drv_sessn_state = NULL;
	ctx->hash_hndlr = NULL;

	k_sem_give(&data->lock);

	LOG_DBG("Hash session freed");
	return 0;
}

static int icm_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ICM_CAPS_SUPPORT;
}

/*
 * Driver initialization
 */
static int icm_init(const struct device *dev)
{
	const struct icm_config *cfg = dev->config;
	struct icm_data *data = dev->data;

	k_sem_init(&data->lock, 1, 1);
	k_sem_init(&data->sync, 0, 1);

	memset(data->sessions, 0, sizeof(data->sessions));
	data->clocks_enabled = false;

	/* Optionally configure interrupt (for async mode in future) */
	if (cfg->irq_config_func) {
		cfg->irq_config_func(dev);
	}

	LOG_INF("ICM crypto driver initialized");
	return 0;
}

/*
 * Driver API
 */
static DEVICE_API(crypto, icm_driver_api) = {
	.query_hw_caps = icm_query_hw_caps,
	.hash_begin_session = icm_hash_begin_session,
	.hash_free_session = icm_hash_free_session,
	/* Cipher operations not supported */
	.cipher_begin_session = NULL,
	.cipher_free_session = NULL,
	.cipher_async_callback_set = NULL,
	.hash_async_callback_set = NULL,
};

/*
 * Device instantiation
 */
#define ICM_INIT(n)                                                         \
	static void icm_irq_config_##n(const struct device *dev);           \
                                                                            \
	static const struct icm_config icm_config_##n = {                   \
		.regs = (icm_registers_t *)DT_INST_REG_ADDR(n),             \
		.irq_num = DT_INST_IRQN(n),                                 \
		.irq_config_func = icm_irq_config_##n,                      \
	};                                                                  \
                                                                            \
	static struct icm_data icm_data_##n;                                \
                                                                            \
	DEVICE_DT_INST_DEFINE(n, icm_init, NULL,                            \
			      &icm_data_##n, &icm_config_##n,               \
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,     \
			      &icm_driver_api);                             \
                                                                            \
	static void icm_irq_config_##n(const struct device *dev)            \
	{                                                                   \
		ARG_UNUSED(dev);                                            \
		/* IRQ configuration for future async support */            \
	}

DT_INST_FOREACH_STATUS_OKAY(ICM_INIT)
