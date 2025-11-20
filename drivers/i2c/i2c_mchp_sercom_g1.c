/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file i2c_mchp_sercom_g1.c
 * @brief I2C driver for Microchip SERCOM_G1 peripheral.
 *
 * This driver provides an implementation of the Zephyr I2C API for the
 * Microchip SERCOM_G1 peripheral.
 *
 */

#include <soc.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/dma.h>
#include <mchp_dt_helper.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>

/*******************************************
 * @brief Devicetree definitions
 ********************************************/
#define DT_DRV_COMPAT microchip_sercom_g1_i2c

/*******************************************
 * Const and Macro Defines
 *******************************************/
/**
 * @brief Register I2C MCHP G1 driver with logging subsystem.
 */
LOG_MODULE_REGISTER(i2c_mchp_sercom_g1, CONFIG_I2C_LOG_LEVEL);

/**
 * @brief Including header here to avoid compilation error
 *
 * As i2c-priv.h header file usage Log messages, LOG_LEVEL has to be defined before.
 */
#include "i2c-priv.h"

#if defined(CONFIG_I2C_MCHP_DMA_DRIVEN) && defined(CONFIG_I2C_TARGET) && \
	defined(CONFIG_I2C_TARGET_BUFFER_MODE)
/* Forward declarations for target DMA helpers */
static int i2c_target_dma_rx_start(const struct device *dev);
static int i2c_target_dma_tx_start(const struct device *dev, uint8_t *ptr, uint32_t len);
#endif

/**
 * @def I2C_MCHP_SUCCESS
 * @brief Macro indicating successful operation.
 */
#define I2C_MCHP_SUCCESS 0

/* Macro to check message direction in read mode */
#define I2C_MCHP_MESSAGE_DIR_READ_MASK 1

#ifndef I2C_INVALID_ADDR
#define I2C_INVALID_ADDR 0x00
#endif

/** I2C Fast Speed: 400 kHz */
#define I2C_MCHP_SPEED_FAST 400000

/** I2C Fast plus Speed: 1 MHz */
#define I2C_MCHP_SPEED_FAST_PLUS 1000000

/* I2C high Speed: 3.4 MHz */
#define I2C_MCHP_SPEED_HIGH_SPEED 3400000

/* Combined max of BAUD_LOW and BAUD.*/
#define I2C_BAUD_LOW_HIGH_MAX 382U

/* i2c start condion setup time 100ns */
#define I2C_MCHP_START_CONDITION_SETUP_TIME (100.0f / 1000000000.0f)

/* I2C message direction: read operation. */
#define I2C_MESSAGE_DIR_READ 1U

#ifdef CONFIG_I2C_MCHP_TRANSFER_TIMEOUT
/* I2C transfer timeout in milliseconds, configurable via Kconfig. */
#define I2C_TRANSFER_TIMEOUT_MSEC K_MSEC(CONFIG_I2C_MCHP_TRANSFER_TIMEOUT)
#else
/* I2C transfer timeout set to infinite if not configured. */
#define I2C_TRANSFER_TIMEOUT_MSEC K_FOREVER
#endif /*CONFIG_I2C_MCHP_TRANSFER_TIMEOUT*/

/* I2C_REGS Register */
#define I2C_REGS ((const struct i2c_mchp_dev_config *)(dev)->config)->regs

/* ACK received status */
#define I2C_MCHP_TARGET_ACK_STATUS_RECEIVED_ACK 0

/* NACK received status */
#define I2C_MCHP_TARGET_ACK_STATUS_RECEIVED_NACK 1

#define I2C_MCHP_STATUS_ERROR_MASK                                                      \
	(SERCOM_I2CM_STATUS_BUSERR_Msk | SERCOM_I2CM_STATUS_ARBLOST_Msk |               \
	 SERCOM_I2CM_STATUS_LOWTOUT_Msk | SERCOM_I2CM_STATUS_MEXTTOUT_Msk |             \
	 SERCOM_I2CM_STATUS_SEXTTOUT_Msk | SERCOM_I2CM_STATUS_LENERR_Msk)

/*******************************************
 * Enum and structs
 *******************************************/
/**
 * @enum i2c_mchp_target_cmd
 * @brief I2C target command options for SERCOM I2C peripheral.
 *
 * Defines commands for ACK/NACK and transaction control in I2C target mode.
 */
enum i2c_mchp_target_cmd {
	I2C_MCHP_TARGET_COMMAND_SEND_ACK = 0,
	I2C_MCHP_TARGET_COMMAND_SEND_NACK,
	I2C_MCHP_TARGET_COMMAND_RECEIVE_ACK_NAK,
	I2C_MCHP_TARGET_COMMAND_WAIT_FOR_START
};

/* Forward declaration for target command API used by target-DMA helpers */
void i2c_target_set_command(const struct device *dev, enum i2c_mchp_target_cmd cmd);

/**
 * @struct i2c_mchp_clock
 * @brief Structure to hold device clock configuration.
 */
struct i2c_mchp_clock {

	/* Clock driver */
	const struct device *clock_dev;

	/* Main clock subsystem. */
	clock_control_subsys_t mclk_sys;

	/* Generic clock subsystem. */
	clock_control_subsys_t gclk_sys;
};

/**
 * @struct i2c_mchp_dma
 * @brief DMA configuration parameters for I2C operations.
 *
 * This structure holds the configuration required to enable DMA-based
 * asynchronous transfers for the I2C peripheral, including device pointers,
 * request lines, and channel numbers for both TX and RX directions.
 */
struct i2c_mchp_dma {

	/* DMA device used for asynchronous operations. */
	const struct device *dma_dev;

	/* DMA request line for TX (transmit) operations. */
	uint8_t tx_dma_request;

	/* DMA channel number for TX (transmit) operations. */
	uint8_t tx_dma_channel;

	/* DMA request line for RX (receive) operations. */
	uint8_t rx_dma_request;

	/* DMA channel number for RX (receive) operations. */
	uint8_t rx_dma_channel;
};

/**
 * @struct i2c_mchp_dev_config
 * @brief Configuration structure for the MCHP I2C driver.
 *
 * This structure contains all necessary configuration parameters for
 * initializing and operating the MCHP I2C driver, including hardware
 * abstraction, clock and pin settings, bitrate, IRQ configuration, and
 * optional DMA support.
 */
struct i2c_mchp_dev_config {

	/* Hardware Abstraction Layer for the I2C peripheral. */
	sercom_registers_t *regs;

	/* Clock configuration for the I2C peripheral. */
	struct i2c_mchp_clock i2c_clock;

	/* Pin configuration for SDA and SCL lines. */
	const struct pinctrl_dev_config *pcfg;

	/* Default bitrate for I2C communication (e.g., 100 kHz, 400 kHz). */
	uint32_t bitrate;

	/* Function pointer to configure IRQ during initialization. */
	void (*irq_config_func)(const struct device *dev);

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
	/* DMA configuration for I2C transfers (enabled if DMA is used). */
	struct i2c_mchp_dma i2c_dma;
#endif /*CONFIG_I2C_MCHP_DMA_DRIVEN*/

	/* Enable peripheral operation in standby sleep mode. */
	uint8_t run_in_standby;
};

/**
 * @struct i2c_mchp_msg
 * @brief Structure representing an I2C message for the MCHP I2C peripheral.
 *
 * This structure encapsulates the data buffer, size, and status information
 * for an I2C message transaction using the MCHP I2C peripheral.
 */
struct i2c_mchp_msg {

	/* Pointer to the data buffer for the I2C message. */
	uint8_t *buffer;

	/* Size of the I2C message in bytes. */
	uint32_t size;

	/* Status of the I2C message, indicating success or error conditions. */
	uint16_t status;
};

/**
 * @struct i2c_mchp_dev_data
 * @brief Structure representing runtime data for the MCHP I2C driver.
 *
 * This structure contains all runtime data required for managing the
 * operation of the MCHP I2C driver, including synchronization primitives,
 * message tracking, configuration, and optional callback and target mode
 * support.
 */
struct i2c_mchp_dev_data {

	/**< Pointer to the I2C device instance. */
	const struct device *dev;

	/* Mutex for protecting access to the I2C driver data. */
	struct k_mutex i2c_bus_mutex;

	/* Semaphore for signaling completion of I2C transfers. */
	struct k_sem i2c_sync_sem;

	/* Structure representing the current I2C message. */
	struct i2c_mchp_msg current_msg;

	/* Pointer to an array of I2C messages being transferred. */
	struct i2c_msg *msgs_array;

	/* Number of messages in the current I2C transfer sequence. */
	uint8_t num_msgs;

	/* Flag indicating whether the device is in target mode. */
	bool target_mode;

	/* Current I2C device configuration settings. */
	uint32_t dev_config;

	/* Address of the I2C target device. */
	uint32_t target_addr;

	/* Variable to track message buffer by indexing. */
	uint8_t msg_index;

#ifdef CONFIG_I2C_CALLBACK
	/* Callback function for asynchronous I2C operations. */
	i2c_callback_t i2c_async_callback;

	/* User data passed to the callback function. */
	void *user_data;
#endif /*CONFIG_I2C_CALLBACK*/

#ifdef CONFIG_I2C_TARGET
	/* Registered target configs (hardware supports up to two addresses). */
	struct i2c_target_config *target_cfgs[2];
	uint8_t target_cfg_count;

	/* Pointer to the config currently being served on the bus. */
	struct i2c_target_config *active_target_cfg;

	/* Data buffer for RX/TX operations in target mode. */
	uint8_t rx_tx_data;

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	/* Target DMA state (buffer-mode) */
	uint8_t *tgt_tx_buf;
	uint32_t tgt_tx_len;
	bool tgt_tx_dma_active;
	bool tgt_tx_buf_active;

	uint8_t __aligned(4) tgt_rx_buf[CONFIG_I2C_MCHP_TARGET_MAX_BUF_SIZE];
	uint32_t tgt_rx_block_size;
	bool tgt_rx_dma_active;
	bool tgt_drdy_masked;

	struct i2c_target_config *dma_rx_target_cfg;
#endif /* CONFIG_I2C_TARGET_BUFFER_MODE */
	/* First byte read after an address match. */
	bool firstReadAfterAddrMatch;
#endif /*CONFIG_I2C_TARGET*/
};

/* target DMA helpers moved below helper functions */

/*******************************************
 * Helper functions
 *******************************************/
#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
static int i2c_dma_write_config(const struct device *dev);
static int i2c_dma_read_config(const struct device *dev);

/**
 * @brief Get the source address for I2C DMA transfers.
 *
 * Returns a pointer to the I2C data register, which serves as the source address
 * for DMA transfers in I2C controller mode. This address is typically used to
 * configure the DMA controller for data transmission.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return Pointer to the I2C data register to be used as the DMA source address.
 */
static inline void *i2c_get_dma_source_addr(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;

#ifdef CONFIG_I2C_TARGET
	if (data->target_mode == true) {
		return (void *)&(I2C_REGS->I2CS.SERCOM_DATA);
	}
#endif /* CONFIG_I2C_TARGET */

	return (void *)&(I2C_REGS->I2CM.SERCOM_DATA);
}

/**
 * @brief Get the destination address for I2C DMA transfers.
 *
 * Returns a pointer to the I2C data register, which serves as the destination address
 * for DMA transfers in I2C controller mode. This address is typically used to
 * configure the DMA controller for data reception.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return Pointer to the I2C data register to be used as the DMA destination address.
 */
static inline void *i2c_get_dma_dest_addr(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;

#ifdef CONFIG_I2C_TARGET
	if (data->target_mode == true) {
		return (void *)&(I2C_REGS->I2CS.SERCOM_DATA);
	}
#endif /* CONFIG_I2C_TARGET */

	return (void *)&(I2C_REGS->I2CM.SERCOM_DATA);
}
#endif /*CONFIG_I2C_MCHP_DMA_DRIVEN*/

#if defined(CONFIG_I2C_MCHP_DMA_DRIVEN) && defined(CONFIG_I2C_TARGET) && \
	defined(CONFIG_I2C_TARGET_BUFFER_MODE)
static void i2c_target_mask_drdy_for_dma(const struct device *dev);
static void i2c_target_unmask_drdy_after_dma(const struct device *dev);
/* Target-mode DMA: RX (host write -> target receive) completion */
static void i2c_target_dma_rx_done(const struct device *dma_dev, void *arg,
					uint32_t channel, int status)
{
	const struct device *dev = arg;
	struct i2c_mchp_dev_data *data = dev->data;
	const struct i2c_mchp_dev_config *const cfg = dev->config;
	struct i2c_target_config *cfg_active = data->dma_rx_target_cfg ?
		data->dma_rx_target_cfg : data->active_target_cfg;
	const struct i2c_target_callbacks *target_cb =
		(cfg_active != NULL) ? cfg_active->callbacks : NULL;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		/* DMA error: stop and clear active state */
		data->tgt_rx_dma_active = false;
		(void)dma_stop(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel);
		i2c_target_unmask_drdy_after_dma(dev);
		return;
	}

	/* Full block received: inform client and re-arm for next block. */
	if (target_cb && target_cb->buf_write_received) {
		target_cb->buf_write_received(cfg_active, data->tgt_rx_buf,
					      data->tgt_rx_block_size);
		LOG_DBG("Target RX DMA block complete (%u bytes)", data->tgt_rx_block_size);
	}

	/* Re-arm for next block if still in write direction; safe to re-arm here */
	if (data->tgt_rx_dma_active) {
		/* Reconfigure and restart RX DMA */
		struct dma_config dma_cfg = {0};
		struct dma_block_config dma_blk = {0};

		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_cfg.source_data_size = 1;
		dma_cfg.dest_data_size = 1;
		dma_cfg.user_data = (void *)dev;
		dma_cfg.dma_callback = i2c_target_dma_rx_done;
		dma_cfg.complete_callback_en = 1;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;
		dma_cfg.dma_slot = cfg->i2c_dma.rx_dma_request;

		dma_blk.block_size = data->tgt_rx_block_size;
		dma_blk.dest_address = (uint32_t)data->tgt_rx_buf;
		dma_blk.source_address = (uint32_t)i2c_get_dma_source_addr(dev);
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		if (dma_config(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel, &dma_cfg) == 0) {
			if (dma_start(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel) == 0) {
				LOG_DBG("Target RX DMA re-armed for %u bytes", data->tgt_rx_block_size);
			} else {
				data->tgt_rx_dma_active = false;
				LOG_ERR("Failed to restart RX DMA");
				i2c_target_unmask_drdy_after_dma(dev);
			}
		} else {
			data->tgt_rx_dma_active = false;
			LOG_ERR("Failed to reconfigure RX DMA");
			i2c_target_unmask_drdy_after_dma(dev);
		}
	}
}

/* Target-mode DMA: TX (host read -> target transmit) completion */
static void i2c_target_dma_tx_done(const struct device *dma_dev, void *arg,
					uint32_t channel, int status)
{
	const struct device *dev = arg;
	struct i2c_mchp_dev_data *data = dev->data;
	const struct i2c_mchp_dev_config *const cfg = dev->config;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	/* Regardless of status, mark TX DMA inactive and reset state */
	data->tgt_tx_dma_active = false;
	uint32_t completed = data->tgt_tx_len;
	data->tgt_tx_buf = NULL;
	data->tgt_tx_len = 0U;
	i2c_target_unmask_drdy_after_dma(dev);

	if (status < 0) {
		(void)dma_stop(cfg->i2c_dma.dma_dev, cfg->i2c_dma.tx_dma_channel);
		/* NACK subsequent reads */
		i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_SEND_NACK);
		LOG_ERR("Target TX DMA failed with status %d", status);
		return;
	}

	/* We have sent all bytes provided by buf_read_requested().
	 * Host controls the final NACK; leave hardware ready in case it clocks more bytes.
	 */
	LOG_DBG("Target TX DMA completed %u bytes", completed);
}

static int i2c_target_dma_rx_start(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;
	const struct i2c_mchp_dev_config *const cfg = dev->config;

	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = i2c_target_dma_rx_done;
	dma_cfg.complete_callback_en = 1;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->i2c_dma.rx_dma_request;

	dma_blk.block_size = data->tgt_rx_block_size;
	dma_blk.dest_address = (uint32_t)data->tgt_rx_buf;
	dma_blk.source_address = (uint32_t)i2c_get_dma_source_addr(dev);
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	int ret = dma_config(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel, &dma_cfg);
	if (ret != 0) {
		return ret;
	}

	ret = dma_start(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel);
	if (ret == 0) {
		LOG_DBG("Target RX DMA armed for %u bytes", data->tgt_rx_block_size);
	}

	return ret;
}

static int i2c_target_dma_tx_start(const struct device *dev, uint8_t *ptr, uint32_t len)
{
	const struct i2c_mchp_dev_config *const cfg = dev->config;

	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};

	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = i2c_target_dma_tx_done;
	dma_cfg.complete_callback_en = 1;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->i2c_dma.tx_dma_request;

	dma_blk.block_size = len;
	dma_blk.source_address = (uint32_t)ptr;
	dma_blk.dest_address = (uint32_t)i2c_get_dma_dest_addr(dev);
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	int ret = dma_config(cfg->i2c_dma.dma_dev, cfg->i2c_dma.tx_dma_channel, &dma_cfg);
	if (ret != 0) {
		return ret;
	}

	ret = dma_start(cfg->i2c_dma.dma_dev, cfg->i2c_dma.tx_dma_channel);
	if (ret == 0) {
		LOG_DBG("Target TX DMA armed for %u bytes", len);
	}

	return ret;
}
#endif /* CONFIG_I2C_MCHP_DMA_DRIVEN && CONFIG_I2C_TARGET && CONFIG_I2C_TARGET_BUFFER_MODE */

/**
 * @brief Perform a software reset on the I2C peripheral.
 *
 * This function triggers a software reset of the I2C peripheral by setting the
 * appropriate bit in the control register. It then waits for the reset operation
 * to complete by polling the synchronization busy flag.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static inline void i2c_swrst(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	i2c_regs->I2CM.SERCOM_CTRLA |= SERCOM_I2CM_CTRLA_SWRST(1);

	/* Wait for synchronization */
	while ((i2c_regs->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SWRST_Msk) ==
	       SERCOM_I2CM_SYNCBUSY_SWRST_Msk) {

		/* Do nothing */
	};
}

/**
 * @brief Read a byte from the I2C data register in controller or target mode.
 *
 * This API reads a single byte from the I2C data register, automatically selecting
 * the appropriate register based on whether the peripheral is operating in controller
 * (master) or target (slave) mode.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return The byte read from the I2C data register.
 */
static inline uint8_t i2c_byte_read(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;
	uint8_t value;

	if ((i2c_regs->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) ==
	    SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) {
		value = (uint8_t)i2c_regs->I2CM.SERCOM_DATA;
	} else {
		value = (uint8_t)i2c_regs->I2CS.SERCOM_DATA;
	}

	return value;
}

/**
 * @brief Write a byte to the I2C data register in controller or target mode.
 *
 * This API writes a single byte to the I2C data register, automatically selecting
 * the appropriate register based on whether the peripheral is operating in controller
 * (master) or target (slave) mode. In controller mode, the function waits for
 * synchronization to complete after writing the data.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param data The byte to write to the I2C data register.
 */
static void i2c_byte_write(const struct device *dev, uint8_t data)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	if ((i2c_regs->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) ==
	    SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) {
		i2c_regs->I2CM.SERCOM_DATA = data;

		/* Wait for synchronization */
		while ((i2c_regs->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) ==
		       SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) {

			/* Do nothing */
		};
	} else {
		i2c_regs->I2CS.SERCOM_DATA = data;
	}
}

/**
 * @brief Enable or disable the I2C peripheral in controller (master) mode.
 *
 * This API enables or disables the I2C peripheral by setting or clearing the
 * enable bit in the control register. It waits for the synchronization process
 * to complete after the operation.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param enable Set to true to enable the controller, false to disable.
 */
static void i2c_controller_enable(const struct device *dev, bool enable)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	if (enable == true) {
		i2c_regs->I2CM.SERCOM_CTRLA |= SERCOM_I2CM_CTRLA_ENABLE(1);
	} else {
		i2c_regs->I2CM.SERCOM_CTRLA &= ~SERCOM_I2CM_CTRLA_ENABLE(1);
	}

	/* Wait for synchronization */
	while ((i2c_regs->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_ENABLE_Msk) ==
	       SERCOM_I2CM_SYNCBUSY_ENABLE_Msk) {

		/* Do nothing */
	};
}

/**
 * @brief Enable or disable RUNSTDBY for I2C controller (master) mode.
 *
 * This function sets or clears the RUNSTDBY bit in the SERCOM CTRLA register
 * for the I2C controller, allowing the peripheral to run in standby mode.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static void i2c_controller_runstandby_enable(const struct device *dev)
{
	const struct i2c_mchp_dev_config *const i2c_cfg = dev->config;
	sercom_registers_t *i2c_regs = i2c_cfg->regs;
	uint32_t reg32_val = i2c_regs->I2CM.SERCOM_CTRLA;

	reg32_val &= ~SERCOM_I2CM_CTRLA_RUNSTDBY_Msk;
	reg32_val |= SERCOM_I2CM_CTRLA_RUNSTDBY(i2c_cfg->run_in_standby);
	i2c_regs->I2CM.SERCOM_CTRLA = reg32_val;
}

/**
 * @brief Enable automatic acknowledgment for the I2C controller.
 *
 * This API configures the I2C controller to automatically send an ACK (acknowledge)
 * after receiving a byte, by clearing the ACKACT bit in the control register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static inline void i2c_set_controller_auto_ack(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	i2c_regs->I2CM.SERCOM_CTRLB =
		((i2c_regs->I2CM.SERCOM_CTRLB & ~SERCOM_I2CM_CTRLB_ACKACT_Msk) |
		 SERCOM_I2CM_CTRLB_ACKACT(0));
}

/**
 * @brief Configure the I2C peripheral to operate in controller (master) mode.
 *
 * This API sets the necessary control register bits to enable controller (master) mode
 * for the I2C peripheral. It enables smart mode features, sets the controller mode,
 * enables SCL low time-out detection, and configures the inactive bus time-out.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static inline void i2c_set_controller_mode(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	/* Enable i2c device smart mode features */
	i2c_regs->I2CM.SERCOM_CTRLB = ((i2c_regs->I2CM.SERCOM_CTRLB & ~SERCOM_I2CM_CTRLB_SMEN_Msk) |
				       SERCOM_I2CM_CTRLB_SMEN(1));

	i2c_regs->I2CM.SERCOM_CTRLA =
		(i2c_regs->I2CM.SERCOM_CTRLA &
		 ~(SERCOM_I2CM_CTRLA_MODE_Msk | SERCOM_I2CM_CTRLA_INACTOUT_Msk |
		   SERCOM_I2CM_CTRLA_LOWTOUTEN_Msk)) |
		(SERCOM_I2CM_CTRLA_MODE(0x5) | SERCOM_I2CM_CTRLA_LOWTOUTEN(1) |
		 SERCOM_I2CM_CTRLA_INACTOUT(0x3));
}

/**
 * @brief Send a Stop condition to terminate I2C communication in controller mode.
 *
 * This API issues a Stop condition on the I2C bus to terminate the current transfer
 * when operating in controller (master) mode. It sets the appropriate bits in the
 * control register and waits for the synchronization process to complete.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static inline void i2c_controller_transfer_stop(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	i2c_regs->I2CM.SERCOM_CTRLB =
		(i2c_regs->I2CM.SERCOM_CTRLB &
		 ~(SERCOM_I2CM_CTRLB_ACKACT_Msk | SERCOM_I2CM_CTRLB_CMD_Msk)) |
		(SERCOM_I2CM_CTRLB_ACKACT(1) | SERCOM_I2CM_CTRLB_CMD(0x3));

	/* Wait for synchronization */
	while ((i2c_regs->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) ==
	       SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) {

		/* Do nothing */
	};
}

/**
 * @brief Set the I2C controller bus state to idle.
 *
 * This API sets the I2C bus state to idle in controller (master) mode by writing
 * to the status register. It then waits for the synchronization process to complete.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static inline void i2c_set_controller_bus_state_idle(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	i2c_regs->I2CM.SERCOM_STATUS = SERCOM_I2CM_STATUS_BUSSTATE(0x1);

	/* Wait for synchronization */
	while ((i2c_regs->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) ==
	       SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) {

		/* Do nothing */
	};
}

/**
 * @brief Get the current I2C status flags in controller (master) mode.
 *
 * This API reads the I2C status register in controller mode and returns a bitmask
 * of status flags indicating various conditions such as bus error, arbitration lost,
 * bus busy, timeouts, and transaction length error.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return Bitmask of current status flags (see @ref i2c_mchp_controller_status_flag_t).
 */
static uint16_t i2c_controller_status_get(const struct device *dev)
{
	uint16_t status_reg_val;
	uint16_t status_flags = 0;

	status_reg_val = I2C_REGS->I2CM.SERCOM_STATUS;

	if ((status_reg_val & SERCOM_I2CM_STATUS_BUSERR_Msk) == SERCOM_I2CM_STATUS_BUSERR_Msk) {
		status_flags |= SERCOM_I2CM_STATUS_BUSERR_Msk;
	}
	if ((status_reg_val & SERCOM_I2CM_STATUS_ARBLOST_Msk) == SERCOM_I2CM_STATUS_ARBLOST_Msk) {
		status_flags |= SERCOM_I2CM_STATUS_ARBLOST_Msk;
	}
	if ((status_reg_val & SERCOM_I2CM_STATUS_BUSSTATE_Msk) ==
	    SERCOM_I2CM_STATUS_BUSSTATE_BUSY) {
		status_flags |= SERCOM_I2CM_STATUS_BUSSTATE_BUSY;
	}
	if ((status_reg_val & SERCOM_I2CM_STATUS_MEXTTOUT_Msk) == SERCOM_I2CM_STATUS_MEXTTOUT_Msk) {
		status_flags |= SERCOM_I2CM_STATUS_MEXTTOUT_Msk;
	}
	if ((status_reg_val & SERCOM_I2CM_STATUS_SEXTTOUT_Msk) == SERCOM_I2CM_STATUS_SEXTTOUT_Msk) {
		status_flags |= SERCOM_I2CM_STATUS_SEXTTOUT_Msk;
	}
	if ((status_reg_val & SERCOM_I2CM_STATUS_LOWTOUT_Msk) == SERCOM_I2CM_STATUS_LOWTOUT_Msk) {
		status_flags |= SERCOM_I2CM_STATUS_LOWTOUT_Msk;
	}
	if ((status_reg_val & SERCOM_I2CM_STATUS_LENERR_Msk) == SERCOM_I2CM_STATUS_LENERR_Msk) {
		status_flags |= SERCOM_I2CM_STATUS_LENERR_Msk;
	}

	return status_flags;
}

/**
 * @brief Clear specific I2C status flags in controller (master) mode.
 *
 * This API clears the specified status flags in the I2C status register by writing
 * the corresponding bits. The function constructs a bitmask based on the provided
 * status flags and writes it to the status register to clear the indicated conditions.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param status_flags Bitmask of status flags to clear

 */
static void i2c_controller_status_clear(const struct device *dev, uint16_t status_flags)
{
	const struct i2c_mchp_dev_config *const cfg = dev->config;
	sercom_registers_t *i2c_regs = cfg->regs;
	uint16_t reg_val = i2c_regs->I2CM.SERCOM_STATUS;

	if ((status_flags & SERCOM_I2CM_STATUS_BUSERR_Msk) == SERCOM_I2CM_STATUS_BUSERR_Msk) {
		reg_val |= SERCOM_I2CM_STATUS_BUSERR(1);
	}
	if ((status_flags & SERCOM_I2CM_STATUS_ARBLOST_Msk) == SERCOM_I2CM_STATUS_ARBLOST_Msk) {
		reg_val |= SERCOM_I2CM_STATUS_ARBLOST(1);
	}
	if ((status_flags & SERCOM_I2CM_STATUS_BUSSTATE_Msk) == SERCOM_I2CM_STATUS_BUSSTATE_Msk) {
		reg_val |= SERCOM_I2CM_STATUS_BUSSTATE(SERCOM_I2CM_STATUS_BUSSTATE_IDLE_Val);
	}
	if ((status_flags & SERCOM_I2CM_STATUS_MEXTTOUT_Msk) == SERCOM_I2CM_STATUS_MEXTTOUT_Msk) {
		reg_val |= SERCOM_I2CM_STATUS_MEXTTOUT(1);
	}
	if ((status_flags & SERCOM_I2CM_STATUS_SEXTTOUT_Msk) == SERCOM_I2CM_STATUS_SEXTTOUT_Msk) {
		reg_val |= SERCOM_I2CM_STATUS_SEXTTOUT(1);
	}
	if ((status_flags & SERCOM_I2CM_STATUS_LOWTOUT_Msk) == SERCOM_I2CM_STATUS_LOWTOUT_Msk) {
		reg_val |= SERCOM_I2CM_STATUS_LOWTOUT(1);
	}
	if ((status_flags & SERCOM_I2CM_STATUS_LENERR_Msk) == SERCOM_I2CM_STATUS_LENERR_Msk) {
		reg_val |= SERCOM_I2CM_STATUS_LENERR(1);
	}

	i2c_regs->I2CM.SERCOM_STATUS = reg_val;
}

/**
 * @brief Enable I2C interrupts in controller (master) mode.
 *
 * This API enables specific I2C interrupts in controller mode by setting the
 * corresponding bits in the interrupt enable set register. The function constructs
 * a bitmask based on the provided interrupt flags and writes it to the register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param controller_int Bitmask of controller interrupt flags to enable
 */
static void i2c_controller_int_enable(const struct device *dev, uint8_t int_enable_mask)
{
	uint8_t int_enable_flags = 0;

	if ((int_enable_mask & SERCOM_I2CM_INTENSET_MB_Msk) == SERCOM_I2CM_INTENSET_MB_Msk) {
		int_enable_flags |= SERCOM_I2CM_INTENSET_MB(1);
	}
	if ((int_enable_mask & SERCOM_I2CM_INTENSET_SB_Msk) == SERCOM_I2CM_INTENSET_SB_Msk) {
		int_enable_flags |= SERCOM_I2CM_INTENSET_SB(1);
	}
	if ((int_enable_mask & SERCOM_I2CM_INTENSET_ERROR_Msk) == SERCOM_I2CM_INTENSET_ERROR_Msk) {
		int_enable_flags |= SERCOM_I2CM_INTENSET_ERROR(1);
	}

	I2C_REGS->I2CM.SERCOM_INTENSET = int_enable_flags;
}

/**
 * @brief Disable I2C interrupts in controller (master) mode.
 *
 * This API disables specific I2C interrupts in controller mode by setting the
 * corresponding bits in the interrupt enable clear register. The function constructs
 * a bitmask based on the provided interrupt flags and writes it to the register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param controller_int Bitmask of controller interrupt flags to disable
 */
static void i2c_controller_int_disable(const struct device *dev, uint8_t int_disable_mask)
{
	uint8_t int_clear_flags = 0;

	if ((int_disable_mask & SERCOM_I2CM_INTENCLR_MB_Msk) == SERCOM_I2CM_INTENCLR_MB_Msk) {
		int_clear_flags |= SERCOM_I2CM_INTENCLR_MB(1);
	}
	if ((int_disable_mask & SERCOM_I2CM_INTENCLR_SB_Msk) == SERCOM_I2CM_INTENCLR_SB_Msk) {
		int_clear_flags |= SERCOM_I2CM_INTENCLR_SB(1);
	}
	if ((int_disable_mask & SERCOM_I2CM_INTENCLR_ERROR_Msk) == SERCOM_I2CM_INTENCLR_ERROR_Msk) {
		int_clear_flags |= SERCOM_I2CM_INTENCLR_ERROR(1);
	}

	I2C_REGS->I2CM.SERCOM_INTENCLR = int_clear_flags;
}

/**
 * @brief Get the I2C controller interrupt flag status.
 *
 * This API reads the I2C controller interrupt flag register and returns a bitmask
 * of currently set interrupt flags, such as master on bus, target on bus, and error.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return Bitmask of currently set interrupt flags.
 */
static uint8_t i2c_controller_int_flag_get(const struct device *dev)
{
	uint8_t flag_reg_val = (uint8_t)I2C_REGS->I2CM.SERCOM_INTFLAG;
	uint8_t interrupt_flags = 0;

	if ((flag_reg_val & SERCOM_I2CM_INTFLAG_MB_Msk) == SERCOM_I2CM_INTFLAG_MB_Msk) {
		interrupt_flags |= SERCOM_I2CM_INTFLAG_MB(1);
	}
	if ((flag_reg_val & SERCOM_I2CM_INTFLAG_SB_Msk) == SERCOM_I2CM_INTFLAG_SB_Msk) {
		interrupt_flags |= SERCOM_I2CM_INTFLAG_SB(1);
	}
	if ((flag_reg_val & SERCOM_I2CM_INTFLAG_ERROR_Msk) == SERCOM_I2CM_INTFLAG_ERROR_Msk) {
		interrupt_flags |= SERCOM_I2CM_INTFLAG_ERROR(1);
	}

	return interrupt_flags;
}

/**
 * @brief Clear specific I2C controller interrupt flags.
 *
 * This API clears the specified interrupt flags in the I2C controller by writing
 * the corresponding bits to the interrupt flag register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param controller_intflag Bitmask of interrupt flags to clear
 */
static void i2c_controller_int_flag_clear(const struct device *dev, uint8_t intflag_mask)
{
	uint8_t flag_clear = 0;

	if ((intflag_mask & SERCOM_I2CM_INTFLAG_MB_Msk) == SERCOM_I2CM_INTFLAG_MB_Msk) {
		flag_clear |= SERCOM_I2CM_INTFLAG_MB(1);
	}
	if ((intflag_mask & SERCOM_I2CM_INTFLAG_SB_Msk) == SERCOM_I2CM_INTFLAG_SB_Msk) {
		flag_clear |= SERCOM_I2CM_INTFLAG_SB(1);
	}
	if ((intflag_mask & SERCOM_I2CM_INTFLAG_ERROR_Msk) == SERCOM_I2CM_INTFLAG_ERROR_Msk) {
		flag_clear |= SERCOM_I2CM_INTFLAG_ERROR(1);
	}

	I2C_REGS->I2CM.SERCOM_INTFLAG = flag_clear;
}

/**
 * @brief Write the target address to the I2C controller address register.
 *
 * This API writes the specified address to the I2C controller's address register.
 * If the address indicates a read operation (based on the direction mask), the function
 * also enables automatic acknowledgment. After writing the address, it waits for
 * the synchronization process to complete.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param addr The 8-bit I2C address (including the R/W bit) to write to the address register.
 */
static void i2c_controller_addr_write(const struct device *dev, uint32_t addr)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	if ((addr & (uint32_t)I2C_MCHP_MESSAGE_DIR_READ_MASK) == I2C_MCHP_MESSAGE_DIR_READ_MASK) {
		i2c_set_controller_auto_ack(dev);
	}

	i2c_regs->I2CM.SERCOM_ADDR = (i2c_regs->I2CM.SERCOM_ADDR & ~SERCOM_I2CM_ADDR_ADDR_Msk) |
				     SERCOM_I2CM_ADDR_ADDR(addr);

	/* Wait for synchronization */
	while ((i2c_regs->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) ==
	       SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) {

		/* Do nothing */
	};
}

/**
 * @brief Calculate the I2C baud rate register value.
 *
 * This API calculates the appropriate baud rate register value for the I2C peripheral
 * based on the desired bitrate and the system clock frequency. It supports standard,
 * fast, fast-plus, and high-speed I2C modes, and ensures the calculated value is within
 * valid hardware limits. The result is written to the provided output pointer.
 *
 * @param bitrate Desired I2C bus speed in Hz (e.g., 100000 for 100 kHz).
 * @param sys_clock_rate System clock frequency in Hz.
 * @param baud_val Pointer to store the calculated baud rate register value.
 * @return true if the calculation was successful, false if the system clock is too low.
 *
 */
static bool i2c_baudrate_calc(uint32_t bitrate, uint32_t sys_clock_rate, uint32_t *baud_val)
{
	uint32_t baud_value = 0U;
	float fsrc_clk_freq = (float)sys_clock_rate;
	float fi2c_clk_speed = (float)bitrate;
	float fbaud_value = 0.0f;
	bool is_calc_success = true;

	/* Reference clock frequency must be at least two times the baud rate */
	if (sys_clock_rate < (2U * bitrate)) {
		is_calc_success = false;
	} else {

		if (bitrate > I2C_SPEED_FAST_PLUS) {

			/* HS mode baud calculation */
			fbaud_value = (fsrc_clk_freq / fi2c_clk_speed) - 2.0f;
			baud_value = (uint32_t)fbaud_value;
		} else {

			/* Standard, FM and FM+ baud calculation */
			fbaud_value =
				(fsrc_clk_freq / fi2c_clk_speed) -
				((fsrc_clk_freq * I2C_MCHP_START_CONDITION_SETUP_TIME) + 10.0f);
			baud_value = (uint32_t)fbaud_value;
		}

		if (bitrate <= I2C_SPEED_FAST) {

			/* For I2C clock speed up to 400 kHz, the value of BAUD<7:0>
			 * determines both SCL_L and SCL_H with SCL_L = SCL_H
			 */
			if (baud_value > (0xFFU * 2U)) {

				/* Set baud rate to the maximum possible value */
				baud_value = 0xFFU;
			} else if (baud_value <= 1U) {

				/* Baud value cannot be 0. Set baud rate to minimum possible
				 * value
				 */
				baud_value = 1U;
			} else {
				baud_value /= 2U;
			}
		} else {

			/* To maintain the ratio of SCL_L:SCL_H to 2:1, the max value of
			 * BAUD_LOW<15:8>:BAUD<7:0> can be 0xFF:0x7F. Hence BAUD_LOW + BAUD
			 * can not exceed 255+127 = 382
			 */
			if (baud_value >= I2C_BAUD_LOW_HIGH_MAX) {

				/* Set baud rate to the minimum possible value while
				 * maintaining SCL_L:SCL_H to 2:1
				 */
				baud_value = (0xFFUL << 8U) | (0x7FU);
			} else if (baud_value <= 3U) {

				/* Baud value cannot be 0. Set baud rate to maximum possible
				 * value while maintaining SCL_L:SCL_H to 2:1
				 */
				baud_value = (2UL << 8U) | 1U;
			} else {

				/* For Fm+ mode, I2C SCL_L:SCL_H to 2:1 */
				baud_value = ((((baud_value * 2U) / 3U) << 8U) | (baud_value / 3U));
			}
		}
		*baud_val = baud_value;
	}

	return is_calc_success;
}

/**
 * @brief Set the I2C baud rate and speed mode for the controller.
 *
 * This API calculates and sets the appropriate baud rate register value and speed mode
 * for the I2C controller based on the desired bitrate and system clock frequency.
 * It supports standard, fast, fast-plus, and high-speed I2C modes, and configures
 * the SDA hold time as required by the selected mode.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param bitrate Desired I2C bus speed in Hz (e.g., 100000 for 100 kHz, 400000 for 400 kHz, 3400000
 * for 3.4 MHz).
 * @param sys_clock_rate System clock frequency in Hz.
 * @return true if the baud rate was successfully set, false otherwise.
 *
 */
static bool i2c_set_baudrate(const struct device *dev, uint32_t bitrate, uint32_t sys_clock_rate)
{
	uint32_t i2c_speed_mode = 0;
	uint32_t baud_value;
	uint32_t hsbaud_value;
	uint32_t sda_hold_time = 0;
	bool is_success = false;

	if (bitrate == I2C_SPEED_HIGH) {

		/* HS mode requires baud values for both FS and HS frequency. First
		 * calculate baud for FS
		 */
		if (i2c_baudrate_calc(I2C_SPEED_FAST, sys_clock_rate, &baud_value) == true) {
			if (i2c_baudrate_calc(bitrate, sys_clock_rate, &hsbaud_value) == true) {
				is_success = true;
				baud_value |= (hsbaud_value << 16U);
				i2c_speed_mode = 2U;
				sda_hold_time = 2U;
			}
		}
	} else {
		if (i2c_baudrate_calc(bitrate, sys_clock_rate, &baud_value) == true) {
			if (bitrate == I2C_SPEED_FAST_PLUS) {
				i2c_speed_mode = 1U;
				sda_hold_time = 1U;
			}
			is_success = true;
		}
	}

	if (is_success == true) {
		sercom_registers_t *i2c_regs =
			((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

		/* Baud rate - controller Baud Rate*/
		i2c_regs->I2CM.SERCOM_BAUD = baud_value;

		i2c_regs->I2CM.SERCOM_CTRLA =
			((i2c_regs->I2CM.SERCOM_CTRLA &
			  (~SERCOM_I2CM_CTRLA_SPEED_Msk | ~SERCOM_I2CM_CTRLA_SDAHOLD_Msk)) |
			 (SERCOM_I2CM_CTRLA_SPEED(i2c_speed_mode) |
			  SERCOM_I2CM_CTRLA_SDAHOLD(sda_hold_time)));
	}

	return is_success;
}

/**
 * @brief Terminates the current I2C operation in case of an error.
 *
 * This function checks for any errors in the I2C operation, stops ongoing DMA transfers
 * (if enabled), clears necessary flags, disables interrupts, stops the I2C transfer, and
 * releases the semaphore to signal the completion of the operation.
 *
 * @param dev Pointer to the device structure for the I2C driver.
 * @return true if the termination is successful, false if an error is detected.
 */
static bool i2c_is_terminate_on_error(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;
	bool retval = true;

	/* Retrieve and store the current I2C status in the device data structure. */
	data->current_msg.status = i2c_controller_status_get(dev);

	/* Check for any I2C errors. If an error is found, terminate early. */
	if (data->current_msg.status == 0) {
		retval = false;
	} else {

		/*
		 * Clear all the status flags that require an explicit clear operation.
		 * Some flags are cleared automatically by writing to specific registers (e.g., ADDR
		 * writes).
		 */
		i2c_controller_status_clear(dev, data->current_msg.status);

		/* Disable all I2C interrupts to prevent further processing. */
		i2c_controller_int_disable(dev, SERCOM_I2CM_INTENSET_Msk);

		/* Stop the I2C transfer explicitly. */
		i2c_controller_transfer_stop(dev);

#ifdef CONFIG_I2C_CALLBACK
		/* Callback to the application for async */
		data->i2c_async_callback(dev, (int)data->current_msg.status, data->user_data);
#else
		/* Release the semaphore to signal the completion of the operation. */
		k_sem_give(&data->i2c_sync_sem);
#endif /*CONFIG_I2C_CALLBACK*/
	}

	/* Return true to indicate successful termination of the operation. */
	return retval;
}

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
/**
 * @brief Configure DMA for I2C transfer depending on direction.
 *
 * This function configures the DMA for either read or write based on
 * the `is_read` flag.
 *
 * @param dev Pointer to the I2C device structure.
 * @param is_read True if the transfer is a read, false for write.
 * @return I2C_MCHP_SUCCESS on success, negative error code on failure.
 */
static int i2c_configure_dma(const struct device *dev, bool is_read)
{
	int retval = I2C_MCHP_SUCCESS;

	if (is_read == true) {
		retval = i2c_dma_read_config(dev);
	} else {
		retval = i2c_dma_write_config(dev);
	}

	return retval;
}

/**
 * @brief Start DMA for I2C transfer depending on direction.
 *
 * This function starts the DMA transfer for either read or write
 * depending on the `is_read` flag.
 *
 * @param dev Pointer to the I2C device structure.
 * @param is_read True if the transfer is a read, false for write.
 * @return I2C_MCHP_SUCCESS on success, negative error code on failure.
 */
static int i2c_start_dma(const struct device *dev, bool is_read)
{
	const struct i2c_mchp_dev_config *cfg = dev->config;
	int retval = I2C_MCHP_SUCCESS;
	uint32_t channel;

	if (is_read == true) {
		channel = cfg->i2c_dma.rx_dma_channel;
	} else {
		channel = cfg->i2c_dma.tx_dma_channel;
	}

	retval = dma_start(cfg->i2c_dma.dma_dev, channel);

	return retval;
}
#endif /*CONFIG_I2C_MCHP_DMA_DRIVEN*/

/**
 * @brief Restart the I2C transaction with the target device.
 *
 * This function handles preparing and restarting an I2C transaction. It configures
 * the address register, handles read or write-specific setups (including optional DMA),
 * and enables the I2C interrupts or DMA operations as needed.
 *
 * @param dev Pointer to the device structure for the I2C driver.
 */
static void i2c_restart(const struct device *dev)
{
	/* Retrieve device data and configuration. */
	struct i2c_mchp_dev_data *data = dev->data;

	/* Prepare the address register (left-shift address by 1 for R/W bit). */
	uint32_t addr_reg = data->target_addr << 1U;

	bool is_read =
		((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ);

	int retval = I2C_MCHP_SUCCESS;

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
	/* Configure DMA if enabled */
	retval = i2c_configure_dma(dev, is_read);
#endif /*CONFIG_I2C_MCHP_DMA_DRIVEN*/

	/* Only proceed if DMA configuration was successful or DMA is not used */
	if (retval == I2C_MCHP_SUCCESS) {

		/* Set read bit if needed */
		if (is_read == true) {
			addr_reg |= 1U;
		}

		/*
		 * Writing the target address to the I2C hardware starts the transaction.
		 * This will issue a START or a repeated START as required.
		 */
		i2c_controller_addr_write(dev, addr_reg);

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN

		/* Start DMA transfer if enabled */
		retval = i2c_start_dma(dev, is_read);

		/* Invoke callback if DMA failed */
		if (retval != I2C_MCHP_SUCCESS) {
#ifdef CONFIG_I2C_CALLBACK
			data->i2c_async_callback(dev, retval, data->user_data);
#endif
		}

#else
		/* Enable I2C interrupts for non-DMA operation */
		i2c_controller_int_enable(dev, SERCOM_I2CM_INTENSET_Msk);
#endif /*CONFIG_I2C_MCHP_DMA_DRIVEN*/
	}
}

#ifdef CONFIG_I2C_TARGET
static struct i2c_target_config *i2c_mchp_find_target_cfg(struct i2c_mchp_dev_data *data,
							  uint16_t addr);
static uint16_t i2c_mchp_get_matched_addr(const struct device *dev);

/**
 * @brief Get the I2C target interrupt flag status.
 *
 * This API reads the I2C target interrupt flag register and returns a bitmask
 * of currently set interrupt flags, such as STOP condition, address match, data ready, and
 * error.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return Bitmask of currently set interrupt flags.
 */
static uint8_t i2c_target_int_flag_get(const struct device *dev)
{
	uint8_t flag_reg_val;
	uint16_t interrupt_flags = 0;

	flag_reg_val = (uint8_t)I2C_REGS->I2CS.SERCOM_INTFLAG;

	if ((flag_reg_val & SERCOM_I2CS_INTFLAG_PREC_Msk) == SERCOM_I2CS_INTFLAG_PREC_Msk) {
		interrupt_flags |= SERCOM_I2CS_INTFLAG_PREC_Msk;
	}
	if ((flag_reg_val & SERCOM_I2CS_INTFLAG_AMATCH_Msk) == SERCOM_I2CS_INTFLAG_AMATCH_Msk) {
		interrupt_flags |= SERCOM_I2CS_INTFLAG_AMATCH_Msk;
	}
	if ((flag_reg_val & SERCOM_I2CS_INTFLAG_DRDY_Msk) == SERCOM_I2CS_INTFLAG_DRDY_Msk) {
		interrupt_flags |= SERCOM_I2CS_INTFLAG_DRDY_Msk;
	}
	if ((flag_reg_val & SERCOM_I2CS_INTFLAG_ERROR_Msk) == SERCOM_I2CS_INTFLAG_ERROR_Msk) {
		interrupt_flags |= SERCOM_I2CS_INTFLAG_ERROR_Msk;
	}

	return interrupt_flags;
}

/**
 * @brief Get the current I2C status flags in target (slave) mode.
 *
 * This API reads the I2C status register in target mode and returns a bitmask
 * of status flags indicating various conditions such as bus error, collision,
 * data direction, and timeouts.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return Bitmask of current status flags (see @ref i2c_mchp_target_status_flag_t).
 */
static uint16_t i2c_target_status_get(const struct device *dev)
{
	uint16_t status_reg_val;
	uint16_t status_flags = 0;

	status_reg_val = I2C_REGS->I2CS.SERCOM_STATUS;

	if ((status_reg_val & SERCOM_I2CS_STATUS_BUSERR_Msk) == SERCOM_I2CS_STATUS_BUSERR_Msk) {
		status_flags |= SERCOM_I2CS_STATUS_BUSERR_Msk;
	}
	if ((status_reg_val & SERCOM_I2CS_STATUS_COLL_Msk) == SERCOM_I2CS_STATUS_COLL_Msk) {
		status_flags |= SERCOM_I2CS_STATUS_COLL_Msk;
	}
	if ((status_reg_val & SERCOM_I2CS_STATUS_DIR_Msk) == SERCOM_I2CS_STATUS_DIR_Msk) {
		status_flags |= SERCOM_I2CS_STATUS_DIR_Msk;
	}
	if ((status_reg_val & SERCOM_I2CS_STATUS_LOWTOUT_Msk) == SERCOM_I2CS_STATUS_LOWTOUT_Msk) {
		status_flags |= SERCOM_I2CS_STATUS_LOWTOUT_Msk;
	}
	if ((status_reg_val & SERCOM_I2CS_STATUS_SEXTTOUT_Msk) == SERCOM_I2CS_STATUS_SEXTTOUT_Msk) {
		status_flags |= SERCOM_I2CS_STATUS_SEXTTOUT_Msk;
	}

	return status_flags;
}

/**
 * @brief Clear specific I2C status flags in target (slave) mode.
 *
 * This API clears the specified status flags in the I2C status register by writing
 * the corresponding bits. The function constructs a bitmask based on the provided
 * status flags and writes it to the status register to clear the indicated conditions.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param status_flags Bitmask of status flags to clear.
 */
static void i2c_target_status_clear(const struct device *dev, uint16_t status_flags)
{
	uint16_t status_clear = 0;

	if ((status_flags & SERCOM_I2CS_STATUS_BUSERR_Msk) == SERCOM_I2CS_STATUS_BUSERR_Msk) {
		status_clear |= SERCOM_I2CS_STATUS_BUSERR(1);
	}
	if ((status_flags & SERCOM_I2CS_STATUS_COLL_Msk) == SERCOM_I2CS_STATUS_COLL_Msk) {
		status_clear |= SERCOM_I2CS_STATUS_COLL(1);
	}
	if ((status_flags & SERCOM_I2CS_STATUS_LOWTOUT_Msk) == SERCOM_I2CS_STATUS_LOWTOUT_Msk) {
		status_clear |= SERCOM_I2CS_STATUS_LOWTOUT(1);
	}
	if ((status_flags & SERCOM_I2CS_STATUS_SEXTTOUT_Msk) == SERCOM_I2CS_STATUS_SEXTTOUT_Msk) {
		status_clear |= SERCOM_I2CS_STATUS_SEXTTOUT(1);
	}

	I2C_REGS->I2CS.SERCOM_STATUS = status_clear;
}

/**
 * @brief Clear specific I2C target interrupt flags.
 *
 * This API clears the specified interrupt flags in the I2C target by writing
 * the corresponding bits to the interrupt flag register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param target_intflag Bitmask of interrupt flags to clear (see @ref
 * i2c_mchp_target_intflag_t).
 */
static void i2c_target_int_flag_clear(const struct device *dev, uint8_t target_intflag)
{
	uint8_t flag_clear = 0;

	if ((target_intflag & SERCOM_I2CS_INTFLAG_PREC_Msk) == SERCOM_I2CS_INTFLAG_PREC_Msk) {
		flag_clear |= SERCOM_I2CS_INTFLAG_PREC(1);
	}
	if ((target_intflag & SERCOM_I2CS_INTFLAG_AMATCH_Msk) == SERCOM_I2CS_INTFLAG_AMATCH_Msk) {
		flag_clear |= SERCOM_I2CS_INTFLAG_AMATCH(1);
	}
	if ((target_intflag & SERCOM_I2CS_INTFLAG_DRDY_Msk) == SERCOM_I2CS_INTFLAG_DRDY_Msk) {
		flag_clear |= SERCOM_I2CS_INTFLAG_DRDY(1);
	}
	if ((target_intflag & SERCOM_I2CS_INTFLAG_ERROR_Msk) == SERCOM_I2CS_INTFLAG_ERROR_Msk) {
		flag_clear |= SERCOM_I2CS_INTFLAG_ERROR(1);
	}

	I2C_REGS->I2CS.SERCOM_INTFLAG = flag_clear;
}

/**
 * @brief Get the ACK/NACK status of the last byte transferred in I2C target mode.
 *
 * This function checks the SERCOM I2C target STATUS register to determine whether
 * the last byte transferred was acknowledged (ACK) or not acknowledged (NACK) by the I2C
 * master.
 *
 * @return uint8_t Returns 0 if the last byte was ACKed by the master,
 *                 or 1 if it was NACKed.
 */
static inline uint8_t i2c_target_get_lastbyte_ack_status(const struct device *dev)
{
	return ((I2C_REGS->I2CS.SERCOM_STATUS & SERCOM_I2CS_STATUS_RXNACK_Msk) != 0U)
		       ? I2C_MCHP_TARGET_ACK_STATUS_RECEIVED_NACK
		       : I2C_MCHP_TARGET_ACK_STATUS_RECEIVED_ACK;
}

/**
 * @brief Set the I2C target (slave) command for the SERCOM peripheral.
 *
 * This function issues a specific command to the I2C target (slave) hardware
 * by updating the SERCOM CTRLB register.
 *
 * @param dev Pointer to the I2C device structure.
 * @param cmd The command to issue to the I2C target.
 */
void i2c_target_set_command(const struct device *dev, enum i2c_mchp_target_cmd cmd)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	/* Clear CMD bits first */
	i2c_regs->I2CS.SERCOM_CTRLB &= ~SERCOM_I2CS_CTRLB_CMD_Msk;

	switch (cmd) {
	case I2C_MCHP_TARGET_COMMAND_SEND_ACK:
		i2c_regs->I2CS.SERCOM_CTRLB =
			(i2c_regs->I2CS.SERCOM_CTRLB & ~SERCOM_I2CS_CTRLB_ACKACT_Msk) |
			SERCOM_I2CS_CTRLB_CMD(0x03UL);
		break;
	case I2C_MCHP_TARGET_COMMAND_SEND_NACK:
		i2c_regs->I2CS.SERCOM_CTRLB =
			(i2c_regs->I2CS.SERCOM_CTRLB | SERCOM_I2CS_CTRLB_ACKACT_Msk) |
			SERCOM_I2CS_CTRLB_CMD(0x03UL);
		break;
	case I2C_MCHP_TARGET_COMMAND_RECEIVE_ACK_NAK:
		i2c_regs->I2CS.SERCOM_CTRLB =
			(i2c_regs->I2CS.SERCOM_CTRLB & ~SERCOM_I2CS_CTRLB_ACKACT_Msk) |
			SERCOM_I2CS_CTRLB_CMD(0x03UL);
		break;
	case I2C_MCHP_TARGET_COMMAND_WAIT_FOR_START:
		i2c_regs->I2CS.SERCOM_CTRLB =
			(i2c_regs->I2CS.SERCOM_CTRLB | SERCOM_I2CS_CTRLB_ACKACT_Msk) |
			SERCOM_I2CS_CTRLB_CMD(0x02UL);
		break;
	default:
		break;
	}
}

/**
 * @brief Handle I2C target address match event.
 *
 * Sends ACK and triggers callbacks based on transfer direction.
 *
 * @param dev Device pointer.
 * @param data Device data pointer.
 * @param target_status Current target status.
 */
static void i2c_target_address_match(const struct device *dev, struct i2c_mchp_dev_data *data,
				     uint16_t target_status)
{
	uint16_t matched_addr = i2c_mchp_get_matched_addr(dev);
	struct i2c_target_config *cfg = i2c_mchp_find_target_cfg(data, matched_addr);
	const struct i2c_target_callbacks *target_cb = NULL;

	if (cfg == NULL) {
		LOG_ERR("No target configuration available to handle address match");
		i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_SEND_NACK);
		return;
	}

	data->active_target_cfg = cfg;
	target_cb = cfg->callbacks;

	if (target_cb == NULL) {
		LOG_ERR("Target callbacks not set");
		i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_SEND_NACK);
		return;
	}

	ARG_UNUSED(matched_addr);

	if ((target_status & SERCOM_I2CS_STATUS_DIR_Msk) == SERCOM_I2CS_STATUS_DIR_Msk) {
		/* Host read (target transmits) */
		LOG_DBG("AMATCH: Host READ request (target TX), status=0x%04x", target_status);
#if defined(CONFIG_I2C_MCHP_DMA_DRIVEN) && defined(CONFIG_I2C_TARGET_BUFFER_MODE)
		/* Reset any previous TX DMA staging info */
		data->tgt_tx_buf_active = false;
		data->tgt_tx_dma_active = false;
		data->tgt_tx_buf = NULL;
		data->tgt_tx_len = 0U;

		if (target_cb->buf_read_requested) {
			uint8_t *ptr = NULL;
			uint32_t len = 0;

			LOG_DBG("Calling buf_read_requested callback");
			if ((target_cb->buf_read_requested(cfg, &ptr, &len) == 0) &&
			    (ptr != NULL) && (len > 0U)) {
				LOG_DBG("buf_read_requested returned %u bytes, first=0x%02x", len, ptr[0]);

				data->tgt_tx_buf = ptr;
				data->tgt_tx_len = len;
				data->tgt_tx_buf_active = true;
				data->firstReadAfterAddrMatch = false;

				int dma_ret = i2c_target_dma_tx_start(dev, data->tgt_tx_buf,
								      data->tgt_tx_len);
				if (dma_ret == 0) {
					data->tgt_tx_dma_active = true;
					i2c_target_mask_drdy_for_dma(dev);
					i2c_target_set_command(dev,
							       I2C_MCHP_TARGET_COMMAND_RECEIVE_ACK_NAK);
					LOG_DBG("Target TX DMA armed for %u bytes", data->tgt_tx_len);
					return;
				}

				LOG_ERR("Target TX DMA start failed: %d", dma_ret);
				data->tgt_tx_buf_active = false;
				data->tgt_tx_buf = NULL;
				data->tgt_tx_len = 0U;
			} else {
				LOG_WRN("buf_read_requested failed or returned invalid data");
			}
		}
#endif /* CONFIG_I2C_MCHP_DMA_DRIVEN && CONFIG_I2C_TARGET_BUFFER_MODE */

		/* Fallback: byte-based */
		LOG_DBG("Using byte-based mode");
		data->firstReadAfterAddrMatch = true;
		if (target_cb->read_requested) {
			target_cb->read_requested(cfg, &data->rx_tx_data);
		}
		i2c_byte_write(dev, data->rx_tx_data);
		i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_RECEIVE_ACK_NAK);
		/* read_processed will be called in DRDY handler for subsequent bytes */
	} else {
		i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_SEND_ACK);

		/* Host write (target receives) */
		if (target_cb->write_requested) {
			target_cb->write_requested(cfg);
		}
#if defined(CONFIG_I2C_MCHP_DMA_DRIVEN) && defined(CONFIG_I2C_TARGET_BUFFER_MODE)
		/* Arm RX DMA into internal buffer */
		data->tgt_rx_block_size = sizeof(data->tgt_rx_buf);
		data->dma_rx_target_cfg = cfg;
		if (i2c_target_dma_rx_start(dev) == 0) {
			data->tgt_rx_dma_active = true;
			i2c_target_mask_drdy_for_dma(dev);
		}
#else
		data->dma_rx_target_cfg = NULL;
#endif /* CONFIG_I2C_MCHP_DMA_DRIVEN && CONFIG_I2C_TARGET_BUFFER_MODE */
	}
}

/**
 * @brief Handle I2C target data ready event.
 *
 * Processes read or write operations depending on the transfer direction.
 *
 * @param dev Device pointer.
 * @param data Device data pointer.
 * @param target_status Current target status.
 */
static void i2c_target_data_ready(const struct device *dev, struct i2c_mchp_dev_data *data,
				  struct i2c_target_config *cfg, uint16_t target_status)
{
	const struct i2c_target_callbacks *target_cb =
		(cfg != NULL) ? cfg->callbacks : NULL;
	int retval = I2C_MCHP_SUCCESS;
	uint8_t last_ack_state = i2c_target_get_lastbyte_ack_status(dev);
	bool last_byte_acked = (last_ack_state == I2C_MCHP_TARGET_ACK_STATUS_RECEIVED_ACK);

	if ((cfg == NULL) || (target_cb == NULL)) {
		LOG_ERR("DRDY with no active target configuration");
		i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_SEND_NACK);
		return;
	}

	if (((target_status & SERCOM_I2CS_STATUS_DIR_Msk) == SERCOM_I2CS_STATUS_DIR_Msk)) {
#if defined(CONFIG_I2C_MCHP_DMA_DRIVEN) && defined(CONFIG_I2C_TARGET_BUFFER_MODE)
		/* TX via DMA: DMA feeds peripheral, nothing to do in DRDY */
		if (data->tgt_tx_dma_active) {
			LOG_DBG("DRDY during DMA TX - ignoring (DMA handles it)");
			return;
		}

		if (data->tgt_tx_buf_active) {
			/* Already provided all bytes via buffer-mode */
			i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_WAIT_FOR_START);
			LOG_DBG("Buffer-mode TX complete; waiting for next START");
			return;
		}
#endif
		if ((data->firstReadAfterAddrMatch == true) || last_byte_acked) {

			/* Host is reading */
			i2c_byte_write(dev, data->rx_tx_data);

			/* first byte read after address match done*/
			data->firstReadAfterAddrMatch = false;

			i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_RECEIVE_ACK_NAK);

			/* Load the next byte for host read*/
			if (target_cb->read_processed) {
				target_cb->read_processed(cfg, &data->rx_tx_data);
			}

		} else {
			i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_WAIT_FOR_START);
		}
	} else {
		/* Host is writing */
#if defined(CONFIG_I2C_MCHP_DMA_DRIVEN) && defined(CONFIG_I2C_TARGET_BUFFER_MODE)
		if (data->tgt_rx_dma_active) {
			/* Keep ACKing while RX DMA consumes data */
			i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_SEND_ACK);
			return;
		}
#endif
		i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_SEND_ACK);
		data->rx_tx_data = i2c_byte_read(dev);
		if (target_cb->write_received) {
			retval = target_cb->write_received(cfg, data->rx_tx_data);
		}
		if (retval != I2C_MCHP_SUCCESS) {
			i2c_target_set_command(dev, I2C_MCHP_TARGET_COMMAND_SEND_NACK);
		}
	}
}

/**
 * @brief Handle target mode interrupts for the I2C peripheral.
 *
 * This function processes I2C target-specific interrupt events such as
 * address match, data ready, and stop condition. It calls the appropriate
 * callbacks based on the interrupt type.
 *
 * @param dev Pointer to the I2C device structure.
 */
static void i2c_target_handler(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;

	/* Retrieve the interrupt status for the I2C peripheral */
	uint8_t int_status = i2c_target_int_flag_get(dev);

	/*Get the current status of target device */
	uint16_t target_status = i2c_target_status_get(dev);

	LOG_DBG("ISR: int_flags=0x%02x, status=0x%04x, tx_dma=%d, rx_dma=%d",
		int_status, target_status, data->tgt_tx_dma_active, data->tgt_rx_dma_active);

	/* Handle error conditions */
	if ((int_status & SERCOM_I2CS_INTFLAG_ERROR_Msk) == SERCOM_I2CS_INTFLAG_ERROR_Msk) {
		i2c_target_int_flag_clear(dev, SERCOM_I2CS_INTFLAG_ERROR_Msk);
		LOG_ERR("Interrupt Error generated (int=0x%02x, status=0x%04x)", int_status, target_status);
		if (target_status & SERCOM_I2CS_STATUS_BUSERR_Msk) {
			LOG_ERR("  - Bus Error");
		}
		if (target_status & SERCOM_I2CS_STATUS_COLL_Msk) {
			LOG_ERR("  - Collision detected");
		}
		if (target_status & SERCOM_I2CS_STATUS_LOWTOUT_Msk) {
			LOG_ERR("  - SCL Low Timeout");
		}
		if (target_status & SERCOM_I2CS_STATUS_SEXTTOUT_Msk) {
			LOG_ERR("  - Slave SCL Low Extend Timeout");
		}
#if defined(CONFIG_I2C_MCHP_DMA_DRIVEN) && defined(CONFIG_I2C_TARGET_BUFFER_MODE)
		/* Clean up any active DMA transfers on error */
		const struct i2c_mchp_dev_config *const cfg = dev->config;
		if (data->tgt_tx_dma_active) {
			(void)dma_stop(cfg->i2c_dma.dma_dev, cfg->i2c_dma.tx_dma_channel);
			data->tgt_tx_dma_active = false;
			LOG_DBG("Target TX DMA stopped due to error");
		}
		data->tgt_tx_buf_active = false;
		if (data->tgt_rx_dma_active) {
			(void)dma_stop(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel);
			data->tgt_rx_dma_active = false;
			LOG_DBG("Target RX DMA stopped due to error");
		}
		/* Reset TX state */
		data->tgt_tx_buf = NULL;
		data->tgt_tx_len = 0;
		data->tgt_tx_buf_active = false;
		i2c_target_unmask_drdy_after_dma(dev);
#endif
		if (data->active_target_cfg && data->active_target_cfg->callbacks &&
		    data->active_target_cfg->callbacks->stop) {
			data->active_target_cfg->callbacks->stop(data->active_target_cfg);
		}
		data->active_target_cfg = NULL;
		data->dma_rx_target_cfg = NULL;
		data->firstReadAfterAddrMatch = false;
	} else {

		/* Handle address match */
		if ((int_status & SERCOM_I2CS_INTFLAG_AMATCH_Msk) ==
		    SERCOM_I2CS_INTFLAG_AMATCH_Msk) {
			LOG_DBG("AMATCH interrupt detected");
			/* Process address match BEFORE clearing flag - hardware needs DATA ready */
			LOG_DBG("Calling address_match handler (AMATCH still set)");
			i2c_target_address_match(dev, data, target_status);
			LOG_DBG("address_match handler returned, now clearing AMATCH flag");
			/* Clear AMATCH flag after DATA is written */
			i2c_target_int_flag_clear(dev, SERCOM_I2CS_INTFLAG_AMATCH_Msk);
			LOG_DBG("AMATCH flag cleared");
		}

		/* Handle data ready (Read/Write Operations) */
		if ((int_status & SERCOM_I2CS_INTFLAG_DRDY_Msk) == SERCOM_I2CS_INTFLAG_DRDY_Msk) {
			LOG_DBG("DRDY interrupt detected");
			i2c_target_data_ready(dev, data, data->active_target_cfg, target_status);
		}
	}

	/* Handle stop condition interrupt */
	if ((int_status & SERCOM_I2CS_INTFLAG_PREC_Msk) == SERCOM_I2CS_INTFLAG_PREC_Msk) {
		LOG_DBG("STOP condition detected");
		i2c_target_int_flag_clear(dev, SERCOM_I2CS_INTFLAG_PREC_Msk);

#if defined(CONFIG_I2C_MCHP_DMA_DRIVEN) && defined(CONFIG_I2C_TARGET_BUFFER_MODE)
		/* Finalize any active target-mode DMA transfers */
		const struct i2c_mchp_dev_config *const cfg = dev->config;
		if (data->tgt_rx_dma_active) {
			struct dma_status st = {0};
			/* Stop DMA first to freeze write-back descriptor */
			(void)dma_stop(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel);
			/* Now get_status reads the write-back descriptor with final BTCNT */
			(void)dma_get_status(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel, &st);
			data->tgt_rx_dma_active = false;

			/* Deliver any residual bytes in the current block */
			struct i2c_target_config *rx_cfg = data->dma_rx_target_cfg ?
				data->dma_rx_target_cfg : data->active_target_cfg;
			const struct i2c_target_callbacks *rx_cb =
				(rx_cfg != NULL) ? rx_cfg->callbacks : NULL;

			if (rx_cb && rx_cb->buf_write_received) {
				uint32_t pending = st.pending_length;
				uint32_t have = 0U;
				if (pending < data->tgt_rx_block_size) {
					have = data->tgt_rx_block_size - pending;
				}
				if (have > 0U) {
					rx_cb->buf_write_received(rx_cfg, data->tgt_rx_buf, have);
					LOG_DBG("Target RX DMA tail delivered %u bytes (pending=%u total_copied=%llu)",
						have, pending, (unsigned long long)st.total_copied);
				}
			}
		}
		if (data->tgt_tx_dma_active) {
			(void)dma_stop(cfg->i2c_dma.dma_dev, cfg->i2c_dma.tx_dma_channel);
			data->tgt_tx_dma_active = false;
			LOG_DBG("Target TX DMA stopped at STOP condition");
		}
		/* Reset TX state */
		data->tgt_tx_buf = NULL;
		data->tgt_tx_len = 0;
		data->tgt_tx_buf_active = false;
		i2c_target_unmask_drdy_after_dma(dev);
#endif /* CONFIG_I2C_MCHP_DMA_DRIVEN && CONFIG_I2C_TARGET_BUFFER_MODE */

		/* Notify that a stop condition was received */
		if (data->active_target_cfg && data->active_target_cfg->callbacks &&
		    data->active_target_cfg->callbacks->stop) {
			data->active_target_cfg->callbacks->stop(data->active_target_cfg);
		}
		data->active_target_cfg = NULL;
		data->dma_rx_target_cfg = NULL;
		data->firstReadAfterAddrMatch = false;
	}

	i2c_target_status_clear(dev, target_status);
}
#endif /* CONFIG_I2C_TARGET */

/**
 * @brief Check if the controller should continue to the next I2C message.
 *
 * Determines whether the current transfer should automatically continue
 * to the next message without issuing a STOP or RESTART condition.
 *
 * @param data Pointer to I2C controller runtime data structure.
 *
 * @return true if the next message can continue without restart, false otherwise.
 */
static bool i2c_controller_check_continue_next(struct i2c_mchp_dev_data *data)
{
	bool continue_next = false;

	/* Check if only one byte is left in the current message, more
	 * messages remain, the direction (read/write) of the current and
	 * next message is the same, and the next message does not have the
	 * RESTART flag set.
	 */
	if ((data->current_msg.size == 1U) && (data->num_msgs > 1U) &&
	    ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) ==
	     (data->msgs_array[data->msg_index + 1U].flags & I2C_MSG_RW_MASK)) &&
	    ((data->msgs_array[data->msg_index + 1U].flags & I2C_MSG_RESTART) == 0U)) {
		continue_next = true;
	}

	return continue_next;
}

/**
 * @brief Handle I2C controller error interrupt.
 *
 * Stops the current I2C transfer, disables interrupts, and notifies
 * the caller using either callback or semaphore depending on mode.
 *
 * @param dev Pointer to the I2C device structure.
 */
static void i2c_handle_controller_error(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;

	i2c_controller_transfer_stop(dev);
	i2c_controller_int_disable(dev, SERCOM_I2CM_INTENSET_Msk);

#ifdef CONFIG_I2C_CALLBACK
	data->i2c_async_callback(dev, (int)data->current_msg.status, data->user_data);
#else
	k_sem_give(&data->i2c_sync_sem);
#endif /*CONFIG_I2C_CALLBACK*/
}

/**
 * @brief Handle controller write-mode interrupts.
 *
 * Sends bytes from the current message buffer to the I2C bus.
 * Handles message completion, next-message continuation, and callback signaling.
 *
 * @param dev Pointer to the I2C device structure.
 * @param continue_next Indicates if the transfer should continue to next message.
 */
static void i2c_handle_controller_write_mode(const struct device *dev, bool continue_next)
{
	struct i2c_mchp_dev_data *data = dev->data;

	if (data->current_msg.size == 0U) {
		i2c_controller_transfer_stop(dev);
		i2c_controller_int_disable(dev, SERCOM_I2CM_INTFLAG_MB_Msk);

		if (data->num_msgs > 1U) {
			data->msg_index++;
			data->num_msgs--;
			data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
			data->current_msg.size = data->msgs_array[data->msg_index].len;
			data->current_msg.status = 0U;
			i2c_restart(dev);
		} else {
#ifdef CONFIG_I2C_CALLBACK
			data->i2c_async_callback(dev, (int)data->current_msg.status,
						 data->user_data);
#else
			k_sem_give(&data->i2c_sync_sem);
#endif /*CONFIG_I2C_CALLBACK*/
		}
	} else {
		i2c_byte_write(dev, *data->current_msg.buffer);
		data->current_msg.buffer++;
		data->current_msg.size--;
	}

	if (continue_next == true) {
		data->msg_index++;
		data->num_msgs--;
		data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
		data->current_msg.size = data->msgs_array[data->msg_index].len;
		data->current_msg.status = 0U;
	}
}

/**
 * @brief Handle controller read-mode interrupts.
 *
 * Reads incoming bytes from the I2C bus into the message buffer.
 * Handles message completion, optional continuation, and callback signaling.
 *
 * @param dev Pointer to the I2C device structure.
 * @param continue_next Indicates if the transfer should continue to next message.
 */
static void i2c_handle_controller_read_mode(const struct device *dev, bool continue_next)
{
	struct i2c_mchp_dev_data *data = dev->data;

	if ((continue_next == false) && (data->current_msg.size == 1U)) {
		i2c_controller_transfer_stop(dev);
	}

#ifndef CONFIG_I2C_MCHP_DMA_DRIVEN
	*data->current_msg.buffer = i2c_byte_read(dev);
	data->current_msg.buffer++;
	data->current_msg.size--;
#endif /*!CONFIG_I2C_MCHP_DMA_DRIVEN*/

	if ((continue_next == false) && (data->current_msg.size == 0U)) {
		i2c_controller_int_disable(dev, SERCOM_I2CM_INTFLAG_SB_Msk);

		if (data->num_msgs > 1U) {
			data->msg_index++;
			data->num_msgs--;
			data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
			data->current_msg.size = data->msgs_array[data->msg_index].len;
			data->current_msg.status = 0U;
			i2c_restart(dev);
		} else {
#ifdef CONFIG_I2C_CALLBACK
			data->i2c_async_callback(dev, (int)data->current_msg.status,
						 data->user_data);
#else
			k_sem_give(&data->i2c_sync_sem);
#endif /*CONFIG_I2C_CALLBACK*/
		}
	}

	if (continue_next == true) {
		data->msg_index++;
		data->num_msgs--;
		data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
		data->current_msg.size = data->msgs_array[data->msg_index].len;
		data->current_msg.status = 0U;
	}
}

/**
 * @brief Interrupt Service Routine for I2C controller/target.
 *
 * Handles I2C interrupt events for both target and controller modes.
 * Dispatches to respective sub-handlers based on the current configuration.
 *
 * @param dev Pointer to the I2C device structure.
 */
static void i2c_mchp_isr(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;
	bool continue_next = false;

#ifdef CONFIG_I2C_TARGET
	/* Check if the device is operating in target mode */
	if (data->target_mode == true) {

		/* Delegate target-specific operations to a dedicated handler */
		i2c_target_handler(dev);
	} else
#endif /*CONFIG_I2C_TARGET*/
	{

		/* Get current interrupt status to identify the cause of the interrupt */
		uint8_t int_status = i2c_controller_int_flag_get(dev);

		/* Terminate if there are any critical errors on the bus */
		if (i2c_is_terminate_on_error(dev) == false) {

			/* Handle ERROR interrupt flag for controller mode transmit and
			 * receive */
			if (int_status == SERCOM_I2CM_INTFLAG_ERROR_Msk) {
				i2c_handle_controller_error(dev);
			} else {

				continue_next = i2c_controller_check_continue_next(data);

				switch (int_status) {
				case SERCOM_I2CM_INTFLAG_MB_Msk:
					i2c_handle_controller_write_mode(dev, continue_next);
					break;
				case SERCOM_I2CM_INTFLAG_SB_Msk:
					i2c_handle_controller_read_mode(dev, continue_next);
					break;
				default:
					break;
				}
			}
		}
	}
}

#ifdef CONFIG_I2C_MCHP_TARGET
/**
 * @brief Enable or disable the I2C peripheral in target (slave) mode.
 *
 * This API enables or disables the I2C peripheral by setting or clearing the
 * enable bit in the control register for target mode. It waits for the
 * synchronization process to complete after the operation.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param enable Set to true to enable the target, false to disable.
 */
static void i2c_target_enable(const struct device *dev, bool enable)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	if (enable == true) {
		i2c_regs->I2CS.SERCOM_CTRLA |= SERCOM_I2CS_CTRLA_ENABLE(1);
	} else {
		i2c_regs->I2CS.SERCOM_CTRLA &= SERCOM_I2CS_CTRLA_ENABLE(0);
	}

	/* Wait for synchronization */
	while ((i2c_regs->I2CS.SERCOM_SYNCBUSY & SERCOM_I2CS_SYNCBUSY_ENABLE_Msk) ==
	       SERCOM_I2CS_SYNCBUSY_ENABLE_Msk) {

		/* Do nothing */
	};
}

/**
 * @brief Enable or disable RUNSTDBY for I2C target (slave) mode.
 *
 * This function sets or clears the RUNSTDBY bit in the SERCOM CTRLA register
 * for the I2C target, allowing the peripheral to run in standby mode.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static void i2c_target_runstandby_enable(const struct device *dev)
{
	const struct i2c_mchp_dev_config *const i2c_cfg = dev->config;
	sercom_registers_t *i2c_regs = i2c_cfg->regs;
	uint32_t reg32_val = i2c_regs->I2CS.SERCOM_CTRLA;

	reg32_val &= ~SERCOM_I2CS_CTRLA_RUNSTDBY_Msk;
	reg32_val |= SERCOM_I2CS_CTRLA_RUNSTDBY(i2c_cfg->run_in_standby);
	i2c_regs->I2CS.SERCOM_CTRLA = reg32_val;
}

/**
 * @brief Configure the I2C peripheral to operate in target (slave) mode.
 *
 * This API sets the necessary control register bits to enable target (slave) mode
 * for the I2C peripheral. It enables smart mode features, sets the target mode,
 * configures SDA hold time, and sets the speed.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static void i2c_set_target_mode(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	/* Enable i2c device smart mode features */
	i2c_regs->I2CS.SERCOM_CTRLB = ((i2c_regs->I2CS.SERCOM_CTRLB & ~SERCOM_I2CS_CTRLB_SMEN_Msk) |
				       SERCOM_I2CS_CTRLB_SMEN(1));

	i2c_regs->I2CS.SERCOM_CTRLA =
		(i2c_regs->I2CS.SERCOM_CTRLA & ~SERCOM_I2CS_CTRLA_MODE_Msk) |
		(SERCOM_I2CS_CTRLA_MODE(0x4) | SERCOM_I2CS_CTRLA_SDAHOLD(0x1) |
		 SERCOM_I2CS_CTRLA_SPEED(0x1));
}

/**
 * @brief Reset the I2C target address register.
 *
 * This API resets the I2C target address register by clearing the address field.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 */
static void i2c_reset_target_addr(const struct device *dev)
{
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	i2c_regs->I2CS.SERCOM_ADDR &=
		~(SERCOM_I2CS_ADDR_ADDR_Msk | SERCOM_I2CS_ADDR_ADDRMASK_Msk |
		  SERCOM_I2CS_ADDR_TENBITEN_Msk);
	i2c_regs->I2CS.SERCOM_CTRLB =
		(i2c_regs->I2CS.SERCOM_CTRLB & ~SERCOM_I2CS_CTRLB_AMODE_Msk) |
		SERCOM_I2CS_CTRLB_AMODE(SERCOM_I2CS_CTRLB_AMODE_MASK_Val);
}

/**
 * @brief Enable I2C target (slave) interrupts.
 *
 * This API enables specific I2C target interrupts by setting the corresponding bits
 * in the interrupt enable set register. The function constructs a bitmask based on
 * the provided interrupt flags and writes it to the register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param target_int Bitmask of target interrupt flags to enable.
 */
static void i2c_target_int_enable(const struct device *dev, uint8_t target_int)
{
	uint8_t int_set = 0;

	if ((target_int & SERCOM_I2CS_INTENSET_PREC_Msk) == SERCOM_I2CS_INTENSET_PREC_Msk) {
		int_set |= SERCOM_I2CS_INTENSET_PREC(1);
	}
	if ((target_int & SERCOM_I2CS_INTENSET_AMATCH_Msk) == SERCOM_I2CS_INTENSET_AMATCH_Msk) {
		int_set |= SERCOM_I2CS_INTENSET_AMATCH(1);
	}
	if ((target_int & SERCOM_I2CS_INTENSET_DRDY_Msk) == SERCOM_I2CS_INTENSET_DRDY_Msk) {
		int_set |= SERCOM_I2CS_INTENSET_DRDY(1);
	}
	if ((target_int & SERCOM_I2CS_INTENSET_ERROR_Msk) == SERCOM_I2CS_INTENSET_ERROR_Msk) {
		int_set |= SERCOM_I2CS_INTENSET_ERROR(1);
	}

	I2C_REGS->I2CS.SERCOM_INTENSET = int_set;
}

/**
 * @brief Disable I2C target (slave) interrupts.
 *
 * This API disables specific I2C target interrupts by setting the corresponding
 * bits in the interrupt enable clear register. The function constructs a bitmask
 * based on the provided interrupt flags and writes it to the register.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @param target_int Bitmask of target interrupt flags to disable (see @ref
 * i2c_mchp_target_interrupt_t).
 */
static void i2c_target_int_disable(const struct device *dev, uint8_t target_int)
{
	uint8_t int_clear = 0;

	if ((target_int & SERCOM_I2CS_INTENCLR_PREC_Msk) == SERCOM_I2CS_INTENCLR_PREC_Msk) {
		int_clear |= SERCOM_I2CS_INTENCLR_PREC(1);
	}
	if ((target_int & SERCOM_I2CS_INTENCLR_AMATCH_Msk) == SERCOM_I2CS_INTENCLR_AMATCH_Msk) {
		int_clear |= SERCOM_I2CS_INTENCLR_AMATCH(1);
	}
	if ((target_int & SERCOM_I2CS_INTENCLR_DRDY_Msk) == SERCOM_I2CS_INTENCLR_DRDY_Msk) {
		int_clear |= SERCOM_I2CS_INTENCLR_DRDY(1);
	}
	if ((target_int & SERCOM_I2CS_INTENCLR_ERROR_Msk) == SERCOM_I2CS_INTENCLR_ERROR_Msk) {
		int_clear |= SERCOM_I2CS_INTENCLR_ERROR(1);
	}

	I2C_REGS->I2CS.SERCOM_INTENCLR = int_clear;
}

#if defined(CONFIG_I2C_MCHP_DMA_DRIVEN) && defined(CONFIG_I2C_TARGET_BUFFER_MODE)
static void i2c_target_mask_drdy_for_dma(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;

	if (!data->tgt_drdy_masked) {
		i2c_target_int_disable(dev, SERCOM_I2CS_INTENCLR_DRDY_Msk);
		data->tgt_drdy_masked = true;
	}
}

static void i2c_target_unmask_drdy_after_dma(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;

	if (data->tgt_drdy_masked && !data->tgt_tx_dma_active && !data->tgt_rx_dma_active) {
		i2c_target_int_enable(dev, SERCOM_I2CS_INTENSET_DRDY_Msk);
		data->tgt_drdy_masked = false;
	}
}
#endif

static bool i2c_mchp_target_matches_addr(const struct i2c_target_config *cfg, uint16_t addr)
{
	uint16_t normalized_addr = addr & 0x3FFU;

	if (cfg == NULL) {
		return false;
	}

	if (cfg->address_mask != 0U) {
		uint16_t must_match = (~cfg->address_mask) & 0x3FFU;

		if (((cfg->address ^ normalized_addr) & must_match) == 0U) {
			return true;
		}
	}

	return (cfg->address & 0x3FFU) == normalized_addr;
}

static struct i2c_target_config *i2c_mchp_find_target_cfg(struct i2c_mchp_dev_data *data,
							  uint16_t addr)
{
	for (size_t i = 0U; i < ARRAY_SIZE(data->target_cfgs); i++) {
		struct i2c_target_config *cfg = data->target_cfgs[i];

		if (cfg == NULL) {
			continue;
		}

		if (i2c_mchp_target_matches_addr(cfg, addr)) {
			return cfg;
		}
	}

	return NULL;
}

static uint16_t i2c_mchp_get_matched_addr(const struct device *dev)
{
	uint8_t raw_data = (uint8_t)I2C_REGS->I2CS.SERCOM_DATA;
	uint16_t inferred_addr = (uint16_t)((raw_data >> 1) & 0x3FFU);

	return inferred_addr;
}

/**
 * @brief Program the SERCOM target address configuration.
 *
 * @param dev Pointer to the I2C device structure.
 * @param cfg Pointer to the stored target configuration.
 */
static void i2c_mchp_apply_target_addrs(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;
	struct i2c_target_config *primary = data->target_cfgs[0];
	struct i2c_target_config *secondary_cfg = data->target_cfgs[1];
	uint32_t addr_reg = 0U;
	uint32_t amode_val = SERCOM_I2CS_CTRLB_AMODE_MASK_Val;
	uint16_t mask = 0U;

	if (primary == NULL) {
		i2c_regs->I2CS.SERCOM_ADDR &=
			~(SERCOM_I2CS_ADDR_ADDR_Msk | SERCOM_I2CS_ADDR_ADDRMASK_Msk |
			  SERCOM_I2CS_ADDR_TENBITEN_Msk);
		return;
	}

	addr_reg |= SERCOM_I2CS_ADDR_ADDR(primary->address & 0x3FFU);

	if ((primary->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U) {
		addr_reg |= SERCOM_I2CS_ADDR_TENBITEN(1);
	}

	if (secondary_cfg != NULL) {
		addr_reg |= SERCOM_I2CS_ADDR_ADDRMASK(secondary_cfg->address & 0x3FFU);
		amode_val = SERCOM_I2CS_CTRLB_AMODE_2_ADDRESSES_Val;
	} else {
		mask = primary->address_mask & 0x3FFU;
		addr_reg |= SERCOM_I2CS_ADDR_ADDRMASK(mask);
		amode_val = SERCOM_I2CS_CTRLB_AMODE_MASK_Val;
	}

	i2c_regs->I2CS.SERCOM_ADDR =
		(i2c_regs->I2CS.SERCOM_ADDR &
		 ~(SERCOM_I2CS_ADDR_ADDR_Msk | SERCOM_I2CS_ADDR_ADDRMASK_Msk |
		   SERCOM_I2CS_ADDR_TENBITEN_Msk)) |
		addr_reg;

	i2c_regs->I2CS.SERCOM_CTRLB =
		(i2c_regs->I2CS.SERCOM_CTRLB & ~SERCOM_I2CS_CTRLB_AMODE_Msk) |
		SERCOM_I2CS_CTRLB_AMODE(amode_val);
}

/**
 * @brief Register the device in I2C target mode.
 *
 * This function configures the I2C peripheral to operate in target mode
 * with the specified target configuration, including address and callback
 * functions.
 *
 * @param dev Pointer to the I2C device structure.
 * @param target_cfg Pointer to the target configuration structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_target_register(const struct device *dev, struct i2c_target_config *target_cfg)
{
	struct i2c_mchp_dev_data *data = dev->data;
	const struct i2c_mchp_dev_config *const cfg = dev->config;
	int slot = -1;
	int retval = 0;
	bool ten_bit;
	uint16_t addr_limit;

	if ((target_cfg == NULL) || (target_cfg->callbacks == NULL)) {
		return -EINVAL;
	}

	if (target_cfg->address == I2C_INVALID_ADDR) {
		LOG_ERR("device can't be registered in target mode with 0x00 address");
		return -EINVAL;
	}

	ten_bit = ((target_cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U);
	addr_limit = ten_bit ? 0x3FFU : 0x7FU;

	if (target_cfg->address > addr_limit) {
		LOG_ERR("target address 0x%x exceeds %s-bit range", target_cfg->address,
			ten_bit ? "10" : "7");
		return -EINVAL;
	}

	if ((target_cfg->address_mask != 0U) && ten_bit) {
		LOG_ERR("10-bit target with address mask unsupported");
		return -EINVAL;
	}

	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	if (data->target_cfg_count >= ARRAY_SIZE(data->target_cfgs)) {
		retval = -EBUSY;
		goto unlock;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(data->target_cfgs); i++) {
		if (data->target_cfgs[i] == target_cfg) {
			retval = -EALREADY;
			goto unlock;
		}

		if ((slot < 0) && (data->target_cfgs[i] == NULL)) {
			slot = (int)i;
		}
	}

	if (slot < 0) {
		retval = -EBUSY;
		goto unlock;
	}

	if ((slot == 1) && data->target_cfgs[0] == NULL) {
		slot = 0;
	}

	if (slot == 1) {
		struct i2c_target_config *primary = data->target_cfgs[0];

		if ((primary == NULL) ||
		    ((primary->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U) ||
		    (primary->address_mask != 0U)
		) {
			retval = -EINVAL;
			goto unlock;
		}

		if (ten_bit || (target_cfg->address_mask != 0U)) {
			retval = -EINVAL;
			goto unlock;
		}
	} else {
		/* nothing additional */
	}

	if (!data->target_mode) {
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
		data->tgt_tx_dma_active = false;
		data->tgt_tx_buf_active = false;
		data->tgt_tx_buf = NULL;
		data->tgt_tx_len = 0U;
		data->tgt_rx_dma_active = false;
		data->tgt_rx_block_size = 0U;
		data->dma_rx_target_cfg = NULL;
		data->tgt_drdy_masked = false;
#endif /* CONFIG_I2C_TARGET_BUFFER_MODE */

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		if (cfg->i2c_dma.dma_dev != NULL) {
			(void)dma_stop(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel);
			(void)dma_stop(cfg->i2c_dma.dma_dev, cfg->i2c_dma.tx_dma_channel);
		}
#endif

		i2c_controller_enable(dev, false);
		i2c_controller_int_disable(dev, SERCOM_I2CM_INTENSET_Msk);
		i2c_controller_int_flag_clear(dev, SERCOM_I2CM_INTFLAG_Msk);
		i2c_controller_status_clear(dev, SERCOM_I2CM_STATUS_Msk);

		i2c_target_enable(dev, false);
		i2c_swrst(dev);
		i2c_set_target_mode(dev);
		i2c_target_runstandby_enable(dev);
		i2c_target_int_disable(dev, SERCOM_I2CS_INTENSET_Msk);
		i2c_target_int_flag_clear(dev, SERCOM_I2CS_INTFLAG_Msk);
		i2c_target_status_clear(dev, SERCOM_I2CS_STATUS_Msk);
	} else {
		i2c_target_enable(dev, false);
	}

	data->target_cfgs[slot] = target_cfg;
	data->target_cfg_count++;
	data->target_mode = true;

	i2c_mchp_apply_target_addrs(dev);

	if (data->target_cfg_count == 1U) {
		i2c_target_int_enable(dev, SERCOM_I2CS_INTENSET_Msk);
		i2c_target_runstandby_enable(dev);
	}

	i2c_target_enable(dev, true);

unlock:
	if ((retval != 0) && (slot >= 0) && (slot < ARRAY_SIZE(data->target_cfgs))) {
		if (data->target_cfgs[slot] == target_cfg) {
			data->target_cfgs[slot] = NULL;
			if (data->target_cfg_count > 0U) {
				data->target_cfg_count--;
			}
			if (data->target_cfg_count == 0U) {
				data->target_mode = false;
			}
			i2c_target_enable(dev, true);
		}
	}

	k_mutex_unlock(&data->i2c_bus_mutex);

	return retval;
}

/**
 * @brief Unregister the device in I2C target mode.
 *
 * This function unregister the I2C peripheral from operate in target mode
 * with the specified target configuration, including address and callback
 * functions.
 *
 * @param dev Pointer to the I2C device structure.
 * @param target_cfg Pointer to the target configuration structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_target_unregister(const struct device *dev,
				      struct i2c_target_config *target_cfg)
{
	struct i2c_mchp_dev_data *data = dev->data;
	int retval = 0;
	int slot = -1;

	if (target_cfg == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	if (!data->target_mode || (data->target_cfg_count == 0U)) {
		retval = -EBUSY;
		goto unlock;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(data->target_cfgs); i++) {
		if (data->target_cfgs[i] == target_cfg) {
			slot = (int)i;
			break;
		}
	}

	if (slot < 0) {
		retval = -EINVAL;
		goto unlock;
	}

	i2c_target_enable(dev, false);

#if defined(CONFIG_I2C_MCHP_DMA_DRIVEN) && defined(CONFIG_I2C_TARGET_BUFFER_MODE)
	const struct i2c_mchp_dev_config *const cfg = dev->config;
	if (data->tgt_rx_dma_active) {
		(void)dma_stop(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel);
		data->tgt_rx_dma_active = false;
	}
	if (data->tgt_tx_dma_active) {
		(void)dma_stop(cfg->i2c_dma.dma_dev, cfg->i2c_dma.tx_dma_channel);
		data->tgt_tx_dma_active = false;
	}
	data->tgt_tx_buf_active = false;
	data->tgt_tx_buf = NULL;
	data->tgt_tx_len = 0U;
	data->tgt_drdy_masked = false;
	data->dma_rx_target_cfg = NULL;
#endif

	data->target_cfgs[slot] = NULL;
	if (data->target_cfg_count > 0U) {
		data->target_cfg_count--;
	}

	if (slot == 0 && data->target_cfgs[1] != NULL) {
		data->target_cfgs[0] = data->target_cfgs[1];
		data->target_cfgs[1] = NULL;
	}

	if (data->active_target_cfg == target_cfg) {
		data->active_target_cfg = NULL;
	}

	if (data->target_cfg_count > 0U) {
		i2c_mchp_apply_target_addrs(dev);
		i2c_target_enable(dev, true);
	} else {
		i2c_target_int_disable(dev, SERCOM_I2CS_INTENSET_Msk);
		i2c_reset_target_addr(dev);
		data->target_mode = false;
		i2c_target_enable(dev, true);
	}

unlock:
	k_mutex_unlock(&data->i2c_bus_mutex);

	return retval;
}
#endif /*CONFIG_I2C_MCHP_TARGET*/

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
/**
 * @brief Callback function for DMA write completion.
 *
 * This function is called when a DMA write transfer is completed. It checks for
 * errors, finalizes the I2C operation if needed, and re-enables the I2C interrupt
 * to handle the final stages of the transaction.
 *
 * @param dma_dev Pointer to the DMA device.
 * @param arg Pointer to the I2C device structure (passed as argument during DMA
 * configuration).
 * @param id DMA transaction ID (not used here).
 * @param error_code DMA operation error code (0 for success, negative for errors).
 */
static void i2c_dma_write_done(const struct device *dma_dev, void *arg, uint32_t id, int error_code)
{
	const struct device *dev = arg;
	struct i2c_mchp_dev_data *data = dev->data;
	const struct i2c_mchp_dev_config *const cfg = dev->config;
	bool continue_next = false;
	int retval = I2C_MCHP_SUCCESS;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);

	/* Lock interrupts to ensure atomic handling of the DMA completion. */
	unsigned int key = irq_lock();

	/* Check I2C operation should be terminated due to an error */
	if (i2c_is_terminate_on_error(dev) != false) {

		/* If termination is required, notify upper layer (async mode). */
#ifdef CONFIG_I2C_CALLBACK
		data->i2c_async_callback(dev, (int)data->current_msg.status, data->user_data);
#endif
		irq_unlock(key);
	}

	/* Check for DMA-specific errors during the transfer. */
	else if (error_code < 0) {
			LOG_ERR("DMA write error on %s: %d", dev->name, error_code);
#ifdef CONFIG_I2C_CALLBACK
			data->i2c_async_callback(dev, error_code, data->user_data);
#endif
		irq_unlock(key);
	} else {

		irq_unlock(key);

		/* Check if next message can continue in same direction */
		if ((data->num_msgs > 1U) &&
		    ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) ==
		     (data->msgs_array[data->msg_index + 1U].flags & I2C_MSG_RW_MASK)) &&
		    ((data->msgs_array[data->msg_index + 1U].flags & I2C_MSG_RESTART) == 0U)) {
			continue_next = true;
		}

		if (continue_next) {
			data->msg_index++;
			data->num_msgs--;
			data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
			data->current_msg.size = data->msgs_array[data->msg_index].len;
			data->current_msg.status = 0U;

			/* Configure DMA and start next transfer */
			if (i2c_dma_write_config(dev) == 0) {
				retval = dma_start(cfg->i2c_dma.dma_dev,
						   cfg->i2c_dma.tx_dma_channel);
				if (retval != 0) {
#ifdef CONFIG_I2C_CALLBACK
					data->i2c_async_callback(dev, retval, data->user_data);
#endif
				}
			}
		} else {
			data->current_msg.size = 0U;
			i2c_controller_int_enable(dev, SERCOM_I2CM_INTENSET_MB_Msk);
		}
	}
}

/**
 * @brief Callback function for DMA read completion.
 *
 * This function is triggered when a DMA read transfer is completed. It checks for
 * errors, handles termination conditions, and allows the ISR to manage the final
 * byte of data and the terminating NACK.
 *
 * @param dma_dev Pointer to the DMA device.
 * @param arg Pointer to the I2C device structure (passed as an argument during DMA
 * configuration).
 * @param id DMA transaction ID (not used here).
 * @param error_code DMA operation error code (0 for success, negative for errors).
 */
static void i2c_dma_read_done(const struct device *dma_dev, void *arg, uint32_t id, int error_code)
{
	const struct device *dev = arg;
	struct i2c_mchp_dev_data *data = dev->data;
	const struct i2c_mchp_dev_config *const cfg = dev->config;

	bool continue_next = false;
	int retval = I2C_MCHP_SUCCESS;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);

    /* Lock interrupts to ensure atomic handling of the DMA completion. */
	unsigned int key = irq_lock();

	/* Check I2C operation should be terminated due to an error */
	if (i2c_is_terminate_on_error(dev) != false) {

			/* If termination is required, notify upper layer if async enabled */
#ifdef CONFIG_I2C_CALLBACK
			data->i2c_async_callback(dev, (int)data->current_msg.status, data->user_data);
#endif
		irq_unlock(key);
	}

	/* Check for DMA-specific errors during the transfer. */
	else if (error_code < 0) {
			LOG_ERR("DMA read error on %s: %d", dev->name, error_code);
#ifdef CONFIG_I2C_CALLBACK
			data->i2c_async_callback(dev, error_code, data->user_data);
#endif
		irq_unlock(key);
	} else {

		irq_unlock(key);

		/* Check if next message can continue in same direction */
		if ((data->num_msgs > 1U) &&
		    ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) ==
		     (data->msgs_array[data->msg_index + 1U].flags & I2C_MSG_RW_MASK)) &&
		    ((data->msgs_array[data->msg_index + 1U].flags & I2C_MSG_RESTART) == 0U)) {
			continue_next = true;
		}

		if (continue_next) {
			data->msg_index++;
			data->num_msgs--;
			data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
			data->current_msg.size = data->msgs_array[data->msg_index].len;
			data->current_msg.status = 0U;

			/* Configure DMA for next read */
			if (i2c_dma_read_config(dev) == 0) {
				retval = dma_start(cfg->i2c_dma.dma_dev,
						   cfg->i2c_dma.rx_dma_channel);
				if (retval != 0) {
#ifdef CONFIG_I2C_CALLBACK
					data->i2c_async_callback(dev, retval, data->user_data);
#endif
				}
			}
		} else {
			data->current_msg.size = 0U;
			i2c_controller_int_enable(dev, SERCOM_I2CM_INTENSET_SB_Msk);
		}
	}
}

/**
 * @brief Configure DMA for I2C write operations.
 *
 * This function sets up the DMA configuration for transferring data from memory
 * to the I2C peripheral during a write operation. It validates the input,
 * initializes DMA configurations, and ensures proper error handling.
 *
 * @param dev Pointer to the I2C device structure.
 * @return true if DMA was successfully configured, false otherwise.
 */
static int i2c_dma_write_config(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;
	const struct i2c_mchp_dev_config *const cfg = dev->config;

	/* Initialize DMA configuration structures. */
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval = I2C_MCHP_SUCCESS;

	/* Configure DMA transfer direction and data size. */
	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = i2c_dma_write_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->i2c_dma.tx_dma_request;

	/* Set up DMA block configuration for the transfer. */
	dma_blk.block_size = data->current_msg.size;
	dma_blk.source_address = (uint32_t)data->current_msg.buffer;
	dma_blk.dest_address = (uint32_t)(i2c_get_dma_dest_addr(dev));
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	/* Configure the DMA with the prepared configurations. */
	retval = dma_config(cfg->i2c_dma.dma_dev, cfg->i2c_dma.tx_dma_channel, &dma_cfg);
		if (retval != I2C_MCHP_SUCCESS) {

			/* Log an error message if DMA configuration fails. */
			LOG_ERR("Write DMA configure on %s failed: %d", dev->name, retval);
#ifdef CONFIG_I2C_CALLBACK
			data->i2c_async_callback(dev, retval, data->user_data);
#endif
		}

	return retval;
}

/**
 * @brief Configure DMA for I2C read operations.
 *
 * This function sets up the DMA configuration for transferring data from the I2C
 * peripheral to memory during a read operation. It validates the conditions for DMA
 * usage, initializes the DMA configuration, and handles errors gracefully.
 *
 * @param dev Pointer to the I2C device structure.
 * @return true if DMA was successfully configured, false otherwise.
 */
static int i2c_dma_read_config(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;
	const struct i2c_mchp_dev_config *const cfg = dev->config;

	/* Initialize DMA configuration structures. */
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int retval = I2C_MCHP_SUCCESS;

	/* Configure DMA transfer direction and data size. */
	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = i2c_dma_read_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->i2c_dma.rx_dma_request;

	/* Configure the DMA block for the transfer. */
	dma_blk.block_size = data->current_msg.size;
	dma_blk.dest_address = (uint32_t)data->current_msg.buffer;
	dma_blk.source_address = (uint32_t)(i2c_get_dma_source_addr(dev));
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	/* Configure the DMA with the prepared configurations. */
	retval = dma_config(cfg->i2c_dma.dma_dev, cfg->i2c_dma.rx_dma_channel, &dma_cfg);
		if (retval != I2C_MCHP_SUCCESS) {
			LOG_ERR("Read DMA configure on %s failed: %d", dev->name, retval);
#ifdef CONFIG_I2C_CALLBACK
			data->i2c_async_callback(dev, retval, data->user_data);
#endif
		}

	return retval;
}
#endif /*CONFIG_I2C_MCHP_DMA_DRIVEN*/

#ifdef CONFIG_I2C_CALLBACK
/**
 * @brief Perform an I2C transfer with callback notification.
 *
 * This function initiates an I2C transfer, either read or write, using DMA or
 * interrupt mode, and registers a callback to notify upon completion. It handles
 * message validation, DMA configuration, interrupt setup, and status
 * initialization.
 *
 * @param dev Pointer to the I2C device structure.
 * @param msgs Pointer to the array of I2C message structures.
 * @param num_msgs Number of messages in the array.
 * @param addr 7-bit or 10-bit I2C target address.
 * @param cb Callback function to invoke on transfer completion.
 * @param user_data User data to pass to the callback function.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				uint16_t addr, i2c_callback_t i2c_async_callback, void *user_data)
{
	struct i2c_mchp_dev_data *data = dev->data;
	uint32_t addr_reg;
	int retval = I2C_MCHP_SUCCESS;
	bool mutex_locked = false;

	/* Validate that there are messages to process. */
	if (num_msgs == 0) {
		retval = -EINVAL;
	}

	/* Check if the device is currently operating in target mode. */
	else if (data->target_mode == true) {
		LOG_ERR("Device currently running in target mode\n");
		retval = -EBUSY;
	} else {

		for (uint8_t i = 0U; (i < num_msgs) && (retval == I2C_MCHP_SUCCESS); i++) {
			if ((msgs[i].len == 0U) || (msgs[i].buf == NULL)) {
				retval = -EINVAL;
			}
		}
	}

	if (retval == I2C_MCHP_SUCCESS) {

		/* Lock the mutex to ensure exclusive access to the I2C device. */
		k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);
		mutex_locked = true;

		/* Initialize data fields for the transfer. */
		data->num_msgs = num_msgs;
		data->msgs_array = msgs;
		data->i2c_async_callback = i2c_async_callback;
		data->user_data = user_data;
		data->target_addr = addr;
		data->msg_index = 0;

		/* Disable I2C interrupts, clear interrupt flags, and reset status
		 * registers. */
		i2c_controller_int_disable(dev, SERCOM_I2CM_INTENSET_Msk);
		i2c_controller_int_flag_clear(dev, SERCOM_I2CM_INTFLAG_Msk);
		i2c_controller_status_clear(dev, SERCOM_I2CM_STATUS_Msk);

		/* Initialize message data for the transfer. */
		data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
		data->current_msg.size = data->msgs_array[data->msg_index].len;
		data->current_msg.status = 0;

		/* Prepare the address register with the 7-bit address and
		 * read/write bit.
		 */
		addr_reg = addr << 1U;
		bool is_read = ((data->msgs_array->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ);

		if (is_read == true) {
			addr_reg |= I2C_MESSAGE_DIR_READ;
		}

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN

		/* Configure DMA */
		retval = i2c_configure_dma(dev, is_read);

		if (retval == I2C_MCHP_SUCCESS) {

			/* Write target address to start transaction */
			i2c_controller_addr_write(dev, addr_reg);

			/* Start DMA transfer */
			retval = i2c_start_dma(dev, is_read);
			if (retval != I2C_MCHP_SUCCESS) {
				LOG_ERR("%s DMA start failed: %d", is_read ? "Read" : "Write",
					retval);
#ifdef CONFIG_I2C_CALLBACK
				data->i2c_async_callback(dev, retval, data->user_data);
#endif
			}
		}
#else
		/* Non-DMA: write address to start transaction and enable interrupts */
		i2c_controller_addr_write(dev, addr_reg);
		i2c_controller_int_enable(dev, SERCOM_I2CM_INTENSET_Msk);
#endif /*CONFIG_I2C_MCHP_DMA_DRIVEN*/
	}

	if (mutex_locked == true) {

		/* Unlock the mutex after completing the setup. */
		k_mutex_unlock(&data->i2c_bus_mutex);
	}

	/* Return the status of the operation. */
	return retval;
}
#endif /*CONFIG_I2C_CALLBACK*/

#if !defined(CONFIG_I2C_MCHP_INTERRUPT_DRIVEN)
static bool i2c_is_nack(const struct device *dev);

static int i2c_poll_wait_for_flag(const struct device *dev, uint8_t flag_mask, bool check_nack)
{
#if CONFIG_I2C_MCHP_TRANSFER_TIMEOUT > 0
	int64_t deadline = k_uptime_get() + CONFIG_I2C_MCHP_TRANSFER_TIMEOUT;
#endif

	while ((i2c_controller_int_flag_get(dev) & flag_mask) != flag_mask) {
		if (check_nack && i2c_is_nack(dev)) {
			LOG_DBG("Detected RXNACK while waiting for flag 0x%02x", flag_mask);
			return -EIO;
		}

		uint16_t status = i2c_controller_status_get(dev);
		uint16_t error_bits = status & I2C_MCHP_STATUS_ERROR_MASK;

		if (error_bits != 0U) {
			LOG_ERR("Controller error while waiting for flag 0x%02x (status=0x%04x)",
				flag_mask, status);
			i2c_controller_status_clear(dev, error_bits);
			return -EIO;
		}

#if CONFIG_I2C_MCHP_TRANSFER_TIMEOUT > 0
		if (k_uptime_get() > deadline) {
			LOG_ERR("Timeout waiting for I2C flag 0x%02x", flag_mask);
			return -ETIMEDOUT;
		}
#endif

		k_yield();
	}

	return I2C_MCHP_SUCCESS;
}

/**
 * @brief Get the NACK status of the I2C controller or target during data transfer.
 *
 * This API checks whether a NACK (Not Acknowledge) condition has occurred during
 * transmit or receive operations in either controller (master) or target (slave)
 * mode. It reads the appropriate status register based on the current I2C mode and
 * returns true if a NACK was detected, or false otherwise.
 *
 * @param dev Pointer to the device structure for the I2C peripheral.
 * @return true if a NACK condition is detected, false otherwise.
 */
static bool i2c_is_nack(const struct device *dev)
{
	bool retval;
	sercom_registers_t *i2c_regs = ((const struct i2c_mchp_dev_config *)(dev)->config)->regs;

	if ((i2c_regs->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) ==
	    SERCOM_I2CM_CTRLA_MODE_I2C_MASTER) {
		if ((i2c_regs->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_RXNACK_Msk) ==
		    SERCOM_I2CM_STATUS_RXNACK_Msk) {
			retval = true;
		} else {
			retval = false;
		}
	} else {
		if ((i2c_regs->I2CS.SERCOM_STATUS & SERCOM_I2CS_STATUS_RXNACK_Msk) ==
		    SERCOM_I2CS_STATUS_RXNACK_Msk) {
			retval = true;
		} else {
			retval = false;
		}
	}

	return retval;
}

/**
 * @brief Perform a polled I2C read operation.
 *
 * This function reads bytes from the I2C bus using a polling mechanism.
 * It checks for a NACK to stop the transfer early if necessary and handles
 * byte-by-byte data reception.
 *
 * @param dev Pointer to the I2C device structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_poll_in(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;
	int retval = I2C_MCHP_SUCCESS;
	bool stop_sent = false;

	/* Check for a NACK condition. If NACK is received, stop the transfer. */
	if (i2c_is_nack(dev) == true) {
		retval = -EIO;
		goto stop;
	} else {

		/* Loop through the message buffer and read each byte from the I2C
		 * bus. */
		for (uint32_t i = 0; i < data->current_msg.size; i++) {
			retval = i2c_poll_wait_for_flag(dev, SERCOM_I2CM_INTFLAG_SB_Msk, false);
			if (retval != I2C_MCHP_SUCCESS) {
				goto stop;
			}

			/* Stop the I2C transfer when reading the last byte. */
			if (i == data->current_msg.size - 1) {
				i2c_controller_transfer_stop(dev);
				stop_sent = true;
			}

			/* Read a byte from the I2C bus and store it in the buffer.
			 */
			data->current_msg.buffer[i] = i2c_byte_read(dev);
		}
	}

	return retval;

stop:
	if (!stop_sent) {
		i2c_controller_transfer_stop(dev);
	}

	return retval;
}

/**
 * @brief Perform a polled I2C write operation.
 *
 * This function writes bytes to the I2C bus using a polling mechanism.
 * It checks for a NACK after each byte is written and terminates the transfer
 * early if a NACK is detected.
 *
 * @param dev Pointer to the I2C device structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_poll_out(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;
	int retval = I2C_MCHP_SUCCESS;

	/* Check for a NACK condition before starting the transfer. */
	if (i2c_is_nack(dev) == true) {
		i2c_controller_transfer_stop(dev);
		retval = -EIO;
	} else {

		/* Loop through the message buffer and write each byte to the I2C
		 * bus. */
		for (uint32_t i = 0; i < data->current_msg.size; i++) {
			retval = i2c_poll_wait_for_flag(dev, SERCOM_I2CM_INTFLAG_MB_Msk, true);
			if (retval != I2C_MCHP_SUCCESS) {
				goto stop;
			}

			/* Write a byte to the I2C bus. */
			i2c_byte_write(dev, data->current_msg.buffer[i]);

			/* Check for a NACK condition after writing each byte. */
			if (i2c_is_nack(dev) == true) {
				retval = -EIO;
				goto stop;
			}
		}

		/* Stop the I2C transfer after all bytes have been written (handled below). */
	}

stop:
	i2c_controller_transfer_stop(dev);
	return retval;
}
#endif /*!CONFIG_I2C_MCHP_INTERRUPT_DRIVEN*/

/**
 * @brief Validate I2C transfer parameters.
 *
 * Checks that the number of messages is non-zero, the device is not in target mode,
 * and each message has a valid length and buffer.
 *
 * @param data Pointer to the device-specific data.
 * @param msgs Pointer to the array of I2C messages.
 * @param num_msgs Number of messages to transfer.
 *
 * @return 0 if parameters are valid, otherwise a negative error code.
 */
static int i2c_validate_transfer_params(const struct i2c_mchp_dev_data *data, struct i2c_msg *msgs,
					uint8_t num_msgs)
{
	int retval = I2C_MCHP_SUCCESS;

	/* Validate input parameters. */
	if (num_msgs == 0) {
		retval = -EINVAL;
	}

	/* Check if the device is currently in target mode. */
	else if (data->target_mode == true) {
		LOG_ERR("Device currently configured in target mode\n");
		retval = -EBUSY;
	} else {

		/* Check for empty messages (invalid read/write). */
		for (uint8_t i = 0; i < num_msgs; i++) {
			if (msgs[i].len == 0 || msgs[i].buf == NULL) {
				retval = -EINVAL;
				break;
			}
		}
	}

	return retval;
}

#ifdef CONFIG_I2C_MCHP_INTERRUPT_DRIVEN
/**
 * @brief Handle errors during interrupt-driven I2C transfer.
 *
 * Waits for the interrupt handler to complete the transfer and checks
 * for timeout or transaction errors such as arbitration loss.
 *
 * @param dev Pointer to the I2C device structure.
 * @param data Pointer to the device-specific data.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_check_interrupt_flag_errors(const struct device *dev, struct i2c_mchp_dev_data *data)
{
	int retval = I2C_MCHP_SUCCESS;

	if (data->current_msg.status != 0) {
		if ((data->current_msg.status & SERCOM_I2CM_STATUS_ARBLOST_Msk) ==
		    SERCOM_I2CM_STATUS_ARBLOST_Msk) {
			LOG_DBG("Arbitration lost on %s", dev->name);
			retval = -EAGAIN;
		} else {
			LOG_ERR("Transaction error on %s: %08X", dev->name,
				data->current_msg.status);
			retval = -EIO;
		}
	}

	return retval;
}
#endif /*CONFIG_I2C_MCHP_INTERRUPT_DRIVEN*/

/**
 * @brief Perform an I2C transfer.
 *
 * Handles reading or writing to the I2C bus with optional interrupt-driven or
 * polled modes. Supports multi-message transfers and checks for potential errors
 * during communication.
 *
 * @param dev Pointer to the I2C device structure.
 * @param msgs Pointer to an array of I2C messages to be processed.
 * @param num_msgs Number of messages in the array.
 * @param addr Target device address on the I2C bus.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			     uint16_t addr)
{
	struct i2c_mchp_dev_data *data = dev->data;
	uint32_t addr_reg;
	int retval = I2C_MCHP_SUCCESS;
	bool mutex_locked = false;

	retval = i2c_validate_transfer_params(data, msgs, num_msgs);

	if (retval == I2C_MCHP_SUCCESS) {
		k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);
		mutex_locked = true;

		/* Disable I2C interrupts, clear interrupt flags, and reset status
		 * registers. */
		i2c_controller_int_disable(dev, SERCOM_I2CM_INTENSET_Msk);
		i2c_controller_int_flag_clear(dev, SERCOM_I2CM_INTFLAG_Msk);
		i2c_controller_status_clear(dev, SERCOM_I2CM_STATUS_Msk);

		/* Set up transfer data. */
		data->num_msgs = num_msgs;
		data->msgs_array = msgs;
		data->msg_index = 0;
		data->target_addr = addr;

		while ((data->num_msgs > 0) && (retval == I2C_MCHP_SUCCESS)) {

			/* Initialize message buffer and size. */
			data->current_msg.buffer = data->msgs_array[data->msg_index].buf;
			data->current_msg.size = data->msgs_array[data->msg_index].len;
			data->current_msg.status = 0;

			/* Set up I2C address register with target address */
			addr_reg = addr << 1U;
			if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) ==
			    I2C_MSG_READ) {
				addr_reg |= I2C_MESSAGE_DIR_READ; /* Read operation. */
			}

			/* Writing the address starts the I2C transaction. */
			i2c_controller_addr_write(dev, addr_reg);

#ifdef CONFIG_I2C_MCHP_INTERRUPT_DRIVEN
			i2c_controller_int_enable(dev, SERCOM_I2CM_INTENSET_Msk);
#else
			/* Process transfer based on message direction (read/write) */
			if ((data->msgs_array[data->msg_index].flags & I2C_MSG_RW_MASK) ==
			    I2C_MSG_READ) {
				retval = i2c_poll_in(dev);

			} else {
				retval = i2c_poll_out(dev);
			}
#endif /*CONFIG_I2C_MCHP_INTERRUPT_DRIVEN*/

#ifdef CONFIG_I2C_MCHP_INTERRUPT_DRIVEN
			/* Wait for transfer completion */
			retval = k_sem_take(&data->i2c_sync_sem, I2C_TRANSFER_TIMEOUT_MSEC);
			if (retval != 0) {
				LOG_ERR("Transfer timeout on %s", dev->name);
				i2c_controller_transfer_stop(dev);
				break;
			}

			/* Check errors */
			retval = i2c_check_interrupt_flag_errors(dev, data);
#endif /*CONFIG_I2C_MCHP_INTERRUPT_DRIVEN*/

			/* Move to the next message in the array. */
			data->num_msgs--;
			data->msg_index++;
		}
	}

	if (mutex_locked == true) {

		/* Unlock the mutex before returning. */
		k_mutex_unlock(&data->i2c_bus_mutex);
	}

	return retval;
}

/**
 * @brief Recover the I2C bus from a hung or stuck state.
 *
 * This function disables the I2C peripheral, applies default pin configuration,
 * and forces the bus to an idle state to recover from any error conditions.
 *
 * @param dev Pointer to the I2C device structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_recover_bus(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;
	const struct i2c_mchp_dev_config *const cfg = dev->config;
	int retval = I2C_MCHP_SUCCESS;

	/* Lock the mutex to ensure exclusive access to the I2C bus. */
	k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

	/* Disable the I2C peripheral to prepare for bus recovery. */
	i2c_controller_enable(dev, false);

	/* Disable I2C interrupts to avoid interference during recovery. */
	i2c_controller_int_disable(dev, SERCOM_I2CM_INTENSET_Msk);

	/* Apply the default pin configuration state. */
	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

	if (retval < I2C_MCHP_SUCCESS) {

		LOG_ERR("Failed to apply default pin state: %d", retval);
	} else {

		/* Re-enable the I2C peripheral. */
		i2c_controller_enable(dev, true);

		/* Force the I2C bus to idle state to recover from a bus hang. */
		i2c_set_controller_bus_state_idle(dev);
	}

	/* Unlock the mutex before returning. */
	k_mutex_unlock(&data->i2c_bus_mutex);

	return retval;
}

/**
 * @brief Retrieve the current I2C device configuration.
 *
 * This function returns the current configuration of the I2C device,
 * such as speed, addressing mode, etc.
 *
 * @param dev Pointer to the I2C device structure.
 * @param dev_config Pointer to store the retrieved configuration.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_mchp_dev_data *data = dev->data;
	int retval = I2C_MCHP_SUCCESS;

	/*Check if the device configuration is valid*/
	if (data->dev_config == 0) {
		retval = -EINVAL;
	} else {

		/* Retrieve the current device configuration */
		*dev_config = data->dev_config;

		LOG_DBG("Retrieved I2C device configuration: 0x%08X", *dev_config);
	}
	return retval;
}

/**
 * @brief Set and apply the I2C bitrate configuration.
 *
 * This function configures the I2C bitrate based on the provided configuration.
 * It ensures proper synchronization, checks for valid speed modes, and updates the
 * device configuration after successful application.
 *
 * @param dev Pointer to the I2C device structure.
 * @param config The desired I2C configuration, including speed settings.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_set_apply_bitrate(const struct device *dev, uint32_t config)
{
	uint32_t sys_clock_rate = 0;
	uint32_t bitrate;
	int retval = I2C_MCHP_SUCCESS;

	/* Determine the bitrate based on the I2C speed configuration */
	switch (I2C_SPEED_GET(config)) {
	case I2C_SPEED_STANDARD:
		bitrate = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		bitrate = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		bitrate = MHZ(1);
		break;
	case I2C_SPEED_HIGH:
		bitrate = MHZ(3.4);
		break;
	default:
		LOG_ERR("Unsupported speed code: %d", I2C_SPEED_GET(config));
		retval = -ENOTSUP;
	}

	if (retval == I2C_MCHP_SUCCESS) {
		const struct i2c_mchp_dev_config *const cfg = dev->config;

		/* Retrieve the clock frequency for baud rate calculation */
		clock_control_get_rate(cfg->i2c_clock.clock_dev, cfg->i2c_clock.gclk_sys,
				       &sys_clock_rate);

		if (sys_clock_rate == 0) {
			LOG_ERR("Failed to retrieve system clock rate.");
			retval = -EIO;
		}

		if (retval == I2C_MCHP_SUCCESS) {

			/*Set the I2C baud rate */
			if (i2c_set_baudrate(dev, bitrate, sys_clock_rate) != true) {
				LOG_ERR("Failed to set I2C baud rate to %u Hz.", bitrate);
				retval = -EIO;
			}
		}
	}

	return retval;
}

/**
 * @brief Configure the I2C interface with the specified settings.
 *
 * This function configures the I2C device based on the provided settings,
 * including the mode (controller/target) and speed. It ensures the configuration
 * is applied safely by disabling the interface before making changes and
 * re-enabling it after.
 *
 * @param dev Pointer to the I2C device structure.
 * @param config Configuration flags specifying mode and speed.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_configure(const struct device *dev, uint32_t config)
{
	struct i2c_mchp_dev_data *data = dev->data;
	int retval = I2C_MCHP_SUCCESS;

	/* Check if the device is currently operating in target mode */
	if (data->target_mode == true) {
		LOG_ERR("Cannot reconfigure while device is in target mode.");
		retval = -EBUSY;
	} else {

		/* Lock the mutex to ensure exclusive access to the I2C bus. */
		k_mutex_lock(&data->i2c_bus_mutex, K_FOREVER);

		/*Disable the I2C interface to allow configuration changes*/
		i2c_controller_enable(dev, false);

		/*Check if the configuration specifies the I2C controller mode*/
		if ((config & I2C_MODE_CONTROLLER) == I2C_MODE_CONTROLLER) {

			/* Set the I2C to controller mode*/
			i2c_set_controller_mode(dev);
		}

		/* Check and configure I2C speed if specified */
		if (I2C_SPEED_GET(config) != 0) {

			/*Set and apply the bitrate for the I2C interface*/
			retval = i2c_set_apply_bitrate(dev, config);
		}

		if (retval == I2C_MCHP_SUCCESS) {

			/*Update the device configuration with the new speed*/
			data->dev_config = I2C_SPEED_GET(config);

			/* Re-enable the I2C interface after configuration */
			i2c_controller_enable(dev, true);

			/* Force the I2C bus state to idle to recover from any
			 * potential errors */
			i2c_set_controller_bus_state_idle(dev);
		}

		/* Unlock the mutex before returning. */
		k_mutex_unlock(&data->i2c_bus_mutex);
	}

	return retval;
}

/**
 * @brief Initialize the I2C peripheral.
 *
 * This function performs the initialization of the I2C hardware, including
 * clock configuration, reset, pin control setup, IRQ configuration, and setting
 * the initial I2C mode and speed. It also initializes synchronization primitives
 * (mutex and semaphore) for the driver.
 *
 * @param dev Pointer to the I2C device structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int i2c_mchp_init(const struct device *dev)
{
	struct i2c_mchp_dev_data *data = dev->data;
	const struct i2c_mchp_dev_config *const cfg = dev->config;
	int retval = I2C_MCHP_SUCCESS;

	/* Enable the clocks for the specified I2C instance */
	retval = clock_control_on(cfg->i2c_clock.clock_dev, cfg->i2c_clock.gclk_sys);
	if (retval == I2C_MCHP_SUCCESS) {
		retval = clock_control_on(cfg->i2c_clock.clock_dev, cfg->i2c_clock.mclk_sys);
	}

	if (retval == I2C_MCHP_SUCCESS) {

		/* Reset all I2C registers*/
		i2c_swrst(dev);

		/* Initialize mutex and semaphore for I2C data structure */
		k_mutex_init(&data->i2c_bus_mutex);
		k_sem_init(&data->i2c_sync_sem, 0, 1);
		data->target_mode = false;

		/* Apply pin control configuration for SDA and SCL lines */
		retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	}

	if (retval == I2C_MCHP_SUCCESS) {

		/* Configure the I2C peripheral with the specified bitrate and
		 * mode*/
		retval = i2c_configure(dev,
				       (I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(cfg->bitrate)));
		if (retval != 0) {
			LOG_ERR("Failed to apply pinctrl state. Error: %d", retval);
		}
	}

	if (retval == I2C_MCHP_SUCCESS) {

		/* Disable the I2C peripheral before further configuration */
		i2c_controller_enable(dev, false);

		/*Configure the IRQ for the I2C peripheral*/
		cfg->irq_config_func(dev);

#ifdef CONFIG_I2C_MCHP_DMA_DRIVEN
		/* Check if the DMA device is ready */
		if (device_is_ready(cfg->i2c_dma.dma_dev) == false) {
			retval = -ENODEV;
		}
#endif /*CONFIG_I2C_MCHP_DMA_DRIVEN*/
	}

	if (retval == I2C_MCHP_SUCCESS) {
		/* Enable runstandby for I2C controller */
		i2c_controller_runstandby_enable(dev);

		/*Enable the I2C peripheral*/
		i2c_controller_enable(dev, true);

		/*Force the I2C bus to idle state*/
		i2c_set_controller_bus_state_idle(dev);
	}

	return retval;
}

/******************************************************************************
 * @brief Zephyr driver instance creation
 *****************************************************************************/
static DEVICE_API(i2c, i2c_mchp_api) = {
	.configure = i2c_mchp_configure,
	.get_config = i2c_mchp_get_config,
	.transfer = i2c_mchp_transfer,
#ifdef CONFIG_I2C_MCHP_TARGET
	.target_register = i2c_mchp_target_register,
	.target_unregister = i2c_mchp_target_unregister,
#endif /*CONFIG_I2C_MCHP_TARGET*/
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = i2c_mchp_transfer_cb,
#endif /*CONFIG_I2C_CALLBACK*/
	.recover_bus = i2c_mchp_recover_bus,
};

#define I2C_MCHP_REG_DEFN(n) .regs = (sercom_registers_t *)DT_INST_REG_ADDR(n),

/* Do the peripheral interrupt related configuration */
#if DT_INST_IRQ_HAS_IDX(0, 3)
#define I2C_MCHP_IRQ_HANDLER(n)                                                                    \
	static void i2c_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		I2C_MCHP_IRQ_CONNECT(n, 0);                                                        \
		I2C_MCHP_IRQ_CONNECT(n, 1);                                                        \
		I2C_MCHP_IRQ_CONNECT(n, 2);                                                        \
		I2C_MCHP_IRQ_CONNECT(n, 3);                                                        \
	}
#else
#define I2C_MCHP_IRQ_HANDLER(n)                                                                    \
	static void i2c_mchp_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		I2C_MCHP_IRQ_CONNECT(n, 0);                                                        \
	}
#endif /*DT_INST_IRQ_HAS_IDX(0, 3)*/

#define I2C_MCHP_CLOCK_DEFN(n)                                                                     \
	.i2c_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                                 \
	.i2c_clock.mclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem)),           \
	.i2c_clock.gclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem)),

#if CONFIG_I2C_MCHP_DMA_DRIVEN
#define I2C_MCHP_DMA_CHANNELS(n)                                                                   \
	.i2c_dma.dma_dev = DEVICE_DT_GET(MCHP_DT_INST_DMA_CTLR(n, tx)),                            \
	.i2c_dma.tx_dma_request = MCHP_DT_INST_DMA_TRIGSRC(n, tx),                                 \
	.i2c_dma.tx_dma_channel = MCHP_DT_INST_DMA_CHANNEL(n, tx),                                 \
	.i2c_dma.rx_dma_request = MCHP_DT_INST_DMA_TRIGSRC(n, rx),                                 \
	.i2c_dma.rx_dma_channel = MCHP_DT_INST_DMA_CHANNEL(n, rx),
#else
#define I2C_MCHP_DMA_CHANNELS(n)
#endif /*CONFIG_I2C_MCHP_DMA_DRIVEN*/

#define I2C_MCHP_IRQ_CONNECT(n, m)                                                                 \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq), DT_INST_IRQ_BY_IDX(n, m, priority),     \
			    i2c_mchp_isr, DEVICE_DT_INST_GET(n), 0);                               \
		irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));                                         \
	} while (false)

#define I2C_MCHP_CONFIG_DEFN(n)                                                                    \
	static const struct i2c_mchp_dev_config i2c_mchp_dev_config_##n = {                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
		.irq_config_func = &i2c_mchp_irq_config_##n,                                       \
		.run_in_standby = DT_INST_PROP(n, run_in_standby_en),                              \
		I2C_MCHP_REG_DEFN(n) I2C_MCHP_CLOCK_DEFN(n) I2C_MCHP_DMA_CHANNELS(n)}

#define I2C_MCHP_DEVICE_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void i2c_mchp_irq_config_##n(const struct device *dev);                             \
	I2C_MCHP_CONFIG_DEFN(n);                                                                   \
	static struct i2c_mchp_dev_data i2c_mchp_dev_data_##n;                                     \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_mchp_init, NULL, &i2c_mchp_dev_data_##n,                  \
				  &i2c_mchp_dev_config_##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, \
				  &i2c_mchp_api);                                                  \
	I2C_MCHP_IRQ_HANDLER(n)

DT_INST_FOREACH_STATUS_OKAY(I2C_MCHP_DEVICE_INIT)
