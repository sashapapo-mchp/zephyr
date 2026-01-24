/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * PUKCC (Public Key Cryptographic Controller) Crypto Driver for Microchip PIC32CX/SAM E5x
 *
 * This driver implements the Zephyr crypto API using the PUKCC hardware
 * accelerator for ECC and RSA operations.
 */

#define DT_DRV_COMPAT microchip_pukcc_g1

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

/* Include PUKCC public API headers */
#include <security/pukcc/pukcc_ecc.h>
#include <security/pukcc/pukcc_rsa.h>

/* Include PUKCC CryptoLib headers for internal implementation */
#include <security/pukcc/CryptoLib_typedef_pb.h>
#include <security/pukcc/CryptoLib_mapping_pb.h>
#include <security/pukcc/CryptoLib_cf_pb.h>
#include <security/pukcc/CryptoLib_Headers_pb.h>
#include <security/pukcc/CryptoLib_Hardware_pb.h>

/*
 * Internal definitions - not exposed in public headers
 */

/* ECC Parameter Length for P-256 curve */
#define PUKCC_ECC_PARAM_LEN   32u
#define PUKCC_PADDING         4u
#define PUKCC_PARAM_RAM_LEN   (PUKCC_ECC_PARAM_LEN + PUKCC_PADDING)
#define PUKCC_BASE_CRYPTO_RAM 0x02011000u

/* ECC Curve Parameters Structure (internal) */
typedef struct {
    uint8_t a[PUKCC_ECC_PARAM_LEN];
    uint8_t modulo_p[PUKCC_ECC_PARAM_LEN];
    uint8_t base_x[PUKCC_ECC_PARAM_LEN];
    uint8_t base_y[PUKCC_ECC_PARAM_LEN];
    uint8_t base_z[PUKCC_ECC_PARAM_LEN];
    uint8_t order[PUKCC_ECC_PARAM_LEN];
    uint8_t one[PUKCC_ECC_PARAM_LEN];
} pukcc_curve_256_t;

/* Memory layout for ECDSA verification (internal) */
#define BASE_ECDSAV_MODULO       PUKCC_BASE_CRYPTO_RAM
#define BASE_ECDSAV_CNS          (BASE_ECDSAV_MODULO + PUKCC_PARAM_RAM_LEN)
#define BASE_ECDSAV_ORDER        (BASE_ECDSAV_CNS + PUKCC_PARAM_RAM_LEN + 8)
#define BASE_ECDSAV_SIGNATURE    (BASE_ECDSAV_ORDER + PUKCC_PARAM_RAM_LEN + 8)
#define BASE_ECDSAV_HASH         (BASE_ECDSAV_SIGNATURE + 2 * PUKCC_PARAM_RAM_LEN)
#define BASE_ECDSAV_POINT_A      (BASE_ECDSAV_HASH + PUKCC_PARAM_RAM_LEN)
#define BASE_ECDSAV_POINT_A_X    (BASE_ECDSAV_POINT_A)
#define BASE_ECDSAV_POINT_A_Y    (BASE_ECDSAV_POINT_A_X + PUKCC_PARAM_RAM_LEN)
#define BASE_ECDSAV_POINT_A_Z    (BASE_ECDSAV_POINT_A_Y + PUKCC_PARAM_RAM_LEN)
#define BASE_ECDSAV_PUBLIC_KEY   (BASE_ECDSAV_POINT_A_Z + PUKCC_PARAM_RAM_LEN)
#define BASE_ECDSAV_PUBLIC_KEY_X (BASE_ECDSAV_PUBLIC_KEY)
#define BASE_ECDSAV_PUBLIC_KEY_Y (BASE_ECDSAV_PUBLIC_KEY_X + PUKCC_PARAM_RAM_LEN)
#define BASE_ECDSAV_PUBLIC_KEY_Z (BASE_ECDSAV_PUBLIC_KEY_Y + PUKCC_PARAM_RAM_LEN)
#define BASE_ECDSAV_A            (BASE_ECDSAV_PUBLIC_KEY_Z + PUKCC_PARAM_RAM_LEN)
#define BASE_ECDSAV_WORKSPACE    (BASE_ECDSAV_A + PUKCC_PARAM_RAM_LEN)

LOG_MODULE_REGISTER(crypto_mchp_pukcc, CONFIG_CRYPTO_LOG_LEVEL);

/* Driver capabilities */
#define PUKCC_CAPS_SUPPORT (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

/* Maximum sessions from Kconfig */
#define PUKCC_MAX_SESSION CONFIG_CRYPTO_MCHP_PUKCC_MAX_SESSION

/* MCLK register access for clock control */
#define MCLK_AHBMASK_REG    (*(volatile uint32_t *)(0x40000800 + 0x10))

/* P-256 curve parameters */
static const pukcc_curve_256_t curve_p256 = {
    /* Elliptical curve equation: y^2 = x^3 + a*x + b */
    .a = {
        0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc,
    },
    .modulo_p = {
        0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    },
    .base_x = {
        0x6b, 0x17, 0xd1, 0xf2, 0xe1, 0x2c, 0x42, 0x47,
        0xf8, 0xbc, 0xe6, 0xe5, 0x63, 0xa4, 0x40, 0xf2,
        0x77, 0x03, 0x7d, 0x81, 0x2d, 0xeb, 0x33, 0xa0,
        0xf4, 0xa1, 0x39, 0x45, 0xd8, 0x98, 0xc2, 0x96,
    },
    .base_y = {
        0x4f, 0xe3, 0x42, 0xe2, 0xfe, 0x1a, 0x7f, 0x9b,
        0x8e, 0xe7, 0xeb, 0x4a, 0x7c, 0x0f, 0x9e, 0x16,
        0x2b, 0xce, 0x33, 0x57, 0x6b, 0x31, 0x5e, 0xce,
        0xcb, 0xb6, 0x40, 0x68, 0x37, 0xbf, 0x51, 0xf5,
    },
    .base_z = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    },
    .order = {
        0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xbc, 0xe6, 0xfa, 0xad, 0xa7, 0x17, 0x9e, 0x84,
        0xf3, 0xb9, 0xca, 0xc2, 0xfc, 0x63, 0x25, 0x51,
    },
    .one = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    }
};

/*
 * Per-session context
 */
struct pukcc_session {
    bool in_use;
    bool started;
};

/*
 * Driver runtime data
 */
struct pukcc_data {
    struct k_sem lock;
    struct pukcc_session sessions[PUKCC_MAX_SESSION];
    bool clocks_enabled;
    PUKCL_PARAM pukcl_param;
    PPUKCL_PARAM p_pukcl_param;
};

static int pukcc_enable_clocks(struct pukcc_data *data)
{
    if (data->clocks_enabled) {
        return 0;
    }

    /* Enable PUKCC AHB clock (bit 20) */
    MCLK_AHBMASK_REG |= MCLK_AHBMASK_PUKCC_Msk;

    /* Verify clock is enabled */
    if (!(MCLK_AHBMASK_REG & MCLK_AHBMASK_PUKCC_Msk)) {
        LOG_ERR("Failed to enable PUKCC clock");
        return -EIO;
    }

    data->clocks_enabled = true;
    LOG_DBG("PUKCC clock enabled");
    return 0;
}

/* Helper: copy data with byte reversal (big-endian to little-endian) */
static void pukcc_memcopy_reverse(uint8_t *dest, const uint8_t *src, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        dest[i] = src[len - i - 1];
    }
    /* Add padding */
    memset(dest + len, 0, PUKCC_PADDING);
}

/* Helper: Initialize PUKCC and run self-test */
static int pukcc_self_test(struct pukcc_data *data)
{
    int timeout = 1000;

    memset(&data->pukcl_param, 0, sizeof(PUKCL_PARAM));
    data->p_pukcl_param = &data->pukcl_param;

    /* Run self-test */
    vPUKCL_Process(SelfTest, data->p_pukcl_param);

    /* Wait for completion */
    while (data->p_pukcl_param->PUKCL_Header.u2Status != PUKCL_OK && timeout > 0) {
        k_busy_wait(10);
        timeout--;
    }

    if (timeout <= 0) {
        LOG_ERR("PUKCC self-test timeout");
        return -ETIMEDOUT;
    }

    /* Verify version and check numbers */
    if (data->p_pukcl_param->P.PUKCL_SelfTest.u4Version != PUKCL_VERSION) {
        LOG_ERR("PUKCC version mismatch: got 0x%08lx, expected 0x%08lx",
                (unsigned long)data->p_pukcl_param->P.PUKCL_SelfTest.u4Version,
                (unsigned long)PUKCL_VERSION);
        return -EIO;
    }

    if (data->p_pukcl_param->P.PUKCL_SelfTest.u4CheckNum1 != 0x6E70DDD2 ||
        data->p_pukcl_param->P.PUKCL_SelfTest.u4CheckNum2 != 0x25C8D64F) {
        LOG_ERR("PUKCC self-test check failed");
        return -EIO;
    }

    LOG_DBG("PUKCC self-test passed");
    return 0;
}

/* Initialize ECDSA verification parameters in crypto RAM */
static void pukcc_setup_ecdsa_verify(struct pukcc_data *data,
                                     const uint8_t *signature,
                                     const uint8_t *hash,
                                     const uint8_t *public_key,
                                     const pukcc_curve_256_t *curve)
{
    /* Copy curve and input parameters to crypto RAM (with byte reversal) */
    pukcc_memcopy_reverse((uint8_t *)BASE_ECDSAV_MODULO, curve->modulo_p, PUKCC_ECC_PARAM_LEN);
    pukcc_memcopy_reverse((uint8_t *)BASE_ECDSAV_POINT_A_X, curve->base_x, PUKCC_ECC_PARAM_LEN);
    pukcc_memcopy_reverse((uint8_t *)BASE_ECDSAV_POINT_A_Y, curve->base_y, PUKCC_ECC_PARAM_LEN);
    pukcc_memcopy_reverse((uint8_t *)BASE_ECDSAV_POINT_A_Z, curve->one, PUKCC_ECC_PARAM_LEN);
    pukcc_memcopy_reverse((uint8_t *)BASE_ECDSAV_A, curve->a, PUKCC_ECC_PARAM_LEN);
    pukcc_memcopy_reverse((uint8_t *)BASE_ECDSAV_ORDER, curve->order, PUKCC_ECC_PARAM_LEN);

    /* Copy signature (R || S) */
    pukcc_memcopy_reverse((uint8_t *)BASE_ECDSAV_SIGNATURE, signature, PUKCC_ECC_PARAM_LEN);
    pukcc_memcopy_reverse((uint8_t *)(BASE_ECDSAV_SIGNATURE + PUKCC_PARAM_RAM_LEN),
                          signature + PUKCC_ECC_PARAM_LEN, PUKCC_ECC_PARAM_LEN);

    /* Copy public key (X || Y) */
    pukcc_memcopy_reverse((uint8_t *)BASE_ECDSAV_PUBLIC_KEY_X, public_key, PUKCC_ECC_PARAM_LEN);
    pukcc_memcopy_reverse((uint8_t *)BASE_ECDSAV_PUBLIC_KEY_Y,
                          public_key + PUKCC_ECC_PARAM_LEN, PUKCC_ECC_PARAM_LEN);
    pukcc_memcopy_reverse((uint8_t *)BASE_ECDSAV_PUBLIC_KEY_Z, curve->one, PUKCC_ECC_PARAM_LEN);

    /* Copy hash */
    pukcc_memcopy_reverse((uint8_t *)BASE_ECDSAV_HASH, hash, PUKCC_ECC_PARAM_LEN);

    /* Setup PUKCL parameters for ECDSA verification */
    PPUKCL_PARAM p = data->p_pukcl_param;
    p->P.PUKCL_ZpEcDsaVerify.nu1ModBase = (nu1)BASE_ECDSAV_MODULO;
    p->P.PUKCL_ZpEcDsaVerify.nu1CnsBase = (nu1)BASE_ECDSAV_CNS;
    p->P.PUKCL_ZpEcDsaVerify.nu1PointABase = (nu1)BASE_ECDSAV_POINT_A;
    p->P.PUKCL_ZpEcDsaVerify.nu1PointPublicKeyGen = (nu1)BASE_ECDSAV_PUBLIC_KEY;
    p->P.PUKCL_ZpEcDsaVerify.nu1PointSignature = (nu1)BASE_ECDSAV_SIGNATURE;
    p->P.PUKCL_ZpEcDsaVerify.nu1OrderPointBase = (nu1)BASE_ECDSAV_ORDER;
    p->P.PUKCL_ZpEcDsaVerify.nu1ABase = (nu1)BASE_ECDSAV_A;
    p->P.PUKCL_ZpEcDsaVerify.nu1Workspace = (nu1)BASE_ECDSAV_WORKSPACE;
    p->P.PUKCL_ZpEcDsaVerify.nu1HashBase = (nu1)BASE_ECDSAV_HASH;
    p->P.PUKCL_ZpEcDsaVerify.u2ModLength = PUKCC_ECC_PARAM_LEN;
    p->P.PUKCL_ZpEcDsaVerify.u2ScalarLength = PUKCC_ECC_PARAM_LEN;
}

/*
 * ECDSA P-256 signature verification
 */
int pukcc_ecdsa_p256_verify_impl(struct pukcc_data *data,
                                  const uint8_t *public_key,
                                  const uint8_t *signature,
                                  const uint8_t *hash)
{
    int ret;

    ret = pukcc_enable_clocks(data);
    if (ret != 0) {
        return ret;
    }

    ret = pukcc_self_test(data);
    if (ret != 0) {
        return ret;
    }

    /* Clear parameter structure before ECDSA setup */
    memset(&data->pukcl_param, 0, sizeof(PUKCL_PARAM));
    data->p_pukcl_param = &data->pukcl_param;

    /* Setup parameters */
    pukcc_setup_ecdsa_verify(data, signature, hash, public_key, &curve_p256);

    /* Execute ECDSA verification */
    vPUKCL_Process(ZpEcDsaVerifyFast, data->p_pukcl_param);

    /* Return status */
    return data->p_pukcl_param->PUKCL_Header.u2Status;
}

/*
 * Public API implementations
 */
int pukcc_ecc_init(void)
{
    /* Clocks will be enabled when needed */
    return 0;
}

static int pukcc_query_hw_caps(const struct device *dev)
{
    ARG_UNUSED(dev);
    return PUKCC_CAPS_SUPPORT;
}

/*
 * Driver initialization
 */
static int pukcc_init(const struct device *dev)
{
    struct pukcc_data *data = dev->data;

    k_sem_init(&data->lock, 1, 1);
    memset(data->sessions, 0, sizeof(data->sessions));
    data->clocks_enabled = false;
    data->p_pukcl_param = &data->pukcl_param;

    LOG_INF("PUKCC crypto driver initialized");
    return 0;
}

/*
 * Driver API - Note: PUKCC is primarily for asymmetric crypto (ECC/RSA)
 * The Zephyr crypto API is focused on hash and symmetric ciphers.
 * For full asymmetric crypto support, applications can use the
 * pukcc_ecdsa_p256_verify function directly.
 */
static DEVICE_API(crypto, pukcc_driver_api) = {
    .query_hw_caps = pukcc_query_hw_caps,
    /* Hash and cipher operations not supported by PUKCC */
    .hash_begin_session = NULL,
    .hash_free_session = NULL,
    .cipher_begin_session = NULL,
    .cipher_free_session = NULL,
    .cipher_async_callback_set = NULL,
    .hash_async_callback_set = NULL,
};

/*
 * Device instantiation
 */
static struct pukcc_data pukcc_data_0;

DEVICE_DT_INST_DEFINE(0, pukcc_init, NULL,
                      &pukcc_data_0, NULL,
                      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,
                      &pukcc_driver_api);

/*
 * Direct PUKCC API for applications
 * These functions bypass the Zephyr crypto API and provide direct
 * access to PUKCC asymmetric crypto operations.
 */

/**
 * @brief Verify ECDSA P-256 signature using PUKCC hardware
 *
 * @param public_key 64-byte public key (X || Y coordinates, big-endian)
 * @param signature  64-byte signature (R || S values, big-endian)
 * @param hash       32-byte SHA-256 hash of the message
 *
 * @return 0 (PUKCL_OK) on successful verification, error code otherwise
 */
int pukcc_ecdsa_p256_verify(const uint8_t *public_key,
                            const uint8_t *signature,
                            const uint8_t *hash)
{
    const struct device *dev = DEVICE_DT_INST_GET(0);
    struct pukcc_data *data;
    int ret;

    if (!device_is_ready(dev)) {
        return -ENODEV;
    }

    data = dev->data;
    k_sem_take(&data->lock, K_FOREVER);

    ret = pukcc_ecdsa_p256_verify_impl(data, public_key, signature, hash);

    k_sem_give(&data->lock);
    return ret;
}

/*
 * RSA Operations
 */

/* RSA memory layout macros (parameterized by modulus length) */
#define RSA_BASE_MODULO(len)     PUKCC_BASE_CRYPTO_RAM
#define RSA_BASE_CNS(len)        (RSA_BASE_MODULO(len) + (len) + 4)
#define RSA_BASE_RBASE(len)      (RSA_BASE_CNS(len) + (len) + 12)
#define RSA_BASE_XBASE(len)      (RSA_BASE_RBASE(len) + 64)
#define RSA_BASE_PRECOMP(len)    (RSA_BASE_XBASE(len) + (len) + 16)
#define RSA_BASE_EXP(len)        (RSA_BASE_PRECOMP(len) + 3 * ((len) + 4) + 8)

/**
 * @brief Initialize PUKCC hardware for RSA operations
 *
 * @return 0 on success, negative error code on failure
 */
int pukcc_rsa_init(void)
{
    const struct device *dev = DEVICE_DT_INST_GET(0);

    if (!device_is_ready(dev)) {
        return -ENODEV;
    }

    return 0;
}

/**
 * @brief Perform RSA modular exponentiation: result = base^exp mod modulo
 *
 * @param modulo       RSA modulus (big-endian)
 * @param modulo_len   Modulus length in bytes (128, 256, or 512)
 * @param base         Base value (input data, big-endian)
 * @param exponent     Public exponent (little-endian with padding)
 * @param exp_len      Exponent length in bytes
 * @param result       Output buffer for result (same size as modulus)
 *
 * @return 0 on success, PUKCL status code on failure
 */
int pukcc_rsa_exp_mod(const uint8_t *modulo,
                      uint32_t modulo_len,
                      const uint8_t *base,
                      const uint8_t *exponent,
                      uint32_t exp_len,
                      uint8_t *result)
{
    const struct device *dev = DEVICE_DT_INST_GET(0);
    struct pukcc_data *data;
    int ret;
    uint16_t status;

    if (!device_is_ready(dev)) {
        return -ENODEV;
    }

    /* Validate modulus length */
    if (modulo_len != 128 && modulo_len != 256 && modulo_len != 512) {
        return -EINVAL;
    }

    data = dev->data;
    k_sem_take(&data->lock, K_FOREVER);

    /* Enable clocks */
    ret = pukcc_enable_clocks(data);
    if (ret != 0) {
        k_sem_give(&data->lock);
        return ret;
    }

    /* Run self-test */
    ret = pukcc_self_test(data);
    if (ret != 0) {
        k_sem_give(&data->lock);
        return ret;
    }

    /* Calculate memory addresses based on modulus length */
    uint32_t base_modulo = RSA_BASE_MODULO(modulo_len);
    uint32_t base_cns = RSA_BASE_CNS(modulo_len);
    uint32_t base_rbase = RSA_BASE_RBASE(modulo_len);
    uint32_t base_xbase = RSA_BASE_XBASE(modulo_len);
    uint32_t base_precomp = RSA_BASE_PRECOMP(modulo_len);
    uint32_t base_exp = RSA_BASE_EXP(modulo_len);

    /* Step 1: Copy modulus to crypto RAM */
    memcpy((uint8_t *)base_modulo, modulo, modulo_len);
    memset((uint8_t *)base_modulo + modulo_len, 0, 4);

    /* Clear CNS and R areas */
    memset((void *)base_cns, 0, modulo_len + 12);
    memset((void *)base_rbase, 0, 64);
    memset((void *)(base_rbase + 64), 0, (modulo_len * 2) + 8);

    /* Step 2: Compute reduction constant (RedMod) */
    memset(&data->pukcl_param, 0, sizeof(PUKCL_PARAM));
    data->p_pukcl_param = &data->pukcl_param;
    data->p_pukcl_param->PUKCL_Header.u2Option = PUKCL_REDMOD_SETUP;
    data->p_pukcl_param->PUKCL_Header.Specific.CarryIn = 0;
    data->p_pukcl_param->PUKCL_Header.Specific.Gf2n = 0;
    data->p_pukcl_param->P.PUKCL_RedMod.u2ModLength = modulo_len;
    data->p_pukcl_param->P.PUKCL_RedMod.nu1ModBase = (nu1)base_modulo;
    data->p_pukcl_param->P.PUKCL_RedMod.nu1CnsBase = (nu1)base_cns;
    data->p_pukcl_param->P.PUKCL_RedMod.nu1RBase = (nu1)base_rbase;
    data->p_pukcl_param->P.PUKCL_RedMod.nu1XBase = (nu1)(base_rbase + 64);

    vPUKCL_Process(RedMod, data->p_pukcl_param);

    status = data->p_pukcl_param->PUKCL_Header.u2Status;
    if (status != PUKCL_OK) {
        LOG_ERR("RSA RedMod failed: 0x%04x", status);
        k_sem_give(&data->lock);
        return status;
    }

    /* Step 3: Copy input data and exponent */
    memcpy((uint8_t *)base_xbase, base, modulo_len);
    memset((uint8_t *)base_xbase + modulo_len, 0, 16);

    /* Clear precomputation area */
    memset((void *)base_precomp, 0, 3 * (modulo_len + 4) + 8);

    /* Copy exponent (already in little-endian format) */
    memcpy((uint8_t *)base_exp, exponent, exp_len);
    if (exp_len < 8) {
        memset((uint8_t *)base_exp + exp_len, 0, 8 - exp_len);
    }

    /* Step 4: Perform modular exponentiation (ExpMod) */
    memset(&data->pukcl_param, 0, sizeof(PUKCL_PARAM));
    data->p_pukcl_param = &data->pukcl_param;
    data->p_pukcl_param->PUKCL_Header.u2Option =
        PUKCL_EXPMOD_FASTRSA | PUKCL_EXPMOD_WINDOWSIZE_1 | PUKCL_EXPMOD_EXPINPUKCCRAM;
    data->p_pukcl_param->P.PUKCL_ExpMod.nu1ModBase = (nu1)base_modulo;
    data->p_pukcl_param->P.PUKCL_ExpMod.u2ModLength = modulo_len;
    data->p_pukcl_param->P.PUKCL_ExpMod.nu1CnsBase = (nu1)base_cns;
    data->p_pukcl_param->P.PUKCL_ExpMod.nu1XBase = (nu1)base_xbase;
    data->p_pukcl_param->P.PUKCL_ExpMod.nu1PrecompBase = (nu1)base_precomp;
    data->p_pukcl_param->P.PUKCL_ExpMod.pfu1ExpBase = (pfu1)base_exp;
    data->p_pukcl_param->P.PUKCL_ExpMod.u2ExpLength = (exp_len + 3) & ~3; /* Round up to 4 */
    data->p_pukcl_param->P.PUKCL_ExpMod.u1Blinding = 0;

    vPUKCL_Process(ExpMod, data->p_pukcl_param);

    status = data->p_pukcl_param->PUKCL_Header.u2Status;
    if (status != PUKCL_OK) {
        LOG_ERR("RSA ExpMod failed: 0x%04x", status);
        k_sem_give(&data->lock);
        return status;
    }

    /* Copy result */
    memcpy(result, (uint8_t *)base_xbase, modulo_len);

    k_sem_give(&data->lock);
    return 0;
}
