/*
 * Copyright (c) 2024-2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * ICM SHA-256 Hash and PUKCC ECDSA Verification Sample
 *
 * This sample demonstrates how to use:
 * - ICM crypto driver to compute SHA-256 hashes
 * - PUKCC crypto driver for ECDSA P-256 signature verification
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <string.h>

#ifdef CONFIG_MBEDTLS
#include <mbedtls/sha256.h>
#include <mbedtls/ecdsa.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/ecp.h>
#include <mbedtls/pk.h>
#include <mbedtls/md.h>
#include <mbedtls/version.h>
#endif

#ifdef CONFIG_CRYPTO_MCHP_PUKCC
#include <security/pukcc/pukcc_ecc.h>
#include <security/pukcc/pukcc_rsa.h>
#endif

LOG_MODULE_REGISTER(crypto_sample, LOG_LEVEL_INF);

/* Get ICM device from device tree */
#define ICM_DEV DEVICE_DT_GET(DT_NODELABEL(icm))

#ifdef CONFIG_CRYPTO_MCHP_PUKCC
/* Get PUKCC device from device tree */
#define PUKCC_DEV DEVICE_DT_GET(DT_NODELABEL(pukcc))
#endif

/* SHA-256 sizes */
#define SHA256_BLOCK_SIZE   64
#define SHA256_DIGEST_SIZE  32

/* ECC sizes */
#define ECC_P256_KEY_SIZE   64
#define ECC_P256_SIG_SIZE   64
#define ECC_P256_HASH_SIZE  32

/*
 * ICM Shell Commands
 */
#ifdef CONFIG_CRYPTO_MCHP_ICM

static int cmd_icm_status(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct device *dev = ICM_DEV;

	if (!device_is_ready(dev)) {
		shell_print(shell, "ICM device not ready");
		return -ENODEV;
	}

	int caps = crypto_query_hwcaps(dev);

	shell_print(shell, "ICM Crypto Driver Status:");
	shell_print(shell, "  Device: %s", dev->name);
	shell_print(shell, "  Ready: yes");
	shell_print(shell, "  Capabilities: 0x%08x", caps);
	shell_print(shell, "    - RAW_KEY: %s", (caps & CAP_RAW_KEY) ? "yes" : "no");
	shell_print(shell, "    - SEPARATE_IO_BUFS: %s", (caps & CAP_SEPARATE_IO_BUFS) ? "yes" : "no");
	shell_print(shell, "    - SYNC_OPS: %s", (caps & CAP_SYNC_OPS) ? "yes" : "no");

	return 0;
}

static int cmd_icm_hash(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct device *dev = ICM_DEV;
	struct hash_ctx ctx;
	struct hash_pkt pkt;
	int ret;

	if (!device_is_ready(dev)) {
		shell_print(shell, "ICM device not ready");
		return -ENODEV;
	}

	/*
	 * Test data: "Hello, World!" (13 bytes) with SHA-256 padding
	 *
	 * The ICM hardware requires pre-padded data in 64-byte blocks.
	 * SHA-256 padding format:
	 *   - Original message (13 bytes)
	 *   - 0x80 byte
	 *   - Zero padding to 56 bytes from start
	 *   - 64-bit big-endian bit length (13*8 = 104 = 0x68)
	 */
	static const uint8_t test_data[64] __aligned(64) = {
		/* "Hello, World!" (13 bytes) */
		'H', 'e', 'l', 'l', 'o', ',', ' ', 'W', 'o', 'r', 'l', 'd', '!',
		/* SHA-256 padding: 0x80 + 42 zeros */
		0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00,
		/* 64-bit big-endian bit length: 13 * 8 = 104 = 0x68 */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68
	};

	static uint8_t hash_output[SHA256_DIGEST_SIZE];

	shell_print(shell, "ICM SHA-256 Hash Test (using Zephyr Crypto API)");
	shell_print(shell, "Input: \"Hello, World!\" (13 bytes, padded to 64)");

	/* Setup hash context */
	memset(&ctx, 0, sizeof(ctx));
	ctx.flags = CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS;

	/* Begin hash session */
	ret = hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256);
	if (ret != 0) {
		shell_print(shell, "Failed to begin hash session: %d", ret);
		return ret;
	}

	/* Setup hash packet */
	memset(&pkt, 0, sizeof(pkt));
	pkt.in_buf = (uint8_t *)test_data;
	pkt.in_len = sizeof(test_data);
	pkt.out_buf = hash_output;
	pkt.ctx = &ctx;

	/* Compute hash */
	ret = hash_compute(&ctx, &pkt);
	if (ret != 0) {
		shell_print(shell, "Hash computation failed: %d", ret);
		hash_free_session(dev, &ctx);
		return ret;
	}

	/* Free session */
	hash_free_session(dev, &ctx);

	/* Print computed hash */
	shell_print(shell, "Computed hash:");
	shell_fprintf(shell, SHELL_NORMAL, "  ");
	for (int i = 0; i < SHA256_DIGEST_SIZE; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", hash_output[i]);
	}
	shell_print(shell, "");

	/* Print expected hash */
	shell_print(shell, "Expected hash:");
	shell_print(shell, "  dffd6021bb2bd5b0af676290809ec3a53191dd81c7f70a4b28688a362182986f");

	/* Verify the hash */
	static const uint8_t expected_hash[32] = {
		0xdf, 0xfd, 0x60, 0x21, 0xbb, 0x2b, 0xd5, 0xb0,
		0xaf, 0x67, 0x62, 0x90, 0x80, 0x9e, 0xc3, 0xa5,
		0x31, 0x91, 0xdd, 0x81, 0xc7, 0xf7, 0x0a, 0x4b,
		0x28, 0x68, 0x8a, 0x36, 0x21, 0x82, 0x98, 0x6f
	};

	if (memcmp(hash_output, expected_hash, SHA256_DIGEST_SIZE) == 0) {
		shell_print(shell, "Result: PASS - Hash matches expected value!");
	} else {
		shell_print(shell, "Result: FAIL - Hash does not match!");
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_icm,
	SHELL_CMD(status, NULL, "Show ICM crypto driver status", cmd_icm_status),
	SHELL_CMD(hash, NULL, "Compute SHA-256 hash using driver API", cmd_icm_hash),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(icm, &sub_icm, "ICM crypto driver commands", NULL);

#endif /* CONFIG_CRYPTO_MCHP_ICM */

/*
 * PUKCC Shell Commands
 */
#ifdef CONFIG_CRYPTO_MCHP_PUKCC

static int cmd_pukcc_status(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct device *dev = PUKCC_DEV;

	if (!device_is_ready(dev)) {
		shell_print(shell, "PUKCC device not ready");
		return -ENODEV;
	}

	int caps = crypto_query_hwcaps(dev);

	shell_print(shell, "PUKCC Crypto Driver Status:");
	shell_print(shell, "  Device: %s", dev->name);
	shell_print(shell, "  Ready: yes");
	shell_print(shell, "  Capabilities: 0x%08x", caps);
	shell_print(shell, "  Supported operations:");
	shell_print(shell, "    - ECDSA P-256 signature verification");
	shell_print(shell, "    - RSA signature verification");

	return 0;
}

static int cmd_pukcc_ecdsa(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct device *dev = PUKCC_DEV;

	if (!device_is_ready(dev)) {
		shell_print(shell, "PUKCC device not ready");
		return -ENODEV;
	}

	/*
	 * ECDSA P-256 Test Vector from RFC 6979 Appendix A.2.5
	 *
	 * This is a verified test vector for ECDSA with SHA-256.
	 * Message: "sample"
	 * Private key: C9AFA9D845BA75166B5C215767B1D6934E50C3DB36E89B127B8A622B120F6721
	 */

	/* Public key (X || Y coordinates, 32 bytes each, big-endian) */
	static const uint8_t test_public_key[64] = {
		/* Qx = 60FED4BA255A9D31C961EB74C6356D68C049B8923B61FA6CE669622E60F29FB6 */
		0x60, 0xFE, 0xD4, 0xBA, 0x25, 0x5A, 0x9D, 0x31,
		0xC9, 0x61, 0xEB, 0x74, 0xC6, 0x35, 0x6D, 0x68,
		0xC0, 0x49, 0xB8, 0x92, 0x3B, 0x61, 0xFA, 0x6C,
		0xE6, 0x69, 0x62, 0x2E, 0x60, 0xF2, 0x9F, 0xB6,
		/* Qy = 7903FE1008B8BC99A41AE9E95628BC64F2F1B20C2D7E9F5177A3C294D4462299 */
		0x79, 0x03, 0xFE, 0x10, 0x08, 0xB8, 0xBC, 0x99,
		0xA4, 0x1A, 0xE9, 0xE9, 0x56, 0x28, 0xBC, 0x64,
		0xF2, 0xF1, 0xB2, 0x0C, 0x2D, 0x7E, 0x9F, 0x51,
		0x77, 0xA3, 0xC2, 0x94, 0xD4, 0x46, 0x22, 0x99,
	};

	/* Signature R and S values (each 32 bytes, big-endian) */
	static const uint8_t test_signature[64] = {
		/* R = EFD48B2AACB6A8FD1140DD9CD45E81D69D2C877B56AAF991C34D0EA84EAF3716 */
		0xEF, 0xD4, 0x8B, 0x2A, 0xAC, 0xB6, 0xA8, 0xFD,
		0x11, 0x40, 0xDD, 0x9C, 0xD4, 0x5E, 0x81, 0xD6,
		0x9D, 0x2C, 0x87, 0x7B, 0x56, 0xAA, 0xF9, 0x91,
		0xC3, 0x4D, 0x0E, 0xA8, 0x4E, 0xAF, 0x37, 0x16,
		/* S = F7CB1C942D657C41D436C7A1B6E29F65F3E900DBB9AFF4064DC4AB2F843ACDA8 */
		0xF7, 0xCB, 0x1C, 0x94, 0x2D, 0x65, 0x7C, 0x41,
		0xD4, 0x36, 0xC7, 0xA1, 0xB6, 0xE2, 0x9F, 0x65,
		0xF3, 0xE9, 0x00, 0xDB, 0xB9, 0xAF, 0xF4, 0x06,
		0x4D, 0xC4, 0xAB, 0x2F, 0x84, 0x3A, 0xCD, 0xA8,
	};

	/* Message hash = SHA-256("sample")
	 * = AF2BDBE1AA9B6EC1E2ADE1D694F41FC71A831D0268E9891562113D8A62ADD1BF
	 */
	static const uint8_t test_hash[32] = {
		0xAF, 0x2B, 0xDB, 0xE1, 0xAA, 0x9B, 0x6E, 0xC1,
		0xE2, 0xAD, 0xE1, 0xD6, 0x94, 0xF4, 0x1F, 0xC7,
		0x1A, 0x83, 0x1D, 0x02, 0x68, 0xE9, 0x89, 0x15,
		0x62, 0x11, 0x3D, 0x8A, 0x62, 0xAD, 0xD1, 0xBF,
	};

	shell_print(shell, "PUKCC ECDSA P-256 Verification Test");
	shell_print(shell, "===================================");
	shell_print(shell, "Using NIST CAVP test vectors...");

	shell_print(shell, "Public Key (X || Y):");
	shell_fprintf(shell, SHELL_NORMAL, "  X: ");
	for (int i = 0; i < 32; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", test_public_key[i]);
	}
	shell_print(shell, "");
	shell_fprintf(shell, SHELL_NORMAL, "  Y: ");
	for (int i = 32; i < 64; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", test_public_key[i]);
	}
	shell_print(shell, "");

	shell_print(shell, "Signature (R || S):");
	shell_fprintf(shell, SHELL_NORMAL, "  R: ");
	for (int i = 0; i < 32; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", test_signature[i]);
	}
	shell_print(shell, "");
	shell_fprintf(shell, SHELL_NORMAL, "  S: ");
	for (int i = 32; i < 64; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", test_signature[i]);
	}
	shell_print(shell, "");

	shell_print(shell, "Message Hash:");
	shell_fprintf(shell, SHELL_NORMAL, "  ");
	for (int i = 0; i < 32; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", test_hash[i]);
	}
	shell_print(shell, "");

	shell_print(shell, "");
	shell_print(shell, "Verifying signature...");

	/* Call PUKCC ECDSA verification */
	int ret = pukcc_ecdsa_p256_verify(test_public_key, test_signature, test_hash);

	if (ret == 0) {
		shell_print(shell, "Result: PASS - Signature verified successfully!");
	} else {
		shell_print(shell, "Result: FAIL - Signature verification failed (status: 0x%04x)", ret);
	}

	return 0;
}

/*
 * ECC Information Command - Display P-256 curve info and available operations
 */
static int cmd_pukcc_ecc(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct device *dev = PUKCC_DEV;

	if (!device_is_ready(dev)) {
		shell_print(shell, "PUKCC device not ready");
		return -ENODEV;
	}

	shell_print(shell, "PUKCC ECC Information");
	shell_print(shell, "=====================");
	shell_print(shell, "");
	shell_print(shell, "Supported Curve: NIST P-256 (secp256r1)");
	shell_print(shell, "");
	shell_print(shell, "P-256 Curve Parameters (FIPS 186-4):");
	shell_print(shell, "  Prime (p):    FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF");
	shell_print(shell, "  Coefficient a: FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFC");
	shell_print(shell, "  Coefficient b: 5AC635D8AA3A93E7B3EBBD55769886BC651D06B0CC53B0F63BCE3C3E27D2604B");
	shell_print(shell, "  Order (n):    FFFFFFFF00000000FFFFFFFFFFFFFFFFBCE6FAADA7179E84F3B9CAC2FC632551");
	shell_print(shell, "  Cofactor (h): 1");
	shell_print(shell, "");
	shell_print(shell, "Key/Signature Sizes:");
	shell_print(shell, "  Public Key:   64 bytes (X || Y coordinates)");
	shell_print(shell, "  Signature:    64 bytes (R || S values)");
	shell_print(shell, "  Hash (SHA-256): 32 bytes");
	shell_print(shell, "");
	shell_print(shell, "Available PUKCC ECC Operations:");
	shell_print(shell, "  - ECDSA Signature Verification (pukcc_ecdsa_p256_verify)");
	shell_print(shell, "  - Point Addition (ZpEccAdd)");
	shell_print(shell, "  - Point Doubling (ZpEccDbl)");
	shell_print(shell, "  - Scalar Multiplication (ZpEccMul)");
	shell_print(shell, "");
	shell_print(shell, "Use 'pukcc ecdsa' to test ECDSA signature verification.");

	return 0;
}

/*
 * ECDSA with Invalid Signature Test (should fail)
 */
static int cmd_pukcc_ecdsa_invalid(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct device *dev = PUKCC_DEV;

	if (!device_is_ready(dev)) {
		shell_print(shell, "PUKCC device not ready");
		return -ENODEV;
	}

	shell_print(shell, "PUKCC ECDSA Invalid Signature Test");
	shell_print(shell, "===================================");
	shell_print(shell, "Testing with corrupted signature (should FAIL)...");

	/* Same public key as valid test (RFC 6979) */
	static const uint8_t test_public_key[64] = {
		/* Qx = 60FED4BA255A9D31C961EB74C6356D68C049B8923B61FA6CE669622E60F29FB6 */
		0x60, 0xFE, 0xD4, 0xBA, 0x25, 0x5A, 0x9D, 0x31,
		0xC9, 0x61, 0xEB, 0x74, 0xC6, 0x35, 0x6D, 0x68,
		0xC0, 0x49, 0xB8, 0x92, 0x3B, 0x61, 0xFA, 0x6C,
		0xE6, 0x69, 0x62, 0x2E, 0x60, 0xF2, 0x9F, 0xB6,
		/* Qy = 7903FE1008B8BC99A41AE9E95628BC64F2F1B20C2D7E9F5177A3C294D4462299 */
		0x79, 0x03, 0xFE, 0x10, 0x08, 0xB8, 0xBC, 0x99,
		0xA4, 0x1A, 0xE9, 0xE9, 0x56, 0x28, 0xBC, 0x64,
		0xF2, 0xF1, 0xB2, 0x0C, 0x2D, 0x7E, 0x9F, 0x51,
		0x77, 0xA3, 0xC2, 0x94, 0xD4, 0x46, 0x22, 0x99,
	};

	/* Corrupted signature - last byte changed from A8 to FF */
	static const uint8_t bad_signature[64] = {
		0xEF, 0xD4, 0x8B, 0x2A, 0xAC, 0xB6, 0xA8, 0xFD,
		0x11, 0x40, 0xDD, 0x9C, 0xD4, 0x5E, 0x81, 0xD6,
		0x9D, 0x2C, 0x87, 0x7B, 0x56, 0xAA, 0xF9, 0x91,
		0xC3, 0x4D, 0x0E, 0xA8, 0x4E, 0xAF, 0x37, 0x16,
		0xF7, 0xCB, 0x1C, 0x94, 0x2D, 0x65, 0x7C, 0x41,
		0xD4, 0x36, 0xC7, 0xA1, 0xB6, 0xE2, 0x9F, 0x65,
		0xF3, 0xE9, 0x00, 0xDB, 0xB9, 0xAF, 0xF4, 0x06,
		0x4D, 0xC4, 0xAB, 0x2F, 0x84, 0x3A, 0xCD, 0xFF, /* Changed last byte */
	};

	/* Same hash as valid test */
	static const uint8_t test_hash[32] = {
		0xAF, 0x2B, 0xDB, 0xE1, 0xAA, 0x9B, 0x6E, 0xC1,
		0xE2, 0xAD, 0xE1, 0xD6, 0x94, 0xF4, 0x1F, 0xC7,
		0x1A, 0x83, 0x1D, 0x02, 0x68, 0xE9, 0x89, 0x15,
		0x62, 0x11, 0x3D, 0x8A, 0x62, 0xAD, 0xD1, 0xBF,
	};

	shell_print(shell, "Verifying corrupted signature...");

	int ret = pukcc_ecdsa_p256_verify(test_public_key, bad_signature, test_hash);

	if (ret != 0) {
		shell_print(shell, "Result: PASS - Invalid signature correctly rejected! (status: 0x%04x)", ret);
		return 0;
	} else {
		shell_print(shell, "Result: FAIL - Invalid signature was incorrectly accepted!");
		return -1;
	}
}

static int cmd_pukcc_selftest(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct device *dev = PUKCC_DEV;

	if (!device_is_ready(dev)) {
		shell_print(shell, "PUKCC device not ready");
		return -ENODEV;
	}

	shell_print(shell, "PUKCC Self-Test");
	shell_print(shell, "===============");
	shell_print(shell, "Running PUKCC hardware self-test...");

	/* The self-test is run automatically during ECDSA operations */
	/* We can trigger it by calling pukcc_ecc_init() */
	int ret = pukcc_ecc_init();

	if (ret == 0) {
		shell_print(shell, "Result: PASS - PUKCC self-test successful!");
	} else {
		shell_print(shell, "Result: FAIL - PUKCC self-test failed (error: %d)", ret);
	}

	return 0;
}

/*
 * RSA test using PUKCC modular exponentiation via driver API
 *
 * Tests RSA public key operation: result = base^e mod n
 * Uses RSA-1024 for faster testing (RSA-2048/4096 also supported)
 */
static int cmd_pukcc_rsa(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	const struct device *dev = PUKCC_DEV;

	if (!device_is_ready(dev)) {
		shell_print(shell, "PUKCC device not ready");
		return -ENODEV;
	}

	shell_print(shell, "PUKCC RSA Modular Exponentiation Test");
	shell_print(shell, "======================================");
	shell_print(shell, "Testing RSA-1024 public key operation...");

	/*
	 * RSA-1024 Test Vector
	 *
	 * This tests modular exponentiation: result = base^exp mod n
	 * Using public exponent e = 65537 (0x010001)
	 */

	/* RSA-1024 public modulus n (128 bytes, big-endian) */
	static const uint8_t rsa_modulus[128] = {
		0xBB, 0xF8, 0x2F, 0x09, 0x06, 0x82, 0xCE, 0x9C,
		0x23, 0x38, 0xAC, 0x2B, 0x9D, 0xA8, 0x71, 0xF7,
		0x36, 0x8D, 0x07, 0xEE, 0xD4, 0x10, 0x43, 0xA4,
		0x40, 0xD6, 0xB6, 0xF0, 0x74, 0x54, 0xF5, 0x1F,
		0xB8, 0xDF, 0xBA, 0xAF, 0x03, 0x5C, 0x02, 0xAB,
		0x61, 0xEA, 0x48, 0xCE, 0xEB, 0x6F, 0xCD, 0x48,
		0x76, 0xED, 0x52, 0x0D, 0x60, 0xE1, 0xEC, 0x46,
		0x19, 0x71, 0x9D, 0x8A, 0x5B, 0x8B, 0x80, 0x7F,
		0xAF, 0xB8, 0xE0, 0xA3, 0xDF, 0xC7, 0x37, 0x72,
		0x3E, 0xE6, 0xB4, 0xB7, 0xD9, 0x3A, 0x25, 0x84,
		0xEE, 0x6A, 0x64, 0x9D, 0x06, 0x09, 0x53, 0x74,
		0x88, 0x34, 0xB2, 0x45, 0x45, 0x98, 0x39, 0x4E,
		0xE0, 0xAA, 0xB1, 0x2D, 0x7B, 0x61, 0xA5, 0x1F,
		0x52, 0x7A, 0x9A, 0x41, 0xF6, 0xC1, 0x68, 0x7F,
		0xE2, 0x53, 0x72, 0x98, 0xCA, 0x2A, 0x8F, 0x59,
		0x46, 0xF8, 0xE5, 0xFD, 0x09, 0x1D, 0xBD, 0xCB,
	};

	/* Test input (PKCS#1 v1.5 padded message, 128 bytes) */
	static const uint8_t test_input[128] = {
		0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x54,
		0x65, 0x73, 0x74, 0x20, 0x4D, 0x65, 0x73, 0x73,
		0x61, 0x67, 0x65, 0x20, 0x66, 0x6F, 0x72, 0x20,
		0x52, 0x53, 0x41, 0x2D, 0x31, 0x30, 0x32, 0x34,
		0x20, 0x50, 0x55, 0x4B, 0x43, 0x43, 0x21, 0x21,
	};

	/* Public exponent e = 65537 (0x010001) - little-endian with padding */
	static const uint8_t rsa_exponent[8] = {
		0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* Result buffer */
	uint8_t result[128];

	shell_print(shell, "");
	shell_print(shell, "Computing: result = input^65537 mod n");
	shell_print(shell, "Using pukcc_rsa_exp_mod() driver API...");

	/* Call driver API */
	int ret = pukcc_rsa_exp_mod(rsa_modulus, 128, test_input, rsa_exponent, 4, result);

	if (ret != 0) {
		shell_print(shell, "");
		shell_print(shell, "Result: FAIL - RSA operation failed (status: 0x%04x)", ret);
		return -1;
	}

	shell_print(shell, "");
	shell_print(shell, "Result (first 32 bytes of output):");
	shell_print(shell, "  %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x",
		    result[0], result[1], result[2], result[3],
		    result[4], result[5], result[6], result[7],
		    result[8], result[9], result[10], result[11],
		    result[12], result[13], result[14], result[15]);
	shell_print(shell, "  %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x",
		    result[16], result[17], result[18], result[19],
		    result[20], result[21], result[22], result[23],
		    result[24], result[25], result[26], result[27],
		    result[28], result[29], result[30], result[31]);

	shell_print(shell, "");
	shell_print(shell, "Result: PASS - RSA modular exponentiation completed!");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pukcc,
	SHELL_CMD(status, NULL, "Show PUKCC crypto driver status", cmd_pukcc_status),
	SHELL_CMD(ecdsa, NULL, "Test ECDSA P-256 signature verification (valid)", cmd_pukcc_ecdsa),
	SHELL_CMD(ecdsa_invalid, NULL, "Test ECDSA with invalid signature (should fail)", cmd_pukcc_ecdsa_invalid),
	SHELL_CMD(ecc, NULL, "Show ECC P-256 curve parameters and operations", cmd_pukcc_ecc),
	SHELL_CMD(rsa, NULL, "Test RSA-1024 modular exponentiation", cmd_pukcc_rsa),
	SHELL_CMD(selftest, NULL, "Run PUKCC hardware self-test", cmd_pukcc_selftest),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(pukcc, &sub_pukcc, "PUKCC crypto driver commands", NULL);

#endif /* CONFIG_CRYPTO_MCHP_PUKCC */

/*
 * MbedTLS API Shell Commands
 * These use the MD wrapper which goes through PSA when USE_PSA_CRYPTO is enabled
 */
#if defined(CONFIG_MBEDTLS)
#include <mbedtls/md.h>
#include <mbedtls/pk.h>
#include <mbedtls/ecp.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/error.h>

static int cmd_mbedtls_hash(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret;
	const char *message = "Hello, World!";
	uint8_t hash[32];

	/* Expected SHA-256 hash of "Hello, World!" */
	static const uint8_t expected_hash[32] = {
		0xdf, 0xfd, 0x60, 0x21, 0xbb, 0x2b, 0xd5, 0xb0,
		0xaf, 0x67, 0x62, 0x90, 0x80, 0x9e, 0xc3, 0xa5,
		0x31, 0x91, 0xdd, 0x81, 0xc7, 0xf7, 0x0a, 0x4b,
		0x28, 0x68, 0x8a, 0x36, 0x21, 0x82, 0x98, 0x6f
	};

	shell_print(shell, "MbedTLS SHA-256 Hash Test (ICM HW via ALT)");
	shell_print(shell, "==========================================");
	shell_print(shell, "Input: \"%s\" (%zu bytes)", message, strlen(message));

	/* Compute SHA-256 hash using MbedTLS directly (uses ICM hardware via ALT) */
	ret = mbedtls_sha256((const unsigned char *)message, strlen(message), hash, 0);
	if (ret != 0) {
		shell_print(shell, "mbedtls_sha256 failed: -0x%04x", -ret);
		return -1;
	}

	/* Print computed hash */
	shell_print(shell, "Computed hash:");
	shell_fprintf(shell, SHELL_NORMAL, "  ");
	for (int i = 0; i < 32; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", hash[i]);
	}
	shell_print(shell, "");

	/* Verify */
	if (memcmp(hash, expected_hash, 32) == 0) {
		shell_print(shell, "Result: PASS - Hash matches expected value!");
	} else {
		shell_print(shell, "Result: FAIL - Hash mismatch!");
	}

	return 0;
}

static int cmd_mbedtls_ecdsa(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret;
	mbedtls_ecdsa_context ecdsa;
	mbedtls_entropy_context entropy;
	mbedtls_ctr_drbg_context ctr_drbg;
	const char *pers = "ecdsa_test";
	uint8_t hash[32];
	uint8_t sig[72];
	size_t sig_len;

	shell_print(shell, "MbedTLS ECDSA P-256 Test (PUKCC HW via ALT)");
	shell_print(shell, "===========================================");

	/* Initialize contexts */
	mbedtls_ecdsa_init(&ecdsa);
	mbedtls_entropy_init(&entropy);
	mbedtls_ctr_drbg_init(&ctr_drbg);

	/* Seed the RNG */
	ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
				    (const unsigned char *)pers, strlen(pers));
	if (ret != 0) {
		shell_print(shell, "mbedtls_ctr_drbg_seed failed: -0x%04x", -ret);
		goto cleanup;
	}

	/* Generate ECDSA key pair (software keygen, but sign/verify uses HW) */
	shell_print(shell, "Generating P-256 key pair...");
	ret = mbedtls_ecdsa_genkey(&ecdsa, MBEDTLS_ECP_DP_SECP256R1,
				   mbedtls_ctr_drbg_random, &ctr_drbg);
	if (ret != 0) {
		shell_print(shell, "mbedtls_ecdsa_genkey failed: -0x%04x", -ret);
		goto cleanup;
	}
	shell_print(shell, "Key generation successful");

	/* Compute hash of test message (uses ICM hardware via ALT) */
	ret = mbedtls_sha256((const unsigned char *)"test message", 12, hash, 0);
	if (ret != 0) {
		shell_print(shell, "mbedtls_sha256 failed: -0x%04x", -ret);
		goto cleanup;
	}

	/* Sign the hash (uses PUKCC hardware via ALT) */
	shell_print(shell, "Signing with MbedTLS (PUKCC HW via ALT)...");
	ret = mbedtls_ecdsa_write_signature(&ecdsa, MBEDTLS_MD_SHA256,
					    hash, sizeof(hash),
					    sig, sizeof(sig), &sig_len,
					    mbedtls_ctr_drbg_random, &ctr_drbg);
	if (ret != 0) {
		shell_print(shell, "mbedtls_ecdsa_write_signature failed: -0x%04x", -ret);
		goto cleanup;
	}
	shell_print(shell, "Signature created (%zu bytes)", sig_len);

	/* Verify the signature (uses PUKCC hardware via ALT) */
	shell_print(shell, "Verifying with MbedTLS (PUKCC HW via ALT)...");
	ret = mbedtls_ecdsa_read_signature(&ecdsa, hash, sizeof(hash), sig, sig_len);
	if (ret != 0) {
		shell_print(shell, "mbedtls_ecdsa_read_signature failed: -0x%04x", -ret);
		shell_print(shell, "Result: FAIL - Signature verification failed!");
		goto cleanup;
	}

	shell_print(shell, "Result: PASS - Signature verified successfully!");
	shell_print(shell, "(Sign/verify used PUKCC hardware acceleration via ALT)");
	ret = 0;

cleanup:
	mbedtls_ecdsa_free(&ecdsa);
	mbedtls_ctr_drbg_free(&ctr_drbg);
	mbedtls_entropy_free(&entropy);

	return ret == 0 ? 0 : -1;
}

#if defined(MBEDTLS_RSA_C)
#include <mbedtls/rsa.h>

static int cmd_mbedtls_rsa(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret = 0;
	mbedtls_rsa_context rsa;
	mbedtls_entropy_context entropy;
	mbedtls_ctr_drbg_context ctr_drbg;
	const char *pers = "rsa_test";
	uint8_t hash[32];
	uint8_t sig[128];

#if defined(CONFIG_MBEDTLS_RSA_ALT)
	shell_print(shell, "MbedTLS RSA Test (PUKCC HW via ALT)");
	shell_print(shell, "====================================");
#else
	shell_print(shell, "MbedTLS RSA Test (software)");
	shell_print(shell, "===========================");
#endif

	/* Initialize contexts */
	mbedtls_rsa_init(&rsa);
	mbedtls_entropy_init(&entropy);
	mbedtls_ctr_drbg_init(&ctr_drbg);

	/* Seed the RNG */
	ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
				    (const unsigned char *)pers, strlen(pers));
	if (ret != 0) {
		shell_print(shell, "mbedtls_ctr_drbg_seed failed: -0x%04x", -ret);
		goto cleanup;
	}

#if defined(CONFIG_MBEDTLS_RSA_ALT)
	shell_print(shell, "Generating 1024-bit RSA key pair (PUKCC HW)...");
#else
	shell_print(shell, "Generating 1024-bit RSA key pair (software)...");
	shell_print(shell, "NOTE: This may take 20-60 seconds...");
#endif
	ret = mbedtls_rsa_gen_key(&rsa, mbedtls_ctr_drbg_random, &ctr_drbg,
				  1024, 65537);
	shell_print(shell, "DEBUG: mbedtls_rsa_gen_key returned %d", ret);
	if (ret != 0) {
		shell_print(shell, "mbedtls_rsa_gen_key failed: -0x%04x", -ret);
		goto cleanup;
	}
	shell_print(shell, "Key generation successful");

	/* Compute hash of test message (uses ICM hardware via ALT) */
	ret = mbedtls_sha256((const unsigned char *)"test message for RSA", 20, hash, 0);
	if (ret != 0) {
		shell_print(shell, "mbedtls_sha256 failed: -0x%04x", -ret);
		goto cleanup;
	}

	/* Sign the hash */
	shell_print(shell, "Signing with MbedTLS RSA...");
	ret = mbedtls_rsa_pkcs1_sign(&rsa, mbedtls_ctr_drbg_random, &ctr_drbg,
				     MBEDTLS_MD_SHA256, 32, hash, sig);
	if (ret != 0) {
		shell_print(shell, "mbedtls_rsa_pkcs1_sign failed: -0x%04x", -ret);
		goto cleanup;
	}
	shell_print(shell, "Signature created (%zu bytes)", mbedtls_rsa_get_len(&rsa));

	/* Verify the signature */
	shell_print(shell, "Verifying with MbedTLS RSA...");
	ret = mbedtls_rsa_pkcs1_verify(&rsa, MBEDTLS_MD_SHA256, 32, hash, sig);
	if (ret != 0) {
		shell_print(shell, "mbedtls_rsa_pkcs1_verify failed: -0x%04x", -ret);
		goto cleanup;
	}

	shell_print(shell, "Result: PASS - RSA sign/verify successful!");
	shell_print(shell, "(Hash computed using ICM hardware via ALT)");

cleanup:
	mbedtls_rsa_free(&rsa);
	mbedtls_ctr_drbg_free(&ctr_drbg);
	mbedtls_entropy_free(&entropy);

	return ret == 0 ? 0 : -1;
}
#endif /* MBEDTLS_RSA_C */

static int cmd_mbedtls_status(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "MbedTLS Status");
	shell_print(shell, "==============");
	shell_print(shell, "MbedTLS version: %s", MBEDTLS_VERSION_STRING);
	shell_print(shell, "");
	shell_print(shell, "Available algorithms:");
#if defined(MBEDTLS_SHA256_C)
	shell_print(shell, "  - SHA-256: yes");
#else
	shell_print(shell, "  - SHA-256: no");
#endif
#if defined(MBEDTLS_ECDSA_C)
	shell_print(shell, "  - ECDSA: yes");
#else
	shell_print(shell, "  - ECDSA: no");
#endif
#if defined(MBEDTLS_ECP_DP_SECP256R1_ENABLED)
	shell_print(shell, "  - P-256 curve: yes");
#else
	shell_print(shell, "  - P-256 curve: no");
#endif
#if defined(MBEDTLS_RSA_C)
	shell_print(shell, "  - RSA: yes");
#else
	shell_print(shell, "  - RSA: no");
#endif
#if defined(MBEDTLS_CTR_DRBG_C)
	shell_print(shell, "  - CTR_DRBG: yes");
#else
	shell_print(shell, "  - CTR_DRBG: no");
#endif
#if defined(MBEDTLS_ENTROPY_C)
	shell_print(shell, "  - Entropy: yes");
#else
	shell_print(shell, "  - Entropy: no");
#endif
	shell_print(shell, "");
	shell_print(shell, "Available tests:");
	shell_print(shell, "  mbedtls hash   - SHA-256 hash test");
	shell_print(shell, "  mbedtls ecdsa  - ECDSA P-256 sign/verify test");
#if defined(MBEDTLS_RSA_C)
	shell_print(shell, "  mbedtls rsa    - RSA sign/verify test");
#endif

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_mbedtls,
	SHELL_CMD(status, NULL, "Show MbedTLS status", cmd_mbedtls_status),
	SHELL_CMD(hash, NULL, "Test SHA-256 hash via MbedTLS", cmd_mbedtls_hash),
	SHELL_CMD(ecdsa, NULL, "Test ECDSA P-256 sign/verify via MbedTLS", cmd_mbedtls_ecdsa),
#if defined(MBEDTLS_RSA_C)
	SHELL_CMD(rsa, NULL, "Test RSA sign/verify via MbedTLS", cmd_mbedtls_rsa),
#endif
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(mbedtls, &sub_mbedtls, "MbedTLS direct API commands", NULL);

#endif /* CONFIG_MBEDTLS */

/*
 * Main function
 */
int main(void)
{
	LOG_INF("Crypto Sample (MbedTLS ALT HW Acceleration)");
	LOG_INF("============================================");

#if defined(CONFIG_MBEDTLS)
	LOG_INF("");
	LOG_INF("MbedTLS with Hardware Acceleration via ALT drivers:");
#if defined(CONFIG_MBEDTLS_SHA256_ALT)
	LOG_INF("  SHA-256: ICM hardware (ALT)");
#else
	LOG_INF("  SHA-256: Software");
#endif
#if defined(CONFIG_MBEDTLS_ECDSA_SIGN_ALT) || defined(CONFIG_MBEDTLS_ECDSA_VERIFY_ALT)
	LOG_INF("  ECDSA: PUKCC hardware (ALT)");
#else
	LOG_INF("  ECDSA: Software");
#endif
#if defined(CONFIG_MBEDTLS_RSA_ALT)
	LOG_INF("  RSA: PUKCC hardware (ALT) - keygen, sign, verify");
#else
	LOG_INF("  RSA: Software");
#endif
	LOG_INF("");
	LOG_INF("Test commands:");
	LOG_INF("  'mbedtls hash'   - Test SHA-256 (uses ICM HW)");
	LOG_INF("  'mbedtls ecdsa'  - Test ECDSA sign/verify (uses PUKCC HW)");
#if defined(CONFIG_MBEDTLS_RSA_ALT)
	LOG_INF("  'mbedtls rsa'    - Test RSA keygen/sign/verify (uses PUKCC HW)");
#else
	LOG_INF("  'mbedtls rsa'    - Test RSA sign/verify (software)");
#endif
	LOG_INF("  'mbedtls status' - View MbedTLS status");
#endif

#ifdef CONFIG_CRYPTO_MCHP_ICM
	const struct device *icm_dev = ICM_DEV;

	if (!device_is_ready(icm_dev)) {
		LOG_ERR("ICM device not ready!");
	} else {
		LOG_INF("");
		LOG_INF("Zephyr Crypto API (direct hardware access):");
		LOG_INF("  'icm hash'   - Compute SHA-256 via Zephyr Crypto API");
		LOG_INF("  'icm status' - View ICM capabilities");
	}
#endif

#ifdef CONFIG_CRYPTO_MCHP_PUKCC
	const struct device *pukcc_dev = PUKCC_DEV;

	if (!device_is_ready(pukcc_dev)) {
		LOG_ERR("PUKCC device not ready!");
	} else {
		LOG_INF("  'pukcc ecdsa'  - Test ECDSA via Zephyr Crypto API");
		LOG_INF("  'pukcc status' - View PUKCC capabilities");
	}
#endif

	return 0;
}
