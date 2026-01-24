.. zephyr:code-sample:: icm_hash
   :name: ICM SHA-256 Hash and PUKCC ECDSA
   :relevant-api: crypto

   Compute SHA-256 hashes using ICM and verify ECDSA signatures using PUKCC.

Overview
********

This sample demonstrates how to use the cryptographic hardware accelerators
on Microchip PIC32CX SG and SAM E54 devices:

**ICM (Integrity Check Monitor):**

- SHA-1, SHA-224, and SHA-256 hash computation
- Memory integrity monitoring (tamper detection)
- DMA-based data processing for efficiency

**PUKCC (Public Key Cryptographic Controller):**

- ECDSA P-256 signature verification
- RSA signature operations
- Modular arithmetic acceleration
- Hardware random number generation support

Requirements
************

This sample requires a board with ICM and PUKCC peripherals, such as:

- PIC32CX SG61 Curiosity Ultra (pic32cx_sg61_cult)
- PIC32CX SG41 Curiosity Ultra (pic32cx_sg41_cult)
- SAM E54 Xplained Pro (sam_e54_xpro)

The ICM and PUKCC nodes must be enabled in the device tree::

    &icm {
        status = "okay";
    };

    &pukcc {
        status = "okay";
    };

Building and Running
********************

Build the sample for your board:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/crypto/icm_hash
   :board: pic32cx_sg61_cult
   :goals: build flash
   :compact:

Sample Output
*************

After flashing, connect to the serial console and use the shell commands:

**ICM Hash Test:**

.. code-block:: console

   uart:~$ icm status
   ICM Crypto Driver Status:
     Device: crypto@42002c00
     Ready: yes
     Capabilities: 0x00000007
       - RAW_KEY: yes
       - SEPARATE_IO_BUFS: yes
       - SYNC_OPS: yes

   uart:~$ icm hash
   ICM SHA-256 Hash Test (using Zephyr Crypto API)
   Input: "Hello, World!" (13 bytes, padded to 64)
   Computed hash:
     dffd6021bb2bd5b0af676290809ec3a53191dd81c7f70a4b28688a362182986f
   Expected hash:
     dffd6021bb2bd5b0af676290809ec3a53191dd81c7f70a4b28688a362182986f
   Result: PASS - Hash matches expected value!

**PUKCC ECDSA Test:**

.. code-block:: console

   uart:~$ pukcc status
   PUKCC Crypto Driver Status:
     Device: crypto@42003000
     Ready: yes
     Capabilities: 0x00000007
     Supported operations:
       - ECDSA P-256 signature verification
       - RSA signature verification

   uart:~$ pukcc ecdsa
   PUKCC ECDSA P-256 Verification Test
   ===================================
   Testing with NIST-style test vectors...
   Verifying signature...
   Result: PASS - Signature verified successfully!

Shell Commands
**************

The sample provides the following shell commands:

**ICM Commands (Zephyr Crypto API):**

- ``icm status`` - Display ICM crypto driver status
- ``icm hash`` - Compute SHA-256 hash of test data ("Hello, World!")

**PUKCC Commands (Zephyr Crypto API):**

- ``pukcc status`` - Display PUKCC crypto driver status
- ``pukcc ecdsa`` - Test ECDSA P-256 signature verification
- ``pukcc selftest`` - Run PUKCC hardware self-test

**PSA Crypto Commands (MbedTLS PSA API with hardware acceleration):**

- ``psa status`` - Display PSA Crypto driver status
- ``psa hash`` - Compute SHA-256 hash via PSA Crypto API
- ``psa ecdsa`` - Test ECDSA P-256 verification via PSA Crypto API

PSA Crypto Integration
**********************

This sample also demonstrates the PSA Crypto API integration with hardware
acceleration. When ``CONFIG_MBEDTLS_PSA_MCHP_ICM_DRIVER`` and
``CONFIG_MBEDTLS_PSA_MCHP_PUKCC_DRIVER`` are enabled, the MbedTLS PSA Crypto
implementation will use the ICM and PUKCC hardware for:

- **SHA-256/SHA-224 hashing** (ICM accelerated)
- **ECDSA P-256 sign/verify** (PUKCC accelerated)
- **RSA encryption/decryption** (PUKCC accelerated)

Technical Details
*****************

ICM Clock Configuration
=======================

The ICM requires two clocks to be enabled:

1. **AHB clock** (MCLK.AHBMASK bit 19) - For memory interface
2. **APBC clock** (MCLK.APBCMASK bit 11) - For register access

PUKCC Clock Configuration
=========================

The PUKCC requires two clocks to be enabled:

1. **AHB clock** (MCLK.AHBMASK bit 20) - For memory interface
2. **APBC clock** (MCLK.APBCMASK bit 12) - For register access

PUKCC also uses dedicated Crypto RAM at address 0x02011000 for
cryptographic operations.

SHA-256 Padding
===============

SHA-256 requires input data to be padded to a multiple of 64 bytes (512 bits).
The padding format is:

1. Original message
2. 0x80 byte
3. Zero bytes until 8 bytes before the next 64-byte boundary
4. 64-bit big-endian bit length of the original message

For "Hello, World!" (13 bytes = 104 bits), the padding is:

- Bytes 0-12: "Hello, World!"
- Byte 13: 0x80
- Bytes 14-55: zeros (42 bytes)
- Bytes 56-63: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x68

ECDSA P-256 Parameters
======================

For ECDSA P-256 signature verification, the following parameters are needed:

- **Public Key**: 64 bytes (X coordinate || Y coordinate, big-endian)
- **Signature**: 64 bytes (R value || S value, big-endian)
- **Hash**: 32 bytes (SHA-256 hash of the message)

The PUKCC uses the NIST P-256 (secp256r1) curve parameters:

- Prime: 2^256 - 2^224 + 2^192 + 2^96 - 1
- Order: 0xFFFFFFFF00000000FFFFFFFFFFFFFFFFBCE6FAADA7179E84F3B9CAC2FC632551

ICM Descriptor Format
=====================

Each ICM region descriptor is 16 bytes:

- **RADDR** (4 bytes): Region start address (64-byte aligned)
- **RCFG** (4 bytes): Region configuration (algorithm, EOM flag)
- **RCTRL** (4 bytes): Region control (TRSIZE = blocks - 1)
- **RNEXT** (4 bytes): Next descriptor address (0 for last)

The descriptor array must be 64-byte aligned.
The hash output buffer must be 128-byte aligned.

References
**********

- PIC32CX SG61 Datasheet, Section: ICM - Integrity Check Monitor
- PIC32CX SG61 Datasheet, Section: PUKCC - Public Key Cryptographic Controller
- SAM D5x/E5x Family Data Sheet, Section: ICM - Integrity Check Monitor
- NIST FIPS 186-4: Digital Signature Standard (DSS)
