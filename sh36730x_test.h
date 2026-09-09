/**
 * @file    sh36730x_test.h
 * @brief   Comprehensive API Test Suite for Sinowealth SH36730X Battery Management AFE.
 * @note    Designed with hardware-safety guards, automatic rollback, and interactive CLI.
 * 
 * Features:
 *  - Hardware Safety First: Read-only prioritized, safe parameter validation,
 *    controlled short-pulse balancing with mandatory auto-off, full state rollback.
 *  - Comprehensive API Coverage: VADC, CADC, Voltages, Temperatures, Current, Protection thresholds.
 *  - Multi-Environment: Native RT-Thread MSH command support & standard standalone C API.
 *  - Automated Statistics: Execution time, PASS/FAIL/WARN tally, CRC error rates.
 */

#ifndef SH36730X_TEST_H
#define SH36730X_TEST_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "sh36730x.h"
#include "soft_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* Configuration Macros                                                       */
/* ========================================================================== */

#define SH36730X_TEST_DEFAULT_I2C_ADDR      SH36730X_TWI_SLAVE_ADDR_7BIT        /**< Default 7-bit I2C address for SH36730X */
#define SH36730X_TEST_DEFAULT_SDA_PIN       GPIO_Pin_12                         /**< Default SDA Pin (PB12) */
#define SH36730X_TEST_DEFAULT_SCL_PIN       GPIO_Pin_13                         /**< Default SCL Pin (PB13) */
#define SH36730X_TEST_DEFAULT_I2C_FREQ_HZ   (100000UL)                          /**< 100 kHz standard mode */
#define SH36730X_TEST_DEFAULT_RSENSE_OHM    (0.001f)                            /**< Default 1 mOhm sense resistor */

/**
 * @brief Test Result Status
 */
typedef enum {
    TEST_RESULT_PASS = 0,
    TEST_RESULT_FAIL,
    TEST_RESULT_WARN,
    TEST_RESULT_SKIP
} test_result_t;

/**
 * @brief Test Suite Summary Statistics
 */
typedef struct {
    uint32_t total_tests;
    uint32_t passed_tests;
    uint32_t failed_tests;
    uint32_t warned_tests;
    uint32_t skipped_tests;
} sh36730x_test_stats_t;

/**
 * @brief Test Context Container
 */
typedef struct {
    sh36730x_t device;              /**< SH36730X device handle */
    soft_i2c_t i2c_bus;             /**< Software I2C bus handle */
    sh36730x_test_stats_t stats;    /**< Current test statistics */
    float rsense_ohm;               /**< Current sense resistor value in Ohms */
    bool is_initialized;            /**< Initialization flag */
} sh36730x_test_context_t;

/* ========================================================================== */
/* Test Function Declarations                                                 */
/* ========================================================================== */

/**
 * @brief Initialize the hardware interface (GPIO/I2C) and the SH36730X test context.
 * @param ctx Pointer to the test context.
 * @param chip_model Target chip model (SH367303, SH367305, or SH367309).
 * @param i2c_addr_7bit 7-bit I2C slave address.
 * @return TEST_RESULT_PASS on success, TEST_RESULT_FAIL on error.
 */
test_result_t sh36730x_test_init_hardware(sh36730x_test_context_t *ctx,
                                          sh36730x_chip_model_t chip_model,
                                          uint8_t i2c_addr_7bit);

/**
 * @brief TC01: Interface, I2C Bus, and Device Initialization Test.
 * @param ctx Pointer to test context.
 */
test_result_t sh36730x_test_tc01_init_and_probe(sh36730x_test_context_t *ctx);

/**
 * @brief TC02: Register Read/Write, Bit-field Update & CRC Integrity Test with Rollback.
 * @param ctx Pointer to test context.
 */
test_result_t sh36730x_test_tc02_register_rw_and_crc(sh36730x_test_context_t *ctx);

/**
 * @brief TC03: Single Cell and Group Voltages Acquisition & Bounds Check.
 * @param ctx Pointer to test context.
 */
test_result_t sh36730x_test_tc03_read_voltages(sh36730x_test_context_t *ctx);

/**
 * @brief TC04: External NTC and Internal Temperature Sensors Acquisition & Range Check.
 * @param ctx Pointer to test context.
 */
test_result_t sh36730x_test_tc04_read_temperatures(sh36730x_test_context_t *ctx);

/**
 * @brief TC05: VADC Sampling Mode and Period Configuration Test with Rollback.
 * @param ctx Pointer to test context.
 */
test_result_t sh36730x_test_tc05_vadc_config(sh36730x_test_context_t *ctx);

/**
 * @brief TC06: CADC Current Measurement & Range Configuration Test with Rollback.
 * @param ctx Pointer to test context.
 */
test_result_t sh36730x_test_tc06_cadc_and_current(sh36730x_test_context_t *ctx);

/**
 * @brief TC07: Hardware Protection Parameters (OV/SC/Delay) Safe Setting & Rollback Test.
 * @param ctx Pointer to test context.
 */
test_result_t sh36730x_test_tc07_protection_params(sh36730x_test_context_t *ctx);

/**
 * @brief TC08: Safe Hardware Cell Balancing Test (Short pulse, strictly disarmed upon exit).
 * @param ctx Pointer to test context.
 */
test_result_t sh36730x_test_tc08_safe_balancing(sh36730x_test_context_t *ctx);

/**
 * @brief TC09: Communication Stress & Long-term Stability Test.
 * @param ctx Pointer to test context.
 * @param iterations Number of consecutive read cycles.
 */
test_result_t sh36730x_test_tc09_stress_test(sh36730x_test_context_t *ctx, uint32_t iterations);

/**
 * @brief TC10: Dump all key diagnostic registers and real-time battery status.
 * @param ctx Pointer to test context.
 */
test_result_t sh36730x_test_dump_status(sh36730x_test_context_t *ctx);

/**
 * @brief Execute all test cases sequentially with hardware safety guarantees and output report.
 * @param ctx Pointer to test context.
 * @return TEST_RESULT_PASS if all pass, TEST_RESULT_FAIL if any critical failure occurred.
 */
test_result_t sh36730x_test_run_all(sh36730x_test_context_t *ctx);

/**
 * @brief Get global singleton test context pointer.
 */
sh36730x_test_context_t* sh36730x_test_get_default_context(void);

#ifdef __cplusplus
}
#endif

#endif /* SH36730X_TEST_H */
