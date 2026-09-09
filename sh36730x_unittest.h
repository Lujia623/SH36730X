/**
 * @file    sh36730x_unittest.h
 * @brief   Unit Testing Framework and Test Cases for Sinowealth SH36730X AFE Driver.
 * @note    Runs fully in-memory with a software Mock AFE device.
 *          - Zero hardware dependencies (safe, no risk of damage to battery/AFE).
 *          - Full API coverage with boundary and parameter checks.
 *          - Built-in Mock I2C & Fault Injection (CRC error, NACK, Timeout).
 */

#ifndef SH36730X_UNITTEST_H
#define SH36730X_UNITTEST_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "sh36730x.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* Unit Test Statistics                                                       */
/* ========================================================================== */

typedef struct {
    uint32_t total_suites;      /**< Total test suites executed */
    uint32_t passed_suites;     /**< Passed test suites */
    uint32_t failed_suites;     /**< Failed test suites */
    uint32_t total_asserts;     /**< Total assertions evaluated */
    uint32_t passed_asserts;    /**< Passed assertions */
    uint32_t failed_asserts;    /**< Failed assertions */
} sh36730x_ut_stats_t;

/* ========================================================================== */
/* Mock Device Context                                                        */
/* ========================================================================== */

#define SH36730X_MOCK_REG_SIZE 0x40

typedef struct {
    uint8_t regs[SH36730X_MOCK_REG_SIZE];   /**< Simulated 64-byte AFE register map */
    bool inject_crc_error;                  /**< Inject CRC error into read response */
    bool inject_comm_error;                 /**< Inject communication failure (e.g. NACK) */
    uint32_t write_count;                   /**< Total successful writes recorded */
    uint32_t read_count;                    /**< Total successful reads recorded */
    uint8_t last_written_reg;               /**< Last written register address */
    uint8_t last_written_val;               /**< Last written byte value */
} sh36730x_mock_afe_t;

/* ========================================================================== */
/* Public Test Runner APIs                                                    */
/* ========================================================================== */

/**
 * @brief Reset Mock AFE registers to default power-on reset (POR) state.
 */
void sh36730x_mock_reset(void);

/**
 * @brief Get pointer to the Mock AFE instance for inspection or test setup.
 */
sh36730x_mock_afe_t* sh36730x_mock_get_instance(void);

/**
 * @brief Initialize a test driver instance wired to the in-memory Mock AFE.
 * @param device Pointer to driver instance to initialize.
 * @param model Chip model (e.g. SH367309).
 */
void sh36730x_ut_setup_mock_device(sh36730x_t *device, sh36730x_chip_model_t model);

/**
 * @brief Run the complete SH36730X API Unit Test Suite.
 * @return true if all test suites and assertions passed, false otherwise.
 */
bool sh36730x_unittest_run_all(void);

#ifdef __cplusplus
}
#endif

#endif /* SH36730X_UNITTEST_H */
