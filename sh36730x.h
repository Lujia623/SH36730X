#ifndef SH36730X_H
#define SH36730X_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "sh36730x_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SH36730X status codes
 */
typedef enum {
    SH36730X_OK = 0,                        /**< Operation completed successfully. */
    SH36730X_ERROR_PARAM = -1,              /**< Invalid argument or uninitialized device. */
    SH36730X_ERROR_NOT_INITIALIZED = -2,    /**< Device not initialized. */
    SH36730X_ERROR_COMMUNICATION = -3,      /**< Communication error. */
    SH36730X_ERROR_TIMEOUT = -4,            /**< Operation timed out. */
    SH36730X_ERROR_CRC = -5,                /**< CRC check failed. */
    SH36730X_ERROR_DEVICE = -6,             /**< Device error. */
    SH36730X_ERROR = -7                     /**< Generic error. */
} sh36730x_status_t;

/**
 * @brief SH36730X chip model identifiers.
 */
typedef enum {
    SH36730X_CHIP_NONE = 0,
    SH36730X_CHIP_SH367303 = 0x03,      /* SH367303 chip model, cell for 3~5 */
    SH36730X_CHIP_SH367305 = 0x05,      /* SH367305 chip model, cell for 6~8 */
    SH36730X_CHIP_SH367309 = 0x09       /* SH367309 chip model, cell for 6~10 */
} sh36730x_chip_model_t;

/**
 * @brief Selection of sampling mode for VADC (VADC_C: Bit 0)
 */
typedef enum {
    SH36730X_VADC_MODE_VOLTAGE_ONLY = SH36730X_VADC_SAMPLE_VOLT_ONLY,   /* 0: Only measure the Cell voltage */
    SH36730X_VADC_MODE_VOLT_AND_TEMP = SH36730X_VADC_SAMPLE_VOLT_TEMP   /* 1: Measure both the Cell voltage and temperature */
} sh36730x_vadc_mode_t;

/**
 * @brief VADC scanning cycle (SCONF3: Bit 2:0)
 */
typedef enum {
    SH36730X_VADC_SCAN_50MS  = SH36730X_SCAN_PERIOD_50MS,       /* 000: 50 ms */
    SH36730X_VADC_SCAN_100MS = SH36730X_SCAN_PERIOD_100MS,      /* 001: 100 ms */
    SH36730X_VADC_SCAN_200MS = SH36730X_SCAN_PERIOD_200MS,      /* 010: 200 ms */
    SH36730X_VADC_SCAN_500MS = SH36730X_SCAN_PERIOD_500MS,      /* 011: 500 ms */
    SH36730X_VADC_SCAN_1S    = SH36730X_SCAN_PERIOD_1S,         /* 100: 1 s */
    SH36730X_VADC_SCAN_2S    = SH36730X_SCAN_PERIOD_2S,         /* 101: 2 s */
    SH36730X_VADC_SCAN_4S    = SH36730X_SCAN_PERIOD_4S,         /* 110: 4 s */
    SH36730X_VADC_SCAN_8S    = SH36730X_SCAN_PERIOD_8S          /* 111: 8 s */
} sh36730x_vadc_scan_period_t;

/**
 * @brief VADC module configuration structure
 */
typedef struct {
    bool enable;                             /**< VADC enable switch (VADC_EN) */
    sh36730x_vadc_mode_t mode;               /**< Sampling mode selection (VADC_C) */
    sh36730x_vadc_scan_period_t scan_period; /**< Scanning cycle selection (SCAN_C) */
} sh36730x_vadc_config_t;

/**
 * @brief Single cell channel
 */
typedef enum {
    SH36730X_CELL_1 = 0,            /**< Cell 1 */
    SH36730X_CELL_2,                /**< Cell 2 */
    SH36730X_CELL_3,                /**< Cell 3 */
    SH36730X_CELL_4,                /**< Cell 4 */
    SH36730X_CELL_5,                /**< Cell 5 */
    SH36730X_CELL_6,                /**< Cell 6 */
    SH36730X_CELL_7,                /**< Cell 7 */
    SH36730X_CELL_8,                /**< Cell 8 */
    SH36730X_CELL_9,                /**< Cell 9 */
    SH36730X_CELL_10                /**< Cell 10 */
} sh36730x_cell_channel_t;

/**
 * @brief Temperature sensor channel (TS1, TS2)
 */
typedef enum {
    SH36730X_TS1,                    /**< external Temperature sensor 1 */
    SH36730X_TS2                     /**< external Temperature sensor 2 */
} sh36730x_temp_ts_channel_t;

/**
 * @brief Internal temperature sensor channel.
 */
typedef enum {
    SH36730X_INTERNAL_TEMP1,        /**< Internal Temperature sensor 1 */
    SH36730X_INTERNAL_TEMP2         /**< Internal Temperature sensor 2 */
} sh36730x_temp_internal_channel_t;

/**
 * @brief Current sense ADC mode.
 */
typedef enum {
    SH36730X_CADC_M_SINGLE = SH36730X_CADC_MODE_SINGLE,             /**< Single mode */
    SH36730X_CADC_M_CONTINUOUS = SH36730X_CADC_MODE_CONTINUOUS,     /**< Continuous mode */
} sh36730x_cadc_mode_t;

/**
 * @brief Current sense ADC full-scale range.
 */
typedef enum {
    SH36730X_CADC_RSNS_50MV =   SH36730X_RSNS_RANGE_50MV,   /**< Sense resistor full-scale range 50 mV */
    SH36730X_CADC_RSNS_100MV = SH36730X_RSNS_RANGE_100MV,   /**< Sense resistor full-scale range 100 mV */
    SH36730X_CADC_RSNS_200MV = SH36730X_RSNS_RANGE_200MV,   /**< Sense resistor full-scale range 200 mV */
    SH36730X_CADC_RSNS_400MV = SH36730X_RSNS_RANGE_400MV    /**< Sense resistor full-scale range 400 mV */
} sh36730x_cadc_rsns_t;

/**
 * @brief Current sense ADC bit resolution.
 */
typedef enum {
    SH36730X_CBTI_C_10BIT = SH36730X_CADC_BITS_10,    /**< Current sense ADC 10-bit resolution */
    SH36730X_CBTI_C_13BIT = SH36730X_CADC_BITS_13,    /**< Current sense ADC 13-bit resolution */
} sh36730x_cbti_c_t;

/**
 * @brief Current sense ADC configuration.
 */
typedef struct {
    bool enable;
    sh36730x_cadc_mode_t cadc_mode;
    sh36730x_cadc_rsns_t cadc_rsns;
    sh36730x_cbti_c_t cbti_c;
} sh36730x_cadc_config_t;

/**
 * @brief Over-voltage delay options.
 */
typedef enum {
    SH36730X_OV_DELAY_1CYCLE = SH36730X_OVT_DELAY_1_CYCLE,                   /**< Over-voltage delay 1 cycle */
    SH36730X_OV_DELAY_2CYCLE = SH36730X_OVT_DELAY_2_CYCLES,                  /**< Over-voltage delay 2 cycles */
    SH36730X_OV_DELAY_4CYCLE = SH36730X_OVT_DELAY_4_CYCLES,                  /**< Over-voltage delay 4 cycles */
    SH36730X_OV_DELAY_8CYCLE = SH36730X_OVT_DELAY_8_CYCLES,                  /**< Over-voltage delay 8 cycles */
    SH36730X_OV_DELAY_16CYCLE = SH36730X_OVT_DELAY_16_CYCLES,                /**< Over-voltage delay 16 cycles */
    SH36730X_OV_DELAY_32CYCLE = SH36730X_OVT_DELAY_32_CYCLES,                /**< Over-voltage delay 32 cycles */
    SH36730X_OV_DELAY_64CYCLE = SH36730X_OVT_DELAY_64_CYCLES,                /**< Over-voltage delay 64 cycles */
    SH36730X_OV_DELAY_128CYCLE = SH36730X_OVT_DELAY_128_CYCLES               /**< Over-voltage delay 128 cycles */
} sh36730x_ov_delay_t;

/**
 * @brief Reset and power-fail options.
 */
typedef enum {
    SH36730X_SCONF2_RESET = 0x00,       /**< reset external MCU function */
    SH36730X_SCONF2_PF    = 0x01        /**< secondary protection (Secondary Protection) function */
} sh36730x_reset_pf_t;

/**
 * @brief Short-circuit voltage options.
 * @note These options correspond to the voltage thresholds for detecting short-circuit conditions.
 */
typedef enum {
    SH36730X_SCV_100MV = SH36730X_SCV_VOLTAGE_100MV,    /**< short-circuit voltage 100 mV */
    SH36730X_SCV_200MV = SH36730X_SCV_VOLTAGE_200MV,    /**< short-circuit voltage 200 mV */
    SH36730X_SCV_300MV = SH36730X_SCV_VOLTAGE_300MV,    /**< short-circuit voltage 300 mV */
    SH36730X_SCV_400MV = SH36730X_SCV_VOLTAGE_400MV     /**< short-circuit voltage 400 mV */
} sh36730x_scv_t;

/**
 * @brief Short-circuit delay options.
 */
typedef enum {
    SH36730X_SCT_50US = SH36730X_SCT_DELAY_50US,        /**< short-circuit delay 50 us */
    SH36730X_SCT_100US = SH36730X_SCT_DELAY_100US,      /**< short-circuit delay 100 us */
    SH36730X_SCT_300US = SH36730X_SCT_DELAY_300US,      /**< short-circuit delay 300 us */
    SH36730X_SCT_500US = SH36730X_SCT_DELAY_500US       /**< short-circuit delay 500 us */
} sh36730x_sct_t;

typedef enum {
    SH36730X_BALANCE_DISABLE = 0,       /* close all balancing */
    SH36730X_BALANCE_ODD,               /* enable balancing for odd channels only (only affects odd bits in the provided mask) */
    SH36730X_BALANCE_EVEN,              /* enable balancing for even channels only (only affects even bits in the provided mask) */
    SH36730X_BALANCE_FORCE_RAW          /* force control according to the raw bitmap (not recommended, the application layer is responsible for safety) */
} sh36730x_balance_mode_t;

/**
 * @brief Performs one atomic I2C write transaction.
 *
 * The callback must generate:
 * START + write address + data + STOP.
 *
 * @return 0 on success; a platform-defined negative value on failure.
 */
typedef int (*sh36730x_i2c_write_fn)(
    void *context,
    uint8_t address_7bit,
    const uint8_t *data,
    size_t size);

/**
 * @brief Performs one atomic I2C write-then-read transaction.
 *
 * The callback must generate:
 * START + write address + tx data + repeated START +
 * read address + rx data + STOP.
 *
 * @return 0 on success; a platform-defined negative value on failure.
 */
typedef int (*sh36730x_i2c_write_read_fn)(
    void *context,
    uint8_t address_7bit,
    const uint8_t *tx_data,
    size_t tx_size,
    uint8_t *rx_data,
    size_t rx_size);

/**
 * @brief Provides an optional blocking delay.
 */
typedef void (*sh36730x_delay_ms_fn)(
    void *context,
    uint32_t milliseconds);

/**
 * @brief Platform interface supplied by the application.
 */
typedef struct {
    void *context;
    sh36730x_i2c_write_fn write;
    sh36730x_i2c_write_read_fn write_read;
    sh36730x_delay_ms_fn delay_ms;
} sh36730x_interface_t;

/**
 * @brief SH36730X device instance.
 *
 * Allocate one instance for every physical SH36730X device.
 * Treat all fields as private after initialization.
 */
typedef struct {
    sh36730x_interface_t interface;
    sh36730x_chip_model_t chip_model;
    uint8_t address_7bit;
    bool initialized;
} sh36730x_t;

/**
 * @brief Initializes the SH36730X device.
 *
 * @param device Pointer to the device instance.
 * @param config Pointer to the device configuration.
 * @return Status of the initialization.
 */
sh36730x_status_t sh36730x_init(sh36730x_t *device);

/**
 * @brief Reads a register from the SH36730X device(Only double-byte operations are supported).
 * @param device Pointer to the device instance.
 * @param register_address The address of the register to read.
 * @param value Pointer to a variable where the read value will be stored.
 * @return Status of the read operation.
 */
sh36730x_status_t sh36730x_read_register(
    sh36730x_t *device,
    uint8_t register_address,
    uint16_t *value);

/**
 * @brief Writes a value to a register of the SH36730X device(Only single-byte operations are supported).
 * @param device Pointer to the device instance.
 * @param register_address The address of the register to write.
 * @param value The value to write to the register.
 * @return Status of the write operation.
 */
sh36730x_status_t sh36730x_write_register(
    sh36730x_t *device,
    uint8_t register_address,
    uint8_t value);

/**
 * @brief Updates specific bits of a register in the SH36730X device.
 * @param device Pointer to the device instance.
 * @param register_address The address of the register to update.
 * @param mask Mask indicating which bits to update, if the low or high mask is set to 0x00, those bits will not be modified.
 * @param value The new value for the specified bits.
 * @return Status of the update operation.
 */
sh36730x_status_t sh36730x_update_registers(
    sh36730x_t *device,
    uint8_t register_address,
    uint16_t mask,
    uint16_t value);

/**
 * @brief Configures the VADC (Voltage Analog-to-Digital Converter) of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param config Pointer to the VADC configuration structure.
 * @return Status of the configuration operation.
 */
sh36730x_status_t sh36730x_vadc_config(
    sh36730x_t *device,
    const sh36730x_vadc_config_t *config);

/**
 * @brief Enables or disables the VADC (Voltage Analog-to-Digital Converter) of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param enable Set to true to enable the VADC, false to disable it.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_vadc_enable(
    sh36730x_t *device,
    bool enable);
/**
 * @brief Sets the scan period for the VADC (Voltage Analog-to-Digital Converter) of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param scan_period The desired scan period for the VADC.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_vadc_set_scan_period(
    sh36730x_t *device,
    sh36730x_vadc_scan_period_t scan_period);

/**
 * @brief Sets the mode for the VADC (Voltage Analog-to-Digital Converter) of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param mode The desired mode for the VADC.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_vadc_set_mode(
    sh36730x_t *device,
    sh36730x_vadc_mode_t mode);

/**
 * @brief Retrieves the voltage of a specific cell in the SH36730X device.
 * @param device Pointer to the device instance.
 * @param cell The cell channel to read the voltage from.
 * @param voltage_mv Pointer to store the retrieved voltage in millivolts.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_get_cell_voltage(
    sh36730x_t *device,
    sh36730x_cell_channel_t cell,
    uint16_t *voltage_mv);

/**
 * @brief Retrieves the voltages of a group of cells in the SH36730X device.
 * @param device Pointer to the device instance.
 * @param min_cell The minimum cell index in the group.
 * @param max_cell The maximum cell index in the group.
 * @param voltages_mv Pointer to store the retrieved voltages in millivolts.
 * @param voltages_mv_len The length of the voltages_mv array.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_get_cells_group_voltage(
    sh36730x_t *device,
    uint8_t min_cell, 
    uint8_t max_cell,
    uint16_t *voltages_mv,
    uint8_t voltages_mv_len);

/**
 * @brief Retrieves the temperature from a specific TS (external Temperature Sensor) channel in the SH36730X device.
 * @param device Pointer to the device instance.
 * @param ts The TS channel to read the temperature from.
 * @param kohm Pointer to store the retrieved temperature in ohm.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_get_temp_ts(
    sh36730x_t *device,
    sh36730x_temp_ts_channel_t ts,
    uint32_t *ohm);

/**
 * @brief Retrieves the temperature from the internal temperature sensor channel in the SH36730X device.
 * @param device Pointer to the device instance.
 * @param internal The internal temperature sensor channel to read the temperature from.
 * @param temperature_C Pointer to store the retrieved temperature in degrees Celsius.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_get_internal_temp(
    sh36730x_t *device,
    sh36730x_temp_internal_channel_t internal,
    float *temperature_C);

/**
 * @brief Configures the CADC (Current Analog-to-Digital Converter) of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param config Pointer to the CADC configuration structure.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_cadc_config(
    sh36730x_t *device,
    sh36730x_cadc_config_t *config);

/**
 * @brief Enables or disables the CADC (Current Analog-to-Digital Converter) of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param enable Boolean flag to enable (true) or disable (false) the CADC.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_cadc_enable(
    sh36730x_t *device,
    bool enable);

/**
 * @brief Sets the RSNS (Collection scope SCONF6[7:6]) configuration of the CADC (Current Analog-to-Digital Converter) in the SH36730X device.
 * @param device Pointer to the device instance.
 * @param cadc_rsns The RSNS configuration to set for the CADC.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_cadc_set_rsns(
    sh36730x_t *device,
    sh36730x_cadc_rsns_t cadc_rsns);

/**
 * @brief Sets the CBTC (SCONF3[5]) configuration of the CADC (Current Analog-to-Digital Converter) in the SH36730X device.
 * @param device Pointer to the device instance.
 * @param cbti_c The CBTC configuration to set for the CADC.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_cadc_set_cbti_c(
    sh36730x_t *device,
    sh36730x_cbti_c_t cbti_c);

/**
 * @brief Sets the mode of the CADC (Current Analog-to-Digital Converter) in the SH36730X device.
 * @param device Pointer to the device instance.
 * @param cadc_mode The mode to set for the CADC.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_cadc_set_mode(
    sh36730x_t *device,
    sh36730x_cadc_mode_t cadc_mode);

/**
 * @brief Retrieves the current measured by the CADC (Current Analog-to-Digital Converter) in the SH36730X device.
 * @param device Pointer to the device instance.
 * @param rsense_ohm The value of the sense resistor in ohms.
 * @param current Pointer to store the retrieved current value.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_get_current(
    sh36730x_t *device,
    float rsense_ohm,
    float *current);

/**
 * @brief Enables or disables the over-voltage protection (OV) of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param enable Boolean flag to enable (true) or disable (false) the OV.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_enable_ov(
    sh36730x_t *device,
    bool enable);

/**
 * @brief Clears the FLAG1 register of the SH36730X device.
 * @param device Pointer to the device instance.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_clear_flag1(
    sh36730x_t *device);

/**
 * @brief Sets the hardware over-voltage (OV) threshold voltage of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param voltage The OV threshold voltage to set.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_set_ov_voltage(
    sh36730x_t *device,
    float voltage);

/**
 * @brief Sets the hardware over-voltage (OV) delay of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param delay The OV delay to set.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_set_ov_delay(
    sh36730x_t *device,
    sh36730x_ov_delay_t delay);

/**
 * @brief Sets the reset power-fail (RESET/PF) option of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param option The RESET/PF option to set.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_set_resetpf(
    sh36730x_t *device,
    sh36730x_reset_pf_t option);

/**
 * @brief Enables or disables the discharge short-circuit protection (SC) of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param enable Boolean flag to enable (true) or disable (false) the SC.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_enable_sc(
    sh36730x_t *device,
    bool enable);

/**
 * @brief Sets the discharge short-circuit protection (SCV) threshold of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param scv The SCV threshold to set.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_set_scv(
    sh36730x_t *device,
    sh36730x_scv_t scv);

/**
 * @brief Sets the discharge short-circuit protection (SCT) threshold of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param sct The SCT threshold to set.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_set_sct(
    sh36730x_t *device,
    sh36730x_sct_t sct);

/**
 * @brief Enables or disables the Charger Detection module of the SH36730X device (SCONF1[0]: CHGR_EN).
 * @param device Pointer to the device instance.
 * @param enable True to enable charger detection, false to disable.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_enable_charger_detection(
    sh36730x_t *device,
    bool enable);

/**
 * @brief Enables or disables the Load Detection module of the SH36730X device (SCONF1[1]: LOAD_EN).
 * @param device Pointer to the device instance.
 * @param enable True to enable load detection, false to disable.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_enable_load_detection(
    sh36730x_t *device,
    bool enable);

/**
 * @brief Retrieves the Charger connection status from BSTATUS[0] (CHGR).
 * @param device Pointer to the device instance.
 * @param connected Pointer to store connection status (true: Charger connected, false: Disconnected).
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_get_charger_status(
    sh36730x_t *device,
    bool *connected);

/**
 * @brief Retrieves the Load connection status from BSTATUS[1] (LOAD).
 * @param device Pointer to the device instance.
 * @param connected Pointer to store connection status (true: Load connected, false: Disconnected).
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_get_load_status(
    sh36730x_t *device,
    bool *connected);




/**
 * @brief Sets the cell balancing configuration of the SH36730X device.
 * @param device Pointer to the device instance.
 * @param mask_10bit 10-bit mask indicating which cells to balance.
 * @param mode The balancing mode to set.
 * @return Status of the operation.
 */
sh36730x_status_t sh36730x_set_balancing(
    sh36730x_t *device,
    uint16_t mask_10bit,
    sh36730x_balance_mode_t mode);



#ifdef __cplusplus
}
#endif
#endif /* SH36730X_H */
