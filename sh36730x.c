/**
 * @file sh36730x.c
 * @brief Implementation of SH36730X driver API functions.
 * @note    - SH367303: 3 to 5 series lithium battery protection & monitoring AFE
 *          - SH367305: 6 to 8 series lithium battery protection & monitoring AFE
 *          - SH367306: 6 to 10 series lithium battery protection & monitoring AFE
 *          The registers and pin definitions across this series are mutually compatible.
 * 
 * @version V1.0
 * @date    2026-09-10
 * @par Change Log:
 * Date       Version  Author                       Description
 * 2026-09-10  V1.0    Rougga(2839754468@qq.com)    Initial release
 */

#include "sh36730x.h"

/**
 * @brief CRC8 lookup table for SH36730X, CRC8 = x^8 + x^2 + x + 1, init = 0x00, polynomial = 0x07
 */
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};


/**
 * @brief Computes the CRC8 checksum for the given data.
 * @param data Pointer to the data buffer.
 * @param size Size of the data buffer.
 * @return Computed CRC8 checksum.
 */
static uint8_t sh36730x_crc8_fast(const uint8_t *data, size_t size)
{
    uint8_t crc = 0x00;

    for (size_t i = 0; i < size; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

sh36730x_status_t sh36730x_init(sh36730x_t *device)
{
    if (device == NULL || \
        device->interface.write == NULL || \
        device->interface.write_read == NULL) {

        return SH36730X_ERROR_PARAM;
    }

    device->initialized = true;
    return SH36730X_OK;
}

sh36730x_status_t sh36730x_read_register(
    sh36730x_t *device,
    uint8_t register_address,
    uint16_t *value)
{
    if (device == NULL || !device->initialized || value == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    /* Validate the register address:
     * - Must not exceed the last register address.
     * - ADC data registers (starting from CELL1H) must be accessed with even addresses only.
     */
    if (register_address > (SH36730X_REG_END_ADDR - 1U) || 
       (register_address >= SH36730X_REG_CELL1H && (register_address & 0x01U) != 0U)) {
        return SH36730X_ERROR_PARAM;
    }

    // Append CRC8 to the register address for communication
    uint8_t rx_buf[3] = {0};

    int result = device->interface.write_read(
        device->interface.context,
        device->address_7bit,
        &register_address,
        1,
        rx_buf,
        3);

    if (result < 0) {
        return SH36730X_ERROR_COMMUNICATION;
    }

    /* Prepare the CRC stream for verification [slave address + write, register address, slave address + read, read data1, read data2] */
    uint8_t crc_stream[5] = {0};
    crc_stream[0] = (uint8_t)(device->address_7bit << 1);               // slave address + write(0)
    crc_stream[1] = register_address;                                   // register address
    crc_stream[2] = (uint8_t)((device->address_7bit << 1) | 0x01);      // slave address after Restart + read(1)
    crc_stream[3] = rx_buf[0];                                          // Read Data1 (high 8 bits)
    crc_stream[4] = rx_buf[1];                                          // Read Data2 (low 8 bits)

    /* Verify the CRC8 checksum of the received data */
    uint8_t expected_crc = sh36730x_crc8_fast(crc_stream, 5);
    if (rx_buf[2] != expected_crc) {
        return SH36730X_ERROR_CRC;
    }

    *value = (rx_buf[0] << 8) | rx_buf[1];

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_write_register(
    sh36730x_t *device,
    uint8_t register_address,
    uint8_t value)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t crc_stream[3] = {0};
    /* Prepare the CRC stream for the write operation [slave address + write, register address, write data] */
    crc_stream[0] = (uint8_t)(device->address_7bit << 1);               // slave address + write(0)
    crc_stream[1] = register_address;                                   // register address
    crc_stream[2] = value;                                              // write data
    
    uint8_t wr_buf[3] = {0};
    wr_buf[0] = register_address;
    wr_buf[1] = value;
    wr_buf[2] = sh36730x_crc8_fast(crc_stream, 3);

    int result = device->interface.write(
        device->interface.context,
        device->address_7bit,
        wr_buf,
        3);

    if (result < 0) {
        return SH36730X_ERROR_COMMUNICATION;
    }

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_update_registers(
    sh36730x_t *device,
    uint8_t register_address,
    uint16_t mask,
    uint16_t value)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    /* Prevent the risk of accidental clearing during "Clear-on-Read" operation. */
    /* When register_address is equal to 0x03 (INT_EN), the read-out 16-bit value is [0x03, 0x04]. */
    /* It is absolutely forbidden for register_address to be 0x02, because when reading [0x02, 0x03], 
    if the hardware crosses the page upwards and touches 0x01 (FLAG2), it will trigger a read-clear operation. */
    if (register_address == SH36730X_REG_FLAG1 || register_address == SH36730X_REG_FLAG2) {
        return SH36730X_ERROR_PARAM;
    }

    /* Read two consecutive bytes through the I2C physical interface. */
    uint16_t current_pair = 0;
    sh36730x_status_t status = sh36730x_read_register(device, register_address, &current_pair);
    if (status != SH36730X_OK) {
        return status;
    }

    /* The structure of current_pair: the high 8 bits are reg[register_address], and the low 8 bits are reg[register_address + 1] */
    uint8_t high_current = (uint8_t)(current_pair >> 8);
    uint8_t low_current  = (uint8_t)(current_pair & 0xFF);

    uint8_t high_mask  = (uint8_t)(mask >> 8);
    uint8_t low_mask   = (uint8_t)(mask & 0xFF);

    uint8_t high_val   = (uint8_t)(value >> 8);
    uint8_t low_val    = (uint8_t)(value & 0xFF);

    /* Calculate the new modified value */
    uint8_t high_new = (high_current & ~high_mask) | (high_val & high_mask);
    uint8_t low_new  = (low_current  & ~low_mask)  | (low_val  & low_mask);

    /* 3. Determine and execute the write for the high byte (register_address) */
    if (high_mask != 0x00 && high_new != high_current) {
        status = sh36730x_write_register(device, register_address, high_new);
        if (status != SH36730X_OK) {
            return status;
        }
    }

    /* Determine and execute the write for the low byte (register_address + 1) */
    if (low_mask != 0x00 && low_new != low_current) {
        status = sh36730x_write_register(device, register_address + 1, low_new);
        if (status != SH36730X_OK) {
            return status;
        }
    }

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_vadc_config(
    sh36730x_t *device,
    const sh36730x_vadc_config_t *config)
{
    if (device == NULL || !device->initialized || config == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf3_val = 0;

    /* Assemble the target data of SCONF3 (0x06) */
    if (config->enable) {
        sconf3_val |= SH36730X_SCONF3_VADC_EN_MASK;
    }

    SH36730X_MODIFY_FIELD(
        sconf3_val,
        SH36730X_SCONF3_VADC_C_MASK,
        SH36730X_SCONF3_VADC_C_POS,
        config->mode);

    SH36730X_MODIFY_FIELD(
        sconf3_val,
        SH36730X_SCONF3_SCAN_C_MASK,
        SH36730X_SCONF3_SCAN_C_POS,
        config->scan_period);
        
    /* Combine the 16-bit mask and value: high 8 bits are SCONF3, low 8 bits are 0x00 (ignore SCONF4) */
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF3_VADC_EN_MASK | SH36730X_SCONF3_VADC_C_MASK | SH36730X_SCONF3_SCAN_C_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf3_val);

    /* Execute read-modify-write: read [0x06, 0x07], only update and write 0x06 */
    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF3,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_vadc_enable(
    sh36730x_t *device,
    bool enable)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf3_val = 0;
    SH36730X_MODIFY_FIELD(
        sconf3_val,
        SH36730X_SCONF3_VADC_EN_MASK,
        SH36730X_SCONF3_VADC_EN_POS,
        enable ? 1 : 0);

    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF3_VADC_EN_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf3_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF3,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_vadc_set_scan_period(
    sh36730x_t *device,
    sh36730x_vadc_scan_period_t scan_period)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf3_val = 0;
    SH36730X_MODIFY_FIELD(
        sconf3_val,
        SH36730X_SCONF3_SCAN_C_MASK,
        SH36730X_SCONF3_SCAN_C_POS,
        scan_period);

    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF3_SCAN_C_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf3_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF3,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_vadc_get_scan_period(
    sh36730x_t *device,
    sh36730x_vadc_scan_period_t *scan_period)
{
    if (device == NULL || !device->initialized || scan_period == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF3, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf3 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *scan_period = (sh36730x_vadc_scan_period_t)SH36730X_READ_FIELD(
        sconf3, SH36730X_SCONF3_SCAN_C_MASK, SH36730X_SCONF3_SCAN_C_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_vadc_set_mode(
    sh36730x_t *device,
    sh36730x_vadc_mode_t mode)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf3_val = 0;
    SH36730X_MODIFY_FIELD(
        sconf3_val,
        SH36730X_SCONF3_VADC_C_MASK,
        SH36730X_SCONF3_VADC_C_POS,
        mode);

    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF3_VADC_C_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf3_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF3,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_vadc_get_mode(
    sh36730x_t *device,
    sh36730x_vadc_mode_t *mode)
{
    if (device == NULL || !device->initialized || mode == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF3, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf3 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *mode = (sh36730x_vadc_mode_t)SH36730X_READ_FIELD(
        sconf3, SH36730X_SCONF3_VADC_C_MASK, SH36730X_SCONF3_VADC_C_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_get_cell_voltage(
    sh36730x_t *device,
    sh36730x_cell_channel_t cell,
    uint16_t *voltage_mv)
{
    if (device == NULL || !device->initialized || voltage_mv == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    if ((device->chip_model == SH36730X_CHIP_SH367303 && cell > SH36730X_CELL_5) || \
        (device->chip_model == SH36730X_CHIP_SH367305 && cell > SH36730X_CELL_8) || \
        (device->chip_model == SH36730X_CHIP_SH367309 && cell > SH36730X_CELL_10)) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t reg_address = SH36730X_REG_CELL_H(((uint8_t)(cell + 1)));
    uint16_t reg_value = 0;
    sh36730x_status_t status = sh36730x_read_register(
        device,
        reg_address,
        &reg_value);
    if (status != SH36730X_OK) {
        return status;
    }
    *voltage_mv = (uint16_t)SH36730X_RAW_TO_CELL_VOLTAGE_MV(reg_value);
    return SH36730X_OK;
}

sh36730x_status_t sh36730x_get_cells_group_voltage(
    sh36730x_t *device,
    uint8_t min_cell, 
    uint8_t max_cell,
    uint16_t *voltages_mv,
    uint8_t voltages_mv_len)
{
    if (device == NULL || !device->initialized || voltages_mv == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    if (min_cell < 1 || max_cell > 10 || min_cell > max_cell) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t num_cells = max_cell - min_cell + 1;
    if (voltages_mv_len < num_cells) {
        return SH36730X_ERROR_PARAM;
    }

    for (uint8_t i = 0; i < num_cells; i++) {
        sh36730x_status_t status = sh36730x_get_cell_voltage(
            device,
            (sh36730x_cell_channel_t)(min_cell - 1 + i),
            &voltages_mv[i]);
        if (status != SH36730X_OK) {
            return status;
        }
    }

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_get_temp_ts(
    sh36730x_t *device,
    sh36730x_temp_ts_channel_t ts,
    uint32_t *ohm)
{
    if (device == NULL || !device->initialized || ohm == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t reg_address = SH36730X_REG_TS1H + (uint8_t)(ts << 1);
    uint16_t reg_value = 0;
    sh36730x_status_t status = sh36730x_read_register(
        device,
        reg_address,
        &reg_value);
    if (status != SH36730X_OK) {
        return status;
    }
    *ohm = (uint32_t)SH36730X_RAW_TO_TS_RESISTANCE_OHM(reg_value, 10000);
    return SH36730X_OK;
}

sh36730x_status_t sh36730x_get_internal_temp(
    sh36730x_t *device,
    sh36730x_temp_internal_channel_t internal,
    float *temperature_C)
{
    if (device == NULL || !device->initialized || temperature_C == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t reg_address = SH36730X_REG_TEMP1H + (uint8_t)(internal << 1);
    uint16_t reg_value = 0;
    sh36730x_status_t status = sh36730x_read_register(
        device,
        reg_address,
        &reg_value);
    if (status != SH36730X_OK) {
        return status;
    }
    *temperature_C = (float)SH36730X_RAW_TO_INTERNAL_TEMP_C_FLOAT(reg_value);
    return SH36730X_OK;
}

sh36730x_status_t sh36730x_cadc_config(
    sh36730x_t *device,
    sh36730x_cadc_config_t *config)
{
    if (device == NULL || !device->initialized || config == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    sh36730x_status_t status;

    /* configure SCONF3 (0x06) */
    uint8_t sconf3_val = 0;

    if (config->enable) {
        sconf3_val |= SH36730X_SCONF3_CADC_EN_MASK;
    }

    SH36730X_MODIFY_FIELD(
        sconf3_val,
        SH36730X_SCONF3_CADC_M_MASK,
        SH36730X_SCONF3_CADC_M_POS,
        config->cadc_mode);

    SH36730X_MODIFY_FIELD(
        sconf3_val,
        SH36730X_SCONF3_CBIT_C_MASK,
        SH36730X_SCONF3_CBIT_C_POS,
        config->cbti_c);

    /* combine 16-bit mask and value*/
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF3_CADC_EN_MASK | SH36730X_SCONF3_CADC_M_MASK | SH36730X_SCONF3_CBIT_C_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf3_val);

    status = sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF3,
        mask_16,
        val_16);

    if (status != SH36730X_OK) {
        return status;
    }

    /* configure SCONF6 (0x09) */
    uint8_t sconf6_val = 0;
    SH36730X_MODIFY_FIELD(
        sconf6_val,
        SH36730X_SCONF6_RSNS_MASK,
        SH36730X_SCONF6_RSNS_POS,
        config->cadc_rsns);

    /* combine 16-bit mask and value*/
    mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF6_RSNS_MASK);
    val_16  = SH36730X_HIGH_BYTE_16(sconf6_val);
    /* perform standard Read-Modify-Write for single-byte SCONF6 */
    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF6,
        mask_16,
        val_16);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_cadc_enable(
    sh36730x_t *device,
    bool enable)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf3_val = 0;
    SH36730X_MODIFY_FIELD(
        sconf3_val,
        SH36730X_SCONF3_CADC_EN_MASK,
        SH36730X_SCONF3_CADC_EN_POS,
        enable);

    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF3_CADC_EN_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf3_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF3,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_cadc_set_rsns(
    sh36730x_t *device,
    sh36730x_cadc_rsns_t cadc_rsns)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf6_val = 0;
    SH36730X_MODIFY_FIELD(
        sconf6_val,
        SH36730X_SCONF6_RSNS_MASK,
        SH36730X_SCONF6_RSNS_POS,
        cadc_rsns);

    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF6_RSNS_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf6_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF6,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_cadc_get_rsns(
    sh36730x_t *device,
    sh36730x_cadc_rsns_t *cadc_rsns)
{
    if (device == NULL || !device->initialized || cadc_rsns == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF6, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf6 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *cadc_rsns = (sh36730x_cadc_rsns_t)SH36730X_READ_FIELD(
        sconf6, SH36730X_SCONF6_RSNS_MASK, SH36730X_SCONF6_RSNS_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_cadc_set_cbti_c(
    sh36730x_t *device,
    sh36730x_cbti_c_t cbti_c)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf3_val = 0;
    SH36730X_MODIFY_FIELD(
        sconf3_val,
        SH36730X_SCONF3_CBIT_C_MASK,
        SH36730X_SCONF3_CBIT_C_POS,
        cbti_c);

    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF3_CBIT_C_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf3_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF3,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_cadc_get_cbti_c(
    sh36730x_t *device,
    sh36730x_cbti_c_t *cbti_c)
{
    if (device == NULL || !device->initialized || cbti_c == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF3, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf3 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *cbti_c = (sh36730x_cbti_c_t)SH36730X_READ_FIELD(
        sconf3, SH36730X_SCONF3_CBIT_C_MASK, SH36730X_SCONF3_CBIT_C_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_cadc_set_mode(
    sh36730x_t *device,
    sh36730x_cadc_mode_t cadc_mode)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf3_val = 0;
    SH36730X_MODIFY_FIELD(
        sconf3_val,
        SH36730X_SCONF3_CADC_M_MASK,
        SH36730X_SCONF3_CADC_M_POS,
        cadc_mode);

    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF3_CADC_M_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf3_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF3,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_cadc_get_mode(
    sh36730x_t *device,
    sh36730x_cadc_mode_t *cadc_mode)
{
    if (device == NULL || !device->initialized || cadc_mode == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF3, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf3 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *cadc_mode = (sh36730x_cadc_mode_t)SH36730X_READ_FIELD(
        sconf3, SH36730X_SCONF3_CADC_M_MASK, SH36730X_SCONF3_CADC_M_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_get_current(
    sh36730x_t *device,
    float rsense_ohm,
    float *current)
{
    if (device == NULL || !device->initialized || current == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    sh36730x_status_t status = SH36730X_OK;
    uint16_t reg_val = 0;
    sh36730x_cadc_rsns_t cadc_rsns;
    float rsns_mv_float = 0.0f;

    status = sh36730x_read_register(
        device,
        SH36730X_REG_SCONF6,
        &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    cadc_rsns = SH36730X_READ_FIELD(SH36730X_EXTRACT_HIGH_BYTE_8(reg_val), SH36730X_SCONF6_RSNS_MASK, SH36730X_SCONF6_RSNS_POS);
    switch (cadc_rsns) {
    case SH36730X_CADC_RSNS_50MV:
        rsns_mv_float = SH36730X_CADC_DIV_50MV;
        break;
    case SH36730X_CADC_RSNS_100MV:
        rsns_mv_float = SH36730X_CADC_DIV_100MV;
        break;
    case SH36730X_CADC_RSNS_200MV:
        rsns_mv_float = SH36730X_CADC_DIV_200MV;
        break;
    case SH36730X_CADC_RSNS_400MV:
        rsns_mv_float = SH36730X_CADC_DIV_400MV;
        break;
    default:
        return SH36730X_ERROR_PARAM;
    }

    status = sh36730x_read_register(
        device,
        SH36730X_REG_CURH,
        &reg_val);

    if (status != SH36730X_OK) {
        return status;
    }

    int16_t raw_code = SH36730X_EXTRACT_CUR_SIGNED13(SH36730X_EXTRACT_HIGH_BYTE_8(reg_val) , SH36730X_EXTRACT_LOW_BYTE_8(reg_val));
    *current = (float)SH36730X_CADC_CALC_CURRENT_MA(raw_code, rsns_mv_float, rsense_ohm);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_enable_ov(
    sh36730x_t *device,
    bool enable)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    sh36730x_status_t status = SH36730X_OK;
    uint8_t sconf1_val = 0;

    SH36730X_MODIFY_FIELD(
        sconf1_val,
        SH36730X_SCONF1_OV_EN_MASK,
        SH36730X_SCONF1_OV_EN_POS,
        enable ? 1 : 0);

    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF1_OV_EN_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf1_val);

    status = sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF1,
        mask_16,
        val_16);

    uint8_t sconf3_val = 0;
    SH36730X_MODIFY_FIELD(
        sconf3_val,
        SH36730X_SCONF3_VADC_EN_MASK,
        SH36730X_SCONF3_VADC_EN_POS,
        1);

    mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF3_VADC_EN_MASK);
    val_16  = SH36730X_HIGH_BYTE_16(sconf3_val);

    status = sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF3,
        mask_16,
        val_16);

    return status;
}

sh36730x_status_t sh36730x_clear_flag1(
    sh36730x_t *device)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf1_val = 0;
    uint16_t reg_val = 0;
    SH36730X_MODIFY_FIELD(
        sconf1_val,
        SH36730X_SCONF1_LTCLR_MASK,
        SH36730X_SCONF1_LTCLR_POS,
        1);


    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF1_LTCLR_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf1_val);

    // Clear the LT flag by writing 1 to the LTCLR field
    sh36730x_status_t status = sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF1,
        mask_16,
        val_16);

    if (status != SH36730X_OK) {
        return status;
    }

    status = sh36730x_read_register(
        device,
        SH36730X_REG_SCONF1,
        &reg_val
    );

    if (status != SH36730X_OK) {
        return status;
    }

    // Check if the LT flag is cleared
    if (SH36730X_TEST_BIT(reg_val, SH36730X_SCONF1_LTCLR_MASK)) {
        status = SH36730X_ERROR;
    }

    return status;
}

sh36730x_status_t sh36730x_get_flag1(
    sh36730x_t *device,
    uint8_t *flag1)
{
    if (device == NULL || !device->initialized || flag1 == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_FLAG1, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    *flag1 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val) & SH36730X_FLAG1_ALL_FLAGS_MASK;
    return SH36730X_OK;
}

sh36730x_status_t sh36730x_get_flag2(
    sh36730x_t *device,
    uint8_t *flag2)
{
    if (device == NULL || !device->initialized || flag2 == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_FLAG2, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    *flag2 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val) & SH36730X_FLAG2_ALL_FLAGS_MASK;
    return SH36730X_OK;
}

sh36730x_status_t sh36730x_get_reset_flag(
    sh36730x_t *device,
    bool *reset_occurred)
{
    if (device == NULL || !device->initialized || reset_occurred == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t flag2 = 0;
    sh36730x_status_t status = sh36730x_get_flag2(device, &flag2);
    if (status != SH36730X_OK) {
        return status;
    }

    *reset_occurred = (SH36730X_READ_FIELD(flag2, SH36730X_FLAG2_RST_MASK, SH36730X_FLAG2_RST_POS) != 0U);
    return SH36730X_OK;
}

sh36730x_status_t sh36730x_set_ov_voltage(
    sh36730x_t *device,
    float voltage)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    sh36730x_status_t status = SH36730X_OK;
    uint8_t sconf1_val = 0;
    
    
    uint16_t raw_ovd = SH36730X_VOLTAGE_MV_TO_OVD(voltage);
    
    /* Ensure raw_ovd does not exceed the maximum allowed value */
    if (raw_ovd > SH36730X_RAW_OVD_MAX_VAL) {
        raw_ovd = SH36730X_RAW_OVD_MAX_VAL;
    }

    /* Split the 10-bit raw OVD value into two separate register values:
       SCONF8 (0x0B) contains the upper 2 bits (OVD.9 and OVD.8) in bits 1 and 0.
       SCONF9 (0x0C) contains the lower 8 bits (OVD.7 ~ OVD.0). */
    uint8_t sconf8_val = (uint8_t)((raw_ovd >> 8) & 0x03U);
    uint8_t sconf9_val = (uint8_t)(raw_ovd & 0xFFU);

    status = sh36730x_write_register(device, SH36730X_REG_SCONF8, sconf8_val);
    if (status != SH36730X_OK) {
        return status;
    }

    status = sh36730x_write_register(device, SH36730X_REG_SCONF9, sconf9_val);
    if (status != SH36730X_OK) {
        return status;
    }

    return status;
}

sh36730x_status_t sh36730x_get_ov_voltage(
    sh36730x_t *device,
    float *voltage)
{
    if (device == NULL || !device->initialized || voltage == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF8, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf8 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    uint8_t sconf9 = SH36730X_EXTRACT_LOW_BYTE_8(reg_val);
    uint16_t ovd_raw = SH36730X_OVD_EXTRACT_10BIT(sconf8, sconf9);

    *voltage = (float)SH36730X_OVD_TO_VOLTAGE_MV(ovd_raw);
    return SH36730X_OK;
}

sh36730x_status_t sh36730x_set_ov_delay(
    sh36730x_t *device,
    sh36730x_ov_delay_t delay)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    if (delay < SH36730X_OV_DELAY_1CYCLE || delay > SH36730X_OV_DELAY_128CYCLE) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF7_OVT_MASK, SH36730X_SCONF7_OVT_POS, delay);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF7_OVT_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    sh36730x_status_t status = sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF7,
        mask_16,
        val_16);

    return status;
}

sh36730x_status_t sh36730x_get_ov_delay(
    sh36730x_t *device,
    sh36730x_ov_delay_t *delay)
{
    if (device == NULL || !device->initialized || delay == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF7, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf7 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *delay = (sh36730x_ov_delay_t)SH36730X_READ_FIELD(
        sconf7, SH36730X_SCONF7_OVT_MASK, SH36730X_SCONF7_OVT_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_set_resetpf(
    sh36730x_t *device,
    sh36730x_reset_pf_t option)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF2_RESET_PF_MASK, SH36730X_SCONF2_RESET_PF_POS, option);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF2_RESET_PF_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    sh36730x_status_t status = sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF2,
        mask_16,
        val_16);

    return status;
}

sh36730x_status_t sh36730x_get_resetpf(
    sh36730x_t *device,
    sh36730x_reset_pf_t *option)
{
    if (device == NULL || !device->initialized || option == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF2, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf2 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *option = (sh36730x_reset_pf_t)SH36730X_READ_FIELD(
        sconf2, SH36730X_SCONF2_RESET_PF_MASK, SH36730X_SCONF2_RESET_PF_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_enable_sc(
    sh36730x_t *device,
    bool enable)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF1_SC_EN_MASK, SH36730X_SCONF1_SC_EN_POS, enable);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF1_SC_EN_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    sh36730x_status_t status = sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF1,
        mask_16,
        val_16);

    return status;
}

sh36730x_status_t sh36730x_set_scv(
    sh36730x_t *device,
    sh36730x_scv_t scv)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF6_SCV_MASK, SH36730X_SCONF6_SCV_POS, scv);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF6_SCV_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    sh36730x_status_t status = sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF6,
        mask_16,
        val_16);

    return status;
}

sh36730x_status_t sh36730x_get_scv(
    sh36730x_t *device,
    sh36730x_scv_t *scv)
{
    if (device == NULL || !device->initialized || scv == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF6, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf6 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *scv = (sh36730x_scv_t)SH36730X_READ_FIELD(
        sconf6, SH36730X_SCONF6_SCV_MASK, SH36730X_SCONF6_SCV_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_set_sct(
    sh36730x_t *device,
    sh36730x_sct_t sct)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF6_SCT_MASK, SH36730X_SCONF6_SCT_POS, sct);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF6_SCT_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    sh36730x_status_t status = sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF6,
        mask_16,
        val_16);

    return status;
}

sh36730x_status_t sh36730x_get_sct(
    sh36730x_t *device,
    sh36730x_sct_t *sct)
{
    if (device == NULL || !device->initialized || sct == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF6, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf6 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *sct = (sh36730x_sct_t)SH36730X_READ_FIELD(
        sconf6, SH36730X_SCONF6_SCT_MASK, SH36730X_SCONF6_SCT_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_enable_charger_detection(
    sh36730x_t *device,
    bool enable)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF1_CHGR_EN_MASK, SH36730X_SCONF1_CHGR_EN_POS, enable ? 1 : 0);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF1_CHGR_EN_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF1,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_enable_load_detection(
    sh36730x_t *device,
    bool enable)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF1_LOAD_EN_MASK, SH36730X_SCONF1_LOAD_EN_POS, enable ? 1 : 0);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF1_LOAD_EN_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF1,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_get_charger_status(
    sh36730x_t *device,
    bool *connected)
{
    if (device == NULL || !device->initialized || connected == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(
        device,
        SH36730X_REG_BSTATUS,
        &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t bstatus = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *connected = (SH36730X_READ_FIELD(bstatus, SH36730X_BSTATUS_CHGR_MASK, SH36730X_BSTATUS_CHGR_POS) != 0U);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_get_load_status(
    sh36730x_t *device,
    bool *connected)
{
    if (device == NULL || !device->initialized || connected == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(
        device,
        SH36730X_REG_BSTATUS,
        &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t bstatus = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *connected = (SH36730X_READ_FIELD(bstatus, SH36730X_BSTATUS_LOAD_MASK, SH36730X_BSTATUS_LOAD_POS) != 0U);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_enable_ctld(
    sh36730x_t *device,
    bool enable)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF1_CTLD_EN_MASK, SH36730X_SCONF1_CTLD_EN_POS, enable ? 1 : 0);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF1_CTLD_EN_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF1,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_set_chs_threshold(
    sh36730x_t *device,
    sh36730x_chs_threshold_t threshold)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF7_CHS_MASK, SH36730X_SCONF7_CHS_POS, threshold);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF7_CHS_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF7,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_get_chs_threshold(
    sh36730x_t *device,
    sh36730x_chs_threshold_t *threshold)
{
    if (device == NULL || !device->initialized || threshold == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF7, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf7 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *threshold = (sh36730x_chs_threshold_t)SH36730X_READ_FIELD(
        sconf7, SH36730X_SCONF7_CHS_MASK, SH36730X_SCONF7_CHS_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_get_charging_status(
    sh36730x_t *device,
    bool *is_charging)
{
    if (device == NULL || !device->initialized || is_charging == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_BSTATUS, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t bstatus = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *is_charging = (SH36730X_READ_FIELD(bstatus, SH36730X_BSTATUS_CHGING_MASK, SH36730X_BSTATUS_CHGING_POS) != 0U);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_get_discharging_status(
    sh36730x_t *device,
    bool *is_discharging)
{
    if (device == NULL || !device->initialized || is_discharging == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_BSTATUS, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t bstatus = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *is_discharging = (SH36730X_READ_FIELD(bstatus, SH36730X_BSTATUS_DSGING_MASK, SH36730X_BSTATUS_DSGING_POS) != 0U);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_set_interrupt_enable(
    sh36730x_t *device,
    uint8_t int_mask)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_INT_EN_ALL_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(int_mask & SH36730X_INT_EN_ALL_MASK);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_INT_EN,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_get_interrupt_enable(
    sh36730x_t *device,
    uint8_t *int_mask)
{
    if (device == NULL || !device->initialized || int_mask == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_INT_EN, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t int_en = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *int_mask = int_en & SH36730X_INT_EN_ALL_MASK;

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_enable_wdt(
    sh36730x_t *device,
    bool enable)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF1_WDT_EN_MASK, SH36730X_SCONF1_WDT_EN_POS, enable ? 1 : 0);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF1_WDT_EN_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF1,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_set_wdt_period(
    sh36730x_t *device,
    sh36730x_wdtt_period_t period)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF7_WDTT_MASK, SH36730X_SCONF7_WDTT_POS, period);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF7_WDTT_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF7,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_get_wdt_period(
    sh36730x_t *device,
    sh36730x_wdtt_period_t *period)
{
    if (device == NULL || !device->initialized || period == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF7, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf7 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *period = (sh36730x_wdtt_period_t)SH36730X_READ_FIELD(
        sconf7, SH36730X_SCONF7_WDTT_MASK, SH36730X_SCONF7_WDTT_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_set_rst_pulse_width(
    sh36730x_t *device,
    sh36730x_rst_pulse_t pulse)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF6_RST_MASK, SH36730X_SCONF6_RST_POS, pulse);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF6_RST_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF6,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_get_rst_pulse_width(
    sh36730x_t *device,
    sh36730x_rst_pulse_t *pulse)
{
    if (device == NULL || !device->initialized || pulse == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF6, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf6 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *pulse = (sh36730x_rst_pulse_t)SH36730X_READ_FIELD(
        sconf6, SH36730X_SCONF6_RST_MASK, SH36730X_SCONF6_RST_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_set_alarm_mode(
    sh36730x_t *device,
    sh36730x_alarm_mode_t mode)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf_val = 0;
    SH36730X_MODIFY_FIELD(sconf_val, SH36730X_SCONF2_ALARM_C_MASK, SH36730X_SCONF2_ALARM_C_POS, mode);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF2_ALARM_C_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF2,
        mask_16,
        val_16);
}

sh36730x_status_t sh36730x_get_alarm_mode(
    sh36730x_t *device,
    sh36730x_alarm_mode_t *mode)
{
    if (device == NULL || !device->initialized || mode == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF2, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf2 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val);
    *mode = (sh36730x_alarm_mode_t)SH36730X_READ_FIELD(
        sconf2, SH36730X_SCONF2_ALARM_C_MASK, SH36730X_SCONF2_ALARM_C_POS);

    return SH36730X_OK;
}

sh36730x_status_t sh36730x_set_balancing(
    sh36730x_t *device,
    uint16_t mask_10bit,
    sh36730x_balance_mode_t mode)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t safe_mask = 0;

    if (device->chip_model == SH36730X_CHIP_SH367303) {
        mask_10bit &= 0x001FU;  // for SH367303, only keep CB1~CB5 (Bit 4:0)
    } else if (device->chip_model == SH36730X_CHIP_SH367305) {
        mask_10bit &= 0x00FFU;  // for SH367305, only keep CB1~CB8 (Bit 7:0)
    }

    switch (mode) {
    case SH36730X_BALANCE_DISABLE:
        safe_mask = 0x0000U;
        break;
    case SH36730X_BALANCE_ODD:
        safe_mask = mask_10bit & SH36730X_CB_ODD_MASK;
        break;
    case SH36730X_BALANCE_EVEN:
        safe_mask = mask_10bit & SH36730X_CB_EVEN_MASK;
        break;
    case SH36730X_BALANCE_FORCE_RAW:
        safe_mask = mask_10bit;
        break;
    default:
        return SH36730X_ERROR_PARAM;
    }

    uint8_t sconf4_val = SH36730X_CB_MAP_TO_SCONF4(safe_mask);
    uint8_t sconf5_val = SH36730X_CB_MAP_TO_SCONF5(safe_mask);
    uint16_t mask_16 = 0x1f1fU;
    uint16_t val_16  = SH36730X_PAIR_16(sconf4_val, sconf5_val);

    sh36730x_status_t status = sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF4,
        mask_16,
        val_16);

    return status;
}

sh36730x_status_t sh36730x_get_balancing(
    sh36730x_t *device,
    uint16_t *mask_10bit)
{
    if (device == NULL || !device->initialized || mask_10bit == NULL) {
        return SH36730X_ERROR_PARAM;
    }

    uint16_t reg_val = 0;
    sh36730x_status_t status = sh36730x_read_register(device, SH36730X_REG_SCONF4, &reg_val);
    if (status != SH36730X_OK) {
        return status;
    }

    uint8_t sconf4 = SH36730X_EXTRACT_HIGH_BYTE_8(reg_val); // CB10~CB6 (Bit 4:0)
    uint8_t sconf5 = SH36730X_EXTRACT_LOW_BYTE_8(reg_val);  // CB5~CB1  (Bit 4:0)

    *mask_10bit = (uint16_t)(((uint16_t)(sconf4 & 0x1FU) << 5U) | (uint16_t)(sconf5 & 0x1FU));
    return SH36730X_OK;
}

sh36730x_status_t sh36730x_enter_power_down(
    sh36730x_t *device)
{
    if (device == NULL || !device->initialized) {
        return SH36730X_ERROR_PARAM;
    }

    /* 1. Step 1: Write PIN[7:0] = 0x33 to SCONF10 (0x0D) to authorize Power-Down */
    sh36730x_status_t status = sh36730x_write_register(
        device,
        SH36730X_REG_SCONF10,
        SH36730X_POWER_DOWN_KEY);
    if (status != SH36730X_OK) {
        return status;
    }

    /* 2. Step 2: Sequentially write PD_EN = 1 to SCONF1 (0x04) without intervening commands */
    uint8_t sconf1_val = 0;
    SH36730X_MODIFY_FIELD(sconf1_val, SH36730X_SCONF1_PD_EN_MASK, SH36730X_SCONF1_PD_EN_POS, 1);
    uint16_t mask_16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF1_PD_EN_MASK);
    uint16_t val_16  = SH36730X_HIGH_BYTE_16(sconf1_val);

    return sh36730x_update_registers(
        device,
        SH36730X_REG_SCONF1,
        mask_16,
        val_16);
}

