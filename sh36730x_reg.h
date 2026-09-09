/**
 * @file    sh36730x_reg.h
 * @brief   Register and bit-field definitions for Sinowealth SH367303 / SH367305 / SH367306 AFE ICs.
 * @note    Reference Manual: 《SH36730X CV1.1A.pdf》
 *          - SH367303: 3 to 5 series lithium battery protection & monitoring AFE
 *          - SH367305: 6 to 8 series lithium battery protection & monitoring AFE
 *          - SH367306: 6 to 10 series lithium battery protection & monitoring AFE
 *          The registers and pin definitions across this series are mutually compatible.
 *
 * @version V1.1
 * @date    2026-09-04
 * @par Change Log:
 * Date       Version  Author                       Description
 * 2026-09-04  V1.0    Rougga(2839754468@qq.com)    Initial release
 */

#ifndef SH36730X_REG_H
#define SH36730X_REG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 1. 8-Bit Register Bitwise & Bitfield Helper Macros                          */
/* ========================================================================== */

/**
 * @brief  Generate an 8-bit single bit mask from bit position.
 * @param  pos  Bit position (0 ~ 7).
 */
#define SH36730X_BIT(pos)                     ((uint8_t)(1U << ((uint8_t)(pos))))

/**
 * @brief  Generate an 8-bit contiguous bitmask between high and low bit indices.
 * @param  high  Highest bit position (0 ~ 7, e.g. 6).
 * @param  low   Lowest bit position (0 ~ 7, e.g. 4).
 * @return 8-bit mask (e.g. SH36730X_GENMASK(6, 4) produces 0b01110000 = 0x70).
 */
#define SH36730X_GENMASK(high, low) \
        ((uint8_t)((((uint8_t)1U << (((uint8_t)(high)) - ((uint8_t)(low)) + 1U)) - 1U) << ((uint8_t)(low))))

/**
 * @brief  Set specific bits in an 8-bit register variable.
 * @param  reg   8-bit register variable (LValue).
 * @param  mask  Bit mask to set.
 */
#define SH36730X_SET_BIT(reg, mask)           ((reg) = (uint8_t)((uint8_t)(reg) | (uint8_t)(mask)))

/**
 * @brief  Clear specific bits in an 8-bit register variable.
 * @param  reg   8-bit register variable (LValue).
 * @param  mask  Bit mask to clear.
 */
#define SH36730X_CLEAR_BIT(reg, mask)         ((reg) = (uint8_t)((uint8_t)(reg) & (uint8_t)(~(uint8_t)(mask))))

/**
 * @brief  Toggle specific bits in an 8-bit register variable.
 * @param  reg   8-bit register variable (LValue).
 * @param  mask  Bit mask to toggle.
 */
#define SH36730X_TOGGLE_BIT(reg, mask)        ((reg) = (uint8_t)((uint8_t)(reg) ^ (uint8_t)(mask)))

/**
 * @brief  Test whether any bit in the specified mask is set.
 * @param  reg   8-bit register variable/expression.
 * @param  mask  Bit mask to test.
 * @return true if any masked bit is 1, false otherwise.
 */
#define SH36730X_TEST_BIT(reg, mask)          ((((uint8_t)(reg)) & ((uint8_t)(mask))) != 0U)

/**
 * @brief  Extract a bit-field from an 8-bit register and right-shift to LSB.
 * @param  reg   8-bit register variable/expression.
 * @param  mask  Bitfield mask.
 * @param  pos   Bitfield start position (shift amount).
 * @return Extracted raw integer value.
 */
#define SH36730X_READ_FIELD(reg, mask, pos)   ((uint8_t)((((uint8_t)(reg)) & ((uint8_t)(mask))) >> ((uint8_t)(pos))))

/**
 * @brief  Modify an 8-bit register bit-field with a new value using Read-Modify-Write.
 * @param  reg   8-bit register variable (LValue).
 * @param  mask  Bitfield mask.
 * @param  pos   Bitfield start position (shift amount).
 * @param  val   New raw integer value to write.
 */
#define SH36730X_MODIFY_FIELD(reg, mask, pos, val) \
                ((reg) = (uint8_t)((((uint8_t)(reg)) & (uint8_t)(~(uint8_t)(mask))) | \
                ((((uint8_t)(val)) << ((uint8_t)(pos))) & (uint8_t)(mask))))

/**
 * @brief  Combine two 8-bit values into a 16-bit value.
 * @param  high  High byte.
 * @param  low   Low byte.
 * @return 16-bit combined value.
 */
#define SH36730X_PAIR_16(high, low)       (((uint16_t)(high) << 8) | (uint8_t)(low))

/**
 * @brief  Extract the high and low bytes from a 16-bit value.
 * @param  high  High byte.
 * @param  low   Low byte.
 * @return 16-bit combined value.
 */
#define SH36730X_HIGH_BYTE_16(high)       ((uint16_t)(high) << 8)
#define SH36730X_LOW_BYTE_16(low)         ((uint16_t)(low))

/**
 * @brief  Extract the high byte (Bits 15..8) from a 16-bit value as uint8_t.
 * @param  val16  16-bit unsigned integer.
 * @return 8-bit high byte.
 */
#define SH36730X_EXTRACT_HIGH_BYTE_8(val16)   ((uint8_t)(((uint16_t)(val16) >> 8) & 0xFFU))

/**
 * @brief  Extract the low byte (Bits 7..0) from a 16-bit value as uint8_t.
 * @param  val16  16-bit unsigned integer.
 * @return 8-bit low byte.
 */
#define SH36730X_EXTRACT_LOW_BYTE_8(val16)    ((uint8_t)((uint16_t)(val16) & 0xFFU))

/* ========================================================================== */
/* 2. TWI Communication & Hardware Constants                                   */
/* ========================================================================== */

/**
 * @brief SH36730X fixed 7-bit TWI slave address (0b0011011 = 0x1B).
 */
#define SH36730X_TWI_SLAVE_ADDR_7BIT          0x1BU

/**
 * @brief SH36730X 8-bit TWI write address (0x36).
 */
#define SH36730X_TWI_WRITE_ADDR_8BIT          0x36U

/**
 * @brief SH36730X 8-bit TWI read address (0x37).
 */
#define SH36730X_TWI_READ_ADDR_8BIT           0x37U

/**
 * @brief SH36730X CRC-8 polynomial: X^8 + X^2 + X + 1 (0x07), initial value: 0x00.
 */
#define SH36730X_CRC8_POLYNOMIAL              0x07U
#define SH36730X_CRC8_INIT_VALUE              0x00U

/**
 * @brief TWI bus timeout threshold in milliseconds (SCL/SDA low > 32ms triggers TWI reset).
 */
#define SH36730X_TWI_TIMEOUT_MS               32U

/**
 * @brief Unlock key written to SCONF10 before enabling Power-Down mode in SCONF1.
 */
#define SH36730X_POWER_DOWN_KEY               0x33U

/* ========================================================================== */
/* 3. Complete Register Address Map (0x00 ~ 0x2B, 44 Registers Total)          */
/* ========================================================================== */

/* --- System Status & Control Registers (0x00 ~ 0x0D) --- */
#define SH36730X_REG_FLAG1                    0x00U   /**< System Flag 1 (Hardware Protection, WDT, TWI Timeout, Read-Only) */
#define SH36730X_REG_FLAG2                    0x01U   /**< System Flag 2 (V33 Reset, CADC/VADC Interrupts, Clear-on-Read) */
#define SH36730X_REG_BSTATUS                  0x02U   /**< System Status (MOS, Charge/Discharge Status, Load/Charger, Read-Only) */
#define SH36730X_REG_INT_EN                   0x03U   /**< ALARM Interrupt Enable Control Register (Read/Write) */
#define SH36730X_REG_SCONF1                   0x04U   /**< System Control Register 1 (Protection Enable & Flag Clear, Read/Write) */
#define SH36730X_REG_SCONF2                   0x05U   /**< System Control Register 2 (MOS Switch & Output Mode Control, Read/Write) */
#define SH36730X_REG_SCONF3                   0x06U   /**< System Control Register 3 (ADC Sampling Config & Scan Period, Read/Write) */
#define SH36730X_REG_SCONF4                   0x07U   /**< System Control Register 4 (Cell Balancing High Channels CB10~CB6, Read/Write) */
#define SH36730X_REG_SCONF5                   0x08U   /**< System Control Register 5 (Cell Balancing Low Channels CB5~CB1, Read/Write) */
#define SH36730X_REG_SCONF6                   0x09U   /**< System Control Register 6 (CADC Range, RST Width, SC Threshold & Delay, Read/Write) */
#define SH36730X_REG_SCONF7                   0x0AU   /**< System Control Register 7 (OV Delay, Charge Detect Threshold, WDT Timeout, Read/Write) */
#define SH36730X_REG_SCONF8                   0x0BU   /**< System Control Register 8 (Hardware OV Threshold High Bits OVD[9:8], Read/Write) */
#define SH36730X_REG_SCONF9                   0x0CU   /**< System Control Register 9 (Hardware OV Threshold Low Bits OVD[7:0], Read/Write) */
#define SH36730X_REG_SCONF10                  0x0DU   /**< System Control Register 10 (Power-Down Authorization Key PIN[7:0], Read/Write) */

/* --- Cell Voltage Data Registers (0x0E ~ 0x21, 10 Cells, 12-bit ADC, Read-Only) --- */
#define SH36730X_REG_CELL1H                   0x0EU   /**< Cell 1 Voltage High 4 bits (CELL1[11:8], VC1-GND) */
#define SH36730X_REG_CELL1L                   0x0FU   /**< Cell 1 Voltage Low 8 bits (CELL1[7:0]) */
#define SH36730X_REG_CELL2H                   0x10U   /**< Cell 2 Voltage High 4 bits (CELL2[11:8], VC2-VC1) */
#define SH36730X_REG_CELL2L                   0x11U   /**< Cell 2 Voltage Low 8 bits (CELL2[7:0]) */
#define SH36730X_REG_CELL3H                   0x12U   /**< Cell 3 Voltage High 4 bits (CELL3[11:8], VC3-VC2 or VC3-GND) */
#define SH36730X_REG_CELL3L                   0x13U   /**< Cell 3 Voltage Low 8 bits (CELL3[7:0]) */
#define SH36730X_REG_CELL4H                   0x14U   /**< Cell 4 Voltage High 4 bits (CELL4[11:8], VC4-VC3) */
#define SH36730X_REG_CELL4L                   0x15U   /**< Cell 4 Voltage Low 8 bits (CELL4[7:0]) */
#define SH36730X_REG_CELL5H                   0x16U   /**< Cell 5 Voltage High 4 bits (CELL5[11:8], VC5-VC4) */
#define SH36730X_REG_CELL5L                   0x17U   /**< Cell 5 Voltage Low 8 bits (CELL5[7:0]) */
#define SH36730X_REG_CELL6H                   0x18U   /**< Cell 6 Voltage High 4 bits (CELL6[11:8], VC6-VC5) */
#define SH36730X_REG_CELL6L                   0x19U   /**< Cell 6 Voltage Low 8 bits (CELL6[7:0]) */
#define SH36730X_REG_CELL7H                   0x1AU   /**< Cell 7 Voltage High 4 bits (CELL7[11:8], VC7-VC6) */
#define SH36730X_REG_CELL7L                   0x1BU   /**< Cell 7 Voltage Low 8 bits (CELL7[7:0]) */
#define SH36730X_REG_CELL8H                   0x1CU   /**< Cell 8 Voltage High 4 bits (CELL8[11:8], VC8-VC7) */
#define SH36730X_REG_CELL8L                   0x1DU   /**< Cell 8 Voltage Low 8 bits (CELL8[7:0]) */
#define SH36730X_REG_CELL9H                   0x1EU   /**< Cell 9 Voltage High 4 bits (CELL9[11:8], VC9-VC8) */
#define SH36730X_REG_CELL9L                   0x1FU   /**< Cell 9 Voltage Low 8 bits (CELL9[7:0]) */
#define SH36730X_REG_CELL10H                  0x20U   /**< Cell 10 Voltage High 4 bits (CELL10[11:8], VC10-VC9) */
#define SH36730X_REG_CELL10L                  0x21U   /**< Cell 10 Voltage Low 8 bits (CELL10[7:0]) */

/* --- External & Internal Temperature Data Registers (0x22 ~ 0x29, 12-bit ADC, Read-Only) --- */
#define SH36730X_REG_TS1H                     0x22U   /**< External NTC TS1 Voltage Ratio High 4 bits (TS1[11:8]) */
#define SH36730X_REG_TS1L                     0x23U   /**< External NTC TS1 Voltage Ratio Low 8 bits (TS1[7:0]) */
#define SH36730X_REG_TS2H                     0x24U   /**< External NTC TS2 Voltage Ratio High 4 bits (TS2[11:8]) */
#define SH36730X_REG_TS2L                     0x25U   /**< External NTC TS2 Voltage Ratio Low 8 bits (TS2[7:0]) */
#define SH36730X_REG_TEMP1H                   0x26U   /**< Internal Die Temperature 1 High 4 bits (TEMP1[11:8]) */
#define SH36730X_REG_TEMP1L                   0x27U   /**< Internal Die Temperature 1 Low 8 bits (TEMP1[7:0]) */
#define SH36730X_REG_TEMP2H                   0x28U   /**< Internal Die Temperature 2 High 4 bits (TEMP2[11:8], SH367305/06) */
#define SH36730X_REG_TEMP2L                   0x29U   /**< Internal Die Temperature 2 Low 8 bits (TEMP2[7:0], SH367305/06) */

/* --- Current Data Registers (0x2A ~ 0x2B, 13-bit/10-bit CADC, Read-Only) --- */
#define SH36730X_REG_CURH                     0x2AU   /**< CADC Current High Byte (Bit 4: Sign CUR[12], Bit 3:0: CUR[11:8]) */
#define SH36730X_REG_CURL                     0x2BU   /**< CADC Current Low Byte (CUR[7:0]) */

/* --- Register Address Boundary & Count Constants --- */
#define SH36730X_REG_START_ADDR               0x00U   /**< Lowest register address */
#define SH36730X_REG_END_ADDR                 0x2BU   /**< Highest register address */
#define SH36730X_REG_TOTAL_COUNT              44U     /**< Total number of registers (0x00 ~ 0x2B) */

/**
 * @brief  Helper macros to get register address for cell voltage (cell_index: 1 ~ 10).
 */
#define SH36730X_REG_CELL_H(cell_index) \
        ((uint8_t)(SH36730X_REG_CELL1H + (((uint8_t)(cell_index) - 1U) * 2U)))
#define SH36730X_REG_CELL_L(cell_index) \
        ((uint8_t)(SH36730X_REG_CELL1L + (((uint8_t)(cell_index) - 1U) * 2U)))

/* ========================================================================== */
/* 4. Register Bitfields, Positions, Masks, and Enumerations                   */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* 0x00 FLAG1 - System Flag Register 1                                        */
/* Access: Read-Only | Reset Value: 0x00                                      */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_FLAG1_RESET_VAL          0x00U

/* Bit 3: SC - Hardware Short-Circuit Protection Flag (0: Normal, 1: Triggered) */
#define SH36730X_FLAG1_SC_POS                 3U
#define SH36730X_FLAG1_SC_MASK                SH36730X_BIT(SH36730X_FLAG1_SC_POS)

/* Bit 2: OV - Hardware Over-Charge Protection Flag (0: Normal, 1: Triggered) */
#define SH36730X_FLAG1_OV_POS                 2U
#define SH36730X_FLAG1_OV_MASK                SH36730X_BIT(SH36730X_FLAG1_OV_POS)

/* Bit 1: WDT - Watchdog Timer Overflow Flag (0: Normal, 1: Overflow) */
#define SH36730X_FLAG1_WDT_POS                1U
#define SH36730X_FLAG1_WDT_MASK               SH36730X_BIT(SH36730X_FLAG1_WDT_POS)

/* Bit 0: TWI - TWI Communication Timeout Flag (0: Normal, 1: Timeout occurred) */
#define SH36730X_FLAG1_TWI_POS                0U
#define SH36730X_FLAG1_TWI_MASK               SH36730X_BIT(SH36730X_FLAG1_TWI_POS)

/* All active flags mask in FLAG1 */
#define SH36730X_FLAG1_ALL_FLAGS_MASK         0x0FU

/* -------------------------------------------------------------------------- */
/* 0x01 FLAG2 - System Flag Register 2                                        */
/* Access: Read-Only (Hardware Clears on Read) | Reset Value: 0x04 (RST=1)     */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_FLAG2_RESET_VAL          0x04U

/* Bit 2: RST - V33 Power-On / Reset Flag (0: No Reset, 1: Reset occurred, Clear-on-Read) */
#define SH36730X_FLAG2_RST_POS                2U
#define SH36730X_FLAG2_RST_MASK               SH36730X_BIT(SH36730X_FLAG2_RST_POS)

/* Bit 1: CADC - CADC Conversion Complete Interrupt Flag (0: None, 1: Complete, Clear-on-Read) */
#define SH36730X_FLAG2_CADC_POS               1U
#define SH36730X_FLAG2_CADC_MASK              SH36730X_BIT(SH36730X_FLAG2_CADC_POS)

/* Bit 0: VADC - VADC Conversion Complete Interrupt Flag (0: None, 1: Complete, Clear-on-Read) */
#define SH36730X_FLAG2_VADC_POS               0U
#define SH36730X_FLAG2_VADC_MASK              SH36730X_BIT(SH36730X_FLAG2_VADC_POS)

/* All active flags mask in FLAG2 */
#define SH36730X_FLAG2_ALL_FLAGS_MASK         0x07U

/* -------------------------------------------------------------------------- */
/* 0x02 BSTATUS - System Status Register                                      */
/* Access: Read-Only | Reset Value: 0x00                                      */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_BSTATUS_RESET_VAL        0x00U

/* Bit 5: DSG - Discharge MOSFET Gate Driver Status (0: OFF, 1: ON) */
#define SH36730X_BSTATUS_DSG_POS              5U
#define SH36730X_BSTATUS_DSG_MASK             SH36730X_BIT(SH36730X_BSTATUS_DSG_POS)

/* Bit 4: CHG - Charge MOSFET Gate Driver Status (0: OFF, 1: ON) */
#define SH36730X_BSTATUS_CHG_POS              4U
#define SH36730X_BSTATUS_CHG_MASK             SH36730X_BIT(SH36730X_BSTATUS_CHG_POS)

/* Bit 3: DSGING - Discharge Current Status (0: Non-Discharging, 1: Discharging) */
#define SH36730X_BSTATUS_DSGING_POS           3U
#define SH36730X_BSTATUS_DSGING_MASK          SH36730X_BIT(SH36730X_BSTATUS_DSGING_POS)

/* Bit 2: CHGING - Charge Current Status (0: Non-Charging, 1: Charging) */
#define SH36730X_BSTATUS_CHGING_POS           2U
#define SH36730X_BSTATUS_CHGING_MASK          SH36730X_BIT(SH36730X_BSTATUS_CHGING_POS)

/* Bit 1: LOAD - Load Connection Status (0: Load Disconnected, 1: Load Connected) */
#define SH36730X_BSTATUS_LOAD_POS             1U
#define SH36730X_BSTATUS_LOAD_MASK            SH36730X_BIT(SH36730X_BSTATUS_LOAD_POS)

/* Bit 0: CHGR - Charger Connection Status (0: Charger Disconnected, 1: Charger Connected) */
#define SH36730X_BSTATUS_CHGR_POS             0U
#define SH36730X_BSTATUS_CHGR_MASK            SH36730X_BIT(SH36730X_BSTATUS_CHGR_POS)

/* -------------------------------------------------------------------------- */
/* 0x03 INT_EN - ALARM Control & Interrupt Enable Register                    */
/* Access: Read/Write | Reset Value: 0x00                                     */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_INT_EN_RESET_VAL         0x00U

/* Bit 6: SC_INT - Hardware Short-Circuit Interrupt Enable (0: Disable, 1: Enable ALARM signal) */
#define SH36730X_INT_EN_SC_INT_POS            6U
#define SH36730X_INT_EN_SC_INT_MASK           SH36730X_BIT(SH36730X_INT_EN_SC_INT_POS)

/* Bit 5: OV_INT - Hardware Over-Charge Interrupt Enable (0: Disable, 1: Enable ALARM signal) */
#define SH36730X_INT_EN_OV_INT_POS            5U
#define SH36730X_INT_EN_OV_INT_MASK           SH36730X_BIT(SH36730X_INT_EN_OV_INT_POS)

/* Bit 4: CD_INT - Charge/Discharge Status Change Interrupt Enable (0: Disable, 1: Enable ALARM signal) */
#define SH36730X_INT_EN_CD_INT_POS            4U
#define SH36730X_INT_EN_CD_INT_MASK           SH36730X_BIT(SH36730X_INT_EN_CD_INT_POS)

/* Bit 3: CADC_INT - CADC Conversion Complete Interrupt Enable (0: Disable, 1: Output Low Pulse on ALARM) */
#define SH36730X_INT_EN_CADC_INT_POS          3U
#define SH36730X_INT_EN_CADC_INT_MASK         SH36730X_BIT(SH36730X_INT_EN_CADC_INT_POS)

/* Bit 2: VADC_INT - VADC Conversion Complete Interrupt Enable (0: Disable, 1: Output Low Pulse on ALARM) */
#define SH36730X_INT_EN_VADC_INT_POS          2U
#define SH36730X_INT_EN_VADC_INT_MASK         SH36730X_BIT(SH36730X_INT_EN_VADC_INT_POS)

/* Bit 1: WDT_INT - Watchdog Timeout Interrupt Enable (0: Disable, 1: Enable ALARM signal) */
#define SH36730X_INT_EN_WDT_INT_POS           1U
#define SH36730X_INT_EN_WDT_INT_MASK          SH36730X_BIT(SH36730X_INT_EN_WDT_INT_POS)

/* Bit 0: TWI_INT - TWI Timeout Interrupt Enable (0: Disable, 1: Enable ALARM signal) */
#define SH36730X_INT_EN_TWI_INT_POS           0U
#define SH36730X_INT_EN_TWI_INT_MASK          SH36730X_BIT(SH36730X_INT_EN_TWI_INT_POS)

/* All interrupts enabled mask in INT_EN */
#define SH36730X_INT_EN_ALL_MASK              0x7FU

/* -------------------------------------------------------------------------- */
/* 0x04 SCONF1 - System Control Register 1                                    */
/* Access: Read/Write | Reset Value: 0x00                                     */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_SCONF1_RESET_VAL         0x00U

/* Bit 7: LTCLR - Clear SC/OV/WDT/TWI Flags in FLAG1 (Write 1 then 0 to clear) */
#define SH36730X_SCONF1_LTCLR_POS             7U
#define SH36730X_SCONF1_LTCLR_MASK            SH36730X_BIT(SH36730X_SCONF1_LTCLR_POS)

/* Bit 6: CTLD_EN - CTLD Pin Function Enable (0: Disable, 1: Enable) */
#define SH36730X_SCONF1_CTLD_EN_POS           6U
#define SH36730X_SCONF1_CTLD_EN_MASK          SH36730X_BIT(SH36730X_SCONF1_CTLD_EN_POS)

/* Bit 5: PD_EN - Power-Down Mode Enable (0: Normal Mode, 1: Power-Down Mode, requires SCONF10=0x33 first) */
#define SH36730X_SCONF1_PD_EN_POS             5U
#define SH36730X_SCONF1_PD_EN_MASK            SH36730X_BIT(SH36730X_SCONF1_PD_EN_POS)

/* Bit 4: WDT_EN - Watchdog Timer Enable (0: Disable Watchdog, 1: Enable Watchdog) */
#define SH36730X_SCONF1_WDT_EN_POS            4U
#define SH36730X_SCONF1_WDT_EN_MASK           SH36730X_BIT(SH36730X_SCONF1_WDT_EN_POS)

/* Bit 3: SC_EN - Hardware Short-Circuit Protection Enable (0: Disable, 1: Enable) */
#define SH36730X_SCONF1_SC_EN_POS             3U
#define SH36730X_SCONF1_SC_EN_MASK            SH36730X_BIT(SH36730X_SCONF1_SC_EN_POS)

/* Bit 2: OV_EN - Hardware Over-Charge Protection Enable (0: Disable, 1: Enable) */
#define SH36730X_SCONF1_OV_EN_POS             2U
#define SH36730X_SCONF1_OV_EN_MASK            SH36730X_BIT(SH36730X_SCONF1_OV_EN_POS)

/* Bit 1: LOAD_EN - Load Detection Module Enable (0: Disable, 1: Enable) */
#define SH36730X_SCONF1_LOAD_EN_POS           1U
#define SH36730X_SCONF1_LOAD_EN_MASK          SH36730X_BIT(SH36730X_SCONF1_LOAD_EN_POS)

/* Bit 0: CHGR_EN - Charger Detection Module Enable (0: Disable, 1: Enable) */
#define SH36730X_SCONF1_CHGR_EN_POS           0U
#define SH36730X_SCONF1_CHGR_EN_MASK          SH36730X_BIT(SH36730X_SCONF1_CHGR_EN_POS)

/* -------------------------------------------------------------------------- */
/* 0x05 SCONF2 - System Control Register 2                                    */
/* Access: Read/Write | Reset Value: 0x00                                     */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_SCONF2_RESET_VAL         0x00U

/* Bit 3: RESET_PF - RESET/PF Pin Function Selection */
#define SH36730X_SCONF2_RESET_PF_POS          3U
#define SH36730X_SCONF2_RESET_PF_MASK         SH36730X_BIT(SH36730X_SCONF2_RESET_PF_POS)
#define SH36730X_PIN_MODE_MCU_RESET           0x00U   /**< 0: External MCU Reset Pin */
#define SH36730X_PIN_MODE_SECOND_PF           0x01U   /**< 1: Secondary Protection Output Pin (PF) */

/* Bit 2: ALARM_C - ALARM Output Signal Mode */
#define SH36730X_SCONF2_ALARM_C_POS           2U
#define SH36730X_SCONF2_ALARM_C_MASK          SH36730X_BIT(SH36730X_SCONF2_ALARM_C_POS)
#define SH36730X_ALARM_OUTPUT_PULSE           0x00U   /**< 0: Output Low Pulse */
#define SH36730X_ALARM_OUTPUT_LEVEL           0x01U   /**< 1: Output Continuous Low Level */

/* Bit 1: DSG_C - Discharge MOSFET Gate Driver Control */
#define SH36730X_SCONF2_DSG_C_POS             1U
#define SH36730X_SCONF2_DSG_C_MASK            SH36730X_BIT(SH36730X_SCONF2_DSG_C_POS)
#define SH36730X_DSG_MOS_DISABLE              0x00U   /**< 0: Discharge MOSFET Forced OFF */
#define SH36730X_DSG_MOS_ENABLE               0x01U   /**< 1: Discharge MOSFET Controlled by Hardware Logic */

/* Bit 0: CHG_C - Charge MOSFET Gate Driver Control */
#define SH36730X_SCONF2_CHG_C_POS             0U
#define SH36730X_SCONF2_CHG_C_MASK            SH36730X_BIT(SH36730X_SCONF2_CHG_C_POS)
#define SH36730X_CHG_MOS_DISABLE              0x00U   /**< 0: Charge MOSFET Forced OFF */
#define SH36730X_CHG_MOS_ENABLE               0x01U   /**< 1: Charge MOSFET Controlled by Hardware Logic */

/* -------------------------------------------------------------------------- */
/* 0x06 SCONF3 - System Control Register 3                                    */
/* Access: Read/Write | Reset Value: 0x00                                     */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_SCONF3_RESET_VAL         0x00U

/* Bit 7: CADC_EN - Current ADC (CADC) Module Enable (0: Disable, 1: Enable) */
#define SH36730X_SCONF3_CADC_EN_POS           7U
#define SH36730X_SCONF3_CADC_EN_MASK          SH36730X_BIT(SH36730X_SCONF3_CADC_EN_POS)

/* Bit 6: CADC_M - CADC Sampling Mode */
#define SH36730X_SCONF3_CADC_M_POS            6U
#define SH36730X_SCONF3_CADC_M_MASK           SH36730X_BIT(SH36730X_SCONF3_CADC_M_POS)
#define SH36730X_CADC_MODE_SINGLE             0x00U   /**< 0: Single Conversion (CADC_EN auto-clears on complete) */
#define SH36730X_CADC_MODE_CONTINUOUS         0x01U   /**< 1: Continuous Conversion */

/* Bit 5: CBIT_C - CADC Resolution Selection */
#define SH36730X_SCONF3_CBIT_C_POS            5U
#define SH36730X_SCONF3_CBIT_C_MASK           SH36730X_BIT(SH36730X_SCONF3_CBIT_C_POS)
#define SH36730X_CADC_BITS_10                 0x00U   /**< 0: 10-bit CADC Resolution */
#define SH36730X_CADC_BITS_13                 0x01U   /**< 1: 13-bit CADC Resolution */

/* Bit 4: VADC_EN - Voltage ADC (VADC) Module Enable (0: Disable, 1: Enable) */
#define SH36730X_SCONF3_VADC_EN_POS           4U
#define SH36730X_SCONF3_VADC_EN_MASK          SH36730X_BIT(SH36730X_SCONF3_VADC_EN_POS)

/* Bit 3: VADC_C - VADC Measurement Channel Mode */
#define SH36730X_SCONF3_VADC_C_POS            3U
#define SH36730X_SCONF3_VADC_C_MASK           SH36730X_BIT(SH36730X_SCONF3_VADC_C_POS)
#define SH36730X_VADC_SAMPLE_VOLT_ONLY        0x00U   /**< 0: Measure Cell Voltages Only */
#define SH36730X_VADC_SAMPLE_VOLT_TEMP        0x01U   /**< 1: Measure Cell Voltages & Temperatures */

/* Bits 2:0: SCAN_C - VADC Scan Period Selection */
#define SH36730X_SCONF3_SCAN_C_POS            0U
#define SH36730X_SCONF3_SCAN_C_MASK           SH36730X_GENMASK(2U, 0U)
#define SH36730X_SCAN_PERIOD_50MS             0x00U   /**< 000: 50 ms Period */
#define SH36730X_SCAN_PERIOD_100MS            0x01U   /**< 001: 100 ms Period */
#define SH36730X_SCAN_PERIOD_200MS            0x02U   /**< 010: 200 ms Period */
#define SH36730X_SCAN_PERIOD_500MS            0x03U   /**< 011: 500 ms Period */
#define SH36730X_SCAN_PERIOD_1S               0x04U   /**< 100: 1 s Period */
#define SH36730X_SCAN_PERIOD_2S               0x05U   /**< 101: 2 s Period */
#define SH36730X_SCAN_PERIOD_4S               0x06U   /**< 110: 4 s Period */
#define SH36730X_SCAN_PERIOD_8S               0x07U   /**< 111: 8 s Period */

/* -------------------------------------------------------------------------- */
/* 0x07 SCONF4 - Cell Balancing Control High Channels CB10~CB6                */
/* Access: Read/Write | Reset Value: 0x00                                     */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_SCONF4_RESET_VAL         0x00U

/* Bits 4:0: CB10 ~ CB6 Balancing Switches (0: Disable, 1: Enable) */
#define SH36730X_SCONF4_CB10_POS              4U
#define SH36730X_SCONF4_CB10_MASK             SH36730X_BIT(SH36730X_SCONF4_CB10_POS)
#define SH36730X_SCONF4_CB9_POS               3U
#define SH36730X_SCONF4_CB9_MASK              SH36730X_BIT(SH36730X_SCONF4_CB9_POS)
#define SH36730X_SCONF4_CB8_POS               2U
#define SH36730X_SCONF4_CB8_MASK              SH36730X_BIT(SH36730X_SCONF4_CB8_POS)
#define SH36730X_SCONF4_CB7_POS               1U
#define SH36730X_SCONF4_CB7_MASK              SH36730X_BIT(SH36730X_SCONF4_CB7_POS)
#define SH36730X_SCONF4_CB6_POS               0U
#define SH36730X_SCONF4_CB6_MASK              SH36730X_BIT(SH36730X_SCONF4_CB6_POS)

#define SH36730X_SCONF4_CB_ALL_MASK           0x1FU   /**< Mask for all high balance channels (CB10~CB6) */

/* -------------------------------------------------------------------------- */
/* 0x08 SCONF5 - Cell Balancing Control Low Channels CB5~CB1                  */
/* Access: Read/Write | Reset Value: 0x00                                     */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_SCONF5_RESET_VAL         0x00U

/* Bits 4:0: CB5 ~ CB1 Balancing Switches (0: Disable, 1: Enable) */
#define SH36730X_SCONF5_CB5_POS               4U
#define SH36730X_SCONF5_CB5_MASK              SH36730X_BIT(SH36730X_SCONF5_CB5_POS)
#define SH36730X_SCONF5_CB4_POS               3U
#define SH36730X_SCONF5_CB4_MASK              SH36730X_BIT(SH36730X_SCONF5_CB4_POS)
#define SH36730X_SCONF5_CB3_POS               2U
#define SH36730X_SCONF5_CB3_MASK              SH36730X_BIT(SH36730X_SCONF5_CB3_POS)
#define SH36730X_SCONF5_CB2_POS               1U
#define SH36730X_SCONF5_CB2_MASK              SH36730X_BIT(SH36730X_SCONF5_CB2_POS)
#define SH36730X_SCONF5_CB1_POS               0U
#define SH36730X_SCONF5_CB1_MASK              SH36730X_BIT(SH36730X_SCONF5_CB1_POS)

#define SH36730X_SCONF5_CB_ALL_MASK           0x1FU   /**< Mask for all low balance channels (CB5~CB1) */

/**
 * @brief  Split 10-bit balance mask (Bit 9~0 for Cell10~1) into SCONF4 & SCONF5 byte values.
 */
#define SH36730X_CB_MAP_TO_SCONF4(cb_10bit)   ((uint8_t)(((uint16_t)(cb_10bit) >> 5U) & 0x1FU))
#define SH36730X_CB_MAP_TO_SCONF5(cb_10bit)   ((uint8_t)(((uint16_t)(cb_10bit)) & 0x1FU))

/**
 * @brief  Masks for all, odd, and even balance channels.
 */
#define SH36730X_CB_ALL_MASK      (0x03FFU)  /* Mask for all balance channels (CB10~CB1) */
#define SH36730X_CB_ODD_MASK      (0x0155U)  /* Mask for odd balance channels (CB1, CB3, CB5, CB7, CB9) */
#define SH36730X_CB_EVEN_MASK     (0x02AAU)  /* Mask for even balance channels (CB2, CB4, CB6, CB8, CB10) */

/* -------------------------------------------------------------------------- */
/* 0x09 SCONF6 - System Control Register 6                                    */
/* Access: Read/Write | Reset Value: 0x00                                     */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_SCONF6_RESET_VAL         0x00U

/* Bits 7:6: RSNS[1:0] - CADC Current Sense Input Full-Scale Voltage Range */
#define SH36730X_SCONF6_RSNS_POS              6U
#define SH36730X_SCONF6_RSNS_MASK             SH36730X_GENMASK(7U, 6U)
#define SH36730X_RSNS_RANGE_400MV             0x00U   /**< 00: 0 ~ 400 mV */
#define SH36730X_RSNS_RANGE_200MV             0x01U   /**< 01: 0 ~ 200 mV */
#define SH36730X_RSNS_RANGE_100MV             0x02U   /**< 10: 0 ~ 100 mV */
#define SH36730X_RSNS_RANGE_50MV              0x03U   /**< 11: 0 ~ 50 mV */

/* Bits 5:4: RST[1:0] - External MCU Reset Output Pulse Width */
#define SH36730X_SCONF6_RST_POS               4U
#define SH36730X_SCONF6_RST_MASK              SH36730X_GENMASK(5U, 4U)
#define SH36730X_RST_PULSE_16MS               0x00U   /**< 00: 16 ms Pulse Width */
#define SH36730X_RST_PULSE_32MS               0x01U   /**< 01: 32 ms Pulse Width */
#define SH36730X_RST_PULSE_128MS              0x02U   /**< 10: 128 ms Pulse Width */
#define SH36730X_RST_PULSE_1S                 0x03U   /**< 11: 1 s Pulse Width */

/* Bits 3:2: SCV[1:0] - Hardware Short-Circuit Protection Voltage Threshold (VRS2-RS1) */
#define SH36730X_SCONF6_SCV_POS               2U
#define SH36730X_SCONF6_SCV_MASK              SH36730X_GENMASK(3U, 2U)
#define SH36730X_SCV_VOLTAGE_100MV            0x00U   /**< 00: 100 mV SC Threshold */
#define SH36730X_SCV_VOLTAGE_200MV            0x01U   /**< 01: 200 mV SC Threshold */
#define SH36730X_SCV_VOLTAGE_300MV            0x02U   /**< 10: 300 mV SC Threshold */
#define SH36730X_SCV_VOLTAGE_400MV            0x03U   /**< 11: 400 mV SC Threshold */

/* Bits 1:0: SCT[1:0] - Hardware Short-Circuit Protection Internal Filter Delay */
#define SH36730X_SCONF6_SCT_POS               0U
#define SH36730X_SCONF6_SCT_MASK              SH36730X_GENMASK(1U, 0U)
#define SH36730X_SCT_DELAY_50US               0x00U   /**< 00: 50 us Delay */
#define SH36730X_SCT_DELAY_100US              0x01U   /**< 01: 100 us Delay */
#define SH36730X_SCT_DELAY_300US              0x02U   /**< 10: 300 us Delay */
#define SH36730X_SCT_DELAY_500US              0x03U   /**< 11: 500 us Delay */

/* -------------------------------------------------------------------------- */
/* 0x0A SCONF7 - System Control Register 7                                    */
/* Access: Read/Write | Reset Value: 0x00                                     */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_SCONF7_RESET_VAL         0x00U

/* Bits 6:4: OVT[2:0] - Hardware Over-Charge Protection Delay (in VADC Scan Cycles) */
#define SH36730X_SCONF7_OVT_POS               4U
#define SH36730X_SCONF7_OVT_MASK              SH36730X_GENMASK(6U, 4U)
#define SH36730X_OVT_DELAY_1_CYCLE            0x00U   /**< 000: 1 VADC Scan Cycle */
#define SH36730X_OVT_DELAY_2_CYCLES           0x01U   /**< 001: 2 VADC Scan Cycles */
#define SH36730X_OVT_DELAY_4_CYCLES           0x02U   /**< 010: 4 VADC Scan Cycles */
#define SH36730X_OVT_DELAY_8_CYCLES           0x03U   /**< 011: 8 VADC Scan Cycles */
#define SH36730X_OVT_DELAY_16_CYCLES          0x04U   /**< 100: 16 VADC Scan Cycles */
#define SH36730X_OVT_DELAY_32_CYCLES          0x05U   /**< 101: 32 VADC Scan Cycles */
#define SH36730X_OVT_DELAY_64_CYCLES          0x06U   /**< 110: 64 VADC Scan Cycles */
#define SH36730X_OVT_DELAY_128_CYCLES         0x07U   /**< 111: 128 VADC Scan Cycles */

/* Bits 3:2: CHS[1:0] - Charge/Discharge State Detection Threshold Voltage */
#define SH36730X_SCONF7_CHS_POS               2U
#define SH36730X_SCONF7_CHS_MASK              SH36730X_GENMASK(3U, 2U)
#define SH36730X_CHS_THRESHOLD_1_4MV          0x00U   /**< 00: 1.4 mV Threshold */
#define SH36730X_CHS_THRESHOLD_3_0MV          0x01U   /**< 01: 3.0 mV Threshold */
#define SH36730X_CHS_THRESHOLD_6_0MV          0x02U   /**< 10: 6.0 mV Threshold */
#define SH36730X_CHS_THRESHOLD_12_0MV         0x03U   /**< 11: 12.0 mV Threshold */

/* Bits 1:0: WDTT[1:0] - Watchdog Timer Overflow Period */
#define SH36730X_SCONF7_WDTT_POS              0U
#define SH36730X_SCONF7_WDTT_MASK             SH36730X_GENMASK(1U, 0U)
#define SH36730X_WDTT_TIMEOUT_30S             0x00U   /**< 00: 30 s Timeout */
#define SH36730X_WDTT_TIMEOUT_10S             0x01U   /**< 01: 10 s Timeout */
#define SH36730X_WDTT_TIMEOUT_2S              0x02U   /**< 10: 2 s Timeout */
#define SH36730X_WDTT_TIMEOUT_500MS           0x03U   /**< 11: 500 ms Timeout */

/* -------------------------------------------------------------------------- */
/* 0x0B & 0x0C SCONF8 / SCONF9 - Hardware OV Threshold                        */
/* Access: Read/Write | Reset Value: SCONF8=0x03, SCONF9=0xFF (OVD=0x3FF)     */
/* Calculation: VOV (mV) = OVD[9:0] * 5.86 mV                                 */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_SCONF8_RESET_VAL         0x03U
#define SH36730X_REG_SCONF9_RESET_VAL         0xFFU

/* SCONF8 (0x0B) Bits 1:0: OVD[9:8] - OV Threshold High 2 bits */
#define SH36730X_SCONF8_OVD_HIGH_POS          0U
#define SH36730X_SCONF8_OVD_HIGH_MASK         SH36730X_GENMASK(1U, 0U)

/* SCONF9 (0x0C) Bits 7:0: OVD[7:0] - OV Threshold Low 8 bits */
#define SH36730X_SCONF9_OVD_LOW_POS           0U
#define SH36730X_SCONF9_OVD_LOW_MASK          SH36730X_GENMASK(7U, 0U)

/**
 * @brief  Split 10-bit OV threshold code (OVD[9:0]) into SCONF8 & SCONF9 byte values.
 */
#define SH36730X_OVD_MAP_TO_SCONF8(ovd_10bit) ((uint8_t)(((uint16_t)(ovd_10bit) >> 8U) & 0x03U))
#define SH36730X_OVD_MAP_TO_SCONF9(ovd_10bit) ((uint8_t)(((uint16_t)(ovd_10bit)) & 0xFFU))

/**
 * @brief  Reassemble 10-bit OV threshold code from SCONF8 & SCONF9 registers.
 */
#define SH36730X_OVD_EXTRACT_10BIT(sconf8, sconf9) \
        ((uint16_t)((((uint16_t)(sconf8) & 0x03U) << 8U) | ((uint16_t)(sconf9) & 0xFFU)))

/* -------------------------------------------------------------------------- */
/* 0x0D SCONF10 - System Control Register 10                                  */
/* Access: Read/Write | Reset Value: 0x00                                     */
/* -------------------------------------------------------------------------- */
#define SH36730X_REG_SCONF10_RESET_VAL        0x00U
#define SH36730X_SCONF10_PIN_POS              0U
#define SH36730X_SCONF10_PIN_MASK             SH36730X_GENMASK(7U, 0U)

/* -------------------------------------------------------------------------- */
/* 0x0E ~ 0x21 CELL1H/L ~ CELL10H/L - Cell Voltages                           */
/* Access: Read-Only | Reset Value: 0x00 | Resolution: 12-bit                 */
/* -------------------------------------------------------------------------- */
#define SH36730X_CELL_H_DATA_POS              0U
#define SH36730X_CELL_H_DATA_MASK             SH36730X_GENMASK(3U, 0U)
#define SH36730X_CELL_L_DATA_POS              0U
#define SH36730X_CELL_L_DATA_MASK             SH36730X_GENMASK(7U, 0U)

/**
 * @brief  Reassemble 12-bit cell voltage ADC raw value (0 ~ 4095).
 */
#define SH36730X_EXTRACT_CELL_ADC(cell_h, cell_l) \
        ((uint16_t)((((uint16_t)(cell_h) & 0x0FU) << 8U) | ((uint16_t)(cell_l) & 0xFFU)))

/* -------------------------------------------------------------------------- */
/* 0x22 ~ 0x25 TS1H/L ~ TS2H/L - External Temperatures                        */
/* Access: Read-Only | Reset Value: 0x00 | Resolution: 12-bit                 */
/* -------------------------------------------------------------------------- */
#define SH36730X_TS_H_DATA_POS                0U
#define SH36730X_TS_H_DATA_MASK               SH36730X_GENMASK(3U, 0U)
#define SH36730X_TS_L_DATA_POS                0U
#define SH36730X_TS_L_DATA_MASK               SH36730X_GENMASK(7U, 0U)

/**
 * @brief  Reassemble 12-bit external temperature TS ADC raw value (0 ~ 4095).
 */
#define SH36730X_EXTRACT_TS_ADC(ts_h, ts_l) \
        ((uint16_t)((((uint16_t)(ts_h) & 0x0FU) << 8U) | ((uint16_t)(ts_l) & 0xFFU)))

/* -------------------------------------------------------------------------- */
/* 0x26 ~ 0x29 TEMP1H/L ~ TEMP2H/L - Internal Temperatures                    */
/* Access: Read-Only | Reset Value: 0x00 | Resolution: 12-bit                 */
/* -------------------------------------------------------------------------- */
#define SH36730X_TEMP_H_DATA_POS              0U
#define SH36730X_TEMP_H_DATA_MASK             SH36730X_GENMASK(3U, 0U)
#define SH36730X_TEMP_L_DATA_POS              0U
#define SH36730X_TEMP_L_DATA_MASK             SH36730X_GENMASK(7U, 0U)

/**
 * @brief  Reassemble 12-bit internal die temperature ADC raw value (0 ~ 4095).
 */
#define SH36730X_EXTRACT_TEMP_ADC(temp_h, temp_l) \
        ((uint16_t)((((uint16_t)(temp_h) & 0x0FU) << 8U) | ((uint16_t)(temp_l) & 0xFFU)))

/* -------------------------------------------------------------------------- */
/* 0x2A ~ 0x2B CURH / CURL - CADC Current Register                            */
/* Access: Read-Only | Reset Value: 0x00 | Resolution: 13-bit / 10-bit         */
/* CURH: Bit 4 is sign CUR[12] (1: Discharging/Negative, 0: Charging/Positive)*/
/*       Bit 3:0 is CUR[11:8]                                                 */
/* CURL: Bit 7:0 is CUR[7:0]                                                  */
/* -------------------------------------------------------------------------- */
#define SH36730X_CURH_SIGN_POS                4U
#define SH36730X_CURH_SIGN_MASK               SH36730X_BIT(SH36730X_CURH_SIGN_POS)
#define SH36730X_CURH_DATA_POS                0U
#define SH36730X_CURH_DATA_MASK               SH36730X_GENMASK(3U, 0U)
#define SH36730X_CURL_DATA_POS                0U
#define SH36730X_CURL_DATA_MASK               SH36730X_GENMASK(7U, 0U)

/**
 * @brief  Extract 13-bit unsigned CADC code (0 ~ 0x1FFF, includes Bit 12 sign).
 */
#define SH36730X_EXTRACT_CUR_RAW13(cur_h, cur_l) \
        ((uint16_t)((((uint16_t)(cur_h) & 0x1FU) << 8U) | ((uint16_t)(cur_l) & 0xFFU)))

/**
 * @brief  Convert 13-bit CADC code to signed 16-bit integer (int16_t) with sign-extension.
 * @note   When Bit 12 is 1 (discharging), sign is extended with 0xE000.
 */
#define SH36730X_EXTRACT_CUR_SIGNED13(cur_h, cur_l) \
        ((int16_t)(((SH36730X_EXTRACT_CUR_RAW13(cur_h, cur_l) & 0x1000U) != 0U) ? \
                (SH36730X_EXTRACT_CUR_RAW13(cur_h, cur_l) | 0xE000U) : \
                SH36730X_EXTRACT_CUR_RAW13(cur_h, cur_l)))

/**
 * @brief  Convert 10-bit CADC code to signed 16-bit integer (int16_t).
 * @note   In 10-bit mode, bits reside in CUR[12:3]; lower 3 bits are ignored.
 */
#define SH36730X_EXTRACT_CUR_SIGNED10(cur_h, cur_l) \
        ((int16_t)(((SH36730X_EXTRACT_CUR_RAW13(cur_h, cur_l) & 0x1000U) != 0U) ? \
                (((SH36730X_EXTRACT_CUR_RAW13(cur_h, cur_l) >> 3U) & 0x03FFU) | 0xFC00U) : \
                ((SH36730X_EXTRACT_CUR_RAW13(cur_h, cur_l) >> 3U) & 0x03FFU)))

/* ========================================================================== */
/* 5. Physical Measurement Conversion Macros & Formulas                        */
/* ========================================================================== */

/**
 * @brief 12-bit VADC full scale counts ($2^{12} = 4096$).
 */
#define SH36730X_VADC_FULL_SCALE              4096.0f

/**
 * @brief 12-bit VADC full scale input voltage: 6.0V (6000 mV).
 */
#define SH36730X_VADC_VREF_MV                 6000.0f
#define SH36730X_VADC_VREF_UV                 6000000UL

/**
 * @brief  Convert 12-bit cell voltage ADC raw value to millivolts (mV).
 * @note   Formula: $V_{\text{cell}}\text{ (mV)} = \frac{\text{CELL} \times 6000}{4096}$
 */
#define SH36730X_RAW_TO_CELL_VOLTAGE_MV(raw_adc) \
        ((uint32_t)(((uint64_t)(raw_adc) * 6000ULL) / 4096ULL))

/**
 * @brief  Convert 12-bit cell voltage ADC raw value to microvolts (uV).
 */
#define SH36730X_RAW_TO_CELL_VOLTAGE_UV(raw_adc) \
        ((uint32_t)(((uint64_t)(raw_adc) * 6000000ULL) / 4096ULL))

/**
 * @brief  Hardware Over-Charge protection step size: 5.86 mV/LSB (5860 uV).
 * @note   Formula: $V_{\text{OV}}\text{ (mV)} = \text{OVD[9:0]} \times 5.86\text{ mV}$
 */
#define SH36730X_OVD_STEP_UV                  5860UL

/**
 * @brief  Maximum raw value for 10-bit OVD register.
 */
#define SH36730X_RAW_OVD_MAX_VAL              0x03FFU

/**
 * @brief  Calculate 10-bit OVD register code from target overcharge threshold voltage in mV.
 */
#define SH36730X_VOLTAGE_MV_TO_OVD(ov_mv) \
        ((uint16_t)(((uint32_t)(ov_mv) * 1000UL + (SH36730X_OVD_STEP_UV / 2UL)) / SH36730X_OVD_STEP_UV))

/**
 * @brief  Calculate overcharge threshold voltage in mV from 10-bit OVD code.
 */
#define SH36730X_OVD_TO_VOLTAGE_MV(ovd_val) \
        ((uint32_t)(((uint32_t)(ovd_val) * SH36730X_OVD_STEP_UV) / 1000UL))

/**
 * @brief  Convert 12-bit TS ADC raw value to external NTC thermistor resistance in Ohms ($\Omega$).
 * @note   Formula: $R_{\text{TS}}\text{ (}\Omega\text{)} = \frac{\text{TS}}{4096 - \text{TS}} \times R_{\text{pullup}}$
 * @param  raw_ts      12-bit TS ADC count (0 ~ 4095).
 * @param  pullup_ohm  Internal pull-up resistor value (typically 10000 $\Omega$).
 */
#define SH36730X_RAW_TO_TS_RESISTANCE_OHM(raw_ts, pullup_ohm) \
        (((raw_ts) >= 4096U) ? 0xFFFFFFFFUL : \
        (uint32_t)(((uint64_t)(raw_ts) * (uint64_t)(pullup_ohm)) / (4096ULL - (uint64_t)(raw_ts))))

/**
 * @brief  Convert internal temperature ADC raw value to Celsius (°C, float).
 * @note   Formula: T (°C) = 0.17 * RAW - 270.0
 */
#define SH36730X_RAW_TO_INTERNAL_TEMP_C_FLOAT(raw_temp) \
        (((float)(raw_temp) * 0.17f) - 270.0f)

/**
 * @brief  Convert internal temperature ADC raw value to deci-degrees (0.1°C, int32_t).
 * @note   E.g., 250 represents 25.0 °C, -400 represents -40.0 °C.
 *         Formula: T_0_1C = 17 * RAW - 2700
 */
#define SH36730X_RAW_TO_INTERNAL_TEMP_0_1C_INT(raw_temp) \
        (((int32_t)(raw_temp) * 17L) - 2700L)

/**
 * @brief 13-bit CADC full scale denominator count (8192 LSB).
 */
#define SH36730X_CADC_13BIT_FULL_SCALE        8192.0f

/**
 * @brief  Full scale denominator constants for different CADC RSNS settings.
 */
#define SH36730X_CADC_DIV_50MV                65536.0f
#define SH36730X_CADC_DIV_100MV               32768.0f
#define SH36730X_CADC_DIV_200MV               16384.0f
#define SH36730X_CADC_DIV_400MV               8192.0f

/**
 * @brief  Calculate CADC current in mA from raw code, full scale denominator, and sense resistor.
 * @param  cur_code      Raw CADC code (signed 13-bit).
 * @param  div_const     Full scale denominator for the current range.
 * @param  rsense_ohm    Sense resistor value in Ohms.
 * @return Current in mA (float).
 */
#define SH36730X_CADC_CALC_CURRENT_MA(cur_code, div_const, rsense_ohm) \
        (((1000.0f * (float)(cur_code)) / (div_const)) / (rsense_ohm))

#ifdef __cplusplus
}
#endif

#endif /* SH36730X_REG_H */


