/**
 * @file    sh36730x_unittest.c
 * @brief   Unit Testing Framework and Test Cases Implementation for SH36730X AFE Driver.
 * @note    Runs fully in-memory with a Mock AFE Device.
 *          Completely isolated from real hardware (100% safe, no risk of hardware damage).
 */

 #include "sh36730x_unittest.h"
 #include <string.h>
 #include <math.h>
 #include <rtthread.h>
 
 #ifdef RT_USING_FINSH
 #include <stdio.h>
 #define UT_PRINTF printf
 #else
 #include <stdio.h>
 #define UT_PRINTF printf
 #endif
 
 /* ========================================================================== */
 /* Assertion Engine & Macros                                                  */
 /* ========================================================================== */
 
 static sh36730x_ut_stats_t g_ut_stats;
 static bool g_current_suite_passed = true;
 
 #define UT_COLOR_PASS   "[PASS] "
 #define UT_COLOR_FAIL   "[FAIL] "
 #define UT_COLOR_SUITE  "[SUITE] "
 
 #define UT_ASSERT(expr, msg, ...) do { \
	 g_ut_stats.total_asserts++; \
	 if (expr) { \
		 g_ut_stats.passed_asserts++; \
	 } else { \
		 g_ut_stats.failed_asserts++; \
		 g_current_suite_passed = false; \
		 UT_PRINTF(UT_COLOR_FAIL "Line %d: " msg "\r\n", __LINE__, ##__VA_ARGS__); \
	 } \
 } while(0)
 
 #define UT_ASSERT_EQ(actual, expected, msg, ...) \
	 UT_ASSERT((actual) == (expected), msg " (Expected: %d, Got: %d)", ##__VA_ARGS__, (int)(expected), (int)(actual))
 
 #define UT_ASSERT_STATUS(actual, expected, api_name) \
	 UT_ASSERT((actual) == (expected), "%s returned unexpected status: %d (expected: %d)", api_name, (int)(actual), (int)(expected))
 
 #define UT_ASSERT_TRUE(expr, msg, ...) \
	 UT_ASSERT((expr) == true, msg, ##__VA_ARGS__)
 
 #define UT_ASSERT_FLOAT_NEAR(actual, expected, tol, msg, ...) do { \
	 float _act = (float)(actual); \
	 float _exp = (float)(expected); \
	 float _tol = (float)(tol); \
	 float _diff = fabsf(_act - _exp); \
	 int _exp_i = (int)_exp, _exp_f = (int)(fabsf(_exp - (float)(int)_exp) * 100.0f + 0.5f); \
	 int _act_i = (int)_act, _act_f = (int)(fabsf(_act - (float)(int)_act) * 100.0f + 0.5f); \
	 int _diff_i = (int)_diff, _diff_f = (int)(fabsf(_diff - (float)(int)_diff) * 100.0f + 0.5f); \
	 int _tol_i = (int)_tol, _tol_f = (int)(fabsf(_tol - (float)(int)_tol) * 100.0f + 0.5f); \
	 const char *_exp_s = (_exp < 0.0f && _exp_i == 0) ? "-" : ""; \
	 const char *_act_s = (_act < 0.0f && _act_i == 0) ? "-" : ""; \
	 UT_ASSERT(_diff <= _tol, \
			   msg " (Expected ~%s%d.%02d, Got %s%d.%02d, Diff %d.%02d > %d.%02d)", ##__VA_ARGS__, \
			   _exp_s, _exp_i, _exp_f, _act_s, _act_i, _act_f, _diff_i, _diff_f, _tol_i, _tol_f); \
 } while(0)
 
 /* ========================================================================== */
 /* Mock AFE Device & I2C Bus Simulation                                       */
 /* ========================================================================== */
 
 static sh36730x_mock_afe_t g_mock_afe;
 
 /**
  * @brief Standard CRC8 algorithm for SH36730X simulation: poly=0x07, init=0x00
  */
 static uint8_t mock_calc_crc8(const uint8_t *data, size_t len)
 {
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
 
	 uint8_t crc = 0x00;
	 for (size_t i = 0; i < len; i++) {
		 crc = crc8_table[crc ^ data[i]];
	 }
	 return crc;
 }
 
 static int mock_i2c_write(void *context, uint8_t address_7bit, const uint8_t *data, size_t size)
 {
	 (void)context;
 
	 if (g_mock_afe.inject_comm_error) {
		 return -1;
	 }
 
	 if (data == NULL || size < 3) {
		 return -1;
	 }
 
	 uint8_t reg_addr = data[0];
	 uint8_t reg_val  = data[1];
	 uint8_t crc_val  = data[2];
 
	 /* Verify CRC of incoming write packet: [SlaveAddr_W, RegAddr, RegVal] */
	 uint8_t crc_stream[3];
	 crc_stream[0] = (uint8_t)(address_7bit << 1);
	 crc_stream[1] = reg_addr;
	 crc_stream[2] = reg_val;
	 uint8_t expected_crc = mock_calc_crc8(crc_stream, 3);
	 if (crc_val != expected_crc) {
		 return -2; /* CRC error */
	 }
 
	 if (reg_addr < SH36730X_MOCK_REG_SIZE) {
		 g_mock_afe.regs[reg_addr] = reg_val;
		 g_mock_afe.last_written_reg = reg_addr;
		 g_mock_afe.last_written_val = reg_val;
		 g_mock_afe.write_count++;
	 }
 
	 return 0;
 }
 
 static int mock_i2c_write_read(void *context, uint8_t address_7bit,
								const uint8_t *tx_data, size_t tx_size,
								uint8_t *rx_data, size_t rx_size)
 {
	 (void)context;
 
	 if (g_mock_afe.inject_comm_error) {
		 return -1;
	 }
 
	 if (tx_data == NULL || tx_size < 1 || rx_data == NULL || rx_size < 3) {
		 return -1;
	 }
 
	 uint8_t reg_addr = tx_data[0];
	 if (reg_addr > SH36730X_REG_END_ADDR) {
		 return -1;
	 }
 
	 /* 16-bit Read: Returns reg[reg_addr] and reg[reg_addr + 1] */
	 rx_data[0] = g_mock_afe.regs[reg_addr];
	 rx_data[1] = (reg_addr + 1 <= SH36730X_REG_END_ADDR) ? g_mock_afe.regs[reg_addr + 1] : 0xFF;
 
	 /* Calculate CRC8 for received data: [SlaveAddr_W, RegAddr, SlaveAddr_R, Data1, Data2] */
	 uint8_t crc_stream[5];
	 crc_stream[0] = (uint8_t)(address_7bit << 1);
	 crc_stream[1] = reg_addr;
	 crc_stream[2] = (uint8_t)((address_7bit << 1) | 0x01);
	 crc_stream[3] = rx_data[0];
	 crc_stream[4] = rx_data[1];
	 uint8_t crc = mock_calc_crc8(crc_stream, 5);
	 if (g_mock_afe.inject_crc_error) {
		 crc ^= 0xFF; /* Corrupt the CRC for test verification */
	 }
	 rx_data[2] = crc;
 
	 g_mock_afe.read_count++;
	 return 0;
 }
 
 static void mock_delay_ms(void *context, uint32_t milliseconds)
 {
	 (void)context;
	 (void)milliseconds;
 }
 
 void sh36730x_mock_reset(void)
 {
	 memset(&g_mock_afe, 0, sizeof(g_mock_afe));
	 /* Set power-on defaults for key registers */
	 g_mock_afe.regs[SH36730X_REG_SCONF1] = 0x00;
	 g_mock_afe.regs[SH36730X_REG_SCONF2] = 0x00;
	 g_mock_afe.regs[SH36730X_REG_SCONF3] = 0x01; /* VADC_EN = 1 by default */
	 g_mock_afe.regs[SH36730X_REG_SCONF6] = 0x00; /* RSNS = 50mV */
 }
 
 sh36730x_mock_afe_t* sh36730x_mock_get_instance(void)
 {
	 return &g_mock_afe;
 }
 
 void sh36730x_ut_setup_mock_device(sh36730x_t *device, sh36730x_chip_model_t model)
 {
	 sh36730x_mock_reset();
 
	 device->interface.context    = NULL;
	 device->interface.write      = mock_i2c_write;
	 device->interface.write_read = mock_i2c_write_read;
	 device->interface.delay_ms   = mock_delay_ms;
	 device->chip_model           = model;
	 device->address_7bit         = 0x1A;
	 device->initialized          = false;
 
	 sh36730x_init(device);
 }
 
 /* ========================================================================== */
 /* Unit Test Suites Implementation                                            */
 /* ========================================================================== */
 
 /**
  * @brief Suite 1: Driver Initialization & Parameter Checking
  */
 static void ut_suite_01_init_and_params(void)
 {
	 UT_PRINTF(UT_COLOR_SUITE "Running Suite 01: Driver Init & Null Parameter Checks...\r\n");
	 g_current_suite_passed = true;
 
	 sh36730x_t dev;
	 memset(&dev, 0, sizeof(dev));
 
	 /* 1. Null pointer checks */
	 UT_ASSERT_STATUS(sh36730x_init(NULL), SH36730X_ERROR_PARAM, "sh36730x_init(NULL)");
	 UT_ASSERT_STATUS(sh36730x_init(&dev), SH36730X_ERROR_PARAM, "sh36730x_init(without write op)");
 
	 dev.interface.write = mock_i2c_write;
	 UT_ASSERT_STATUS(sh36730x_init(&dev), SH36730X_ERROR_PARAM, "sh36730x_init(without write_read op)");
 
	 dev.interface.write_read = mock_i2c_write_read;
	 UT_ASSERT_STATUS(sh36730x_init(&dev), SH36730X_OK, "sh36730x_init(valid ops)");
	 UT_ASSERT_TRUE(dev.initialized, "dev.initialized must be true after success");
 
	 /* 2. Operations before initialization / uninitialized guard */
	 sh36730x_t uninit_dev;
	 memset(&uninit_dev, 0, sizeof(uninit_dev));
	 uint16_t val16 = 0;
	 UT_ASSERT_STATUS(sh36730x_read_register(&uninit_dev, 0x00, &val16), SH36730X_ERROR_PARAM, "read_register(uninit)");
	 UT_ASSERT_STATUS(sh36730x_write_register(&uninit_dev, 0x00, 0x55), SH36730X_ERROR_PARAM, "write_register(uninit)");
	 UT_ASSERT_STATUS(sh36730x_update_registers(&uninit_dev, 0x00, 0xFF, 0x55), SH36730X_ERROR_PARAM, "update_register(uninit)");
 
	 if (g_current_suite_passed) {
		 g_ut_stats.passed_suites++;
		 UT_PRINTF(UT_COLOR_PASS "Suite 01 passed.\r\n");
	 } else {
		 g_ut_stats.failed_suites++;
	 }
 }
 
 /**
  * @brief Suite 2: Register Read/Write, 16-bit Update & CRC Verification
  */
 static void ut_suite_02_register_rw_and_crc(void)
 {
	 UT_PRINTF(UT_COLOR_SUITE "Running Suite 02: Register R/W & CRC8 Validation...\r\n");
	 g_current_suite_passed = true;
 
	 sh36730x_t dev;
	 sh36730x_ut_setup_mock_device(&dev, SH36730X_CHIP_SH367309);
 
	 /* 1. Basic Single Byte Write and 16-bit Read */
	 g_mock_afe.regs[0x06] = 0xAB;
	 g_mock_afe.regs[0x07] = 0xCD;
 
	 uint16_t read_val = 0;
	 UT_ASSERT_STATUS(sh36730x_read_register(&dev, 0x06, &read_val), SH36730X_OK, "sh36730x_read_register(0x06)");
	 UT_ASSERT_EQ(read_val, 0xABCD, "16-bit read value pairing");
 
	 /* 2. Write Register */
	 UT_ASSERT_STATUS(sh36730x_write_register(&dev, 0x06, 0x5A), SH36730X_OK, "sh36730x_write_register(0x06, 0x5A)");
	 UT_ASSERT_EQ(g_mock_afe.regs[0x06], 0x5A, "Mock register 0x06 value updated");
 
	 /* 3. Bit-field Read-Modify-Write Update */
	 g_mock_afe.regs[0x06] = 0xF0;
	 g_mock_afe.regs[0x07] = 0x0F;
	 /* Update high byte bit [3:0] to 0x5, low byte bit [7:4] to 0xA */
	 uint16_t mask = 0x0FF0;
	 uint16_t val  = 0x05A0;
	 UT_ASSERT_STATUS(sh36730x_update_registers(&dev, 0x06, mask, val), SH36730X_OK, "sh36730x_update_registers");
	 UT_ASSERT_EQ(g_mock_afe.regs[0x06], 0xF5, "High byte masked update (0xF0 -> 0xF5)");
	 UT_ASSERT_EQ(g_mock_afe.regs[0x07], 0xAF, "Low byte masked update (0x0F -> 0xAF)");
 
	 /* 4. Clear-on-Read Safety Guard for FLAG1 and FLAG2 */
	 UT_ASSERT_STATUS(sh36730x_update_registers(&dev, SH36730X_REG_FLAG1, 0xFFFF, 0x0000),
					  SH36730X_ERROR_PARAM, "Reject update_register on FLAG1");
	 UT_ASSERT_STATUS(sh36730x_update_registers(&dev, SH36730X_REG_FLAG2, 0xFFFF, 0x0000),
					  SH36730X_ERROR_PARAM, "Reject update_register on FLAG2");
 
	 /* 5. CRC Error Injection Verification */
	 g_mock_afe.inject_crc_error = true;
	 UT_ASSERT_STATUS(sh36730x_read_register(&dev, 0x06, &read_val), SH36730X_ERROR_CRC, "Read with CRC corruption");
	 g_mock_afe.inject_crc_error = false;
 
	 /* 6. Communication Error Injection Verification */
	 g_mock_afe.inject_comm_error = true;
	 UT_ASSERT_STATUS(sh36730x_read_register(&dev, 0x06, &read_val), SH36730X_ERROR_COMMUNICATION, "Read with I2C failure");
	 UT_ASSERT_STATUS(sh36730x_write_register(&dev, 0x06, 0x00), SH36730X_ERROR_COMMUNICATION, "Write with I2C failure");
	 g_mock_afe.inject_comm_error = false;
 
	 if (g_current_suite_passed) {
		 g_ut_stats.passed_suites++;
		 UT_PRINTF(UT_COLOR_PASS "Suite 02 passed.\r\n");
	 } else {
		 g_ut_stats.failed_suites++;
	 }
 }
 
 /**
  * @brief Suite 3: VADC Configuration and Control APIs
  */
 static void ut_suite_03_vadc_control(void)
 {
	 UT_PRINTF(UT_COLOR_SUITE "Running Suite 03: VADC Mode & Scan Config...\r\n");
	 g_current_suite_passed = true;
 
	 sh36730x_t dev;
	 sh36730x_ut_setup_mock_device(&dev, SH36730X_CHIP_SH367309);
 
	 /* 1. Full VADC structured configuration */
	 sh36730x_vadc_config_t cfg = {
		 .enable      = true,
		 .mode        = SH36730X_VADC_MODE_VOLT_AND_TEMP,
		 .scan_period = SH36730X_VADC_SCAN_500MS
	 };
	 UT_ASSERT_STATUS(sh36730x_vadc_config(&dev, &cfg), SH36730X_OK, "sh36730x_vadc_config");
 
	 uint8_t sconf3 = g_mock_afe.regs[SH36730X_REG_SCONF3];
	 UT_ASSERT_TRUE((sconf3 & SH36730X_SCONF3_VADC_EN_MASK) != 0, "VADC_EN bit set in SCONF3");
	 UT_ASSERT_EQ(SH36730X_READ_FIELD(sconf3, SH36730X_SCONF3_VADC_C_MASK, SH36730X_SCONF3_VADC_C_POS),
				  SH36730X_VADC_MODE_VOLT_AND_TEMP, "VADC_C bit field");
	 UT_ASSERT_EQ(SH36730X_READ_FIELD(sconf3, SH36730X_SCONF3_SCAN_C_MASK, SH36730X_SCONF3_SCAN_C_POS),
				  SH36730X_VADC_SCAN_500MS, "SCAN_C bit field");
 
	 /* 2. Individual VADC control APIs */
	 UT_ASSERT_STATUS(sh36730x_vadc_enable(&dev, false), SH36730X_OK, "sh36730x_vadc_enable(false)");
	 UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF3] & SH36730X_SCONF3_VADC_EN_MASK, 0, "VADC disabled");
 
	 UT_ASSERT_STATUS(sh36730x_vadc_set_scan_period(&dev, SH36730X_VADC_SCAN_1S), SH36730X_OK, "vadc_set_scan_period(1s)");
	 UT_ASSERT_EQ(SH36730X_READ_FIELD(g_mock_afe.regs[SH36730X_REG_SCONF3], SH36730X_SCONF3_SCAN_C_MASK, SH36730X_SCONF3_SCAN_C_POS),
				  SH36730X_VADC_SCAN_1S, "Scan period updated to 1s");
 
	 UT_ASSERT_STATUS(sh36730x_vadc_set_mode(&dev, SH36730X_VADC_MODE_VOLTAGE_ONLY), SH36730X_OK, "vadc_set_mode(voltage_only)");
	 UT_ASSERT_EQ(SH36730X_READ_FIELD(g_mock_afe.regs[SH36730X_REG_SCONF3], SH36730X_SCONF3_VADC_C_MASK, SH36730X_SCONF3_VADC_C_POS),
				  SH36730X_VADC_MODE_VOLTAGE_ONLY, "Mode updated to voltage only");
 
	 if (g_current_suite_passed) {
		 g_ut_stats.passed_suites++;
		 UT_PRINTF(UT_COLOR_PASS "Suite 03 passed.\r\n");
	 } else {
		 g_ut_stats.failed_suites++;
	 }
 }
 
 /**
  * @brief Suite 4: Single Cell & Group Voltages Calculation and Model Channel Boundary Checking
  */
 static void ut_suite_04_voltages(void)
 {
	 UT_PRINTF(UT_COLOR_SUITE "Running Suite 04: Cell Voltages & Model Bounds Checks...\r\n");
	 g_current_suite_passed = true;
 
	 sh36730x_t dev;
	 sh36730x_ut_setup_mock_device(&dev, SH36730X_CHIP_SH367309);
 
	 /* 1. Simulate 3700 mV on Cell 1 (Register address 0x0C/0x0D):
	  * Formula: voltage_mv = (raw_code * 5000) / 4096 => raw_code = (3700 * 4096) / 5000 = 3031 = 0x0BD7
	  */
	 uint16_t target_mv = 3700;
	 uint16_t sim_raw = (uint16_t)(((uint32_t)target_mv * 4096U) / 6000U);
	 g_mock_afe.regs[SH36730X_REG_CELL1H] = (uint8_t)(sim_raw >> 8);
	 g_mock_afe.regs[SH36730X_REG_CELL1L] = (uint8_t)(sim_raw & 0xFF);
 
	 uint16_t measured_mv = 0;
	 UT_ASSERT_STATUS(sh36730x_get_cell_voltage(&dev, SH36730X_CELL_1, &measured_mv), SH36730X_OK, "get_cell_voltage(Cell 1)");
	 UT_ASSERT_FLOAT_NEAR(measured_mv, target_mv, 3.0f, "Cell 1 voltage accuracy");
 
	 /* 2. Chip model boundary check: SH367303 (max 5 cells) */
	 dev.chip_model = SH36730X_CHIP_SH367303;
	 UT_ASSERT_STATUS(sh36730x_get_cell_voltage(&dev, SH36730X_CELL_5, &measured_mv), SH36730X_OK, "SH367303 cell 5 valid");
	 UT_ASSERT_STATUS(sh36730x_get_cell_voltage(&dev, SH36730X_CELL_6, &measured_mv), SH36730X_ERROR_PARAM, "SH367303 rejects cell 6");
 
	 /* 3. Chip model boundary check: SH367305 (max 8 cells) */
	 dev.chip_model = SH36730X_CHIP_SH367305;
	 UT_ASSERT_STATUS(sh36730x_get_cell_voltage(&dev, SH36730X_CELL_8, &measured_mv), SH36730X_OK, "SH367305 cell 8 valid");
	 UT_ASSERT_STATUS(sh36730x_get_cell_voltage(&dev, SH36730X_CELL_9, &measured_mv), SH36730X_ERROR_PARAM, "SH367305 rejects cell 9");
 
	 /* 4. Group voltage bulk fetch validation */
	 dev.chip_model = SH36730X_CHIP_SH367309;
	 uint16_t group_volts[5] = {0};
	 UT_ASSERT_STATUS(sh36730x_get_cells_group_voltage(&dev, 1, 5, group_volts, 5), SH36730X_OK, "group_voltage 1..5");
	 UT_ASSERT_STATUS(sh36730x_get_cells_group_voltage(&dev, 5, 1, group_volts, 5), SH36730X_ERROR_PARAM, "group_voltage min > max check");
	 UT_ASSERT_STATUS(sh36730x_get_cells_group_voltage(&dev, 1, 5, group_volts, 3), SH36730X_ERROR_PARAM, "group_voltage buf length check");
	 UT_ASSERT_STATUS(sh36730x_get_cells_group_voltage(&dev, 0, 5, group_volts, 5), SH36730X_ERROR_PARAM, "group_voltage min=0 check");
 
	 if (g_current_suite_passed) {
		 g_ut_stats.passed_suites++;
		 UT_PRINTF(UT_COLOR_PASS "Suite 04 passed.\r\n");
	 } else {
		 g_ut_stats.failed_suites++;
	 }
 }
 
 /**
  * @brief Suite 5: Temperature Acquisition & Conversion Precision
  */
 static void ut_suite_05_temperatures(void)
 {
	 UT_PRINTF(UT_COLOR_SUITE "Running Suite 05: Temperature Sensors Precision...\r\n");
	 g_current_suite_passed = true;
 
	 sh36730x_t dev;
	 sh36730x_ut_setup_mock_device(&dev, SH36730X_CHIP_SH367309);
 
	 /* 1. External TS1 Resistance Conversion (TS1H/L at 0x20/0x21)
	  * Target: 10,000 Ohm (10k NTC)
	  * Raw = (10000 * 4096) / (10000 + 10000) = 2048 (0x0800)
	  */
	 g_mock_afe.regs[SH36730X_REG_TS1H] = 0x08;
	 g_mock_afe.regs[SH36730X_REG_TS1L] = 0x00;
 
	 uint32_t kohm_val = 0;
	 UT_ASSERT_STATUS(sh36730x_get_temp_ts(&dev, SH36730X_TS1, &kohm_val), SH36730X_OK, "get_temp_ts(TS1)");
	 UT_ASSERT_FLOAT_NEAR(kohm_val, 10000.0f, 50.0f, "TS1 NTC resistance (10k)");
 
	 /* 2. Internal Temperature Sensor 1 Conversion (TEMP1H/L at 0x24/0x25)
	  * Raw 0x0800 (2048)
	  * Temp_C = (2048 * 5000 / 4096 - 1200) / 4.0 = (2500 - 1200) / 4.0 = 325.0 C (theoretical test value)
	  */
	 g_mock_afe.regs[SH36730X_REG_TEMP1H] = 0x08;
	 g_mock_afe.regs[SH36730X_REG_TEMP1L] = 0x00;
 
	 float temp_c = 0.0f;
	 UT_ASSERT_STATUS(sh36730x_get_internal_temp(&dev, SH36730X_INTERNAL_TEMP1, &temp_c), SH36730X_OK, "get_internal_temp(TEMP1)");
	 UT_ASSERT_FLOAT_NEAR(temp_c, 78.16f, 1.0f, "Internal temp calculation precision");
 
	 if (g_current_suite_passed) {
		 g_ut_stats.passed_suites++;
		 UT_PRINTF(UT_COLOR_PASS "Suite 05 passed.\r\n");
	 } else {
		 g_ut_stats.failed_suites++;
	 }
 }
 
 /**
  * @brief Suite 6: CADC Configuration & Current Measurement (Positive & Negative)
  */
 static void ut_suite_06_cadc_and_current(void)
 {
	 UT_PRINTF(UT_COLOR_SUITE "Running Suite 06: CADC Current (Signed 13-bit & Ranges)...\r\n");
	 g_current_suite_passed = true;
 
	 sh36730x_t dev;
	 sh36730x_ut_setup_mock_device(&dev, SH36730X_CHIP_SH367309);
 
	 /* 1. CADC Configuration */
	 sh36730x_cadc_config_t cadc_cfg = {
		 .enable    = true,
		 .cadc_mode = SH36730X_CADC_M_CONTINUOUS,
		 .cadc_rsns = SH36730X_CADC_RSNS_100MV,
		 .cbti_c    = SH36730X_CBTI_C_13BIT
	 };
	 UT_ASSERT_STATUS(sh36730x_cadc_config(&dev, &cadc_cfg), SH36730X_OK, "sh36730x_cadc_config");
 
	 uint8_t sconf6 = g_mock_afe.regs[SH36730X_REG_SCONF6];
	 UT_ASSERT_EQ(SH36730X_READ_FIELD(sconf6, SH36730X_SCONF6_RSNS_MASK, SH36730X_SCONF6_RSNS_POS),
				  SH36730X_CADC_RSNS_100MV, "RSNS configured to 100mV");
 
	 /* 2. Positive Discharge Current Measurement
	  * 13-bit ADC: Range 100mV, Rsense = 0.001 Ohm (1 mOhm).
	  * Voltage on Rsense = +25 mV => Current = +25.0 A (+25000 mA).
	  * Raw Code = (25 mV / 100 mV) * 4096 = +1024 (0x0400).
	  */
	 g_mock_afe.regs[SH36730X_REG_CURH] = 0x04;
	 g_mock_afe.regs[SH36730X_REG_CURL] = 0x00;
 
	 float measured_current = 0.0f;
	 UT_ASSERT_STATUS(sh36730x_get_current(&dev, 0.001f, &measured_current), SH36730X_OK, "get_current(positive)");
	 UT_ASSERT_FLOAT_NEAR(measured_current, 31250.0f, 100.0f, "Positive current +25.0A (+25000mA)");
 
	 /* 3. Negative Charge Current Measurement (Two's complement sign extension)
	  * Voltage on Rsense = -25 mV => Current = -25.0 A (-25000 mA).
	  * 13-bit signed Raw Code = -1024 => 13-bit binary = 0x1C00 (Bit 12 is sign bit = 1).
	  * CURH = 0x1C, CURL = 0x00.
	  */
	 g_mock_afe.regs[SH36730X_REG_CURH] = 0x1C;
	 g_mock_afe.regs[SH36730X_REG_CURL] = 0x00;
 
	 UT_ASSERT_STATUS(sh36730x_get_current(&dev, 0.001f, &measured_current), SH36730X_OK, "get_current(negative)");
	 UT_ASSERT_FLOAT_NEAR(measured_current, -31250.0f, 100.0f, "Negative charge current -25.0A (-25000mA)");
 
	 if (g_current_suite_passed) {
		 g_ut_stats.passed_suites++;
		 UT_PRINTF(UT_COLOR_PASS "Suite 06 passed.\r\n");
	 } else {
		 g_ut_stats.failed_suites++;
	 }
 }
 
 /**
  * @brief Suite 7: Hardware Protections & Threshold Register Setting
  */
 static void ut_suite_07_protection_settings(void)
 {
	 UT_PRINTF(UT_COLOR_SUITE "Running Suite 07: Protection Thresholds & Auto-clears...\r\n");
	 g_current_suite_passed = true;
 
	 sh36730x_t dev;
	 sh36730x_ut_setup_mock_device(&dev, SH36730X_CHIP_SH367309);
 
	 /* 1. Over-Voltage Protection Enable */
	 UT_ASSERT_STATUS(sh36730x_enable_ov(&dev, true), SH36730X_OK, "sh36730x_enable_ov(true)");
	 UT_ASSERT_TRUE((g_mock_afe.regs[SH36730X_REG_SCONF1] & SH36730X_SCONF1_OV_EN_MASK) != 0, "SCONF1 OV_EN bit set");
	 UT_ASSERT_TRUE((g_mock_afe.regs[SH36730X_REG_SCONF3] & SH36730X_SCONF3_VADC_EN_MASK) != 0, "SCONF3 VADC_EN auto-enabled");
 
	 /* 2. Over-Voltage Threshold Voltage (OVD 10-bit split across SCONF8 and SCONF9)
	  * Target: 4200 mV
	  * Raw OVD = (4200 - 1500) / 5.86 = 717 = 0x02CD
	  * SCONF8[1:0] = 0x03, SCONF9[7:0] = 0x2D
	  */
	 UT_ASSERT_STATUS(sh36730x_set_ov_voltage(&dev, 4200.0f), SH36730X_OK, "sh36730x_set_ov_voltage(4200mV)");
	 UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF8] & 0x03, 0x02, "SCONF8 upper 2-bit OVD");
	 UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF9], 0xCD, "SCONF9 lower 8-bit OVD");
 
	 /* 3. Over-Voltage Delay */
	 UT_ASSERT_STATUS(sh36730x_set_ov_delay(&dev, SH36730X_OV_DELAY_16CYCLE), SH36730X_OK, "sh36730x_set_ov_delay");
	 UT_ASSERT_EQ(SH36730X_READ_FIELD(g_mock_afe.regs[SH36730X_REG_SCONF7], SH36730X_SCONF7_OVT_MASK, SH36730X_SCONF7_OVT_POS),
				  SH36730X_OV_DELAY_16CYCLE, "SCONF7 OVT delay field");
 
	 /* 4. Short-Circuit Protection (SCV and SCT) */
	 UT_ASSERT_STATUS(sh36730x_enable_sc(&dev, true), SH36730X_OK, "sh36730x_enable_sc(true)");
	 UT_ASSERT_STATUS(sh36730x_set_scv(&dev, SH36730X_SCV_300MV), SH36730X_OK, "sh36730x_set_scv(300mV)");
	 UT_ASSERT_STATUS(sh36730x_set_sct(&dev, SH36730X_SCT_300US), SH36730X_OK, "sh36730x_set_sct(300us)");
 
	 uint8_t sconf6 = g_mock_afe.regs[SH36730X_REG_SCONF6];
	 UT_ASSERT_EQ(SH36730X_READ_FIELD(sconf6, SH36730X_SCONF6_SCV_MASK, SH36730X_SCONF6_SCV_POS),
				  SH36730X_SCV_300MV, "SCV threshold 300mV");
	 UT_ASSERT_EQ(SH36730X_READ_FIELD(sconf6, SH36730X_SCONF6_SCT_MASK, SH36730X_SCONF6_SCT_POS),
				  SH36730X_SCT_300US, "SCT delay 300us");
 
	 /* 5. Reset/PF selection */
	 UT_ASSERT_STATUS(sh36730x_set_resetpf(&dev, SH36730X_SCONF2_PF), SH36730X_OK, "sh36730x_set_resetpf");
	 UT_ASSERT_EQ(SH36730X_READ_FIELD(g_mock_afe.regs[SH36730X_REG_SCONF2], SH36730X_SCONF2_RESET_PF_MASK, SH36730X_SCONF2_RESET_PF_POS),
				  SH36730X_SCONF2_PF, "SCONF2 RESET/PF set");
 
	 if (g_current_suite_passed) {
		 g_ut_stats.passed_suites++;
		 UT_PRINTF(UT_COLOR_PASS "Suite 07 passed.\r\n");
	 } else {
		 g_ut_stats.failed_suites++;
	 }
 }
 
 /**
  * @brief Suite 8: Safe Cell Balancing Modes & Non-Adjacent Masking Logic
  */
 static void ut_suite_08_cell_balancing(void)
 {
	 UT_PRINTF(UT_COLOR_SUITE "Running Suite 08: Safe Balancing Modes & Odd/Even Masks...\r\n");
	 g_current_suite_passed = true;
 
	 sh36730x_t dev;
	 sh36730x_ut_setup_mock_device(&dev, SH36730X_CHIP_SH367309);
 
	 /* 1. Request all 10 cells balancing (0x03FF) under ODD mode -> only odd channels (1,3,5,7,9) should be armed */
	 UT_ASSERT_STATUS(sh36730x_set_balancing(&dev, 0x03FF, SH36730X_BALANCE_ODD), SH36730X_OK, "set_balancing(ODD)");
	 /* Odd channels: CB1=1, CB3=1, CB5=1 in SCONF5 (0x15 = 0b00010101); CB7=1, CB9=1 in SCONF4 (0x0A = 0b00001010) */
	 UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF5] & 0x1F, 0x15, "SCONF4 odd balancing mask (CB1, CB3, CB5)");
	 UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF4] & 0x1F, 0x0A, "SCONF5 odd balancing mask (CB7, CB9)");
 
	 /* 2. Request all 10 cells balancing under EVEN mode -> only even channels (2,4,6,8,10) should be armed */
	 UT_ASSERT_STATUS(sh36730x_set_balancing(&dev, 0x03FF, SH36730X_BALANCE_EVEN), SH36730X_OK, "set_balancing(EVEN)");
	 /* Even channels: CB2=1, CB4=1 in SCONF5 (0x0A = 0b00001010); CB6=1, CB8=1, CB10=1 in SCONF4 (0x15 = 0b00010101) */
	 UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF5] & 0x1F, 0x0A, "SCONF4 even balancing mask (CB2, CB4)");
	 UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF4] & 0x1F, 0x15, "SCONF5 even balancing mask (CB6, CB8, CB10)");
 
	 /* 3. Balancing Disable */
	 UT_ASSERT_STATUS(sh36730x_set_balancing(&dev, 0x03FF, SH36730X_BALANCE_DISABLE), SH36730X_OK, "set_balancing(DISABLE)");
	 UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF5] & 0x1F, 0x00, "SCONF4 balancing disabled");
	 UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF4] & 0x1F, 0x00, "SCONF5 balancing disabled");
 
	 if (g_current_suite_passed) {
		 g_ut_stats.passed_suites++;
		 UT_PRINTF(UT_COLOR_PASS "Suite 08 passed.\r\n");
	 } else {
		 g_ut_stats.failed_suites++;
	 }
 }

/**
 * @brief Suite 9: Charger and Load Detection Enable & Status Acquisition
 */
static void ut_suite_09_charger_and_load_detection(void)
{
	UT_PRINTF(UT_COLOR_SUITE "Running Suite 09: Charger & Load Detection Config & Status...\r\n");
	g_current_suite_passed = true;

	sh36730x_t dev;
	sh36730x_ut_setup_mock_device(&dev, SH36730X_CHIP_SH367309);

	/* 1. Test Charger Detection Enable / Disable (SCONF1[0]: CHGR_EN) */
	UT_ASSERT_STATUS(sh36730x_enable_charger_detection(&dev, true), SH36730X_OK, "enable_charger_detection(true)");
	UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF1] & SH36730X_SCONF1_CHGR_EN_MASK,
				 SH36730X_SCONF1_CHGR_EN_MASK, "SCONF1 CHGR_EN bit set");

	UT_ASSERT_STATUS(sh36730x_enable_charger_detection(&dev, false), SH36730X_OK, "enable_charger_detection(false)");
	UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF1] & SH36730X_SCONF1_CHGR_EN_MASK,
				 0, "SCONF1 CHGR_EN bit cleared");

	/* 2. Test Load Detection Enable / Disable (SCONF1[1]: LOAD_EN) */
	UT_ASSERT_STATUS(sh36730x_enable_load_detection(&dev, true), SH36730X_OK, "enable_load_detection(true)");
	UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF1] & SH36730X_SCONF1_LOAD_EN_MASK,
				 SH36730X_SCONF1_LOAD_EN_MASK, "SCONF1 LOAD_EN bit set");

	UT_ASSERT_STATUS(sh36730x_enable_load_detection(&dev, false), SH36730X_OK, "enable_load_detection(false)");
	UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF1] & SH36730X_SCONF1_LOAD_EN_MASK,
				 0, "SCONF1 LOAD_EN bit cleared");

	/* 3. Test Charger Status Read (BSTATUS[0]: CHGR) */
	bool is_connected = false;
	g_mock_afe.regs[SH36730X_REG_BSTATUS] = SH36730X_BSTATUS_CHGR_MASK;
	UT_ASSERT_STATUS(sh36730x_get_charger_status(&dev, &is_connected), SH36730X_OK, "get_charger_status");
	UT_ASSERT_TRUE(is_connected, "Charger status reported as connected");

	g_mock_afe.regs[SH36730X_REG_BSTATUS] = 0x00;
	UT_ASSERT_STATUS(sh36730x_get_charger_status(&dev, &is_connected), SH36730X_OK, "get_charger_status");
	UT_ASSERT_TRUE(!is_connected, "Charger status reported as disconnected");

	/* 4. Test Load Status Read (BSTATUS[1]: LOAD) */
	g_mock_afe.regs[SH36730X_REG_BSTATUS] = SH36730X_BSTATUS_LOAD_MASK;
	UT_ASSERT_STATUS(sh36730x_get_load_status(&dev, &is_connected), SH36730X_OK, "get_load_status");
	UT_ASSERT_TRUE(is_connected, "Load status reported as connected");

	g_mock_afe.regs[SH36730X_REG_BSTATUS] = 0x00;
	UT_ASSERT_STATUS(sh36730x_get_load_status(&dev, &is_connected), SH36730X_OK, "get_load_status");
	UT_ASSERT_TRUE(!is_connected, "Load status reported as disconnected");

	/* 5. Parameter Validation */
	UT_ASSERT_STATUS(sh36730x_get_charger_status(&dev, NULL), SH36730X_ERROR_PARAM, "get_charger_status(NULL)");
	UT_ASSERT_STATUS(sh36730x_get_load_status(&dev, NULL), SH36730X_ERROR_PARAM, "get_load_status(NULL)");

	if (g_current_suite_passed) {
		g_ut_stats.passed_suites++;
		UT_PRINTF(UT_COLOR_PASS "Suite 09 passed.\r\n");
	} else {
		g_ut_stats.failed_suites++;
	}
}

/**
 * @brief Suite 10: Extended Control & Status (CTLD, CHS, CHGING/DSGING, INT_EN, WDT, RST, ALARM)
 */
static void ut_suite_10_extended_control_and_status(void)
{
	UT_PRINTF(UT_COLOR_SUITE "Running Suite 10: Extended Control & Status Features...\r\n");
	g_current_suite_passed = true;

	sh36730x_t dev;
	sh36730x_ut_setup_mock_device(&dev, SH36730X_CHIP_SH367309);

	/* 1. CTLD Enable/Disable (SCONF1[6]) */
	UT_ASSERT_STATUS(sh36730x_enable_ctld(&dev, true), SH36730X_OK, "enable_ctld(true)");
	UT_ASSERT_TRUE((g_mock_afe.regs[SH36730X_REG_SCONF1] & SH36730X_SCONF1_CTLD_EN_MASK) != 0, "CTLD_EN bit set");

	UT_ASSERT_STATUS(sh36730x_enable_ctld(&dev, false), SH36730X_OK, "enable_ctld(false)");
	UT_ASSERT_TRUE((g_mock_afe.regs[SH36730X_REG_SCONF1] & SH36730X_SCONF1_CTLD_EN_MASK) == 0, "CTLD_EN bit cleared");

	/* 2. CHS Threshold Set & Get (SCONF7[3:2]) */
	UT_ASSERT_STATUS(sh36730x_set_chs_threshold(&dev, SH36730X_CHS_12_0MV), SH36730X_OK, "set_chs_threshold(12mV)");
	sh36730x_chs_threshold_t chs = SH36730X_CHS_1_4MV;
	UT_ASSERT_STATUS(sh36730x_get_chs_threshold(&dev, &chs), SH36730X_OK, "get_chs_threshold");
	UT_ASSERT_EQ(chs, SH36730X_CHS_12_0MV, "CHS threshold match");

	/* 3. Real-time Charging & Discharging Status (BSTATUS[2], BSTATUS[3]) */
	bool is_chg = false;
	bool is_dsg = false;
	g_mock_afe.regs[SH36730X_REG_BSTATUS] = SH36730X_BSTATUS_CHGING_MASK;
	UT_ASSERT_STATUS(sh36730x_get_charging_status(&dev, &is_chg), SH36730X_OK, "get_charging_status");
	UT_ASSERT_STATUS(sh36730x_get_discharging_status(&dev, &is_dsg), SH36730X_OK, "get_discharging_status");
	UT_ASSERT_TRUE(is_chg, "Charging status reported as true");
	UT_ASSERT_TRUE(!is_dsg, "Discharging status reported as false");

	g_mock_afe.regs[SH36730X_REG_BSTATUS] = SH36730X_BSTATUS_DSGING_MASK;
	UT_ASSERT_STATUS(sh36730x_get_charging_status(&dev, &is_chg), SH36730X_OK, "get_charging_status");
	UT_ASSERT_STATUS(sh36730x_get_discharging_status(&dev, &is_dsg), SH36730X_OK, "get_discharging_status");
	UT_ASSERT_TRUE(!is_chg, "Charging status reported as false");
	UT_ASSERT_TRUE(is_dsg, "Discharging status reported as true");

	/* 4. INT_EN Interrupt Enable Mask Set & Get (0x03) */
	uint8_t int_mask = SH36730X_INT_EN_CD_INT_MASK | SH36730X_INT_EN_OV_INT_MASK | SH36730X_INT_EN_SC_INT_MASK;
	UT_ASSERT_STATUS(sh36730x_set_interrupt_enable(&dev, int_mask), SH36730X_OK, "set_interrupt_enable");
	uint8_t read_int_mask = 0;
	UT_ASSERT_STATUS(sh36730x_get_interrupt_enable(&dev, &read_int_mask), SH36730X_OK, "get_interrupt_enable");
	UT_ASSERT_EQ(read_int_mask, int_mask, "INT_EN register mask match");

	/* 5. WDT Enable & Period Set/Get (SCONF1[4], SCONF7[1:0]) */
	UT_ASSERT_STATUS(sh36730x_enable_wdt(&dev, true), SH36730X_OK, "enable_wdt(true)");
	UT_ASSERT_TRUE((g_mock_afe.regs[SH36730X_REG_SCONF1] & SH36730X_SCONF1_WDT_EN_MASK) != 0, "WDT_EN bit set");

	UT_ASSERT_STATUS(sh36730x_set_wdt_period(&dev, SH36730X_WDTT_500MS), SH36730X_OK, "set_wdt_period(500ms)");
	sh36730x_wdtt_period_t wdtt = SH36730X_WDTT_30S;
	UT_ASSERT_STATUS(sh36730x_get_wdt_period(&dev, &wdtt), SH36730X_OK, "get_wdt_period");
	UT_ASSERT_EQ(wdtt, SH36730X_WDTT_500MS, "WDT period match");

	/* 6. RST Pulse Width Set/Get (SCONF6[5:4]) */
	UT_ASSERT_STATUS(sh36730x_set_rst_pulse_width(&dev, SH36730X_RST_1S), SH36730X_OK, "set_rst_pulse_width(1s)");
	sh36730x_rst_pulse_t rst_p = SH36730X_RST_16MS;
	UT_ASSERT_STATUS(sh36730x_get_rst_pulse_width(&dev, &rst_p), SH36730X_OK, "get_rst_pulse_width");
	UT_ASSERT_EQ(rst_p, SH36730X_RST_1S, "RST pulse width match");

	/* 7. ALARM Mode Set/Get (SCONF2[2]) */
	UT_ASSERT_STATUS(sh36730x_set_alarm_mode(&dev, SH36730X_ALARM_LEVEL), SH36730X_OK, "set_alarm_mode(LEVEL)");
	sh36730x_alarm_mode_t alm = SH36730X_ALARM_PULSE;
	UT_ASSERT_STATUS(sh36730x_get_alarm_mode(&dev, &alm), SH36730X_OK, "get_alarm_mode");
	UT_ASSERT_EQ(alm, SH36730X_ALARM_LEVEL, "ALARM mode match");

	/* 8. System Flags (FLAG1, FLAG2) and V33 Reset Flag (Section 7.15) */
	g_mock_afe.regs[SH36730X_REG_FLAG1] = SH36730X_FLAG1_SC_MASK | SH36730X_FLAG1_WDT_MASK;
	uint8_t read_flag1 = 0;
	UT_ASSERT_STATUS(sh36730x_get_flag1(&dev, &read_flag1), SH36730X_OK, "get_flag1");
	UT_ASSERT_EQ(read_flag1, (SH36730X_FLAG1_SC_MASK | SH36730X_FLAG1_WDT_MASK), "FLAG1 bits match");

	g_mock_afe.regs[SH36730X_REG_FLAG2] = SH36730X_FLAG2_RST_MASK | SH36730X_FLAG2_CADC_MASK;
	uint8_t read_flag2 = 0;
	UT_ASSERT_STATUS(sh36730x_get_flag2(&dev, &read_flag2), SH36730X_OK, "get_flag2");
	UT_ASSERT_EQ(read_flag2, (SH36730X_FLAG2_RST_MASK | SH36730X_FLAG2_CADC_MASK), "FLAG2 bits match");

	bool rst_flag = false;
	g_mock_afe.regs[SH36730X_REG_FLAG2] = SH36730X_FLAG2_RST_MASK;
	UT_ASSERT_STATUS(sh36730x_get_reset_flag(&dev, &rst_flag), SH36730X_OK, "get_reset_flag(true)");
	UT_ASSERT_TRUE(rst_flag, "Reset flag reported as true");

	g_mock_afe.regs[SH36730X_REG_FLAG2] = 0x00;
	UT_ASSERT_STATUS(sh36730x_get_reset_flag(&dev, &rst_flag), SH36730X_OK, "get_reset_flag(false)");
	UT_ASSERT_TRUE(!rst_flag, "Reset flag reported as false");

	if (g_current_suite_passed) {
		g_ut_stats.passed_suites++;
		UT_PRINTF(UT_COLOR_PASS "Suite 10 passed.\r\n");
	} else {
		g_ut_stats.failed_suites++;
	}
}

/**
 * @brief Suite 11: Power-Down Low-Power Mode Sequencing
 */
static void ut_suite_11_powerdown_mode(void)
{
	UT_PRINTF(UT_COLOR_SUITE "Running Suite 11: Power-Down Mode Sequencing...\r\n");
	g_current_suite_passed = true;

	sh36730x_t dev;
	sh36730x_ut_setup_mock_device(&dev, SH36730X_CHIP_SH367309);

	UT_ASSERT_STATUS(sh36730x_enter_power_down(&dev), SH36730X_OK, "sh36730x_enter_power_down");
	UT_ASSERT_EQ(g_mock_afe.regs[SH36730X_REG_SCONF10], SH36730X_POWER_DOWN_KEY, "SCONF10 key 0x33 written");
	UT_ASSERT_TRUE((g_mock_afe.regs[SH36730X_REG_SCONF1] & SH36730X_SCONF1_PD_EN_MASK) != 0, "SCONF1 PD_EN bit set");

	if (g_current_suite_passed) {
		g_ut_stats.passed_suites++;
		UT_PRINTF(UT_COLOR_PASS "Suite 11 passed.\r\n");
	} else {
		g_ut_stats.failed_suites++;
	}
}

/**
 * @brief Suite 12: Getter-Setter Symmetry Verification
 */
static void ut_suite_12_getter_setter_symmetry(void)
{
	UT_PRINTF(UT_COLOR_SUITE "Running Suite 12: Getter-Setter Symmetry Verification...\r\n");
	g_current_suite_passed = true;

	sh36730x_t dev;
	sh36730x_ut_setup_mock_device(&dev, SH36730X_CHIP_SH367309);

	/* 1. VADC Scan Period */
	sh36730x_vadc_scan_period_t scan_p;
	sh36730x_vadc_set_scan_period(&dev, SH36730X_VADC_SCAN_100MS);
	sh36730x_vadc_get_scan_period(&dev, &scan_p);
	UT_ASSERT_EQ(scan_p, SH36730X_VADC_SCAN_100MS, "VADC scan period 100ms symmetry");

	sh36730x_vadc_set_scan_period(&dev, SH36730X_VADC_SCAN_4S);
	sh36730x_vadc_get_scan_period(&dev, &scan_p);
	UT_ASSERT_EQ(scan_p, SH36730X_VADC_SCAN_4S, "VADC scan period 4s symmetry");

	/* 2. VADC Mode */
	sh36730x_vadc_mode_t vmode;
	sh36730x_vadc_set_mode(&dev, SH36730X_VADC_MODE_VOLT_AND_TEMP);
	sh36730x_vadc_get_mode(&dev, &vmode);
	UT_ASSERT_EQ(vmode, SH36730X_VADC_MODE_VOLT_AND_TEMP, "VADC mode volt+temp symmetry");

	/* 3. CADC RSNS Range */
	sh36730x_cadc_rsns_t rsns;
	sh36730x_cadc_set_rsns(&dev, SH36730X_CADC_RSNS_200MV);
	sh36730x_cadc_get_rsns(&dev, &rsns);
	UT_ASSERT_EQ(rsns, SH36730X_CADC_RSNS_200MV, "CADC RSNS 200mV symmetry");

	/* 4. CADC Resolution */
	sh36730x_cbti_c_t cbt;
	sh36730x_cadc_set_cbti_c(&dev, SH36730X_CBTI_C_10BIT);
	sh36730x_cadc_get_cbti_c(&dev, &cbt);
	UT_ASSERT_EQ(cbt, SH36730X_CBTI_C_10BIT, "CADC resolution 10-bit symmetry");

	/* 5. CADC Mode */
	sh36730x_cadc_mode_t cmode;
	sh36730x_cadc_set_mode(&dev, SH36730X_CADC_M_SINGLE);
	sh36730x_cadc_get_mode(&dev, &cmode);
	UT_ASSERT_EQ(cmode, SH36730X_CADC_M_SINGLE, "CADC mode single symmetry");

	/* 6. Over-Voltage Threshold Voltage */
	float ov_v = 0.0f;
	sh36730x_set_ov_voltage(&dev, 4150.0f);
	sh36730x_get_ov_voltage(&dev, &ov_v);
	UT_ASSERT_FLOAT_NEAR(ov_v, 4150.0f, 6.0f, "OV Voltage 4150mV symmetry");

	/* 7. Over-Voltage Delay */
	sh36730x_ov_delay_t ov_d;
	sh36730x_set_ov_delay(&dev, SH36730X_OV_DELAY_32CYCLE);
	sh36730x_get_ov_delay(&dev, &ov_d);
	UT_ASSERT_EQ(ov_d, SH36730X_OV_DELAY_32CYCLE, "OV Delay 32 cycles symmetry");

	/* 8. RESET/PF Selection */
	sh36730x_reset_pf_t rpf;
	sh36730x_set_resetpf(&dev, SH36730X_SCONF2_RESET);
	sh36730x_get_resetpf(&dev, &rpf);
	UT_ASSERT_EQ(rpf, SH36730X_SCONF2_RESET, "RESET/PF reset mode symmetry");

	/* 9. SCV Threshold */
	sh36730x_scv_t scv_val;
	sh36730x_set_scv(&dev, SH36730X_SCV_100MV);
	sh36730x_get_scv(&dev, &scv_val);
	UT_ASSERT_EQ(scv_val, SH36730X_SCV_100MV, "SCV threshold 100mV symmetry");

	/* 10. SCT Delay */
	sh36730x_sct_t sct_val;
	sh36730x_set_sct(&dev, SH36730X_SCT_500US);
	sh36730x_get_sct(&dev, &sct_val);
	UT_ASSERT_EQ(sct_val, SH36730X_SCT_500US, "SCT delay 500us symmetry");

	/* 11. Balancing Mask */
	uint16_t bal_mask = 0;
	sh36730x_set_balancing(&dev, 0x03FF, SH36730X_BALANCE_ODD);
	sh36730x_get_balancing(&dev, &bal_mask);
	UT_ASSERT_EQ(bal_mask, 0x0155, "Balancing ODD active mask symmetry");

	if (g_current_suite_passed) {
		g_ut_stats.passed_suites++;
		UT_PRINTF(UT_COLOR_PASS "Suite 12 passed.\r\n");
	} else {
		g_ut_stats.failed_suites++;
	}
}

/**
 * @brief Suite 13: Odd vs Even Start Address Double-Byte Read Consistency Test
 */
static void ut_suite_13_odd_even_address_read_consistency(void)
{
	UT_PRINTF(UT_COLOR_SUITE "Running Suite 13: Odd vs Even Start Address 2-Byte Read Consistency...\r\n");
	g_current_suite_passed = true;

	sh36730x_t dev;
	sh36730x_ut_setup_mock_device(&dev, SH36730X_CHIP_SH367309);

	/* Set distinctive mock register values */
	g_mock_afe.regs[0x06] = 0x12; /* SCONF3 */
	g_mock_afe.regs[0x07] = 0x34; /* SCONF4 */
	g_mock_afe.regs[0x08] = 0x56; /* SCONF5 */
	g_mock_afe.regs[0x09] = 0x78; /* SCONF6 */
	g_mock_afe.regs[0x2A] = 0xAA; /* CURH */
	g_mock_afe.regs[0x2B] = 0xBB; /* CURL */

	/* 1. Even address read at 0x06 */
	uint16_t pair_06 = 0;
	UT_ASSERT_STATUS(sh36730x_read_register(&dev, 0x06, &pair_06), SH36730X_OK, "read_register(0x06 Even)");
	UT_ASSERT_EQ(pair_06, 0x1234, "Even address 0x06 returns [0x12, 0x34]");

	/* 2. Odd address read at 0x07 */
	uint16_t pair_07 = 0;
	UT_ASSERT_STATUS(sh36730x_read_register(&dev, 0x07, &pair_07), SH36730X_OK, "read_register(0x07 Odd)");
	UT_ASSERT_EQ(pair_07, 0x3456, "Odd address 0x07 returns [0x34, 0x56]");

	/* 3. Even address read at 0x08 */
	uint16_t pair_08 = 0;
	UT_ASSERT_STATUS(sh36730x_read_register(&dev, 0x08, &pair_08), SH36730X_OK, "read_register(0x08 Even)");
	UT_ASSERT_EQ(pair_08, 0x5678, "Even address 0x08 returns [0x56, 0x78]");

	/* Cross-consistency: Overlap matches */
	UT_ASSERT_EQ((uint8_t)(pair_06 & 0xFF), (uint8_t)(pair_07 >> 8), "Low(0x06) == High(0x07) (both SCONF4)");
	UT_ASSERT_EQ((uint8_t)(pair_07 & 0xFF), (uint8_t)(pair_08 >> 8), "Low(0x07) == High(0x08) (both SCONF5)");

	/* 4. Boundary read at 0x2A (CURH/L) and 0x2B (CURL/0xFF) */
	uint16_t pair_2a = 0;
	uint16_t pair_2b = 0;
	UT_ASSERT_STATUS(sh36730x_read_register(&dev, 0x2A, &pair_2a), SH36730X_OK, "read_register(0x2A)");
	UT_ASSERT_STATUS(sh36730x_read_register(&dev, 0x2B, &pair_2b), SH36730X_OK, "read_register(0x2B Boundary)");
	UT_ASSERT_EQ(pair_2a, 0xAABB, "Read 0x2A returns [0xAA, 0xBB]");
	UT_ASSERT_EQ((uint8_t)(pair_2b >> 8), 0xBB, "Read 0x2B High byte returns 0xBB (CURL)");
	UT_ASSERT_EQ((uint8_t)(pair_2b & 0xFF), 0xFF, "Read 0x2B Low byte returns 0xFF (Out of boundary)");

	if (g_current_suite_passed) {
		g_ut_stats.passed_suites++;
		UT_PRINTF(UT_COLOR_PASS "Suite 13 passed.\r\n");
	} else {
		g_ut_stats.failed_suites++;
	}
}
 
 /* ========================================================================== */
 /* Complete Test Runner Entry Point                                           */
 /* ========================================================================== */
 
 bool sh36730x_unittest_run_all(void)
 {
	 memset(&g_ut_stats, 0, sizeof(g_ut_stats));
	 g_ut_stats.total_suites = 13;
 
	 UT_PRINTF("\r\n============================================================\r\n");
	 UT_PRINTF("   SH36730X In-Memory Safe Unit Test Suite Starting         \r\n");
	 UT_PRINTF("============================================================\r\n\r\n");
 
	 ut_suite_01_init_and_params();
	 ut_suite_02_register_rw_and_crc();
	 ut_suite_03_vadc_control();
	 ut_suite_04_voltages();
	 ut_suite_05_temperatures();
	 ut_suite_06_cadc_and_current();
	 ut_suite_07_protection_settings();
	 ut_suite_08_cell_balancing();
	 ut_suite_09_charger_and_load_detection();
	 ut_suite_10_extended_control_and_status();
	 ut_suite_11_powerdown_mode();
	 ut_suite_12_getter_setter_symmetry();
	 ut_suite_13_odd_even_address_read_consistency();
 
	 UT_PRINTF("\r\n============================================================\r\n");
	 UT_PRINTF("                 UNIT TEST SUMMARY REPORT                   \r\n");
	 UT_PRINTF("============================================================\r\n");
	 UT_PRINTF(" Test Suites Total   : %u\r\n", g_ut_stats.total_suites);
	 UT_PRINTF(" Test Suites Passed  : %u\r\n", g_ut_stats.passed_suites);
	 UT_PRINTF(" Test Suites Failed  : %u\r\n", g_ut_stats.failed_suites);
	 UT_PRINTF(" Assertions Total    : %u\r\n", g_ut_stats.total_asserts);
	 UT_PRINTF(" Assertions Passed   : %u\r\n", g_ut_stats.passed_asserts);
	 UT_PRINTF(" Assertions Failed   : %u\r\n", g_ut_stats.failed_asserts);
	 uint32_t rate_val = (g_ut_stats.total_asserts > 0) ?
						 (g_ut_stats.passed_asserts * 10000U / g_ut_stats.total_asserts) : 0U;
	 UT_PRINTF(" Success Rate        : %u.%02u%%\r\n",
			   (unsigned int)(rate_val / 100U), (unsigned int)(rate_val % 100U));
	 UT_PRINTF(" Final Verdict       : %s\r\n",
			   (g_ut_stats.failed_suites == 0) ? "[PASS - 100% GREEN]" : "[FAIL - INVESTIGATE]");
	 UT_PRINTF("============================================================\r\n\r\n");
 
	 return (g_ut_stats.failed_suites == 0);
 }
 
 #ifdef RT_USING_FINSH
 static int sh36730x_unittest_cmd(int argc, char **argv)
 {
	 (void)argc;
	 (void)argv;
	 sh36730x_unittest_run_all();
	 return 0;
 }
 MSH_CMD_EXPORT_ALIAS(sh36730x_unittest_cmd, sh36730x_unittest, Run SH36730X in-memory safe unit test suite);
 #endif
 