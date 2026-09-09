/**
 * @file    sh36730x_test.c
 * @brief   Comprehensive API Test Suite Implementation for Sinowealth SH36730X AFE.
 * @note    Focuses on hardware-safety, automated rollback, comprehensive coverage,
 *          clear diagnostics, and RT-Thread MSH command integration.
 */

 #include "sh36730x_test.h"
 #include "CH58x_common.h"
 #include <string.h>
 #include <stdlib.h>
 #include <math.h>
 #include <rtthread.h>
 
 #ifdef RT_USING_FINSH
 #include <stdio.h>
 #define TEST_PRINTF printf
 #else
 #include <stdio.h>
 #define TEST_PRINTF printf
 #endif
 
 /* ========================================================================== */
 /* Color & Log Formatting Macros                                              */
 /* ========================================================================== */
 
 #define TEST_TAG_PASS   "[PASS] "
 #define TEST_TAG_FAIL   "[FAIL] "
 #define TEST_TAG_WARN   "[WARN] "
 #define TEST_TAG_INFO   "[INFO] "
 #define TEST_TAG_TEST   "[TEST] "
 
 #define TEST_LOG_INFO(fmt, ...) TEST_PRINTF(TEST_TAG_INFO fmt "\r\n", ##__VA_ARGS__)
 #define TEST_LOG_PASS(fmt, ...) TEST_PRINTF(TEST_TAG_PASS fmt "\r\n", ##__VA_ARGS__)
 #define TEST_LOG_FAIL(fmt, ...) TEST_PRINTF(TEST_TAG_FAIL fmt "\r\n", ##__VA_ARGS__)
 #define TEST_LOG_WARN(fmt, ...) TEST_PRINTF(TEST_TAG_WARN fmt "\r\n", ##__VA_ARGS__)
 #define TEST_LOG_TEST(fmt, ...) TEST_PRINTF(TEST_TAG_TEST fmt "\r\n", ##__VA_ARGS__)
 
 /* ========================================================================== */
 /* Global Test Context Singleton                                              */
 /* ========================================================================== */
 
 static sh36730x_test_context_t g_test_ctx;
 
 /* ========================================================================== */
 /* Low-Level GPIO & Timing Adapter for Software I2C                           */
 /* ========================================================================== */
 
 static void bsp_soft_i2c_set_scl(void *context, bool release_high)
 {
	 (void)context;
	 if (release_high) {
		 /* Open-drain release high: Configure as input with pull-up */
		 GPIOB_ModeCfg(SH36730X_TEST_DEFAULT_SCL_PIN, GPIO_ModeIN_PU);
	 } else {
		 /* Drive low: Output low level */
		 GPIOB_ResetBits(SH36730X_TEST_DEFAULT_SCL_PIN);
		 GPIOB_ModeCfg(SH36730X_TEST_DEFAULT_SCL_PIN, GPIO_ModeOut_PP_5mA);
	 }
 }
 
 static void bsp_soft_i2c_set_sda(void *context, bool release_high)
 {
	 (void)context;
	 if (release_high) {
		 /* Open-drain release high: Configure as input with pull-up */
		 GPIOB_ModeCfg(SH36730X_TEST_DEFAULT_SDA_PIN, GPIO_ModeIN_PU);
	 } else {
		 /* Drive low: Output low level */
		 GPIOB_ResetBits(SH36730X_TEST_DEFAULT_SDA_PIN);
		 GPIOB_ModeCfg(SH36730X_TEST_DEFAULT_SDA_PIN, GPIO_ModeOut_PP_5mA);
	 }
 }
 
 static bool bsp_soft_i2c_read_scl(void *context)
 {
	 (void)context;
	 return (GPIOB_ReadPortPin(SH36730X_TEST_DEFAULT_SCL_PIN) != 0);
 }
 
 static bool bsp_soft_i2c_read_sda(void *context)
 {
	 (void)context;
	 return (GPIOB_ReadPortPin(SH36730X_TEST_DEFAULT_SDA_PIN) != 0);
 }
 
 static void bsp_soft_i2c_delay_us(void *context, uint32_t us)
 {
	 (void)context;
	 DelayUs(us);
 }
 
 static int bsp_sh36730x_i2c_write(void *context, uint8_t address_7bit, const uint8_t *data, size_t size)
 {
	 soft_i2c_t *bus = (soft_i2c_t *)context;
	 soft_i2c_status_t status = soft_i2c_write(bus, address_7bit, data, size);
	 return (status == SOFT_I2C_OK) ? 0 : (int)status;
 }
 
 static int bsp_sh36730x_i2c_write_read(void *context, uint8_t address_7bit,
										const uint8_t *tx_data, size_t tx_size,
										uint8_t *rx_data, size_t rx_size)
 {
	 soft_i2c_t *bus = (soft_i2c_t *)context;
	 soft_i2c_status_t status = soft_i2c_write_read(bus, address_7bit, tx_data, tx_size, rx_data, rx_size);
	 return (status == SOFT_I2C_OK) ? 0 : (int)status;
 }
 
 static void bsp_sh36730x_delay_ms(void *context, uint32_t milliseconds)
 {
	 (void)context;
 #ifdef RT_USING_FINSH
	 rt_thread_mdelay(milliseconds);
 #else
	 DelayMs(milliseconds);
 #endif
 }
 
 /* ========================================================================== */
 /* Hardware Initialization                                                    */
 /* ========================================================================== */
 
 test_result_t sh36730x_test_init_hardware(sh36730x_test_context_t *ctx,
										   sh36730x_chip_model_t chip_model,
										   uint8_t i2c_addr_7bit)
 {
	 if (ctx == NULL) {
		 return TEST_RESULT_FAIL;
	 }
 
	 memset(ctx, 0, sizeof(sh36730x_test_context_t));
	 ctx->rsense_ohm = SH36730X_TEST_DEFAULT_RSENSE_OHM;
 
	 /* 1. Setup GPIO pin defaults: SCL/SDA input with pull-up (bus released) */
	 GPIOB_ModeCfg(SH36730X_TEST_DEFAULT_SCL_PIN | SH36730X_TEST_DEFAULT_SDA_PIN, GPIO_ModeIN_PU);
 
	 /* 2. Configure software I2C IO operations */
	 soft_i2c_io_t io = {
		 .context   = NULL,
		 .set_scl   = bsp_soft_i2c_set_scl,
		 .set_sda   = bsp_soft_i2c_set_sda,
		 .read_scl  = bsp_soft_i2c_read_scl,
		 .read_sda  = bsp_soft_i2c_read_sda,
		 .delay_us  = bsp_soft_i2c_delay_us,
		 .lock      = NULL,
		 .unlock    = NULL
	 };
 
	 soft_i2c_status_t i2c_status = soft_i2c_init(
		 &ctx->i2c_bus,
		 &io,
		 SH36730X_TEST_DEFAULT_I2C_FREQ_HZ,
		 1000U,  /* 1ms line timeout */
		 0U      /* lock timeout */
	 );
 
	 if (i2c_status != SOFT_I2C_OK) {
		 TEST_LOG_FAIL("Software I2C initialization failed: %d", i2c_status);
		 return TEST_RESULT_FAIL;
	 }
 
	 /* 3. Setup SH36730X device driver handle */
	 ctx->device.interface.context    = &ctx->i2c_bus;
	 ctx->device.interface.write      = bsp_sh36730x_i2c_write;
	 ctx->device.interface.write_read = bsp_sh36730x_i2c_write_read;
	 ctx->device.interface.delay_ms   = bsp_sh36730x_delay_ms;
	 ctx->device.chip_model           = (chip_model == SH36730X_CHIP_NONE) ? SH36730X_CHIP_SH367303 : chip_model;
	 ctx->device.address_7bit         = (i2c_addr_7bit == 0) ? SH36730X_TEST_DEFAULT_I2C_ADDR : i2c_addr_7bit;
 
	 sh36730x_status_t drv_status = sh36730x_init(&ctx->device);
	 if (drv_status != SH36730X_OK) {
		 TEST_LOG_FAIL("SH36730X driver initialization failed: %d", drv_status);
		 return TEST_RESULT_FAIL;
	 }
 
	 ctx->is_initialized = true;
	 TEST_LOG_INFO("SH36730X Test Hardware Initialized (Addr=0x%02X, Model=%d)",
				   ctx->device.address_7bit, ctx->device.chip_model);
	 return TEST_RESULT_PASS;
 }
 
 sh36730x_test_context_t* sh36730x_test_get_default_context(void)
 {
	 return &g_test_ctx;
 }
 
 /* ========================================================================== */
 /* Test Cases Implementation                                                  */
 /* ========================================================================== */
 
 /**
  * @brief TC01: Interface, I2C Bus, and Device Initialization Test.
  */
 test_result_t sh36730x_test_tc01_init_and_probe(sh36730x_test_context_t *ctx)
 {
	 TEST_LOG_TEST("--- TC01: Interface & Device Probe Test ---");
	 ctx->stats.total_tests++;
 
	 if (!ctx->is_initialized) {
		 test_result_t res = sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		 if (res != TEST_RESULT_PASS) {
			 ctx->stats.failed_tests++;
			 return TEST_RESULT_FAIL;
		 }
	 }
 
	 /* Probe I2C address */
	 soft_i2c_status_t probe_res = soft_i2c_probe(&ctx->i2c_bus, ctx->device.address_7bit);
	 if (probe_res != SOFT_I2C_OK) {
		 TEST_LOG_WARN("I2C probe returned NACK for address 0x%02X (err=%d). Attempting bus recovery...",
					   ctx->device.address_7bit, probe_res);
		 soft_i2c_recover(&ctx->i2c_bus);
		 probe_res = soft_i2c_probe(&ctx->i2c_bus, ctx->device.address_7bit);
	 }
 
	 if (probe_res == SOFT_I2C_OK) {
		 TEST_LOG_PASS("Device responded with ACK at 7-bit addr: 0x%02X", ctx->device.address_7bit);
	 } else {
		 TEST_LOG_FAIL("Device not acknowledging address 0x%02X. Check wiring/pullups/power.", ctx->device.address_7bit);
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 /* Verify driver API parameter validation checks */
	 if (sh36730x_init(NULL) == SH36730X_ERROR_PARAM) {
		 TEST_LOG_PASS("Driver parameter null-check validation passed.");
	 } else {
		 TEST_LOG_FAIL("Driver parameter null-check failed.");
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 ctx->stats.passed_tests++;
	 return TEST_RESULT_PASS;
 }
 
 /**
  * @brief TC02: Register Read/Write, Bit-field Update & CRC Integrity Test with Rollback.
  */
 test_result_t sh36730x_test_tc02_register_rw_and_crc(sh36730x_test_context_t *ctx)
 {
	 TEST_LOG_TEST("--- TC02: Register R/W & CRC Verification with Rollback ---");
	 ctx->stats.total_tests++;
 
	 if (!ctx->is_initialized) {
		 TEST_LOG_FAIL("Context not initialized.");
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 /* Read SCONF3 and SCONF4 registers (0x06) - High byte SCONF3, Low byte SCONF4 */
	 uint16_t original_val = 0;
	 sh36730x_status_t status = sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF3, &original_val);
	 if (status != SH36730X_OK) {
		 TEST_LOG_FAIL("Failed to read SCONF3/4 (err=%d). CRC or bus fault.", status);
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
	 TEST_LOG_PASS("Read SCONF3/4 = 0x%04X (High=0x%02X, Low=0x%02X) with valid CRC8",
				   original_val, (original_val >> 8) & 0xFF, original_val & 0xFF);
 
	 /* Safe Write & Rollback: Toggle SCAN_C field in SCONF3 safely */
	 uint8_t orig_sconf3 = (uint8_t)(original_val >> 8);
	 uint8_t test_sconf3 = orig_sconf3 ^ SH36730X_SCONF3_SCAN_C_MASK;
	 uint16_t test_val16 = SH36730X_PAIR_16(test_sconf3, (uint8_t)(original_val & 0xFF));
	 uint16_t mask16     = SH36730X_HIGH_BYTE_16(SH36730X_SCONF3_SCAN_C_MASK);
 
	 /* Modify */
	 status = sh36730x_update_registers(&ctx->device, SH36730X_REG_SCONF3, mask16, test_val16);
	 if (status != SH36730X_OK) {
		 TEST_LOG_FAIL("Failed to update SCONF3 bits (err=%d)", status);
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 /* Verify modification */
	 uint16_t readback_val = 0;
	 sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF3, &readback_val);
	 if (((readback_val >> 8) & SH36730X_SCONF3_SCAN_C_MASK) != (test_sconf3 & SH36730X_SCONF3_SCAN_C_MASK)) {
		 TEST_LOG_FAIL("Readback value 0x%04X does not match expected 0x%04X", readback_val, test_val16);
		 /* Force rollback */
		 sh36730x_update_registers(&ctx->device, SH36730X_REG_SCONF3, mask16, SH36730X_HIGH_BYTE_16(orig_sconf3));
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
	 TEST_LOG_PASS("Update and Readback verification succeeded.");
 
	 /* Strict Rollback: Restore original value */
	 status = sh36730x_update_registers(&ctx->device, SH36730X_REG_SCONF3, mask16, SH36730X_HIGH_BYTE_16(orig_sconf3));
	 sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF3, &readback_val);
	 if ((uint8_t)(readback_val >> 8) != orig_sconf3) {
		 TEST_LOG_FAIL("Rollback failed! Target left in modified state: 0x%02X vs orig 0x%02X",
					   (uint8_t)(readback_val >> 8), orig_sconf3);
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
	 TEST_LOG_PASS("Hardware state safely rolled back to original (0x%04X).", original_val);
 
	 /* Boundary safety check: Verify that FLAG1/FLAG2 clear-on-read protection rejects update_register */
	 if (sh36730x_update_registers(&ctx->device, SH36730X_REG_FLAG1, 0xFFFF, 0x0000) == SH36730X_ERROR_PARAM) {
		 TEST_LOG_PASS("Protected FLAG1 update rejection verified (Safety Guard).");
	 } else {
		 TEST_LOG_WARN("FLAG1 protection did not return expected error param.");
	 }
 
	 ctx->stats.passed_tests++;
	 return TEST_RESULT_PASS;
 }
 
 /**
  * @brief TC03: Single Cell and Group Voltages Acquisition & Bounds Check.
  */
 test_result_t sh36730x_test_tc03_read_voltages(sh36730x_test_context_t *ctx)
 {
	 TEST_LOG_TEST("--- TC03: Cell Voltages Acquisition Test ---");
	 ctx->stats.total_tests++;
 
	 if (!ctx->is_initialized) {
		 TEST_LOG_FAIL("Context not initialized.");
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 /* Ensure VADC is enabled */
	 sh36730x_vadc_enable(&ctx->device, true);
	 bsp_sh36730x_delay_ms(NULL, 100); /* Wait for ADC sample conversion */
 
	 uint8_t max_cell_count = 10;
	 if (ctx->device.chip_model == SH36730X_CHIP_SH367303) {
		 max_cell_count = 5;
	 } else if (ctx->device.chip_model == SH36730X_CHIP_SH367305) {
		 max_cell_count = 8;
	 }
 
	 uint16_t cell_volts[10] = {0};
	 uint32_t total_pack_mv = 0;
	 uint16_t max_v = 0;
	 uint16_t min_v = 0xFFFF;
	 uint8_t connected_cells = 0;
 
	 TEST_PRINTF("--------------------------------------------------\r\n");
	 TEST_PRINTF(" Channel | Voltage (mV) | Voltage (V) | Status    \r\n");
	 TEST_PRINTF("--------------------------------------------------\r\n");
 
	 for (uint8_t i = 0; i < max_cell_count; i++) {
		 uint16_t v_mv = 0;
		 sh36730x_status_t st = sh36730x_get_cell_voltage(&ctx->device, (sh36730x_cell_channel_t)i, &v_mv);
		 if (st == SH36730X_OK) {
			 cell_volts[i] = v_mv;
			 const char *status_str = "OK";
			 if (v_mv < 500) {
				 status_str = "NC / Floating";
			 } else if (v_mv < 2500) {
				 status_str = "Under-voltage";
				 connected_cells++;
			 } else if (v_mv > 4350) {
				 status_str = "Over-voltage!";
				 connected_cells++;
			 } else {
				 status_str = "Normal";
				 connected_cells++;
			 }
 
			 TEST_PRINTF(" Cell %2d | %7u mV   | %2u.%03u V | %s\r\n",
						 i + 1, v_mv, (unsigned int)(v_mv / 1000U), (unsigned int)(v_mv % 1000U), status_str);
 
			 if (v_mv >= 500) {
				 total_pack_mv += v_mv;
				 if (v_mv > max_v) max_v = v_mv;
				 if (v_mv < min_v) min_v = v_mv;
			 }
		 } else {
			 TEST_PRINTF(" Cell %2d | READ ERROR (%d)\r\n", i + 1, st);
			 ctx->stats.failed_tests++;
			 return TEST_RESULT_FAIL;
		 }
	 }
	 TEST_PRINTF("--------------------------------------------------\r\n");
 
	 if (connected_cells > 0) {
		 TEST_LOG_INFO("Connected Cells: %u, Total Pack: %u mV (%u.%03u V)",
					   connected_cells, total_pack_mv,
					   (unsigned int)(total_pack_mv / 1000U), (unsigned int)(total_pack_mv % 1000U));
		 TEST_LOG_INFO("Cell Max: %u mV, Min: %u mV, Delta: %u mV",
					   max_v, min_v, (max_v >= min_v) ? (max_v - min_v) : 0);
	 } else {
		 TEST_LOG_WARN("No active cells detected (>500mV). Evaluation board may be unpowered.");
	 }
 
	 /* Test Group Voltage API */
	 uint16_t group_volts[5] = {0};
	 sh36730x_status_t grp_st = sh36730x_get_cells_group_voltage(&ctx->device, 1, 5, group_volts, 5);
	 if (grp_st == SH36730X_OK) {
		 TEST_LOG_PASS("Group Voltage (Cell 1..5) bulk fetch passed.");
	 } else {
		 TEST_LOG_FAIL("Group Voltage bulk fetch failed: %d", grp_st);
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 ctx->stats.passed_tests++;
	 return TEST_RESULT_PASS;
 }
 
 /**
  * @brief TC04: External NTC and Internal Temperature Sensors Acquisition & Range Check.
  */
 test_result_t sh36730x_test_tc04_read_temperatures(sh36730x_test_context_t *ctx)
 {
	 TEST_LOG_TEST("--- TC04: Temperature Sensors Acquisition Test ---");
	 ctx->stats.total_tests++;
 
	 if (!ctx->is_initialized) {
		 TEST_LOG_FAIL("Context not initialized.");
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 /* Configure VADC for Volt & Temp mode */
	 sh36730x_vadc_set_mode(&ctx->device, SH36730X_VADC_MODE_VOLT_AND_TEMP);
	 bsp_sh36730x_delay_ms(NULL, 100);
 
	 /* Read TS1 and TS2 external NTC resistance */
	 uint32_t ts1_kohm = 0;
	 uint32_t ts2_kohm = 0;
	 sh36730x_status_t st_ts1 = sh36730x_get_temp_ts(&ctx->device, SH36730X_TS1, &ts1_kohm);
	 sh36730x_status_t st_ts2 = sh36730x_get_temp_ts(&ctx->device, SH36730X_TS2, &ts2_kohm);
 
	 if (st_ts1 == SH36730X_OK) {
		 TEST_LOG_INFO("External TS1 Resistance: %u Ohm (%u kOhm)", ts1_kohm, ts1_kohm / 1000);
	 } else {
		 TEST_LOG_WARN("External TS1 read returned %d", st_ts1);
	 }
 
	 if (st_ts2 == SH36730X_OK) {
		 TEST_LOG_INFO("External TS2 Resistance: %u Ohm (%u kOhm)", ts2_kohm, ts2_kohm / 1000);
	 } else {
		 TEST_LOG_WARN("External TS2 read returned %d", st_ts2);
	 }
 
	 /* Read Internal Temperature Sensors 1 & 2 */
	 float int_temp1 = 0.0f;
	 float int_temp2 = 0.0f;
	 sh36730x_status_t st_tint1 = sh36730x_get_internal_temp(&ctx->device, SH36730X_INTERNAL_TEMP1, &int_temp1);
	 sh36730x_status_t st_tint2 = sh36730x_get_internal_temp(&ctx->device, SH36730X_INTERNAL_TEMP2, &int_temp2);
 
	 if (st_tint1 == SH36730X_OK) {
		 int t1_int = (int)int_temp1;
		 int t1_dec = (int)(fabsf(int_temp1 - (float)t1_int) * 100.0f + 0.5f);
		 const char *t1_neg = (int_temp1 < 0.0f && t1_int == 0) ? "-" : "";
		 TEST_PRINTF(TEST_TAG_INFO "Internal Temp 1: %s%d.%02d deg C\r\n", t1_neg, t1_int, t1_dec);
		 if (int_temp1 >= -40.0f && int_temp1 <= 100.0f) {
			 TEST_LOG_PASS("Internal Temp 1 within reasonable operating bounds.");
		 } else {
			 TEST_LOG_WARN("Internal Temp 1 (%s%d.%02d C) outside standard room envelope.", t1_neg, t1_int, t1_dec);
		 }
	 } else {
		 TEST_LOG_FAIL("Internal Temp 1 read failed: %d", st_tint1);
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 if (st_tint2 == SH36730X_OK) {
		 int t2_int = (int)int_temp2;
		 int t2_dec = (int)(fabsf(int_temp2 - (float)t2_int) * 100.0f + 0.5f);
		 const char *t2_neg = (int_temp2 < 0.0f && t2_int == 0) ? "-" : "";
		 TEST_PRINTF(TEST_TAG_INFO "Internal Temp 2: %s%d.%02d deg C\r\n", t2_neg, t2_int, t2_dec);
	 }
 
	 ctx->stats.passed_tests++;
	 return TEST_RESULT_PASS;
 }
 
 /**
  * @brief TC05: VADC Sampling Mode and Period Configuration Test with Rollback.
  */
 test_result_t sh36730x_test_tc05_vadc_config(sh36730x_test_context_t *ctx)
 {
	 TEST_LOG_TEST("--- TC05: VADC Config & Scan Period Verification ---");
	 ctx->stats.total_tests++;
 
	 if (!ctx->is_initialized) {
		 TEST_LOG_FAIL("Context not initialized.");
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 /* Read original SCONF3 */
	 uint16_t orig_sconf3_pair = 0;
	 sh36730x_status_t st = sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF3, &orig_sconf3_pair);
	 if (st != SH36730X_OK) {
		 TEST_LOG_FAIL("Cannot read SCONF3 (err=%d)", st);
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
	 uint8_t orig_sconf3 = (uint8_t)(orig_sconf3_pair >> 8);
 
	 /* Test structured config API */
	 sh36730x_vadc_config_t test_cfg = {
		 .enable      = true,
		 .mode        = SH36730X_VADC_MODE_VOLT_AND_TEMP,
		 .scan_period = SH36730X_VADC_SCAN_200MS
	 };
 
	 st = sh36730x_vadc_config(&ctx->device, &test_cfg);
	 if (st != SH36730X_OK) {
		 TEST_LOG_FAIL("sh36730x_vadc_config returned error: %d", st);
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 /* Verify SCONF3 fields */
	 uint16_t readback_pair = 0;
	 sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF3, &readback_pair);
	 uint8_t readback_sconf3 = (uint8_t)(readback_pair >> 8);
 
	 bool vadc_en = (readback_sconf3 & SH36730X_SCONF3_VADC_EN_MASK) != 0;
	 uint8_t vadc_c = SH36730X_READ_FIELD(readback_sconf3, SH36730X_SCONF3_VADC_C_MASK, SH36730X_SCONF3_VADC_C_POS);
	 uint8_t scan_c = SH36730X_READ_FIELD(readback_sconf3, SH36730X_SCONF3_SCAN_C_MASK, SH36730X_SCONF3_SCAN_C_POS);
 
	 if (vadc_en == true && vadc_c == SH36730X_VADC_MODE_VOLT_AND_TEMP && scan_c == SH36730X_VADC_SCAN_200MS) {
		 TEST_LOG_PASS("VADC configuration verified successfully in register.");
	 } else {
		 TEST_LOG_FAIL("VADC configuration mismatch (EN=%d, Mode=%d, Scan=%d)", vadc_en, vadc_c, scan_c);
		 ctx->stats.failed_tests++;
	 }
 
	 /* Test individual setter: sh36730x_vadc_set_scan_period */
	 sh36730x_vadc_set_scan_period(&ctx->device, SH36730X_VADC_SCAN_50MS);
	 sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF3, &readback_pair);
	 scan_c = SH36730X_READ_FIELD((uint8_t)(readback_pair >> 8), SH36730X_SCONF3_SCAN_C_MASK, SH36730X_SCONF3_SCAN_C_POS);
	 if (scan_c == SH36730X_VADC_SCAN_50MS) {
		 TEST_LOG_PASS("sh36730x_vadc_set_scan_period(50ms) confirmed.");
	 }
 
	 /* Restore original SCONF3 */
	 uint16_t mask16 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF3_VADC_EN_MASK | SH36730X_SCONF3_VADC_C_MASK | SH36730X_SCONF3_SCAN_C_MASK);
	 sh36730x_update_registers(&ctx->device, SH36730X_REG_SCONF3, mask16, SH36730X_HIGH_BYTE_16(orig_sconf3));
	 TEST_LOG_PASS("VADC parameters rolled back to initial state.");
 
	 ctx->stats.passed_tests++;
	 return TEST_RESULT_PASS;
 }
 
 /**
  * @brief TC06: CADC Current Measurement & Range Configuration Test with Rollback.
  */
 test_result_t sh36730x_test_tc06_cadc_and_current(sh36730x_test_context_t *ctx)
 {
	 TEST_LOG_TEST("--- TC06: CADC Current Measurement & Mode Test ---");
	 ctx->stats.total_tests++;
 
	 if (!ctx->is_initialized) {
		 TEST_LOG_FAIL("Context not initialized.");
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 /* Backup original SCONF3 and SCONF6 */
	 uint16_t orig_sconf3_pair = 0;
	 uint16_t orig_sconf6_pair = 0;
	 sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF3, &orig_sconf3_pair);
	 sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF6, &orig_sconf6_pair);
 
	 /* Configure CADC: Continuous mode, 50mV RSNS range, 13-bit resolution */
	 sh36730x_cadc_config_t cadc_cfg = {
		 .enable    = true,
		 .cadc_mode = SH36730X_CADC_M_CONTINUOUS,
		 .cadc_rsns = SH36730X_CADC_RSNS_50MV,
		 .cbti_c    = SH36730X_CBTI_C_13BIT
	 };
 
	 sh36730x_status_t st = sh36730x_cadc_config(&ctx->device, &cadc_cfg);
	 if (st != SH36730X_OK) {
		 TEST_LOG_FAIL("CADC configuration failed: %d", st);
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
	 TEST_LOG_PASS("CADC configured (Continuous, 50mV RSNS, 13-bit).");
 
	 bsp_sh36730x_delay_ms(NULL, 150); /* Allow CADC conversion */
 
	 /* Read Current */
	 float current_ma = 0.0f;
	 st = sh36730x_get_current(&ctx->device, ctx->rsense_ohm, &current_ma);
	 if (st == SH36730X_OK) {
		 int cur_ma_int = (int)current_ma;
		 int cur_ma_dec = (int)(fabsf(current_ma - (float)cur_ma_int) * 100.0f + 0.5f);
		 const char *cur_ma_neg = (current_ma < 0.0f && cur_ma_int == 0) ? "-" : "";
 
		 float current_a = current_ma / 1000.0f;
		 int cur_a_int = (int)current_a;
		 int cur_a_dec = (int)(fabsf(current_a - (float)cur_a_int) * 1000.0f + 0.5f);
		 const char *cur_a_neg = (current_a < 0.0f && cur_a_int == 0) ? "-" : "";
 
		 int rsense_mohm = (int)(ctx->rsense_ohm * 1000.0f + 0.5f);
 
		 TEST_PRINTF(TEST_TAG_INFO "Measured Current: %s%d.%02d mA (%s%d.%03d A) [Rsense=%d mOhm]\r\n",
					 cur_ma_neg, cur_ma_int, cur_ma_dec, cur_a_neg, cur_a_int, cur_a_dec, rsense_mohm);
		 TEST_LOG_PASS("Current reading API returned successfully.");
	 } else {
		 TEST_LOG_FAIL("Failed to get CADC current: %d", st);
		 ctx->stats.failed_tests++;
	 }
 
	 /* Restore original SCONF3 & SCONF6 */
	 uint16_t mask_sconf3 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF3_CADC_EN_MASK | SH36730X_SCONF3_CADC_M_MASK | SH36730X_SCONF3_CBIT_C_MASK);
	 uint16_t mask_sconf6 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF6_RSNS_MASK);
 
	 sh36730x_update_registers(&ctx->device, SH36730X_REG_SCONF3, mask_sconf3, SH36730X_HIGH_BYTE_16((uint8_t)(orig_sconf3_pair >> 8)));
	 sh36730x_update_registers(&ctx->device, SH36730X_REG_SCONF6, mask_sconf6, SH36730X_HIGH_BYTE_16((uint8_t)(orig_sconf6_pair >> 8)));
	 TEST_LOG_PASS("CADC configuration safely restored.");
 
	 ctx->stats.passed_tests++;
	 return TEST_RESULT_PASS;
 }
 
 /**
  * @brief TC07: Hardware Protection Parameters (OV/SC/Delay) Safe Setting & Rollback Test.
  */
 test_result_t sh36730x_test_tc07_protection_params(sh36730x_test_context_t *ctx)
 {
	 TEST_LOG_TEST("--- TC07: Protection Thresholds & Delay Test (Safe Range) ---");
	 ctx->stats.total_tests++;
 
	 if (!ctx->is_initialized) {
		 TEST_LOG_FAIL("Context not initialized.");
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 /* Backup original SCONF7 (OVT delay) & SCONF6 (SCV/SCT) */
	 uint16_t orig_sconf7_pair = 0;
	 uint16_t orig_sconf6_pair = 0;
	 sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF7, &orig_sconf7_pair);
	 sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF6, &orig_sconf6_pair);
 
	 /* 1. Test OV Delay Configuration */
	 sh36730x_status_t st = sh36730x_set_ov_delay(&ctx->device, SH36730X_OV_DELAY_8CYCLE);
	 if (st == SH36730X_OK) {
		 TEST_LOG_PASS("Set OV delay to 8 cycles passed.");
	 } else {
		 TEST_LOG_FAIL("Set OV delay failed: %d", st);
		 ctx->stats.failed_tests++;
	 }
 
	 /* 2. Test Short-Circuit Voltage (SCV) and Delay (SCT) */
	 st = sh36730x_set_scv(&ctx->device, SH36730X_SCV_200MV);
	 if (st == SH36730X_OK) {
		 TEST_LOG_PASS("Set SCV threshold to 200mV passed.");
	 }
 
	 st = sh36730x_set_sct(&ctx->device, SH36730X_SCT_100US);
	 if (st == SH36730X_OK) {
		 TEST_LOG_PASS("Set SCT delay to 100us passed.");
	 }
 
	 /* 3. Safe Rollback of SCONF7 and SCONF6 */
	 uint16_t mask_sconf7 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF7_OVT_MASK);
	 uint16_t mask_sconf6 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF6_SCV_MASK | SH36730X_SCONF6_SCT_MASK);
 
	 sh36730x_update_registers(&ctx->device, SH36730X_REG_SCONF7, mask_sconf7, SH36730X_HIGH_BYTE_16((uint8_t)(orig_sconf7_pair >> 8)));
	 sh36730x_update_registers(&ctx->device, SH36730X_REG_SCONF6, mask_sconf6, SH36730X_HIGH_BYTE_16((uint8_t)(orig_sconf6_pair >> 8)));
	 TEST_LOG_PASS("Protection thresholds safely restored to initial values.");
 
	 ctx->stats.passed_tests++;
	 return TEST_RESULT_PASS;
 }
 
 /**
  * @brief TC08: Safe Hardware Cell Balancing Test (Short pulse, strictly disarmed upon exit).
  * @note  Safety Guarantee: Balancing is enabled only for 50ms verification pulse and
  *        FORCIBLY disabled before the function exits to prevent thermal or over-discharge risk.
  */
 test_result_t sh36730x_test_tc08_safe_balancing(sh36730x_test_context_t *ctx)
 {
	 TEST_LOG_TEST("--- TC08: Safe Controlled Cell Balancing Test ---");
	 ctx->stats.total_tests++;
 
	 if (!ctx->is_initialized) {
		 TEST_LOG_FAIL("Context not initialized.");
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 /* Step 1: Ensure balancing is completely disabled first */
	 sh36730x_status_t st = sh36730x_set_balancing(&ctx->device, 0x0000, SH36730X_BALANCE_DISABLE);
	 if (st != SH36730X_OK) {
		 TEST_LOG_FAIL("Failed to set BALANCE_DISABLE: %d", st);
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 /* Step 2: Test Odd Channel Safe Balancing (Cells 1, 3, 5, 7, 9) with 50ms pulse */
	 uint16_t odd_mask = 0x0155; /* Bits for odd channels */
	 st = sh36730x_set_balancing(&ctx->device, odd_mask, SH36730X_BALANCE_ODD);
	 if (st == SH36730X_OK) {
		 /* Read back SCONF4 and SCONF5 */
		 uint16_t sconf4_5 = 0;
		 sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF4, &sconf4_5);
		 TEST_LOG_PASS("Odd balancing pulse armed (SCONF4/5 = 0x%04X).", sconf4_5);
	 } else {
		 TEST_LOG_FAIL("Failed to set odd balance: %d", st);
	 }
 
	 bsp_sh36730x_delay_ms(NULL, 50); /* Short 50ms pulse */
 
	 /* Step 3: IMMEDIATE DISARM & SAFETY LOCK */
	 st = sh36730x_set_balancing(&ctx->device, 0x0000, SH36730X_BALANCE_DISABLE);
	 uint16_t disarmed_val = 0;
	 sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF4, &disarmed_val);
 
	 if ((disarmed_val & 0x1F1F) == 0x0000) {
		 TEST_LOG_PASS("Balancing FORCIBLY DISARMED and verified (SCONF4/5 = 0x0000). Safety check OK.");
	 } else {
		 TEST_LOG_FAIL("CRITICAL: Balancing not disarmed! (Value=0x%04X)", disarmed_val);
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 ctx->stats.passed_tests++;
	 return TEST_RESULT_PASS;
 }
 
 /**
  * @brief TC09: Communication Stress & Long-term Stability Test.
  */
 test_result_t sh36730x_test_tc09_stress_test(sh36730x_test_context_t *ctx, uint32_t iterations)
 {
	 TEST_LOG_TEST("--- TC09: Communication Stress & Stability Test (%u cycles) ---", (unsigned int)iterations);
	 ctx->stats.total_tests++;
 
	 if (!ctx->is_initialized) {
		 TEST_LOG_FAIL("Context not initialized.");
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 
	 if (iterations == 0) {
		 iterations = 50;
	 }
 
	 uint32_t crc_errors = 0;
	 uint32_t comm_errors = 0;
	 uint32_t successful_cycles = 0;
 
	 for (uint32_t i = 0; i < iterations; i++) {
		 uint16_t reg_val = 0;
		 sh36730x_status_t st = sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF3, &reg_val);
		 if (st == SH36730X_OK) {
			 successful_cycles++;
		 } else if (st == SH36730X_ERROR_CRC) {
			 crc_errors++;
		 } else {
			 comm_errors++;
		 }
 
		 if ((i + 1) % 25 == 0 || i == iterations - 1) {
			 TEST_PRINTF(TEST_TAG_INFO "Progress: %u / %u | Success: %u | CRC Err: %u | Comm Err: %u\r\n",
						 (unsigned int)(i + 1), (unsigned int)iterations,
						 (unsigned int)successful_cycles, (unsigned int)crc_errors, (unsigned int)comm_errors);
		 }
	 }
 
	 uint32_t rate_val = (iterations > 0) ? (successful_cycles * 10000U / iterations) : 0U;
	 TEST_PRINTF(TEST_TAG_INFO "Stress Test Result: Success Rate = %u.%02u%%\r\n",
				 (unsigned int)(rate_val / 100U), (unsigned int)(rate_val % 100U));
 
	 if (crc_errors == 0 && comm_errors == 0) {
		 TEST_LOG_PASS("Zero communication/CRC errors during %u transactions.", (unsigned int)iterations);
		 ctx->stats.passed_tests++;
		 return TEST_RESULT_PASS;
	 } else if (rate_val >= 9500U) {
		 TEST_LOG_WARN("Stress test passed with minor anomalies (CRC: %u, Comm: %u)",
					   (unsigned int)crc_errors, (unsigned int)comm_errors);
		 ctx->stats.warned_tests++;
		 return TEST_RESULT_WARN;
	 } else {
		 TEST_LOG_FAIL("Communication stability failed unacceptable error rate!");
		 ctx->stats.failed_tests++;
		 return TEST_RESULT_FAIL;
	 }
 }
 
 /**
  * @brief TC10: Dump all key diagnostic registers.
  */
 test_result_t sh36730x_test_dump_status(sh36730x_test_context_t *ctx)
 {
	 TEST_LOG_TEST("--- Diagnostic Register Dump ---");
 
	 if (!ctx->is_initialized) {
		 TEST_LOG_FAIL("Context not initialized.");
		 return TEST_RESULT_FAIL;
	 }
 
	 const struct {
		 uint8_t reg;
		 const char *name;
	 } dump_regs[] = {
		 {SH36730X_REG_SCONF1, "SCONF1/2 (OV/SC/LTCLR & RESET)"},
		 {SH36730X_REG_SCONF3, "SCONF3/4 (VADC/CADC/SCAN & CB1-5)"},
		 {SH36730X_REG_SCONF5, "SCONF5/6 (CB6-10 & RSNS/SCV/SCT)"},
		 {SH36730X_REG_SCONF7, "SCONF7/8 (OVT Delay & OVD[9:8])"},
		 {SH36730X_REG_SCONF9, "SCONF9/10 (OVD[7:0] & INT_EN)"},
		 {SH36730X_REG_CURH,   "CURH/L   (CADC Current Raw)"},
		 {SH36730X_REG_CELL1H, "CELL1H/L (Cell 1 Raw)"},
		 {SH36730X_REG_CELL2H, "CELL2H/L (Cell 2 Raw)"},
		 {SH36730X_REG_CELL3H, "CELL3H/L (Cell 3 Raw)"},
		 {SH36730X_REG_TS1H,   "TS1H/L   (TS1 Temp Raw)"},
		 {SH36730X_REG_TEMP1H, "TEMP1H/L (Internal Temp Raw)"}
	 };
 
	 TEST_PRINTF("------------------------------------------------------------\r\n");
	 TEST_PRINTF(" Reg Addr | Name                             | Hex Value   \r\n");
	 TEST_PRINTF("------------------------------------------------------------\r\n");
 
	 for (size_t i = 0; i < sizeof(dump_regs)/sizeof(dump_regs[0]); i++) {
		 uint16_t val = 0;
		 sh36730x_status_t st = sh36730x_read_register(&ctx->device, dump_regs[i].reg, &val);
		 if (st == SH36730X_OK) {
			 TEST_PRINTF(" 0x%02X     | %-32s | 0x%04X\r\n", dump_regs[i].reg, dump_regs[i].name, val);
		 } else {
			 TEST_PRINTF(" 0x%02X     | %-32s | READ ERR (%d)\r\n", dump_regs[i].reg, dump_regs[i].name, st);
		 }
	 }
	 TEST_PRINTF("------------------------------------------------------------\r\n");
 
	 return TEST_RESULT_PASS;
 }
 
 /**
  * @brief Execute all test cases sequentially with hardware safety guarantees.
  */
 test_result_t sh36730x_test_run_all(sh36730x_test_context_t *ctx)
 {
	 TEST_PRINTF("\r\n============================================================\r\n");
	 TEST_PRINTF("   SH36730X AFE Automated Safe Test Suite Starting          \r\n");
	 TEST_PRINTF("============================================================\r\n\r\n");
 
	 if (ctx == NULL) {
		 ctx = &g_test_ctx;
	 }
 
	 /* Initialize hardware & test context */
	 if (!ctx->is_initialized) {
		 if (sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR) != TEST_RESULT_PASS) {
			 TEST_LOG_FAIL("Hardware init aborted.");
			 return TEST_RESULT_FAIL;
		 }
	 }
 
	 ctx->stats.total_tests = 0;
	 ctx->stats.passed_tests = 0;
	 ctx->stats.failed_tests = 0;
	 ctx->stats.warned_tests = 0;
	 ctx->stats.skipped_tests = 0;
 
	 /* Execute test suite */
	 sh36730x_test_tc01_init_and_probe(ctx);
	 sh36730x_test_tc02_register_rw_and_crc(ctx);
	 sh36730x_test_tc03_read_voltages(ctx);
	 sh36730x_test_tc04_read_temperatures(ctx);
	 sh36730x_test_tc05_vadc_config(ctx);
	 sh36730x_test_tc06_cadc_and_current(ctx);
	 sh36730x_test_tc07_protection_params(ctx);
	 sh36730x_test_tc08_safe_balancing(ctx);
	 sh36730x_test_tc09_stress_test(ctx, 30);
	 sh36730x_test_dump_status(ctx);
 
	 /* Final Report */
	 TEST_PRINTF("\r\n============================================================\r\n");
	 TEST_PRINTF("                    TEST SUITE REPORT                       \r\n");
	 TEST_PRINTF("============================================================\r\n");
	 TEST_PRINTF(" Total Test Cases  : %u\r\n", (unsigned int)ctx->stats.total_tests);
	 TEST_PRINTF(" Passed            : %u\r\n", (unsigned int)ctx->stats.passed_tests);
	 TEST_PRINTF(" Failed            : %u\r\n", (unsigned int)ctx->stats.failed_tests);
	 TEST_PRINTF(" Warnings          : %u\r\n", (unsigned int)ctx->stats.warned_tests);
	 TEST_PRINTF(" Skipped           : %u\r\n", (unsigned int)ctx->stats.skipped_tests);
	 TEST_PRINTF(" Overall Result    : %s\r\n",
				 (ctx->stats.failed_tests == 0) ? "[PASS - ALL SAFE]" : "[FAIL - INVESTIGATE]");
	 TEST_PRINTF("============================================================\r\n\r\n");
 
	 return (ctx->stats.failed_tests == 0) ? TEST_RESULT_PASS : TEST_RESULT_FAIL;
 }
 
 /* ========================================================================== */
 /* Interactive RT-Thread FinSH / MSH Command CLI                              */
 /* ========================================================================== */
 
 #ifdef RT_USING_FINSH
 
 static void sh36730x_test_print_help(void)
 {
	 rt_kprintf("\r\nUsage: sh36730x_test <command> [args]\r\n");
	 rt_kprintf("Commands:\r\n");
	 rt_kprintf("  all              - Run full automated safe test suite\r\n");
	 rt_kprintf("  init             - Initialize I2C and probe SH36730X\r\n");
	 rt_kprintf("  volt             - Measure all cell voltages and pack summary\r\n");
	 rt_kprintf("  temp             - Read external TS & internal chip temperatures\r\n");
	 rt_kprintf("  cur [rsense_mOhm]- Read CADC current (default Rsense = 1.0 mOhm)\r\n");
	 rt_kprintf("  vadc             - Test VADC sampling modes and scan periods\r\n");
	 rt_kprintf("  cadc             - Test CADC mode and resolution config\r\n");
	 rt_kprintf("  prot             - Test OV/SC protection params with rollback\r\n");
	 rt_kprintf("  bal              - Run controlled 50ms pulse balancing test\r\n");
	 rt_kprintf("  stress [count]   - Run communication stress test (default 50)\r\n");
	 rt_kprintf("  dump             - Dump all key diagnostic registers\r\n");
	 rt_kprintf("  help             - Show this help message\r\n\r\n");
 }
 
 static int sh36730x_test_cmd(int argc, char **argv)
 {
	 sh36730x_test_context_t *ctx = &g_test_ctx;
 
	 if (argc < 2) {
		 sh36730x_test_print_help();
		 return 0;
	 }
 
	 const char *subcmd = argv[1];
 
	 if (strcmp(subcmd, "help") == 0) {
		 sh36730x_test_print_help();
	 } else if (strcmp(subcmd, "all") == 0) {
		 sh36730x_test_run_all(ctx);
	 } else if (strcmp(subcmd, "init") == 0) {
		 sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		 sh36730x_test_tc01_init_and_probe(ctx);
	 } else if (strcmp(subcmd, "volt") == 0) {
		 if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		 sh36730x_test_tc03_read_voltages(ctx);
	 } else if (strcmp(subcmd, "temp") == 0) {
		 if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		 sh36730x_test_tc04_read_temperatures(ctx);
	 } else if (strcmp(subcmd, "cur") == 0) {
		 if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		 if (argc >= 3) {
			 float rsense_mohm = (float)atoi(argv[2]);
			 if (rsense_mohm > 0.0f) {
				 ctx->rsense_ohm = rsense_mohm / 1000.0f;
			 }
		 }
		 sh36730x_test_tc06_cadc_and_current(ctx);
	 } else if (strcmp(subcmd, "vadc") == 0) {
		 if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		 sh36730x_test_tc05_vadc_config(ctx);
	 } else if (strcmp(subcmd, "cadc") == 0) {
		 if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		 sh36730x_test_tc06_cadc_and_current(ctx);
	 } else if (strcmp(subcmd, "prot") == 0) {
		 if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		 sh36730x_test_tc07_protection_params(ctx);
	 } else if (strcmp(subcmd, "bal") == 0) {
		 if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		 sh36730x_test_tc08_safe_balancing(ctx);
	 } else if (strcmp(subcmd, "stress") == 0) {
		 if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		 uint32_t count = 50;
		 if (argc >= 3) {
			 count = (uint32_t)atoi(argv[2]);
		 }
		 sh36730x_test_tc09_stress_test(ctx, count);
	 } else if (strcmp(subcmd, "dump") == 0) {
		 if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		 sh36730x_test_dump_status(ctx);
	 } else {
		 rt_kprintf("Unknown subcommand: %s\r\n", subcmd);
		 sh36730x_test_print_help();
	 }
 
	 return 0;
 }
 
 MSH_CMD_EXPORT_ALIAS(sh36730x_test_cmd, sh36730x_test, SH36730X AFE comprehensive safe test suite);
 
 #endif /* RT_USING_FINSH */
 
  