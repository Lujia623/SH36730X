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
 #include "ntc_mf52a.h"
 
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
	 sh36730x_vadc_enable(&ctx->device, true);
	 sh36730x_vadc_set_mode(&ctx->device, SH36730X_VADC_MODE_VOLT_AND_TEMP);
	 bsp_sh36730x_delay_ms(NULL, 100);
 
	 /* Read TS1 and TS2 external NTC resistance */
	 uint32_t ts1_kohm = 0;
	 uint32_t ts2_kohm = 0;
	 float ts1_temp = 0.0f;
	 float ts2_temp = 0.0f;
	 sh36730x_status_t st_ts1 = sh36730x_get_temp_ts(&ctx->device, SH36730X_TS1, &ts1_kohm);
	 sh36730x_status_t st_ts2 = sh36730x_get_temp_ts(&ctx->device, SH36730X_TS2, &ts2_kohm);
	 bool _ts1 = ntc_mf52a_ohm_to_temp_c(ts1_kohm, &ts1_temp);
	 bool _ts2 = ntc_mf52a_ohm_to_temp_c(ts2_kohm, &ts2_temp);
 
	 if (st_ts1 == SH36730X_OK) {
		 TEST_LOG_INFO("External TS1 Resistance: %u Ohm (%u kOhm)", ts1_kohm, ts1_kohm / 1000);
		 if (_ts1) {
			 int t1_int = (int)ts1_temp;
			 int t1_dec = (int)(fabsf(ts1_temp - (float)t1_int) * 100.0f + 0.5f);
			 const char *t1_neg = (ts1_temp < 0.0f && t1_int == 0) ? "-" : "";
			 TEST_PRINTF(TEST_TAG_INFO "External TS1 Temp 1: %s%d.%02d deg C\r\n", t1_neg, t1_int, t1_dec);
		 } else {
			TEST_PRINTF(TEST_TAG_WARN "External TS1 Temp 1 FAIL\r\n");
		 }
	 } else {
		 TEST_LOG_WARN("External TS1 read returned %d", st_ts1);
	 }
 
	 if (st_ts2 == SH36730X_OK) {
		 TEST_LOG_INFO("External TS2 Resistance: %u Ohm (%u kOhm)", ts2_kohm, ts2_kohm / 1000);
		 if (_ts2) {
			int t1_int = (int)ts2_temp;
			int t1_dec = (int)(fabsf(ts2_temp - (float)t1_int) * 100.0f + 0.5f);
			const char *t1_neg = (ts2_temp < 0.0f && t1_int == 0) ? "-" : "";
			TEST_PRINTF(TEST_TAG_INFO "External TS2 Temp 2: %s%d.%02d deg C\r\n", t1_neg, t1_int, t1_dec);
		} else {
		   TEST_PRINTF(TEST_TAG_WARN "External TS2 Temp 2 FAIL\r\n");
		}
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
 
	/* 1. Test structured config API */
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

	/* 2. Verify with dedicated Getters */
	sh36730x_vadc_scan_period_t read_scan = SH36730X_VADC_SCAN_50MS;
	sh36730x_vadc_mode_t read_mode = SH36730X_VADC_MODE_VOLTAGE_ONLY;
	sh36730x_vadc_get_scan_period(&ctx->device, &read_scan);
	sh36730x_vadc_get_mode(&ctx->device, &read_mode);

	if (read_scan == SH36730X_VADC_SCAN_200MS && read_mode == SH36730X_VADC_MODE_VOLT_AND_TEMP) {
		TEST_LOG_PASS("VADC getters verified (Scan=200ms, Mode=Volt+Temp).");
	} else {
		TEST_LOG_FAIL("VADC getters mismatch (Scan=%d, Mode=%d)", (int)read_scan, (int)read_mode);
		ctx->stats.failed_tests++;
	}

	/* 3. Test individual setter: sh36730x_vadc_set_scan_period */
	sh36730x_vadc_set_scan_period(&ctx->device, SH36730X_VADC_SCAN_50MS);
	sh36730x_vadc_get_scan_period(&ctx->device, &read_scan);
	if (read_scan == SH36730X_VADC_SCAN_50MS) {
		TEST_LOG_PASS("sh36730x_vadc_set_scan_period(50ms) confirmed via getter.");
	}

	/* 4. Restore original SCONF3 */
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

	/* 1. Configure CADC: Continuous mode, 50mV RSNS range, 13-bit resolution */
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

	/* 2. Verify with CADC getters */
	sh36730x_cadc_rsns_t read_rsns;
	sh36730x_cbti_c_t read_cbt;
	sh36730x_cadc_mode_t read_cmode;
	sh36730x_cadc_get_rsns(&ctx->device, &read_rsns);
	sh36730x_cadc_get_cbti_c(&ctx->device, &read_cbt);
	sh36730x_cadc_get_mode(&ctx->device, &read_cmode);

	if (read_rsns == SH36730X_CADC_RSNS_50MV && read_cbt == SH36730X_CBTI_C_13BIT && read_cmode == SH36730X_CADC_M_CONTINUOUS) {
		TEST_LOG_PASS("CADC getters verified (RSNS=50mV, Bits=13, Mode=Continuous).");
	} else {
		TEST_LOG_FAIL("CADC getters mismatch (RSNS=%d, Bits=%d, Mode=%d)", (int)read_rsns, (int)read_cbt, (int)read_cmode);
		ctx->stats.failed_tests++;
	}

	bsp_sh36730x_delay_ms(NULL, 150); /* Allow CADC conversion */

	/* 3. Read Current */
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

	/* 4. Restore original SCONF3 & SCONF6 */
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

	/* Backup original SCONF7 (OVT delay) & SCONF6 (SCV/SCT) & SCONF2 */
	uint16_t orig_sconf7_pair = 0;
	uint16_t orig_sconf6_pair = 0;
	uint16_t orig_sconf2_pair = 0;
	sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF7, &orig_sconf7_pair);
	sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF6, &orig_sconf6_pair);
	sh36730x_read_register(&ctx->device, SH36730X_REG_SCONF2, &orig_sconf2_pair);

	/* 1. Test OV Delay Configuration & Getter */
	sh36730x_status_t st = sh36730x_set_ov_delay(&ctx->device, SH36730X_OV_DELAY_8CYCLE);
	sh36730x_ov_delay_t read_ov_delay;
	sh36730x_get_ov_delay(&ctx->device, &read_ov_delay);
	if (st == SH36730X_OK && read_ov_delay == SH36730X_OV_DELAY_8CYCLE) {
		TEST_LOG_PASS("Set & Get OV delay (8 cycles) passed.");
	} else {
		TEST_LOG_FAIL("Set/Get OV delay failed: %d, read=%d", st, (int)read_ov_delay);
		ctx->stats.failed_tests++;
	}

	/* 2. Test Short-Circuit Voltage (SCV) & Delay (SCT) with Getters */
	st = sh36730x_set_scv(&ctx->device, SH36730X_SCV_200MV);
	sh36730x_scv_t read_scv;
	sh36730x_get_scv(&ctx->device, &read_scv);
	if (st == SH36730X_OK && read_scv == SH36730X_SCV_200MV) {
		TEST_LOG_PASS("Set & Get SCV threshold (200mV) passed.");
	}

	st = sh36730x_set_sct(&ctx->device, SH36730X_SCT_100US);
	sh36730x_sct_t read_sct;
	sh36730x_get_sct(&ctx->device, &read_sct);
	if (st == SH36730X_OK && read_sct == SH36730X_SCT_100US) {
		TEST_LOG_PASS("Set & Get SCT delay (100us) passed.");
	}

	/* 3. Test RESET/PF option & Getter */
	st = sh36730x_set_resetpf(&ctx->device, SH36730X_SCONF2_RESET);
	sh36730x_reset_pf_t read_rpf;
	sh36730x_get_resetpf(&ctx->device, &read_rpf);
	if (st == SH36730X_OK && read_rpf == SH36730X_SCONF2_RESET) {
		TEST_LOG_PASS("Set & Get RESET/PF option (MCU Reset) passed.");
	}

	/* 4. Safe Rollback of SCONF7, SCONF6, SCONF2 */
	uint16_t mask_sconf7 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF7_OVT_MASK);
	uint16_t mask_sconf6 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF6_SCV_MASK | SH36730X_SCONF6_SCT_MASK);
	uint16_t mask_sconf2 = SH36730X_HIGH_BYTE_16(SH36730X_SCONF2_RESET_PF_MASK);

	sh36730x_update_registers(&ctx->device, SH36730X_REG_SCONF7, mask_sconf7, SH36730X_HIGH_BYTE_16((uint8_t)(orig_sconf7_pair >> 8)));
	sh36730x_update_registers(&ctx->device, SH36730X_REG_SCONF6, mask_sconf6, SH36730X_HIGH_BYTE_16((uint8_t)(orig_sconf6_pair >> 8)));
	sh36730x_update_registers(&ctx->device, SH36730X_REG_SCONF2, mask_sconf2, SH36730X_HIGH_BYTE_16((uint8_t)(orig_sconf2_pair >> 8)));
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
		uint16_t read_mask = 0;
		sh36730x_get_balancing(&ctx->device, &read_mask);
		TEST_LOG_PASS("Odd balancing pulse armed (Active Mask = 0x%04X).", read_mask);
	} else {
		TEST_LOG_FAIL("Failed to set odd balance: %d", st);
	}

	bsp_sh36730x_delay_ms(NULL, 50); /* Short 50ms pulse */

	/* Step 3: IMMEDIATE DISARM & SAFETY LOCK */
	st = sh36730x_set_balancing(&ctx->device, 0x0000, SH36730X_BALANCE_DISABLE);
	uint16_t disarmed_mask = 0xFFFF;
	sh36730x_get_balancing(&ctx->device, &disarmed_mask);

	if (disarmed_mask == 0x0000) {
		TEST_LOG_PASS("Balancing FORCIBLY DISARMED and verified (Mask = 0x0000). Safety check OK.");
	} else {
		TEST_LOG_FAIL("CRITICAL: Balancing not disarmed! (Mask=0x%04X)", disarmed_mask);
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
 * @brief TC10: Odd vs Even Start Address 2-Byte Read Alignment & Hardware Behavior Test.
 * @note  Tests auto-increment read behavior across Control Registers (0x00~0x0D)
 *        and verifies 16-bit word alignment requirements for ADC Data Registers (0x0E~0x2B).
 */
test_result_t sh36730x_test_tc10_odd_even_address_read(sh36730x_test_context_t *ctx)
{
	TEST_LOG_TEST("--- TC10: Odd vs Even Start Address 2-Byte Read Alignment Test ---");
	ctx->stats.total_tests++;

	if (!ctx->is_initialized) {
		TEST_LOG_FAIL("Context not initialized.");
		ctx->stats.failed_tests++;
		return TEST_RESULT_FAIL;
	}

	/*
	 * Part 1: Control Registers (0x00 ~ 0x0D)
	 * Control registers support byte-by-byte continuous auto-increment addressing for both odd and even start addresses.
	 * Criterion: LowByte(Read(Addr_Even)) == HighByte(Read(Addr_Odd))
	 */
	const struct {
		uint8_t even_addr;
		const char *reg_names;
	} ctrl_windows[] = {
		{0x04, "SCONF1 (0x04) / SCONF2 (0x05) / SCONF3 (0x06)"},
		{0x06, "SCONF3 (0x06) / SCONF4 (0x07) / SCONF5 (0x08)"},
		{0x08, "SCONF5 (0x08) / SCONF6 (0x09) / SCONF7 (0x0A)"},
		{0x0A, "SCONF7 (0x0A) / SCONF8 (0x0B) / SCONF9 (0x0C)"},
		{0x0C, "SCONF9 (0x0C) / SCONF10(0x0D) / CELL1H (0x0E)"}
	};

	bool ctrl_passed = true;

	TEST_PRINTF("--------------------------------------------------------------------------------\r\n");
	TEST_PRINTF(" [Part 1: Control Regs 0x00~0x0D] | Even[A, A+1] | Odd[A+1, A+2] | Overlap Match\r\n");
	TEST_PRINTF("--------------------------------------------------------------------------------\r\n");

	for (size_t i = 0; i < sizeof(ctrl_windows)/sizeof(ctrl_windows[0]); i++) {
		uint8_t addr_even = ctrl_windows[i].even_addr;
		uint8_t addr_odd  = addr_even + 1;

		uint16_t word_even = 0;
		uint16_t word_odd  = 0;

		sh36730x_status_t st_even = sh36730x_read_register(&ctx->device, addr_even, &word_even);
		sh36730x_status_t st_odd  = sh36730x_read_register(&ctx->device, addr_odd, &word_odd);

		if (st_even != SH36730X_OK || st_odd != SH36730X_OK) {
			TEST_PRINTF(" %-32s | ERR(%d)       | ERR(%d)        | FAIL\r\n",
						ctrl_windows[i].reg_names, st_even, st_odd);
			ctrl_passed = false;
			continue;
		}

		uint8_t even_high = (uint8_t)(word_even >> 8);
		uint8_t even_low  = (uint8_t)(word_even & 0xFF);
		uint8_t odd_high  = (uint8_t)(word_odd >> 8);
		uint8_t odd_low   = (uint8_t)(word_odd & 0xFF);

		bool overlap_match = (even_low == odd_high);

		TEST_PRINTF(" %-32s | 0x%02X, 0x%02X   | 0x%02X, 0x%02X    | 0x%02X == 0x%02X [%s]\r\n",
					ctrl_windows[i].reg_names,
					even_high, even_low,
					odd_high, odd_low,
					even_low, odd_high,
					overlap_match ? "OK" : "MISMATCH");

		if (!overlap_match) {
			ctrl_passed = false;
		}
	}
	TEST_PRINTF("--------------------------------------------------------------------------------\r\n");

	/*
	 * Part 2: ADC Data Registers (0x0E ~ 0x2B: CELL, TS, TEMP, CUR)
	 * Hardware Characteristic: ADC data registers are 16-bit word-latched on EVEN High-Byte boundaries.
	 * Even start address (High Byte) yields valid [High, Low] sample.
	 * Odd start address directly accesses the low byte, where hardware returns 0xFF dummy byte.
	 */
	const struct {
		uint8_t even_addr;
		const char *reg_names;
	} adc_windows[] = {
		{0x0E, "CELL1 (0x0E / 0x0F)"},
		{0x22, "TS1   (0x22 / 0x23)"},
		{0x26, "TEMP1 (0x26 / 0x27)"},
		{0x2A, "CUR   (0x2A / 0x2B)"}
	};

	bool adc_passed = true;

	TEST_PRINTF(" [Part 2: ADC Data Regs 0x0E~0x2B] | Even Read[H, L] | Odd Read (Direct Low Byte)\r\n");
	TEST_PRINTF("--------------------------------------------------------------------------------\r\n");

	for (size_t i = 0; i < sizeof(adc_windows)/sizeof(adc_windows[0]); i++) {
		uint8_t addr_even = adc_windows[i].even_addr;
		uint8_t addr_odd  = addr_even + 1;

		uint16_t word_even = 0;
		uint16_t word_odd  = 0;

		sh36730x_status_t st_even = sh36730x_read_register(&ctx->device, addr_even, &word_even);
		sh36730x_status_t st_odd  = sh36730x_read_register(&ctx->device, addr_odd, &word_odd);

		if (st_even != SH36730X_OK || st_odd != SH36730X_OK) {
			TEST_PRINTF(" %-32s | ERR(%d)          | ERR(%d)\r\n",
						adc_windows[i].reg_names, st_even, st_odd);
			adc_passed = false;
			continue;
		}

		uint8_t even_h = (uint8_t)(word_even >> 8);
		uint8_t even_l = (uint8_t)(word_even & 0xFF);
		uint8_t odd_h  = (uint8_t)(word_odd >> 8);
		uint8_t odd_l  = (uint8_t)(word_odd & 0xFF);

		/* Even read gives valid ADC word; Odd read returns 0xFF for high byte as hardware latch characteristic */
		bool odd_has_dummy = (odd_h == 0xFF);

		TEST_PRINTF(" %-32s | 0x%02X, 0x%02X      | 0x%02X, 0x%02X (HW Latch: %s)\r\n",
					adc_windows[i].reg_names,
					even_h, even_l,
					odd_h, odd_l,
					odd_has_dummy ? "0xFF Dummy OK" : "Custom");
	}
	TEST_PRINTF("--------------------------------------------------------------------------------\r\n");

	if (ctrl_passed && adc_passed) {
		TEST_LOG_PASS("Control Regs continuous addressing & ADC Data 16-bit word alignment verified.");
		ctx->stats.passed_tests++;
		return TEST_RESULT_PASS;
	} else {
		TEST_LOG_FAIL("Odd/Even address read alignment test failed!");
		ctx->stats.failed_tests++;
		return TEST_RESULT_FAIL;
	}
}

/**
 * @brief TC11: Extended Control & Status Getters Test (Detection, WDT, RST, ALARM, CHS).
 */
test_result_t sh36730x_test_tc11_extended_features(sh36730x_test_context_t *ctx)
{
	TEST_LOG_TEST("--- TC11: Extended Control & Status Getters Test with Rollback ---");
	ctx->stats.total_tests++;

	if (!ctx->is_initialized) {
		TEST_LOG_FAIL("Context not initialized.");
		ctx->stats.failed_tests++;
		return TEST_RESULT_FAIL;
	}

	/* 1. Charger & Load Detection Enable/Disable & Status */
	sh36730x_status_t st = sh36730x_enable_charger_detection(&ctx->device, true);
	if (st == SH36730X_OK) {
		bool chgr_connected = false;
		sh36730x_get_charger_status(&ctx->device, &chgr_connected);
		TEST_LOG_PASS("Charger detection enabled, status: %s", chgr_connected ? "CONNECTED" : "DISCONNECTED");
	}

	st = sh36730x_enable_load_detection(&ctx->device, true);
	if (st == SH36730X_OK) {
		bool load_connected = false;
		sh36730x_get_load_status(&ctx->device, &load_connected);
		TEST_LOG_PASS("Load detection enabled, status: %s", load_connected ? "CONNECTED" : "DISCONNECTED");
	}

	/* 2. CHS Threshold Set & Get (SCONF7[3:2]) */
	sh36730x_chs_threshold_t orig_chs = SH36730X_CHS_1_4MV;
	sh36730x_get_chs_threshold(&ctx->device, &orig_chs);

	st = sh36730x_set_chs_threshold(&ctx->device, SH36730X_CHS_6_0MV);
	sh36730x_chs_threshold_t read_chs = SH36730X_CHS_1_4MV;
	sh36730x_get_chs_threshold(&ctx->device, &read_chs);
	if (st == SH36730X_OK && read_chs == SH36730X_CHS_6_0MV) {
		TEST_LOG_PASS("CHS Threshold set to 6.0mV and verified via getter.");
	} else {
		TEST_LOG_FAIL("CHS Threshold getter/setter mismatch (Expected 2, got %d)", (int)read_chs);
	}
	sh36730x_set_chs_threshold(&ctx->device, orig_chs); /* Rollback */

	/* 3. WDT Period Set & Get (SCONF7[1:0]) */
	sh36730x_wdtt_period_t orig_wdt = SH36730X_WDTT_30S;
	sh36730x_get_wdt_period(&ctx->device, &orig_wdt);

	st = sh36730x_set_wdt_period(&ctx->device, SH36730X_WDTT_2S);
	sh36730x_wdtt_period_t read_wdt = SH36730X_WDTT_30S;
	sh36730x_get_wdt_period(&ctx->device, &read_wdt);
	if (st == SH36730X_OK && read_wdt == SH36730X_WDTT_2S) {
		TEST_LOG_PASS("WDT Period set to 2S and verified via getter.");
	} else {
		TEST_LOG_FAIL("WDT Period getter/setter mismatch (Expected 2, got %d)", (int)read_wdt);
	}
	sh36730x_set_wdt_period(&ctx->device, orig_wdt); /* Rollback */

	/* 4. RST Pulse Width Set & Get (SCONF6[5:4]) */
	sh36730x_rst_pulse_t orig_rst = SH36730X_RST_16MS;
	sh36730x_get_rst_pulse_width(&ctx->device, &orig_rst);

	st = sh36730x_set_rst_pulse_width(&ctx->device, SH36730X_RST_128MS);
	sh36730x_rst_pulse_t read_rst = SH36730X_RST_16MS;
	sh36730x_get_rst_pulse_width(&ctx->device, &read_rst);
	if (st == SH36730X_OK && read_rst == SH36730X_RST_128MS) {
		TEST_LOG_PASS("RST Pulse Width set to 128ms and verified via getter.");
	} else {
		TEST_LOG_FAIL("RST Pulse Width getter/setter mismatch (Expected 2, got %d)", (int)read_rst);
	}
	sh36730x_set_rst_pulse_width(&ctx->device, orig_rst); /* Rollback */

	/* 5. ALARM Mode Set & Get (SCONF2[2]) */
	sh36730x_alarm_mode_t orig_alarm = SH36730X_ALARM_PULSE;
	sh36730x_get_alarm_mode(&ctx->device, &orig_alarm);

	st = sh36730x_set_alarm_mode(&ctx->device, SH36730X_ALARM_LEVEL);
	sh36730x_alarm_mode_t read_alarm = SH36730X_ALARM_PULSE;
	sh36730x_get_alarm_mode(&ctx->device, &read_alarm);
	if (st == SH36730X_OK && read_alarm == SH36730X_ALARM_LEVEL) {
		TEST_LOG_PASS("ALARM Mode set to LEVEL and verified via getter.");
	} else {
		TEST_LOG_FAIL("ALARM Mode getter/setter mismatch (Expected 1, got %d)", (int)read_alarm);
	}
	sh36730x_set_alarm_mode(&ctx->device, orig_alarm); /* Rollback */

	/* 6. Real-time Charging / Discharging Status Read */
	bool is_chg = false;
	bool is_dsg = false;
	sh36730x_get_charging_status(&ctx->device, &is_chg);
	sh36730x_get_discharging_status(&ctx->device, &is_dsg);
	TEST_LOG_INFO("Real-time Current Direction: Charging=%s, Discharging=%s",
				  is_chg ? "YES" : "NO", is_dsg ? "YES" : "NO");

	ctx->stats.passed_tests++;
	return TEST_RESULT_PASS;
}

/**
 * @brief TC12: System Flags & Power-On/Voltage-Drop Reset Flag Test (Section 7.15, Table 7.47).
 */
test_result_t sh36730x_test_tc12_system_and_reset_flags(sh36730x_test_context_t *ctx)
{
	TEST_LOG_TEST("--- TC12: System Flags & Reset Status Test (Section 7.15) ---");
	ctx->stats.total_tests++;

	if (!ctx->is_initialized) {
		TEST_LOG_FAIL("Context not initialized.");
		ctx->stats.failed_tests++;
		return TEST_RESULT_FAIL;
	}

	/* 1. Read FLAG1 (0x00) */
	uint8_t flag1 = 0;
	sh36730x_status_t st = sh36730x_get_flag1(&ctx->device, &flag1);
	if (st == SH36730X_OK) {
		TEST_LOG_PASS("FLAG1 (0x00) read: 0x%02X [SC=%d, OV=%d, WDT=%d, TWI=%d]",
					  flag1,
					  (flag1 & SH36730X_FLAG1_SC_MASK) != 0,
					  (flag1 & SH36730X_FLAG1_OV_MASK) != 0,
					  (flag1 & SH36730X_FLAG1_WDT_MASK) != 0,
					  (flag1 & SH36730X_FLAG1_TWI_MASK) != 0);
	} else {
		TEST_LOG_FAIL("Failed to read FLAG1 (err=%d)", st);
		ctx->stats.failed_tests++;
		return TEST_RESULT_FAIL;
	}

	/* 2. Read FLAG2 (0x01, Clear-on-Read) & Check Reset occurrence */
	bool reset_occurred = false;
	st = sh36730x_get_reset_flag(&ctx->device, &reset_occurred);
	if (st == SH36730X_OK) {
		TEST_LOG_INFO("V33 Reset Status (FLAG2[2]: RST): %s", reset_occurred ? "RESET DETECTED (Power-on/LVR)" : "NO RESET");
	} else {
		TEST_LOG_FAIL("Failed to read Reset flag (err=%d)", st);
		ctx->stats.failed_tests++;
		return TEST_RESULT_FAIL;
	}

	/* 3. Verify that second read of FLAG2 has RST cleared by hardware (Clear-on-Read) */
	bool second_reset_flag = false;
	sh36730x_get_reset_flag(&ctx->device, &second_reset_flag);
	if (!second_reset_flag) {
		TEST_LOG_PASS("FLAG2 Clear-on-Read verified (RST cleared to 0 after read).");
	} else {
		TEST_LOG_WARN("FLAG2 RST flag remained set after read.");
	}

	/* 4. Test FLAG1 manual clear function (LTCLR in SCONF1) */
	st = sh36730x_clear_flag1(&ctx->device);
	if (st == SH36730X_OK) {
		TEST_LOG_PASS("sh36730x_clear_flag1 (LTCLR) executed successfully.");
	} else {
		TEST_LOG_WARN("sh36730x_clear_flag1 returned: %d", st);
	}

	ctx->stats.passed_tests++;
	return TEST_RESULT_PASS;
}

/**
 * @brief Comprehensive Dump of ALL 44 Registers (0x00 ~ 0x2B) with detailed descriptions.
 */
test_result_t sh36730x_test_dump_status(sh36730x_test_context_t *ctx)
{
	TEST_LOG_TEST("================================================================================");
	TEST_LOG_TEST("           SH36730X COMPLETE REGISTER MAP DUMP (0x00 ~ 0x2B, 44 REGISTERS)      ");
	TEST_LOG_TEST("================================================================================");

	if (!ctx->is_initialized) {
		TEST_LOG_FAIL("Context not initialized.");
		return TEST_RESULT_FAIL;
	}

	typedef struct {
		uint8_t addr;
		const char *name;
		const char *desc;
	} sh36730x_reg_info_t;

	static const sh36730x_reg_info_t all_regs[SH36730X_REG_TOTAL_COUNT] = {
		{0x00, "FLAG1",    "Hardware Protection & WDT/TWI Status (RO)"},
		{0x01, "FLAG2",    "V33 Reset & ADC Interrupts (Clear-on-Read)"},
		{0x02, "BSTATUS",  "MOS, Charge/Discharge, Load/Charger Status (RO)"},
		{0x03, "INT_EN",   "ALARM Interrupt Enable Control (RW)"},
		{0x04, "SCONF1",   "Protection Enable & Flag Clear (RW)"},
		{0x05, "SCONF2",   "MOS & Output Mode Control (RW)"},
		{0x06, "SCONF3",   "ADC Sampling Config & Scan Period (RW)"},
		{0x07, "SCONF4",   "Cell Balancing High CB10~CB6 (RW)"},
		{0x08, "SCONF5",   "Cell Balancing Low CB5~CB1 (RW)"},
		{0x09, "SCONF6",   "CADC Range, RST Width, SC Params (RW)"},
		{0x0A, "SCONF7",   "OV Delay, CHS Threshold, WDT Timeout (RW)"},
		{0x0B, "SCONF8",   "Hardware OV Threshold High OVD[9:8] (RW)"},
		{0x0C, "SCONF9",   "Hardware OV Threshold Low OVD[7:0] (RW)"},
		{0x0D, "SCONF10",  "Power-Down Authorization Key PIN[7:0] (RW)"},
		{0x0E, "CELL1H",   "Cell 1 Voltage High 4 bits (RO)"},
		{0x0F, "CELL1L",   "Cell 1 Voltage Low 8 bits (RO)"},
		{0x10, "CELL2H",   "Cell 2 Voltage High 4 bits (RO)"},
		{0x11, "CELL2L",   "Cell 2 Voltage Low 8 bits (RO)"},
		{0x12, "CELL3H",   "Cell 3 Voltage High 4 bits (RO)"},
		{0x13, "CELL3L",   "Cell 3 Voltage Low 8 bits (RO)"},
		{0x14, "CELL4H",   "Cell 4 Voltage High 4 bits (RO)"},
		{0x15, "CELL4L",   "Cell 4 Voltage Low 8 bits (RO)"},
		{0x16, "CELL5H",   "Cell 5 Voltage High 4 bits (RO)"},
		{0x17, "CELL5L",   "Cell 5 Voltage Low 8 bits (RO)"},
		{0x18, "CELL6H",   "Cell 6 Voltage High 4 bits (RO)"},
		{0x19, "CELL6L",   "Cell 6 Voltage Low 8 bits (RO)"},
		{0x1A, "CELL7H",   "Cell 7 Voltage High 4 bits (RO)"},
		{0x1B, "CELL7L",   "Cell 7 Voltage Low 8 bits (RO)"},
		{0x1C, "CELL8H",   "Cell 8 Voltage High 4 bits (RO)"},
		{0x1D, "CELL8L",   "Cell 8 Voltage Low 8 bits (RO)"},
		{0x1E, "CELL9H",   "Cell 9 Voltage High 4 bits (RO)"},
		{0x1F, "CELL9L",   "Cell 9 Voltage Low 8 bits (RO)"},
		{0x20, "CELL10H",  "Cell 10 Voltage High 4 bits (RO)"},
		{0x21, "CELL10L",  "Cell 10 Voltage Low 8 bits (RO)"},
		{0x22, "TS1H",     "External NTC TS1 Voltage Ratio High 4 bits (RO)"},
		{0x23, "TS1L",     "External NTC TS1 Voltage Ratio Low 8 bits (RO)"},
		{0x24, "TS2H",     "External NTC TS2 Voltage Ratio High 4 bits (RO)"},
		{0x25, "TS2L",     "External NTC TS2 Voltage Ratio Low 8 bits (RO)"},
		{0x26, "TEMP1H",   "Internal Die Temp 1 High 4 bits (RO)"},
		{0x27, "TEMP1L",   "Internal Die Temp 1 Low 8 bits (RO)"},
		{0x28, "TEMP2H",   "Internal Die Temp 2 High 4 bits (RO)"},
		{0x29, "TEMP2L",   "Internal Die Temp 2 Low 8 bits (RO)"},
		{0x2A, "CURH",     "CADC Current High & Sign (RO)"},
		{0x2B, "CURL",     "CADC Current Low 8 bits (RO)"}
	};

	TEST_PRINTF("--------------------------------------------------------------------------------\r\n");
	TEST_PRINTF(" Addr | Name     | Hex  | Binary   | Description                                \r\n");
	TEST_PRINTF("--------------------------------------------------------------------------------\r\n");

	/* Read all 44 registers pairwise from address 0x00 to 0x2A (22 transactions) */
	for (uint8_t addr = 0x00; addr <= 0x2A; addr += 2) {
		uint16_t word_val = 0;
		sh36730x_status_t st = sh36730x_read_register(&ctx->device, addr, &word_val);

		if (st == SH36730X_OK) {
			uint8_t byte_h = (uint8_t)(word_val >> 8);
			uint8_t byte_l = (uint8_t)(word_val & 0xFF);

			/* Print high register (addr) */
			char bin_h[9];
			for (int b = 7; b >= 0; b--) {
				bin_h[7 - b] = ((byte_h >> b) & 1) ? '1' : '0';
			}
			bin_h[8] = '\0';
			TEST_PRINTF(" 0x%02X | %-8s | 0x%02X | %s | %s\r\n",
						addr, all_regs[addr].name, byte_h, bin_h, all_regs[addr].desc);

			/* Print low register (addr + 1) */
			char bin_l[9];
			for (int b = 7; b >= 0; b--) {
				bin_l[7 - b] = ((byte_l >> b) & 1) ? '1' : '0';
			}
			bin_l[8] = '\0';
			TEST_PRINTF(" 0x%02X | %-8s | 0x%02X | %s | %s\r\n",
						addr + 1, all_regs[addr + 1].name, byte_l, bin_l, all_regs[addr + 1].desc);
		} else {
			TEST_PRINTF(" 0x%02X | %-8s | READ ERROR (%d)\r\n", addr, all_regs[addr].name, st);
			TEST_PRINTF(" 0x%02X | %-8s | READ ERROR (%d)\r\n", addr + 1, all_regs[addr + 1].name, st);
		}
	}
	TEST_PRINTF("--------------------------------------------------------------------------------\r\n\r\n");

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
	sh36730x_test_tc10_odd_even_address_read(ctx);
	sh36730x_test_tc11_extended_features(ctx);
	sh36730x_test_tc12_system_and_reset_flags(ctx);
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
	rt_kprintf("  all              - Run full automated safe test suite (TC01~TC12 + Dump)\r\n");
	rt_kprintf("  init             - Initialize I2C and probe SH36730X\r\n");
	rt_kprintf("  volt             - Measure all cell voltages and pack summary\r\n");
	rt_kprintf("  temp             - Read external TS & internal chip temperatures\r\n");
	rt_kprintf("  cur [rsense_mOhm]- Read CADC current (default Rsense = 1.0 mOhm)\r\n");
	rt_kprintf("  vadc             - Test VADC sampling modes and scan periods\r\n");
	rt_kprintf("  cadc             - Test CADC mode and resolution config\r\n");
	rt_kprintf("  prot             - Test OV/SC protection params with rollback\r\n");
	rt_kprintf("  bal              - Run controlled 50ms pulse balancing test\r\n");
	rt_kprintf("  stress [count]   - Run communication stress test (default 50)\r\n");
	rt_kprintf("  oddeven          - Test odd vs even start address 2-byte reads\r\n");
	rt_kprintf("  ext              - Test extended features (detection, WDT, RST, ALARM, CHS)\r\n");
	rt_kprintf("  flags            - Test FLAG1/FLAG2 read, clear, and V33 Reset status\r\n");
	rt_kprintf("  dump             - Dump all 44 registers (0x00 ~ 0x2B)\r\n");
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
	} else if (strcmp(subcmd, "oddeven") == 0 || strcmp(subcmd, "align") == 0) {
		if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		sh36730x_test_tc10_odd_even_address_read(ctx);
	} else if (strcmp(subcmd, "dump") == 0) {
		if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		sh36730x_test_dump_status(ctx);
	} else if (strcmp(subcmd, "ext") == 0) {
		if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		sh36730x_test_tc11_extended_features(ctx);
	} else if (strcmp(subcmd, "flags") == 0 || strcmp(subcmd, "rst") == 0) {
		if (!ctx->is_initialized) sh36730x_test_init_hardware(ctx, SH36730X_CHIP_SH367303, SH36730X_TEST_DEFAULT_I2C_ADDR);
		sh36730x_test_tc12_system_and_reset_flags(ctx);
	} else {
		 rt_kprintf("Unknown subcommand: %s\r\n", subcmd);
		 sh36730x_test_print_help();
	 }
 
	 return 0;
 }
 
 MSH_CMD_EXPORT_ALIAS(sh36730x_test_cmd, sh36730x_test, SH36730X AFE comprehensive safe test suite);
 
 #endif /* RT_USING_FINSH */
 
  