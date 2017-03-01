#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>
#include <stdio.h>
#include <stdint.h>


#include "utils.h"



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TESTS FUNCTION PROTOTYPES
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int TCPClient_tests(void);
/*****************************************************************************/
int SimClient_tests(void);
/*****************************************************************************/


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TESTS FOR Utils MODULE.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void test_f_to_scaled_u32(void** state) {
	float var = 0.02f;
	uint32_t scaling = 1000000;
	uint32_t u32_var = f_to_scaled_u32(var, scaling);
	assert_int_equal(20000, u32_var);
}

void test_f_to_scaled_i32(void** state) {
	float var = -0.02f;
	uint32_t scaling = 1000;
	int32_t i32_var = f_to_scaled_i32(var, scaling);
	assert_int_equal(-20, i32_var);
}

void test_u32_to_u8_arr(void** state) {
	uint32_t var = 0xffeeaabb;
	uint8_t expected_arr[] = { 0xff, 0xee, 0xaa, 0xbb };
	uint8_t var_arr[4];
	u32_to_u8_arr(var, var_arr);
	assert_memory_equal(var_arr, expected_arr, sizeof(uint32_t));
}

void test_i32_to_u8_arr(void** state) {
	int32_t var = -1;
	uint8_t expected_arr[] = { 0xff, 0xff, 0xff, 0xff };
	uint8_t var_arr[4];
	i32_to_u8_arr(var, var_arr);
	assert_memory_equal(var_arr, expected_arr, sizeof(int32_t));
}

void test_u8_arr_to_i32(void** state) {
	uint8_t max_arr[] = { 0xff, 0xff, 0xff, 0xff };
	uint8_t zero_arr[] = { 0x00, 0x00, 0x00, 0x00 };
	assert_int_equal(u8_arr_to_i32(max_arr), -1);
	assert_int_equal(u8_arr_to_i32(zero_arr), 0);
}

void test_u8_arr_to_u32(void **state) {
	uint8_t max_arr[] = { 0xff, 0xff, 0xff, 0xff };
	uint8_t zero_arr[] = { 0x00, 0x00, 0x00, 0x00 };
	assert_int_equal(u8_arr_to_u32(max_arr), 0xffffffff);
	assert_int_equal(u8_arr_to_i32(zero_arr), 0);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// END OF TESTS: Utils.
/*****************************************************************************/

/* A test case that does nothing and succeeds. */
static void null_test_success(void **state) {
	(void)state; /* unused */
}

int RunAllTests(void) {
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	TESTS: TCPClient MODULE.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	(void)TCPClient_tests();

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	TESTS: TCPClient MODULE.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	(void)SimClient_tests();
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	const struct CMUnitTest tests[] = {
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TESTS: Dummy.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		cmocka_unit_test(null_test_success),
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TESTS: Utils.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		cmocka_unit_test(test_f_to_scaled_i32),
		cmocka_unit_test(test_f_to_scaled_u32),
		cmocka_unit_test(test_i32_to_u8_arr),
		cmocka_unit_test(test_u32_to_u8_arr),
		cmocka_unit_test(test_u8_arr_to_u32),
		cmocka_unit_test(test_u8_arr_to_i32),
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	};
	return cmocka_run_group_tests(tests, NULL, NULL);
}