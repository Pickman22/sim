#include "utils.h"
#include <assert.h>
#include <string.h>

uint32_t f_to_scaled_u32(float var, uint32_t scaling) {
	return (uint32_t)(var * scaling);
}

int32_t f_to_scaled_i32(float var, uint32_t scaling) {
	return (int32_t)(var * scaling);
}

void u32_to_u8_arr(uint32_t var, uint8_t* arr) {
	assert(arr);
	memset(arr, 0, sizeof(uint32_t));
	arr[0] = (uint8_t)(var >> 24);
	arr[1] = (uint8_t)(var >> 16);
	arr[2] = (uint8_t)(var >> 8);
	arr[3] = (uint8_t)(var);
}

void i32_to_u8_arr(int32_t var, uint8_t* arr) {
	assert(arr);
	u32_to_u8_arr((uint32_t)var, arr);
}

uint32_t u8_arr_to_u32(uint8_t const * const arr) {
	assert(arr);
	uint32_t tmp = arr[0];
	tmp |= arr[1] << 8;
	tmp |= arr[2] << 16;
	tmp |= arr[3] << 24;
	return tmp;
}

int32_t u8_arr_to_i32(uint8_t const * const arr) {
	assert(arr);
	return (int32_t)u8_arr_to_u32(arr);
}