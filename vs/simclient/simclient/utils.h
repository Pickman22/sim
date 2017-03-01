#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

uint32_t f_to_scaled_u32(float var, uint32_t scaling);

int32_t f_to_scaled_i32(float var, uint32_t scaling);

void u32_to_u8_arr(uint32_t var, uint8_t* arr);

void i32_to_u8_arr(int32_t var, uint8_t* arr);

uint32_t u8_arr_to_u32(uint8_t const * const arr);

int32_t u8_arr_to_i32(uint8_t const * const arr);

#endif