#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

union fl32u8
{
    float fl;
    uint8_t u8[4];
};

union fl32i8
{
    float fl;
    int8_t i8[4];
};

union u32u8
{
    uint32_t u32;
    uint8_t u8[4];
};

union i32i8
{
    int32_t i32;
    int8_t i8[4];
};  

union i32u8
{
    int32_t i32;
    uint8_t u8[4];
};

#endif // TYPES_H