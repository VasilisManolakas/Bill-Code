/*
 * clamp.h
 *
 *  Created on: Sep 7, 2025
 *      Author: Vasilis Manolakas
 */
#ifndef UTIL_CLAMP_H
#define UTIL_CLAMP_H

#include <stdint.h>
#include <limits.h>
#pragma once

static  int16_t clamp_i16(long v) {
    if (v < INT16_MIN) return INT16_MIN;
    if (v > INT16_MAX) return INT16_MAX;
    return (int16_t)v;
}

static int16_t clamp_i16_range(long v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (int16_t)v;
}

#endif // UTIL_CLAMP_H

