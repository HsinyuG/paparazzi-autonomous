/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H

#ifndef AVOID_OPTICFLOW_ID
#define AVOID_OPTICFLOW_ID 10
#endif

#include "modules/ahrs/ahrs.h"
#include "modules/ahrs/ahrs_int_cmpl_quat.h"
#include "modules/ahrs/ahrs_aligner.h"
#include "modules/ins/ins.h"

// settings
extern float oa_color_count_frac;
extern float angular_vel;
extern float maxDistance;
extern float k_vel;
extern float k_psi;
extern float k_lpf;
extern float opticflow_free_space_threshold;

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

extern void opticflow_avoider_init(void);
extern void opticflow_avoider_periodic(void);

#endif

