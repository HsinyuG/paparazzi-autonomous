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

// settings
extern float oa_color_count_frac;
extern float angular_vel;
extern float maxDistance;
extern float k_vel;

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

extern void edge_avoider_init(void);
extern void edge_avoider_periodic(void);

#endif

