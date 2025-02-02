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
// #include "cv_detect_color_object.h"
#include "cv_detect_yuv.h"
// settings
extern float oa_color_count_frac;

extern float maxDistance;
extern bool random_rotate_avoid;//True: make a random turn when the obstacle is found.
extern bool compare_middle_left_right;
extern bool enable_bounds_detect;//True: enable optitrack to help the drone to stay in Cyberzoo.
extern bool slight_turn;//True: the drone will use slight turn to slightly change the direction when it found the left or right should be a better direction.
extern uint16_t threshold_move_back;
extern float proportion_move_back;
extern float proportion_predict;
extern float ang_vel;// the angular velocity of the drone to make a turn.
extern bool use_vel_control;
extern float forward_vel;// forward velocity for the drone to move forward.
extern float backward_vel;// backward velocity for the drone to stop.
extern uint16_t min_accel_cnt;
extern float hysteresis_coeff;

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

extern void green_tracker_init(void);
extern void green_tracker_periodic(void);

#endif
