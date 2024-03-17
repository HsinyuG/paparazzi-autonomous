/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_color_object.h
 * Assumes the color_object consists of a continuous color and checks
 * if you are over the defined color_object or not
 */

#ifndef COLOR_OBJECT_DETECTOR_CV_H
#define COLOR_OBJECT_DETECTOR_CV_H

#include <stdint.h>
#include <stdbool.h>
#ifndef ACTION_LEFT
#define ACTION_LEFT 0
#endif
#ifndef ACTION_FORWARD_LEFT
#define ACTION_FORWARD_LEFT 1
#endif
#ifndef ACTION_FORWARD
#define ACTION_FORWARD 2
#endif
#ifndef ACTION_FORWARD_RIGHT
#define ACTION_FORWARD_RIGHT 3
#endif
#ifndef ACTION_RIGHT
#define ACTION_RIGHT 4
#endif
// Module settings
extern uint8_t cod_lum_min1;
extern uint8_t cod_lum_max1;
extern uint8_t cod_cb_min1;
extern uint8_t cod_cb_max1;
extern uint8_t cod_cr_min1;
extern uint8_t cod_cr_max1;
extern uint8_t cod_bottom_height1;

extern uint8_t cod_lum_min2;
extern uint8_t cod_lum_max2;
extern uint8_t cod_cb_min2;
extern uint8_t cod_cb_max2;
extern uint8_t cod_cr_min2;
extern uint8_t cod_cr_max2;
extern uint8_t cod_bottom_height2;

extern bool cod_draw1;
extern bool cod_draw2;

extern float num_parts;
extern float green_threshold_proportion;

// Module functions
extern void color_object_detector_init(void);
extern void color_object_detector_periodic(void);

#endif /* COLOR_OBJECT_DETECTOR_CV_H */
