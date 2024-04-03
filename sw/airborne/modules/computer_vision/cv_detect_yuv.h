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

#ifndef GREEN_DETECTOR
#define GREEN_DETECTOR 0
#endif
#ifndef GROUND_DETECTOR
#define GROUND_DETECTOR 1
#endif
#ifndef EDGE_DETECTOR
#define EDGE_DETECTOR 2
#endif

// Module settings
// this is the yuv threshold for the floor detection
extern uint8_t cod_lum_min1;
extern uint8_t cod_lum_max1;
extern uint8_t cod_cb_min1;
extern uint8_t cod_cb_max1;
extern uint8_t cod_cr_min1;
extern uint8_t cod_cr_max1;

extern uint8_t cod_lum_min2;
extern uint8_t cod_lum_max2;
extern uint8_t cod_cb_min2;
extern uint8_t cod_cb_max2;
extern uint8_t cod_cr_min2;
extern uint8_t cod_cr_max2;

extern bool detect_tree;

//this is the yuv threshold for the tree detection
extern uint8_t tree_y_min;
extern uint8_t tree_y_max;
extern uint8_t tree_u_min;
extern uint8_t tree_u_max;
extern uint8_t tree_v_min;
extern uint8_t tree_v_max;
extern uint8_t tree_top_height; // only camera 1 is front camera

extern bool cod_draw1;
extern bool cod_draw2;
extern bool small_window;// True: use a smaller window in the bottom part of the image
extern float compare_threshold;// True: compare the number of the green pixel in the left and right part of the image 

extern float sigma1;
extern float tlow1;
extern float thigh1;
extern uint16_t pixel_variance_threshold;

extern uint32_t threshold_middle;// threshold for the number of the pixels in the middle part of the image(for ground detection)
extern uint32_t threshold_sideways;// threshold for the number of the pixels in the left and right part of the image(for ground detection)

extern uint32_t middle_threshold_pixel;
extern uint32_t sideways_threshold_pixel;
extern uint32_t tree_threshold_pixel;// threshold for the number of the pixels that stands for a tree in the top part of the image

//this is the variables for the detection window
extern float image_middle_proportion;
extern float middle_threshold_proportion;// threshold for the number of the pixels in the middle part of the image(for color detection)
extern float sideways_threshold_proportion;// threshold for the number of the pixels in the left and right part of the image(for color detection)
extern float tree_threshold_proportion;
extern uint8_t green_bottom_height;// the height of the detection window in the bottom of the image
extern uint8_t detection_mode;

extern float edge_detect_proportion;

// Module functions
extern void color_object_detector_init(void);
extern void color_object_detector_periodic(void);

#endif /* COLOR_OBJECT_DETECTOR_CV_H */
