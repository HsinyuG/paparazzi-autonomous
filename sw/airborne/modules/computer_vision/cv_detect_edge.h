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

#ifndef EDGE_DETECTOR_CV_H
#define EDGH_DETECTOR_CV_H

#include <stdint.h>
#include <stdbool.h>

extern float sigma1;
extern float tlow1;
extern float thigh1;
extern uint16_t pixel_variance_threshold;

extern float sigma2;
extern float tlow2;
extern float thigh2;

extern bool cod_draw1;
extern bool cod_draw2;

// Module functions
extern void edge_detector_init(void);
extern void edge_detector_periodic(void);

#endif /* COLOR_OBJECT_DETECTOR_CV_H */
