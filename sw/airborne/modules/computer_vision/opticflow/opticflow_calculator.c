/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2016 Kimberly McGuire <k.n.mcguire@tudelft.nl
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow/opticflow_calculator.c
 * @brief Estimate velocity from optic flow.
 *
 */

#include "std.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Own Header
#include "opticflow_calculator.h"

// Computer Vision
#include "lib/vision/image.h"
#include "lib/vision/lucas_kanade.h"
#include "lib/vision/fast_rosten.h"
#include "lib/vision/act_fast.h"
#include "lib/vision/edge_flow.h"
#include "lib/vision/undistortion.h"
#include "size_divergence.h"
#include "linear_flow_fit.h"
#include "modules/sonar/agl_dist.h"
#include "generated/airframe.h"

// to get the definition of front_camera / bottom_camera
#include BOARD_CONFIG

// whether to show the flow and corners:
#define OPTICFLOW_SHOW_CORNERS 0

#define EXHAUSTIVE_FAST 0
#define ACT_FAST 1
// TODO: these are now adapted, but perhaps later could be a setting:
uint16_t n_time_steps[2] = {10, 10};
uint16_t n_agents[2] = {25, 25};

// What methods are run to determine divergence, lateral flow, etc.
// SIZE_DIV looks at line sizes and only calculates divergence
#define SIZE_DIV 1
// LINEAR_FIT makes a linear optical flow field fit and extracts a lot of information:
// relative velocities in x, y, z (divergence / time to contact), the slope of the surface, and the surface roughness.
#define LINEAR_FIT 0

#ifndef OPTICFLOW_CORNER_METHOD
#define OPTICFLOW_CORNER_METHOD ACT_FAST
#endif

#ifndef OPTICFLOW_CORNER_METHOD_CAMERA2
#define OPTICFLOW_CORNER_METHOD_CAMERA2 ACT_FAST
#endif
PRINT_CONFIG_VAR(OPTICFLOW_CORNER_METHOD)
PRINT_CONFIG_VAR(OPTICFLOW_CORNER_METHOD_CAMERA2)

/* Set the default values */
#ifndef OPTICFLOW_MAX_TRACK_CORNERS
#define OPTICFLOW_MAX_TRACK_CORNERS 25
#endif

#ifndef OPTICFLOW_MAX_TRACK_CORNERS_CAMERA2
#define OPTICFLOW_MAX_TRACK_CORNERS_CAMERA2 25
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_TRACK_CORNERS)
PRINT_CONFIG_VAR(OPTICFLOW_MAX_TRACK_CORNERS_CAMERA2)

#ifndef OPTICFLOW_WINDOW_SIZE
#define OPTICFLOW_WINDOW_SIZE 10
#endif

#ifndef OPTICFLOW_WINDOW_SIZE_CAMERA2
#define OPTICFLOW_WINDOW_SIZE_CAMERA2 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_WINDOW_SIZE)
PRINT_CONFIG_VAR(OPTICFLOW_WINDOW_SIZE_CAMERA2)

#ifndef OPTICFLOW_SEARCH_DISTANCE
#define OPTICFLOW_SEARCH_DISTANCE 20
#endif

#ifndef OPTICFLOW_SEARCH_DISTANCE_CAMERA2
#define OPTICFLOW_SEARCH_DISTANCE_CAMERA2 20
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SEARCH_DISTANCE)
PRINT_CONFIG_VAR(OPTICFLOW_SEARCH_DISTANCE_CAMERA2)

#ifndef OPTICFLOW_SUBPIXEL_FACTOR
#define OPTICFLOW_SUBPIXEL_FACTOR 10
#endif

#ifndef OPTICFLOW_SUBPIXEL_FACTOR_CAMERA2
#define OPTICFLOW_SUBPIXEL_FACTOR_CAMERA2 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SUBPIXEL_FACTOR)
PRINT_CONFIG_VAR(OPTICFLOW_SUBPIXEL_FACTOR_CAMERA2)

#ifndef OPTICFLOW_RESOLUTION_FACTOR
#define OPTICFLOW_RESOLUTION_FACTOR 100
#endif

#ifndef OPTICFLOW_RESOLUTION_FACTOR_CAMERA2
#define OPTICFLOW_RESOLUTION_FACTOR_CAMERA2 100
#endif
PRINT_CONFIG_VAR(OPTICFLOW_RESOLUTION_FACTOR)
PRINT_CONFIG_VAR(OPTICFLOW_RESOLUTION_FACTOR_CAMERA2)

#ifndef OPTICFLOW_MAX_ITERATIONS
#define OPTICFLOW_MAX_ITERATIONS 10
#endif

#ifndef OPTICFLOW_MAX_ITERATIONS_CAMERA2
#define OPTICFLOW_MAX_ITERATIONS_CAMERA2 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_ITERATIONS)
PRINT_CONFIG_VAR(OPTICFLOW_MAX_ITERATIONS_CAMERA2)

#ifndef OPTICFLOW_THRESHOLD_VEC
#define OPTICFLOW_THRESHOLD_VEC 2
#endif

#ifndef OPTICFLOW_THRESHOLD_VEC_CAMERA2
#define OPTICFLOW_THRESHOLD_VEC_CAMERA2 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_THRESHOLD_VEC)
PRINT_CONFIG_VAR(OPTICFLOW_THRESHOLD_VEC_CAMERA2)

#ifndef OPTICFLOW_PYRAMID_LEVEL
#define OPTICFLOW_PYRAMID_LEVEL 2
#endif

#ifndef OPTICFLOW_PYRAMID_LEVEL_CAMERA2
#define OPTICFLOW_PYRAMID_LEVEL_CAMERA2 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_PYRAMID_LEVEL)
PRINT_CONFIG_VAR(OPTICFLOW_PYRAMID_LEVEL_CAMERA2)

#ifndef OPTICFLOW_FAST9_ADAPTIVE
#define OPTICFLOW_FAST9_ADAPTIVE TRUE
#endif

#ifndef OPTICFLOW_FAST9_ADAPTIVE_CAMERA2
#define OPTICFLOW_FAST9_ADAPTIVE_CAMERA2 TRUE
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_ADAPTIVE)
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_ADAPTIVE_CAMERA2)

#ifndef OPTICFLOW_FAST9_THRESHOLD
#define OPTICFLOW_FAST9_THRESHOLD 20
#endif

#ifndef OPTICFLOW_FAST9_THRESHOLD_CAMERA2
#define OPTICFLOW_FAST9_THRESHOLD_CAMERA2 20
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_THRESHOLD)
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_THRESHOLD_CAMERA2)

#ifndef OPTICFLOW_FAST9_MIN_DISTANCE
#define OPTICFLOW_FAST9_MIN_DISTANCE 10
#endif

#ifndef OPTICFLOW_FAST9_MIN_DISTANCE_CAMERA2
#define OPTICFLOW_FAST9_MIN_DISTANCE_CAMERA2 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_MIN_DISTANCE)
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_MIN_DISTANCE_CAMERA2)

#ifndef OPTICFLOW_FAST9_PADDING
#define OPTICFLOW_FAST9_PADDING 20
#endif

#ifndef OPTICFLOW_FAST9_PADDING_CAMERA2
#define OPTICFLOW_FAST9_PADDING_CAMERA2 20
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_PADDING)
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_PADDING_CAMERA2)

// thresholds FAST9 that are currently not set from the GCS:
#define FAST9_LOW_THRESHOLD 5
#define FAST9_HIGH_THRESHOLD 60


#ifndef OPTICFLOW_METHOD
#define OPTICFLOW_METHOD 0
#endif

#ifndef OPTICFLOW_METHOD_CAMERA2
#define OPTICFLOW_METHOD_CAMERA2 0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_METHOD)
PRINT_CONFIG_VAR(OPTICFLOW_METHOD_CAMERA2)

#if OPTICFLOW_METHOD > 1
#error WARNING: Both Lukas Kanade and EdgeFlow are NOT selected camera1
#endif

#if OPTICFLOW_METHOD_CAMERA2 > 1
#error WARNING: Both Lukas Kanade and EdgeFlow are NOT selected camera2
#endif

#ifndef OPTICFLOW_DEROTATION
#define OPTICFLOW_DEROTATION TRUE
#endif

#ifndef OPTICFLOW_DEROTATION_CAMERA2
#define OPTICFLOW_DEROTATION_CAMERA2 TRUE
#endif
PRINT_CONFIG_VAR(OPTICFLOW_DEROTATION)
PRINT_CONFIG_VAR(OPTICFLOW_DEROTATION_CAMERA2)

#ifndef OPTICFLOW_DEROTATION_CORRECTION_FACTOR_X
#define OPTICFLOW_DEROTATION_CORRECTION_FACTOR_X 1.0
#endif

#ifndef OPTICFLOW_DEROTATION_CORRECTION_FACTOR_X_CAMERA2
#define OPTICFLOW_DEROTATION_CORRECTION_FACTOR_X_CAMERA2 1.0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_DEROTATION_CORRECTION_FACTOR_X)
PRINT_CONFIG_VAR(OPTICFLOW_DEROTATION_CORRECTION_FACTOR_X_CAMERA2)

#ifndef OPTICFLOW_DEROTATION_CORRECTION_FACTOR_Y
#define OPTICFLOW_DEROTATION_CORRECTION_FACTOR_Y 1.0
#endif

#ifndef OPTICFLOW_DEROTATION_CORRECTION_FACTOR_Y_CAMERA2
#define OPTICFLOW_DEROTATION_CORRECTION_FACTOR_Y_CAMERA2 1.0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_DEROTATION_CORRECTION_FACTOR_Y)
PRINT_CONFIG_VAR(OPTICFLOW_DEROTATION_CORRECTION_FACTOR_Y_CAMERA2)

#ifndef OPTICFLOW_MEDIAN_FILTER
#define OPTICFLOW_MEDIAN_FILTER FALSE
#endif

#ifndef OPTICFLOW_MEDIAN_FILTER_CAMERA2
#define OPTICFLOW_MEDIAN_FILTER_CAMERA2 FALSE
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MEDIAN_FILTER)
PRINT_CONFIG_VAR(OPTICFLOW_MEDIAN_FILTER_CAMERA2)

#ifndef OPTICFLOW_FEATURE_MANAGEMENT
#define OPTICFLOW_FEATURE_MANAGEMENT 0
#endif

#ifndef OPTICFLOW_FEATURE_MANAGEMENT_CAMERA2
#define OPTICFLOW_FEATURE_MANAGEMENT_CAMERA2 0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FEATURE_MANAGEMENT)
PRINT_CONFIG_VAR(OPTICFLOW_FEATURE_MANAGEMENT_CAMERA2)

#ifndef OPTICFLOW_FAST9_REGION_DETECT
#define OPTICFLOW_FAST9_REGION_DETECT 1
#endif

#ifndef OPTICFLOW_FAST9_REGION_DETECT_CAMERA2
#define OPTICFLOW_FAST9_REGION_DETECT_CAMERA2 1
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_REGION_DETECT)
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_REGION_DETECT_CAMERA2)

#ifndef OPTICFLOW_FAST9_NUM_REGIONS
#define OPTICFLOW_FAST9_NUM_REGIONS 9
#endif

#ifndef OPTICFLOW_FAST9_NUM_REGIONS_CAMERA2
#define OPTICFLOW_FAST9_NUM_REGIONS_CAMERA2 9
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_NUM_REGIONS)
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_NUM_REGIONS_CAMERA2)

#ifndef OPTICFLOW_ACTFAST_LONG_STEP
#define OPTICFLOW_ACTFAST_LONG_STEP 10
#endif

#ifndef OPTICFLOW_ACTFAST_LONG_STEP_CAMERA2
#define OPTICFLOW_ACTFAST_LONG_STEP_CAMERA2 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_ACTFAST_LONG_STEP)
PRINT_CONFIG_VAR(OPTICFLOW_ACTFAST_LONG_STEP_CAMERA2)

#ifndef OPTICFLOW_ACTFAST_SHORT_STEP
#define OPTICFLOW_ACTFAST_SHORT_STEP 2
#endif

#ifndef OPTICFLOW_ACTFAST_SHORT_STEP_CAMERA2
#define OPTICFLOW_ACTFAST_SHORT_STEP_CAMERA2 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_ACTFAST_SHORT_STEP)
PRINT_CONFIG_VAR(OPTICFLOW_ACTFAST_SHORT_STEP_CAMERA2)

#ifndef OPTICFLOW_ACTFAST_GRADIENT_METHOD
#define OPTICFLOW_ACTFAST_GRADIENT_METHOD 1
#endif

#ifndef OPTICFLOW_ACTFAST_GRADIENT_METHOD_CAMERA2
#define OPTICFLOW_ACTFAST_GRADIENT_METHOD_CAMERA2 1
#endif
PRINT_CONFIG_VAR(OPTICFLOW_ACTFAST_GRADIENT_METHOD)
PRINT_CONFIG_VAR(OPTICFLOW_ACTFAST_GRADIENT_METHOD_CAMERA2)

#ifndef OPTICFLOW_ACTFAST_MIN_GRADIENT
#define OPTICFLOW_ACTFAST_MIN_GRADIENT 10
#endif

#ifndef OPTICFLOW_ACTFAST_MIN_GRADIENT_CAMERA2
#define OPTICFLOW_ACTFAST_MIN_GRADIENT_CAMERA2 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_ACTFAST_MIN_GRADIENT)
PRINT_CONFIG_VAR(OPTICFLOW_ACTFAST_MIN_GRADIENT_CAMERA2)

// Defaults for ARdrone
#ifndef OPTICFLOW_BODY_TO_CAM_PHI
#define OPTICFLOW_BODY_TO_CAM_PHI 0
#endif
#ifndef OPTICFLOW_BODY_TO_CAM_THETA
#define OPTICFLOW_BODY_TO_CAM_THETA 0
#endif
#ifndef OPTICFLOW_BODY_TO_CAM_PSI
#define OPTICFLOW_BODY_TO_CAM_PSI -M_PI_2
#endif

#ifndef OPTICFLOW_BODY_TO_CAM_PHI_CAMERA2
#define OPTICFLOW_BODY_TO_CAM_PHI_CAMERA2 0
#endif
#ifndef OPTICFLOW_BODY_TO_CAM_THETA_CAMERA2
#define OPTICFLOW_BODY_TO_CAM_THETA_CAMERA2 0
#endif
#ifndef OPTICFLOW_BODY_TO_CAM_PSI_CAMERA2
#define OPTICFLOW_BODY_TO_CAM_PSI_CAMERA2 -M_PI_2
#endif

// Tracking back flow to make the accepted flow vectors more robust:
// Default is false, as it does take extra processing time
#ifndef OPTICFLOW_TRACK_BACK
#define OPTICFLOW_TRACK_BACK FALSE
#endif

#ifndef OPTICFLOW_TRACK_BACK_CAMERA2
#define OPTICFLOW_TRACK_BACK_CAMERA2 FALSE
#endif
PRINT_CONFIG_VAR(OPTICFLOW_TRACK_BACK)
PRINT_CONFIG_VAR(OPTICFLOW_TRACK_BACK_CAMERA2)

// Whether to draw the flow on the image:
// False by default, since it changes the image and costs time.
#ifndef OPTICFLOW_SHOW_FLOW
#define OPTICFLOW_SHOW_FLOW FALSE
#endif

#ifndef OPTICFLOW_SHOW_FLOW_CAMERA2
#define OPTICFLOW_SHOW_FLOW_CAMERA2 FALSE
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SHOW_FLOW)
PRINT_CONFIG_VAR(OPTICFLOW_SHOW_FLOW_CAMERA2)


//Include median filter
#include "filters/median_filter.h"
struct MedianFilter3Float vel_filt;
struct FloatRMat body_to_cam[2];

/* Functions only used here */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);
static int cmp_flow(const void *a, const void *b);
static int cmp_array(const void *a, const void *b);
static void manage_flow_features(struct image_t *img, struct opticflow_t *opticflow,
                                 struct opticflow_result_t *result);

static struct flow_t *predict_flow_vectors(struct flow_t *flow_vectors, uint16_t n_points, float phi_diff,
    float theta_diff, float psi_diff, struct opticflow_t *opticflow);
/**
 * Initialize the opticflow calculator
 * @param[out] *opticflow The new optical flow calculator
 */
void opticflow_calc_init(struct opticflow_t opticflow[])
{
  /* Set the default values */
  opticflow[0].method = OPTICFLOW_METHOD; //0 = LK_fast9, 1 = Edgeflow
  opticflow[0].window_size = OPTICFLOW_WINDOW_SIZE;
  opticflow[0].search_distance = OPTICFLOW_SEARCH_DISTANCE;
  opticflow[0].derotation = OPTICFLOW_DEROTATION; //0 = OFF, 1 = ON
  opticflow[0].derotation_correction_factor_x = OPTICFLOW_DEROTATION_CORRECTION_FACTOR_X;
  opticflow[0].derotation_correction_factor_y = OPTICFLOW_DEROTATION_CORRECTION_FACTOR_Y;
  opticflow[0].track_back = OPTICFLOW_TRACK_BACK;
  opticflow[0].show_flow = OPTICFLOW_SHOW_FLOW;
  opticflow[0].max_track_corners = OPTICFLOW_MAX_TRACK_CORNERS;
  opticflow[0].subpixel_factor = OPTICFLOW_SUBPIXEL_FACTOR;
  if (opticflow[0].subpixel_factor == 0) {
    opticflow[0].subpixel_factor = 10;
  }
  opticflow[0].resolution_factor = OPTICFLOW_RESOLUTION_FACTOR;
  opticflow[0].max_iterations = OPTICFLOW_MAX_ITERATIONS;
  opticflow[0].threshold_vec = OPTICFLOW_THRESHOLD_VEC;
  opticflow[0].pyramid_level = OPTICFLOW_PYRAMID_LEVEL;
  opticflow[0].median_filter = OPTICFLOW_MEDIAN_FILTER;
  opticflow[0].feature_management = OPTICFLOW_FEATURE_MANAGEMENT;
  opticflow[0].fast9_region_detect = OPTICFLOW_FAST9_REGION_DETECT;
  opticflow[0].fast9_num_regions = OPTICFLOW_FAST9_NUM_REGIONS;

  opticflow[0].fast9_adaptive = OPTICFLOW_FAST9_ADAPTIVE;
  opticflow[0].fast9_threshold = OPTICFLOW_FAST9_THRESHOLD;
  opticflow[0].fast9_min_distance = OPTICFLOW_FAST9_MIN_DISTANCE;
  opticflow[0].fast9_padding = OPTICFLOW_FAST9_PADDING;
  opticflow[0].fast9_rsize = FAST9_MAX_CORNERS;
  opticflow[0].fast9_ret_corners = calloc(opticflow[0].fast9_rsize, sizeof(struct point_t));

  opticflow[0].corner_method = OPTICFLOW_CORNER_METHOD;
  opticflow[0].actfast_long_step = OPTICFLOW_ACTFAST_LONG_STEP;
  opticflow[0].actfast_short_step = OPTICFLOW_ACTFAST_SHORT_STEP;
  opticflow[0].actfast_min_gradient = OPTICFLOW_ACTFAST_MIN_GRADIENT;
  opticflow[0].actfast_gradient_method = OPTICFLOW_ACTFAST_GRADIENT_METHOD;

  opticflow[0].camera = &OPTICFLOW_CAMERA;
  opticflow[0].id = 0;

  struct FloatEulers euler_cam1 = {OPTICFLOW_BODY_TO_CAM_PHI, OPTICFLOW_BODY_TO_CAM_THETA, OPTICFLOW_BODY_TO_CAM_PSI};
  float_rmat_of_eulers(&body_to_cam[0], &euler_cam1);

#ifdef OPTICFLOW_CAMERA2
  opticflow[1].method = OPTICFLOW_METHOD_CAMERA2; //0 = LK_fast9, 1 = Edgeflow
  opticflow[1].window_size = OPTICFLOW_WINDOW_SIZE_CAMERA2;
  opticflow[1].search_distance = OPTICFLOW_SEARCH_DISTANCE_CAMERA2;
  opticflow[1].derotation = OPTICFLOW_DEROTATION_CAMERA2; //0 = OFF, 1 = ON
  opticflow[1].derotation_correction_factor_x = OPTICFLOW_DEROTATION_CORRECTION_FACTOR_X_CAMERA2;
  opticflow[1].derotation_correction_factor_y = OPTICFLOW_DEROTATION_CORRECTION_FACTOR_Y_CAMERA2;
  opticflow[1].track_back = OPTICFLOW_TRACK_BACK_CAMERA2;
  opticflow[1].show_flow = OPTICFLOW_SHOW_FLOW_CAMERA2;
  opticflow[1].max_track_corners = OPTICFLOW_MAX_TRACK_CORNERS_CAMERA2;
  opticflow[1].subpixel_factor = OPTICFLOW_SUBPIXEL_FACTOR_CAMERA2;
  if (opticflow[1].subpixel_factor == 0) {
    opticflow[1].subpixel_factor = 10;
  }
  opticflow[1].resolution_factor = OPTICFLOW_RESOLUTION_FACTOR_CAMERA2;
  opticflow[1].max_iterations = OPTICFLOW_MAX_ITERATIONS_CAMERA2;
  opticflow[1].threshold_vec = OPTICFLOW_THRESHOLD_VEC_CAMERA2;
  opticflow[1].pyramid_level = OPTICFLOW_PYRAMID_LEVEL_CAMERA2;
  opticflow[1].median_filter = OPTICFLOW_MEDIAN_FILTER_CAMERA2;
  opticflow[1].feature_management = OPTICFLOW_FEATURE_MANAGEMENT_CAMERA2;
  opticflow[1].fast9_region_detect = OPTICFLOW_FAST9_REGION_DETECT_CAMERA2;
  opticflow[1].fast9_num_regions = OPTICFLOW_FAST9_NUM_REGIONS_CAMERA2;

  opticflow[1].fast9_adaptive = OPTICFLOW_FAST9_ADAPTIVE_CAMERA2;
  opticflow[1].fast9_threshold = OPTICFLOW_FAST9_THRESHOLD_CAMERA2;
  opticflow[1].fast9_min_distance = OPTICFLOW_FAST9_MIN_DISTANCE_CAMERA2;
  opticflow[1].fast9_padding = OPTICFLOW_FAST9_PADDING_CAMERA2;
  opticflow[1].fast9_rsize = FAST9_MAX_CORNERS;
  opticflow[1].fast9_ret_corners = calloc(opticflow[0].fast9_rsize, sizeof(struct point_t));

  opticflow[1].corner_method = OPTICFLOW_CORNER_METHOD_CAMERA2;
  opticflow[1].actfast_long_step = OPTICFLOW_ACTFAST_LONG_STEP_CAMERA2;
  opticflow[1].actfast_short_step = OPTICFLOW_ACTFAST_SHORT_STEP_CAMERA2;
  opticflow[1].actfast_min_gradient = OPTICFLOW_ACTFAST_MIN_GRADIENT_CAMERA2;
  opticflow[1].actfast_gradient_method = OPTICFLOW_ACTFAST_GRADIENT_METHOD_CAMERA2;

  opticflow[1].camera = &OPTICFLOW_CAMERA2;
  opticflow[1].id = 1;

  struct FloatEulers euler_cam2 = {OPTICFLOW_BODY_TO_CAM_PHI_CAMERA2, OPTICFLOW_BODY_TO_CAM_THETA_CAMERA2,
           OPTICFLOW_BODY_TO_CAM_PSI_CAMERA2
  };
  float_rmat_of_eulers(&body_to_cam[1], &euler_cam2);
#endif
}



bool cal_opticflow_vector(struct opticflow_t *opticflow, struct image_t *img,
                       struct opticflow_result_t *result, float *flow_length)
{

    if (opticflow->just_switched_method==0) {
    printf("%d,%d\n",img->w,img->h);
    // Create the image buffers
    printf("11111");
    image_create(&opticflow->img_gray, img->w, img->h, IMAGE_GRAYSCALE);
    image_create(&opticflow->prev_img_gray, img->w, img->h, IMAGE_GRAYSCALE);
    
    // Set the previous values
    opticflow->got_first_img = true;
    opticflow->just_switched_method = true;
    }
    image_to_grayscale(img, &opticflow->img_gray);
    
    // if we only receive the first image, quit the function
    if (opticflow->got_first_img) {
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    opticflow->got_first_img = false;
    printf("we are going out");
    return false;
    }
    
    // if we receive the image and the previous image
    //image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    
    // get the width and the height of the image
    int width = img->w;
    int height = img->h;
    uint8_t keep_bad_points = 1;
    
    width = width/20;
    height = height/20;

    //create point_t variables to save the position of the choosen point
    struct point_t *new_points = calloc(width*height, sizeof(struct point_t));
    
    
    for (int i=0;i<width;i++){
    	for (int j=0;j<height;j++){
    		new_points[i*(height-1)+j].x = i*20;
    		new_points[i*(height-1)+j].y = j*20;
    	};
    };

    // change some parameters in the algorithm
    result->tracked_cnt = 312;
    opticflow->max_track_corners = 312;
    opticflow->window_size = 50;
    //opticflow->pyramid_level = 10;


    //printf("%d,%d,%d,%d,%d,%d",result->tracked_cnt,opticflow->subpixel_factor,opticflow->max_iterations,opticflow->threshold_vec,opticflow->max_track_corners,opticflow->pyramid_level);
 
    struct flow_t *vectors = opticFlowLK(&(opticflow->img_gray), &opticflow->prev_img_gray, new_points,
                                       &result->tracked_cnt,
                                       opticflow->window_size / 2, opticflow->subpixel_factor, opticflow->max_iterations,
                                       opticflow->threshold_vec, opticflow->max_track_corners, opticflow->pyramid_level, keep_bad_points);
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    //printf("we get the vectors");

    //define the variable to save the flow in the right and left side
    double flow_sum_left = 0.0, flow_sum_right = 0.0;
    //printf("%d,%d\n",width,height);

    for (int i=0;i<width;i++){
    	for (int j=0;j<height;j++){
        //printf("%u,%u",vectors[i*(height-1)+j].flow_x,vectors[i*(height-1)+j].flow_y);
        double magnitude = sqrt(vectors[i*(height-1)+j].flow_x * vectors[i*(height-1)+j].flow_x + vectors[i*(height-1)+j].flow_y * vectors[i*(height-1)+j].flow_y);
        //printf("maginitude %f %d,%d,\n",magnitude,i,j);
        if ( 4 < j && j < height/ 2 ) {
        // left
          flow_sum_left += magnitude;
        } 
        else if (j<21 && j>=height/2)
        {
        // right
          flow_sum_right += magnitude;
        }

    	};
    };
    flow_sum_left = flow_sum_left/156;
    flow_sum_right = flow_sum_right/156;

    flow_length[0] = flow_sum_left;
    flow_length[1] = flow_sum_right;

    return true;
   
}

