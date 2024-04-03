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
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
// #include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv_detect_yuv.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/edge_detection/canny_edge.c"
#include "modules/computer_vision/edge_detection/hysteresis.c"
#include "modules/computer_vision/edge_detection/pgm_io.c"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;
uint8_t green_bottom_height = 80; // only camera 1 is front camera

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

/*
 * This is the threshold for the comparison between the histogram of the front camera and bottom camera.
 * We use a dot production to compare the histograms of the front camera and bottom camera.
 */
uint32_t threshold_middle = 8000;
uint32_t threshold_sideways = 5000;


/*
 * This is the threshold for the proprotion of the green pixel in the bottom part of the image.
 * If the proportion falls below the threshold, it indicates the detection of an obstacle.
 */
float image_middle_proportion = 0.33f;
float middle_threshold_proportion = 0.5f;
float sideways_threshold_proportion = 0.2f;

float edge_detect_proportion = 0.8f;

uint8_t detection_mode = GREEN_DETECTOR;

// sum of the green pixel in the left, middle and left part of the image
uint32_t sum_left = 0;
uint32_t sum_mid = 0;
uint32_t sum_right = 0; 

// Edge Filter Settings
float sigma1 = 3.0;
float tlow1 = 0.5;
float thigh1 = 0.95;

// float sigma2 = 0.6;
// float tlow2 = 0.5;
// float thigh2 = 0.8;
u_int16_t pixel_variance_threshold = 3;

uint16_t free_space_threshold = 10;

bool small_window = true; // true: the drone is using a small central window
float compare_threshold = 1.1f;

/*
 * This is the variable to reveive the message from the floor detection with bottom camera
 */
struct ground_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
  uint16_t yd[256];
  uint16_t ud[256];
  uint16_t vd[256];
  uint8_t strategy;
};

/*
 * This is the variable to reveive the message from the floor detection with bottom camera.
 * The most important message is the number of the green pixels in three parts of the image.
 */
struct green_object_t {
  uint32_t count_middle;
  uint32_t count_left;
  uint32_t count_right;
  uint32_t threshold_middle;
  uint32_t threshold_sideways;
  uint32_t strategy;
  bool updated;
};

/*
 * This is the variable to reveive the message from the edge detection
 */
struct edge_t {
  // int32_t x_c;
  // int32_t y_c;
  uint16_t highest_column_idx;
  uint16_t shorest_column_idx;
  uint16_t highest_edge_height;
  uint16_t shortest_edge_height;
  int16_t strategy;
  bool updated;
};

struct ground_object_t global_filters_ground[2];
struct green_object_t global_filters_green[2];
struct edge_t global_filters_edge[2];

// Function
// green detection
void find_object_counts(struct image_t *img, bool draw,
                              uint32_t* count_left, uint32_t* count_middle, uint32_t* count_right,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint8_t bottom_height);

// ground detection
uint32_t histogram_front(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint16_t *yd, uint16_t *ud, uint16_t *vd, uint8_t *strategy);
void histogram_bottom(struct image_t *img, uint16_t *yd, uint16_t *ud, uint16_t *vd);
// the function to calculate the relationship parameter between two histogram
uint32_t multiply(uint16_t *yd_bot, uint16_t *ud_bot, uint16_t *vd_bot, uint16_t *yd, uint16_t *ud, uint16_t *vd);

// edge detection
void find_edge(struct edge_t *local_filter_ptr, struct image_t *img, bool draw, float sigma, float tlow, float thigh);
uint16_t median_filter(uint16_t *array, uint16_t length);
// void compare (const void * a, const void * b);          
int compare (const void * a, const void * b);               
unsigned char * YUV2Gray(struct image_t *img);






/*
 * this is the function for the object detector of the front camera
 * if we use GROUND DETECTION:
 * 	1. We will get the histogram of three different part in the front camera
 * 	2. If we have get the histogram from the bottom camera, we will make a comparision between these histograms
 * if we use COLOR DETECTION:
 * 	1. Filter the green color from the three parts of the image.
 * 	2. Make the strategy according to the proportion of the green pixels in three parts and their threshold.
 */
static struct image_t *object_detector_front(struct image_t *img, uint8_t filter)
{
    uint8_t lum_min, lum_max;
    uint8_t cb_min, cb_max;
    uint8_t cr_min, cr_max;
    bool draw;

    switch (filter){
      case 1:
        lum_min = cod_lum_min1;
        lum_max = cod_lum_max1;
        cb_min = cod_cb_min1;
        cb_max = cod_cb_max1;
        cr_min = cod_cr_min1;
        cr_max = cod_cr_max1;
        draw = cod_draw1;
        break;
      case 2:
        // lum_min = cod_lum_min2;
        // lum_max = cod_lum_max2;
        // cb_min = cod_cb_min2;
        // cb_max = cod_cb_max2;
        // cr_min = cod_cr_min2;
        // cr_max = cod_cr_max2;
        // draw = cod_draw2;
        printf("ERROR, wrong front camera ID in airframe!");
        break;
      default:
        return img;
    };

  if(detection_mode == GROUND_DETECTOR)
  {
    //initialize the variable to get the histogram from the bottom camera
    uint16_t yd[256] = {0};
    uint16_t ud[256] = {0};
    uint16_t vd[256] = {0};
    if (global_filters_ground[1].updated == true){
        pthread_mutex_lock(&mutex);
        memcpy(yd, global_filters_ground[1].yd, 256*sizeof(uint16_t));
        memcpy(ud, global_filters_ground[1].ud, 256*sizeof(uint16_t));
        memcpy(vd, global_filters_ground[1].vd, 256*sizeof(uint16_t));
        pthread_mutex_unlock(&mutex);
    }
    
    //initialize the strategy parameter
    uint8_t strategy = 0;

    int32_t x_c, y_c;

    // Filter and find centroid
    uint32_t count = histogram_front(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, yd, ud, vd, &strategy);

    pthread_mutex_lock(&mutex);
    global_filters_ground[filter-1].color_count = count;
    global_filters_ground[filter-1].x_c = x_c;
    global_filters_ground[filter-1].y_c = y_c;
    global_filters_ground[filter-1].updated = true;
    
    // save the strategy in the global filter, so we could send the message through the global filter
    global_filters_ground[filter-1].strategy = strategy;
    pthread_mutex_unlock(&mutex);

  }
  else if (detection_mode == GREEN_DETECTOR)
  {
    // uint8_t lum_min, lum_max;
    // uint8_t cb_min, cb_max;
    // uint8_t cr_min, cr_max;
    // bool draw;

    // switch (filter){
    //   case 1:
    //     lum_min = cod_lum_min1;
    //     lum_max = cod_lum_max1;
    //     cb_min = cod_cb_min1;
    //     cb_max = cod_cb_max1;
    //     cr_min = cod_cr_min1;
    //     cr_max = cod_cr_max1;
    //     draw = cod_draw1;
    //     break;
    //   case 2:
    //     lum_min = cod_lum_min2;
    //     lum_max = cod_lum_max2;
    //     cb_min = cod_cb_min2;
    //     cb_max = cod_cb_max2;
    //     cr_min = cod_cr_min2;
    //     cr_max = cod_cr_max2;
    //     draw = cod_draw2;
    //     break;
    //   default:
    //     return img;
    // };

    uint32_t count_left = 0;
    uint32_t count_middle = 0; // !!! if not initialize again, just define it, it will be very large values, seems use last loop's result
    uint32_t count_right = 0;
    // Filter and find centroid
    find_object_counts(
      img, draw, &count_left, &count_middle, &count_right,
      lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, green_bottom_height);
    // VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
    // VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
          // hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

    pthread_mutex_lock(&mutex);
    //global_filters_green[filter-1].color_count = count;
    
    uint32_t middle_threshold_pixel;
    uint32_t sideways_threshold_pixel;
    
    /*
     * This is the part to make the strategy according to the proportion and the threshold.
     */
    
    if (small_window == true){ // if we choose to use the smaller window in the image

	    middle_threshold_pixel = middle_threshold_proportion * 160 * green_bottom_height;
	    sideways_threshold_pixel = sideways_threshold_proportion * 80 * green_bottom_height;
	    if (count_middle < middle_threshold_pixel || count_left < sideways_threshold_pixel || count_right < sideways_threshold_pixel)
	    {
	      if (count_left > count_right)
	      {
		global_filters_green[filter-1].strategy = ACTION_LEFT;
	      }
	      else
	      {
		global_filters_green[filter-1].strategy = ACTION_RIGHT;
	      }
	    }
	    else
	    {
	      if (count_left > compare_threshold * count_right && count_left * 2 > compare_threshold * count_middle)
	      {
		global_filters_green[filter-1].strategy = ACTION_FORWARD_LEFT;
	      }
	      else if (count_right > compare_threshold * count_left && count_right * 2 > compare_threshold * count_middle)
	      {
		global_filters_green[filter-1].strategy = ACTION_FORWARD_RIGHT;
	      }
	      else
	      {
		global_filters_green[filter-1].strategy = ACTION_FORWARD;
	      }
	    }
    }
    else {
        middle_threshold_pixel = middle_threshold_proportion * img->h * (image_middle_proportion) * green_bottom_height;
    	sideways_threshold_pixel = sideways_threshold_proportion * img->h * (1.f-image_middle_proportion)/2.f * green_bottom_height;
    	if (count_middle < middle_threshold_pixel || count_left < sideways_threshold_pixel || count_right < sideways_threshold_pixel)
    	{
	      if (count_left > count_right)
	      {
		global_filters_green[filter-1].strategy = ACTION_LEFT;
	      }
	      else
	      {
		global_filters_green[filter-1].strategy = ACTION_RIGHT;
	      }
	    }
	    else
	    {
	      if (count_left > compare_threshold * count_right && count_left > compare_threshold * count_middle)
	      {
		global_filters_green[filter-1].strategy = ACTION_FORWARD_LEFT;
	      }
	      else if (count_right > compare_threshold * count_left && count_right > compare_threshold * count_middle)
	      {
		global_filters_green[filter-1].strategy = ACTION_FORWARD_RIGHT;
	      }
	      else
	      {
		global_filters_green[filter-1].strategy = ACTION_FORWARD;
	      }
    	}
    }
    
    /*
     * Save the message within the global variable, ready to be included in the ABI message when sent.
     */
    global_filters_green[filter-1].threshold_middle = middle_threshold_pixel;
    global_filters_green[filter-1].count_middle = count_middle;
    global_filters_green[filter-1].threshold_sideways = sideways_threshold_pixel;
    global_filters_green[filter-1].count_left = count_left;
    global_filters_green[filter-1].count_right = count_right;
    global_filters_green[filter-1].updated = true;
    pthread_mutex_unlock(&mutex);


  }
  else if (detection_mode == EDGE_DETECTOR)
  {
    float sigma, tlow, thigh;
    sigma = sigma1;
    tlow = tlow1;
    thigh = thigh1;
    // Filter and find centroid
    struct edge_t local_filter;
    find_edge(&local_filter, img, draw, sigma, tlow, thigh);
    if (local_filter.shortest_edge_height < free_space_threshold)
    {
      if (local_filter.shorest_column_idx < img->h * 0.5f) local_filter.strategy = ACTION_RIGHT;
      else local_filter.strategy = ACTION_LEFT;
    }
    else 
    {
      if (local_filter.highest_column_idx < img->h * (0.5f - edge_detect_proportion/2.f + edge_detect_proportion/3.f)) {
          local_filter.strategy = ACTION_LEFT;
        } else if (local_filter.highest_column_idx > img->h * (0.5f + edge_detect_proportion/2.f - edge_detect_proportion/3.f)) {
          local_filter.strategy = ACTION_RIGHT;
        } else {
          local_filter.strategy = ACTION_FORWARD;
        }
    }
    
    /*
     * Save the message within the global variable, ready to be included in the ABI message when sent.
     */
    pthread_mutex_lock(&mutex);
    global_filters_edge[filter-1].highest_column_idx = local_filter.highest_column_idx;
    global_filters_edge[filter-1].highest_edge_height = local_filter.highest_edge_height;
    global_filters_edge[filter-1].shorest_column_idx = local_filter.shorest_column_idx;
    global_filters_edge[filter-1].shortest_edge_height = local_filter.shortest_edge_height;
    global_filters_edge[filter-1].strategy = local_filter.strategy;
    global_filters_edge[filter-1].updated = true;
    pthread_mutex_unlock(&mutex);
  }
  return img;
}






/*
 * this is the function for the object detector of the bottom camera
 * 1. we will get the histogram of color in the bottom camerar
 * 2. we will save the histogram in a global variable
 */
static struct image_t *object_detector_bottom(struct image_t *img, uint8_t filter)
{
  if(detection_mode == GROUND_DETECTOR)
  {
    uint16_t yd[256] = {0};
    uint16_t ud[256] = {0};
    uint16_t vd[256] = {0};
    
    // Filter and find centroid
    histogram_bottom (img, yd, ud, vd);

    pthread_mutex_lock(&mutex);
    global_filters_ground[filter-1].color_count = 0;
    global_filters_ground[filter-1].x_c = 0;
    global_filters_ground[filter-1].y_c = 0;
    global_filters_ground[filter-1].updated = true;
    
    // save the hue and intensity of the bottom camera to the global variables
    memcpy(global_filters_ground[filter-1].yd, yd, 256*sizeof(uint16_t));
    memcpy(global_filters_ground[filter-1].ud, ud, 256*sizeof(uint16_t));
    memcpy(global_filters_ground[filter-1].vd, vd, 256*sizeof(uint16_t));
    pthread_mutex_unlock(&mutex);
    

  }
  else if (detection_mode == GREEN_DETECTOR || detection_mode == EDGE_DETECTOR)
  {
    // do nothing if we choose the other modes
  }
  return img;
}





/*
 * this is the callback function used in cv_add_to_device
 */
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector_front(img, 1);
}

struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector_bottom(img, 2);
}

void color_object_detector_init(void)
{
  memset(global_filters_ground, 0, 2*sizeof(struct ground_object_t));
  memset(global_filters_green, 0, 2*sizeof(struct green_object_t));
  memset(global_filters_edge, 0, 2*sizeof(struct edge_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2 // true
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2 // not true
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
#endif

#ifdef THRESHOLD_MIDDLE
threshold_middle = THRESHOLD_MIDDLE;
threshold_sideways = THRESHOLD_SIDEWAYS;
green_bottom_height = GREEN_BOTTOM_HEIGHT;
small_window = SMALL_WINDOW;

image_middle_proportion = IMAGE_MIDDLE_PROPORTION;
middle_threshold_proportion = MIDDLE_THRESHOLD_PROPORTION;
sideways_threshold_proportion = SIDEWAYS_THRESHOLD_PROPORTION;

sigma1 = EDGE_DETECTOR_SIGMA1;
tlow1 = EDGE_DETECTOR_TLOW1;
thigh1 = EDGE_DETECTOR_THIGH1;
pixel_variance_threshold = PIXEL_VAR_THRESHOLD;

edge_detect_proportion = EDGE_DETECT_PROPORTION;

detection_mode = DETECTION_MODE;

compare_threshold = COMPARE_THRESHOLD;
#endif
}






/*
 * This is the function to count the green pixels in three parts of the image.
 * The green filter is made with YUV threshold.
 */
void find_object_counts(struct image_t *img, bool draw,
                              uint32_t* count_left, uint32_t* count_middle, uint32_t* count_right,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint8_t bottom_height)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;
  // uint8_t *count_rows = malloc(img->h * sizeof(uint8_t));

  // debug
  // uint64_t loop_count = 0;
  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y++) {
    uint8_t count_row = 0;
    for (uint16_t x = 0; x < bottom_height; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        count_row ++;
        cnt ++;
        tot_x += x;
        tot_y += y;
        // printf("bound1 = %d\n", img->w);
        // printf("bound2 = %f\n", 1-1/num_parts);
        // printf("bound3 = %f\n", img->w * (1-1/num_parts));
        // if (!(loop_count % 100)) printf("bound1 = %d\n", img->w); //{printf("current x: %u, ", x);}
        if (small_window == true){
        if (y < 180&& y > 100 ) {
		  *count_left += 1;
		  // if (!(loop_count % 100)) {printf("1\n");}
		} else if (y > 340 && y < 420 ) {
		  *count_right += 1;
		  // if (!(loop_count % 100)) {printf("2\n");}
		} else if (y > 180 && y < 340) {
		  *count_middle += 1;
		  // if (!(loop_count % 100)) {printf("3\n");}
		}
        }
        else {
		if (y < img->h * (0.5 - image_middle_proportion/2.0f)) {
		  *count_left += 1;
		  // if (!(loop_count % 100)) {printf("1\n");}
		} else if (y > img->h * (0.5 + image_middle_proportion/2.0f)) {
		  *count_right += 1;
		  // if (!(loop_count % 100)) {printf("2\n");}
		} else {
		  *count_middle += 1;
		  // if (!(loop_count % 100)) {printf("3\n");}
        }
	}

        if (draw){
          *yp = 255;  // make pixel brighter in image
        }
      }
      // loop_count ++;
    }
    // count_rows[y] = count_row;
  }
  // printf("loop_count = %u\n", loop_count);
  /*
  if (cnt > 0) {
    *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }
  */
  // return count_rows;
  return;
}






/* this function is to calculate the color histogram from the image of the front camera
 * it will get the histogram of the bottom camera with a pointer of an array
 * it will make a comparision between these two histogram and make a strategy
 */
uint32_t histogram_front(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint16_t *yd, uint16_t *ud, uint16_t *vd, uint8_t *strategy)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;
  
  // initialie the variables to save the color histogram in three different area
  uint16_t histogram_yd_right[256] = {0};
  uint16_t histogram_yd_mid[256] = {0};
  uint16_t histogram_yd_left[256] = {0};
  uint16_t histogram_ud_right[256] = {0};
  uint16_t histogram_ud_mid[256] = {0};
  uint16_t histogram_ud_left[256] = {0};
  uint16_t histogram_vd_right[256] = {0};
  uint16_t histogram_vd_mid[256] = {0};
  uint16_t histogram_vd_left[256] = {0};

  // Go through all the pixels and get the yuv information of the image
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      if (y>100 && y<180 && x>0 && x<40){
        histogram_yd_left[*yp] += 4;
        histogram_ud_left[*up] += 4;
        histogram_vd_left[*vp] += 4;
      }
      if (y>180 && y<340 && x>0 && x<40){
        histogram_yd_mid[*yp] += 2;
        histogram_ud_mid[*up] += 2;
        histogram_vd_mid[*vp] += 2;
      }
      if (y>340 && y<420 && x>0 && x<40){
        histogram_yd_right[*yp] += 4;
        histogram_ud_right[*up] += 4;
        histogram_vd_right[*vp] += 4;
      }
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        cnt ++;
        tot_x += x;
        tot_y += y;
        if (draw){
          *yp = 255;  // make pixel brighter in image
        }
      }
    }
  }
  if (cnt > 0) {
    *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }
  
  
  // compare the histogram with the histogram from the bottom
  sum_left = multiply(yd, ud, vd, histogram_yd_left, histogram_ud_left, histogram_vd_left);
  sum_mid = multiply(yd, ud, vd, histogram_yd_mid, histogram_ud_mid, histogram_vd_mid);
  sum_right = multiply(yd, ud, vd, histogram_yd_right, histogram_ud_right, histogram_vd_right); 

  // choose the biggest relationship and set the strategy
  // uint32_t biggest_direction = sum_mid;
  
  if (sum_mid < threshold_middle * 1000 || sum_left < threshold_sideways * 1000 || sum_right < threshold_sideways * 1000)  {
      if (sum_left < sum_right){
        *strategy = ACTION_RIGHT;
      }
      else {
        *strategy = ACTION_LEFT;
      } 
  }
  else {
    if (sum_left > sum_right && sum_left > sum_mid) {
      *strategy = ACTION_FORWARD_LEFT;
    }
    else if (sum_right > sum_left && sum_right > sum_mid) {
      *strategy = ACTION_FORWARD_RIGHT;
    }
    else {*strategy = ACTION_FORWARD;}
  }
  
  return cnt;
}






/* this function is to calculate the color histogram from the image of the bottom camera
 * it will send the histogram back with a pointer of an array
 * we only choose the central part of the bottom image to get the range of the yuv channel 
 */
void histogram_bottom(struct image_t *img, uint16_t *yd, uint16_t *ud, uint16_t *vd)
{
  // initialize three histogram to save the y,u,v message
  uint8_t *buffer = img->buf;
  
  // now we get the HSI message from the YUV image of the front camera
  for (uint16_t y = 0; y < 100; y++) {
    for (uint16_t x = 0; x < 100; x ++) {
        uint8_t *yp, *up, *vp;
        if (x % 2 == 0) {
        // Even x
        up = &buffer[(70 * 2 + y * 2) * img->w + (70 *2 + 2 * x)];      // U
        yp = &buffer[(70 * 2 + y * 2) * img->w + (70 *2 + 2 * x) + 1];  // Y1
        vp = &buffer[(70 * 2 + y * 2) * img->w + (70 *2 + 2 * x) + 2];  // V
        } else {
        // Uneven x
        up = &buffer[(70 * 2 + y * 2) * img->w + (70 *2 + 2 * x) - 2];  // U
        vp = &buffer[(70 * 2 + y * 2) * img->w + (70 *2 + 2 * x)];      // V
        yp = &buffer[(70 * 2 + y * 2) * img->w + (70 *2 + 2 * x) + 1];  // Y2
      	}
        yd[*yp] += 1;
        ud[*up] += 1;
        vd[*vp] += 1;
      }
    }
}





/* This function is to make comparison between two histograms.
 * The similaritiy is calculated through a dot production between two histogram.
 */
uint32_t multiply(uint16_t *yd_bot, uint16_t *ud_bot, uint16_t *vd_bot, uint16_t *yd, uint16_t *ud, uint16_t *vd){
  uint32_t sum_yd = 0;
  uint32_t sum_ud = 0;
  uint32_t sum_vd = 0;
  for (uint16_t y = 0; y < 256; y++) {
    sum_yd = sum_yd + yd_bot[y]*yd[y];
    sum_ud = sum_ud + ud_bot[y]*ud[y]; 
    sum_vd = sum_vd + vd_bot[y]*vd[y]; 
  }
  uint32_t sum = (sum_yd + sum_ud + sum_vd)/3;
  return sum;
}





/* This function is to turn the YUV image into grayscale image.
 */
unsigned char * YUV2Gray(struct image_t *img)
{
  uint8_t *buffer = img->buf;
  unsigned char *gray = (unsigned char *)malloc(img->w * img->h * sizeof(unsigned char));

  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      gray[y * img->w + x] = *yp;
    }
  }
  return gray;
}





/* This function is to filter the median number from an array
 */
uint16_t median_filter(uint16_t *array, uint16_t length)
{
  uint16_t *array_sorted = (uint16_t *)malloc(length * sizeof(uint16_t));
  memcpy(array_sorted, array, length * sizeof(uint16_t));
  qsort(array_sorted, length, sizeof(uint16_t), compare);
  uint16_t median = array_sorted[length/2];
  free(array_sorted);
  return median;
}

int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}






/* This function is to find the edge in the image
 */
void find_edge(struct edge_t *local_filter_ptr, struct image_t *img, bool draw, float sigma, float tlow, float thigh)
{
  uint8_t *buffer = img->buf;
  uint32_t rows = img->h;
  uint32_t cols = img->w;
  unsigned char *edge;

  // Convert to grayscale
  unsigned char *gray = YUV2Gray(img);

  // Canny edge detection
  canny(gray, rows, cols, sigma, tlow, thigh, &edge, NULL);
  // printf("%u", gray[20]);
  free(gray);


  // Go through all the pixels
  for (uint16_t y = 0; y < rows; y++) {
    for (uint16_t x = 0; x < cols; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * cols + 2 * x];      // U
        yp = &buffer[y * 2 * cols + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * cols + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * cols + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * cols + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * cols + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * cols + 2 * x];      // V
        yp = &buffer[y * 2 * cols + 2 * x + 1];  // Y2
      }
      if ( edge[y * cols + x] < 100) {
        if (draw){
          *yp = 255;  // make pixel brighter in image
        }
      }
    }
  }

  /* 
   * find the lowest edge for each column
   * note - the image is rotated 90 degrees clockwise, 
   * here rows refer to columns in the original image and vice versa
   */
  uint32_t edge_array_length = rows * cols;

  uint32_t start_index = 51;
  uint32_t end_index = 350;

  uint16_t *first_edge_x_each_row = (uint16_t *)malloc(rows * sizeof(uint16_t));
  uint16_t *first_edge_x_each_row_filtered = (uint16_t *)malloc(rows-2 * sizeof(uint16_t));
  for (uint16_t i=0; i<rows; i++) {
    first_edge_x_each_row[i] = cols+1; // set initial value to top of image
  }
  for (uint32_t i=0; i<edge_array_length; i++) {
    // printf("%u", edge[i]);
    uint32_t current_index = i; // edge_array_length - i - 1;
    uint16_t current_x = current_index % cols;
    uint16_t current_y = current_index / cols;

    if (current_y > start_index && current_y < end_index) // only use the middle part of the image
    {
      // If no values are assigned to the current row and there is no edge at the current index, assign the current edge as the first edge
      if ((first_edge_x_each_row[current_y]==cols+1) && !edge[current_index]) {
        first_edge_x_each_row[current_y] = current_x;
      }
      
    // If the current column is the last column and the first edge for the current row is still the initial value, assign the first column as the edge
    }
    if (current_x == cols-1 && first_edge_x_each_row[current_y] == cols+1) {
      first_edge_x_each_row[current_y] = 0;
    }

  }
  
  // find the column with highest free space from bottom
  uint16_t row_with_largest_space = rows/2;
  uint16_t row_with_shortest_space = rows/2;
  int16_t longest_edge_length = -1; // max x is < 255 so okay, longest from botton to top - 240 pixels
  uint32_t shortest_edge_length = 1000;

  // median filtering, find the largest space
  for (uint32_t i=start_index + 2; i < end_index - 2; i++) {
    // first_edge_x_each_row_filtered[i-1] = (first_edge_x_each_row[i-1] + first_edge_x_each_row[i] + first_edge_x_each_row[i+1])/3;
    uint16_t edge_array[3] = {first_edge_x_each_row[i-1], first_edge_x_each_row[i], first_edge_x_each_row[i+1]};
    first_edge_x_each_row_filtered[i-1] = median_filter(edge_array, 3);
    
    // // if variance of edge lengths in edge array is too high, ignore this row, only used to guide the drone!
    // if (abs(first_edge_x_each_row[i-1] - first_edge_x_each_row[i]) > pixel_variance_threshold || 
    //     abs(first_edge_x_each_row[i] - first_edge_x_each_row[i+1]) > pixel_variance_threshold) {
    //   continue;
    // }
  

    if (first_edge_x_each_row_filtered[i-1] > longest_edge_length) {
      longest_edge_length = first_edge_x_each_row_filtered[i-1];
      row_with_largest_space = i;
    }

    if (first_edge_x_each_row_filtered[i-1] < shortest_edge_length && first_edge_x_each_row_filtered[i-1] != 0) {
      shortest_edge_length = first_edge_x_each_row_filtered[i-1];
      row_with_shortest_space = i;
    }
  } 
  
  if (longest_edge_length == -1) {longest_edge_length = cols;}
  if (shortest_edge_length == 1000) {shortest_edge_length = 0;} // no edge, force the drone to turn

  // color the 3 rows with largest space, neon green
  for (uint16_t i=0; i<cols; i++) {
    for (uint16_t j=row_with_largest_space-1; j<row_with_largest_space+2; j++) {
      uint8_t *yp, *up, *vp;
      if (i % 2 == 0) {
        // Even x
        up = &buffer[j * 2 * cols + 2 * i];      // U
        yp = &buffer[j * 2 * cols + 2 * i + 1];  // Y1
        vp = &buffer[j * 2 * cols + 2 * i + 2];  // V
        //yp = &buffer[j * 2 * cols + 2 * i + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[j * 2 * cols + 2 * i - 2];  // U
        //yp = &buffer[j * 2 * cols + 2 * i - 1]; // Y1
        vp = &buffer[j * 2 * cols + 2 * i];      // V
        yp = &buffer[j * 2 * cols + 2 * i + 1];  // Y2
      }
      *yp = 184;  // make pixel brighter in image
      *up = 128;  // make pixel brighter in image
      *vp = 255;  // make pixel brighter in image
    }
  }

  // color the 3 rows with shortest space, neon green
  for (uint16_t i=0; i<cols; i++) {
    for (uint16_t j=row_with_shortest_space-1; j<row_with_shortest_space+2; j++) {
      uint8_t *yp, *up, *vp;
      if (i % 2 == 0) {
        // Even x
        up = &buffer[j * 2 * cols + 2 * i];      // U
        yp = &buffer[j * 2 * cols + 2 * i + 1];  // Y1
        vp = &buffer[j * 2 * cols + 2 * i + 2];  // V
        //yp = &buffer[j * 2 * cols + 2 * i + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[j * 2 * cols + 2 * i - 2];  // U
        //yp = &buffer[j * 2 * cols + 2 * i - 1]; // Y1
        vp = &buffer[j * 2 * cols + 2 * i];      // V
        yp = &buffer[j * 2 * cols + 2 * i + 1];  // Y2
      }
      *yp = 184;  // make pixel brighter in image
      *up = 255;  // make pixel brighter in image
      *vp = 128;  // make pixel brighter in image
    }
  }


  // TODO: sliding window
  // buffer[least_first_edge_y * 2 * cols + 2 * highest_column_x - 2] = -37.52415; // U
  // buffer[least_first_edge_y * 2 * cols + 2 * highest_column_x] = 157.27575;     // V
  // buffer[least_first_edge_y * 2 * cols + 2 * highest_column_x + 1] = 76.245;    // Y

  // printf("free direction: %u, ", row_with_largest_space);
  // printf("free space: %u, ", longest_edge_length);
  // printf("in 0 - %u\n", rows);
  free(first_edge_x_each_row);
  free(first_edge_x_each_row_filtered);
  local_filter_ptr->highest_edge_height = longest_edge_length;
  local_filter_ptr->shortest_edge_height = shortest_edge_length;
  local_filter_ptr->shorest_column_idx = row_with_shortest_space;
  local_filter_ptr->highest_column_idx = row_with_largest_space;
  return;
}


/* This function is the periodic function to send the message.
 * 1. Get the detection information from global_filters and save the message in local_filters.
 * 2. Send the information through the abi message.
 */
void color_object_detector_periodic(void)
{
  // this is for the ground detection
  if (detection_mode == GROUND_DETECTOR)
  {
    global_filters_green[0].updated = false;
    global_filters_green[1].updated = false;

    global_filters_edge[0].updated = false;
    global_filters_edge[1].updated = false;

    static struct ground_object_t local_filters[2];
    
    // read the message from the global_filters
    pthread_mutex_lock(&mutex);
    memcpy(local_filters, global_filters_ground, 2*sizeof(struct ground_object_t));
    pthread_mutex_unlock(&mutex);
    
    // send the information
    if(local_filters[0].updated){
      AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, 
        local_filters[0].x_c, 
        local_filters[0].y_c,
        0, 
        0, 
        local_filters[0].color_count, 
        local_filters[0].strategy);
      local_filters[0].updated = false;
      printf("Ground, L = %d, M = %d, R = %d\n", sum_left, sum_mid, sum_right);
    }
  }
  
  // this is for the green detection
  else if (detection_mode == GREEN_DETECTOR)
  {
    global_filters_ground[0].updated = false;
    global_filters_ground[1].updated = false;

    global_filters_edge[0].updated = false;
    global_filters_edge[1].updated = false;

    static struct green_object_t local_filters[2];
    
    // read the message from the global_filters
    pthread_mutex_lock(&mutex);
    memcpy(local_filters, global_filters_green, 2*sizeof(struct green_object_t));
    pthread_mutex_unlock(&mutex);
    
    // send the information
    if(local_filters[0].updated){
      AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, 
        (int16_t)local_filters[0].count_middle, 
        (int16_t)local_filters[0].count_left,
        (int16_t)local_filters[0].count_right, 
        (int16_t)local_filters[0].threshold_sideways, 
        (int32_t)local_filters[0].threshold_middle,
        (int16_t)local_filters[0].strategy);
      local_filters[0].updated = false;

      printf("Green, L = %d, R = %d, thS = %d, M = %d, thM = %d\n", local_filters[0].count_left, local_filters[0].count_right, \
        local_filters[0].threshold_sideways, local_filters[0].count_middle, local_filters[0].threshold_middle);
    }
  }
  
  // this is for the edge detection
  else if (detection_mode == EDGE_DETECTOR)
  {
    global_filters_green[0].updated = false;
    global_filters_green[1].updated = false;

    global_filters_ground[0].updated = false;
    global_filters_ground[1].updated = false;

    static struct edge_t local_filters[2];
    pthread_mutex_lock(&mutex);
    memcpy(local_filters, global_filters_edge, 2*sizeof(struct edge_t));
    pthread_mutex_unlock(&mutex);

    if(local_filters[0].updated){
      AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, 
        0, 
        0,
        0, 
        (int16_t)local_filters[0].highest_edge_height, 
        (int32_t)local_filters[0].highest_column_idx,
        local_filters[0].strategy); // third last element is the the free space height, int32_t, range (0,240)
      local_filters[0].updated = false;

      printf("Edge, high = %d, hi = %d, low = %d, li = %d\n", 
            local_filters[0].highest_edge_height, 
            local_filters[0].highest_column_idx, 
            local_filters[0].shortest_edge_height,
            local_filters[0].shorest_column_idx);
    }
  }
}

