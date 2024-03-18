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
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
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

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
  uint16_t hue[256];
  uint16_t intensity[256];
};
struct color_object_t global_filters[2];

// Function
uint32_t histogram_front(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint16_t *hue, uint16_t *intensity, uint8_t *strategy);
void histogram_bottom(struct image_t *img, uint16_t *hue, uint16_t *intensity);
                              
// the function to turn the rgb image to hsi
void turn_yuv_hsi(struct image_t *img, uint8_t *buf_HSI);

// the function to calculate the relationship parameter between two histogram
uint32_t multiply(uint16_t *hue_bot, uint16_t *intensity_bot, uint16_t *hue, uint16_t *intensity);

/*
 * this is the function for the object detector of the front camera
 * 1. we will get the histogram of three different part in the front camera
 * 2. if we have get the histogram from the bottom camera, we will make a comparision between these histograms
 */
static struct image_t *object_detector_front(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;
  
  //initialize the variable to get the histogram from the bottom camera
  uint16_t hue[256] = {0};
  uint16_t intensity[256] = {0};
  if (global_filters[1].updated == true){
      memcpy(hue, global_filters[1].hue, 256*sizeof(uint16_t));
      memcpy(intensity, global_filters[1].intensity, 256*sizeof(uint16_t));
  }
  
  //initialize the strategy parameter
  uint8_t strategy = 0;

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
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;

  // Filter and find centroid
  uint32_t count = histogram_front(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, hue, intensity, &strategy);
  //VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  //VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
        hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}


/*
 * this is the function for the object detector of the bottom camera
 * 1. we will get the histogram of color in the bottom camerar
 * 2. we will save the histogram in a global variable
 */
static struct image_t *object_detector_bottom(struct image_t *img, uint8_t filter)
{

  uint16_t hue[256] = {0};
  uint16_t intensity[256] = {0};
  
  // Filter and find centroid
  histogram_bottom (img, hue, intensity);

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = 0;
  global_filters[filter-1].x_c = 0;
  global_filters[filter-1].y_c = 0;
  global_filters[filter-1].updated = true;
  
  // save the hue and intensity of the bottom camera to the global variables
  memcpy(global_filters[filter-1].hue, hue, 256*sizeof(uint16_t));
  memcpy(global_filters[filter-1].intensity, intensity, 256*sizeof(uint16_t));
  pthread_mutex_unlock(&mutex);
  

  return img;
}

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
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
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

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
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
}

/* this function is to calculate the color histogram from the image of the front camera
 * it will get the histogram of the bottom camera with a pointer of an array
 * it will make a comparision between these two histogram and make a strategy
 */
uint32_t histogram_front(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint16_t *hue, uint16_t *intensity, uint8_t *strategy)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;
  // initialie the variables to save the color histogram in three different area
  uint16_t histogram_hue_right[256] = {0};
  uint16_t histogram_hue_mid[256] = {0};
  uint16_t histogram_hue_left[256] = {0};
  uint16_t histogram_intensity_right[256] = {0};
  uint16_t histogram_intensity_mid[256] = {0};
  uint16_t histogram_intensity_left[256] = {0};

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
  
  //create two variables to save the RGB image and the HSI image
  uint8_t *array = (uint8_t *)malloc(2 * img->w * img->h * sizeof(uint8_t));

  // get the HSI from the YUV image
  turn_yuv_hsi(img,array);
  
  // now we get the HSI message from the YUV image of the front camera
  for (uint16_t n = 0; n < 3; n++) {
    for (uint16_t y = 0; y < 100; y++) {
      for (uint16_t x = 0; x < 50; x ++) {
          uint8_t hue = array[(50*2 + y * 2 + n*150*2) * img->w + 2 * x];
          uint8_t intensity =  array[(50*2 + y * 2 + n*150*2) * img->w + 2 * x + 1];
          if (n==0){
            histogram_hue_left[hue] += 1;
            histogram_intensity_left[intensity] += 1;
          }
          else if(n==1){
            histogram_hue_mid[hue] += 1;
            histogram_intensity_mid[intensity] += 1;
          }
          else if(n==2){
            histogram_hue_right[hue] += 1;
            histogram_intensity_right[intensity] += 1;
          }
        }
      }
    }
  printf("front %d\n",histogram_hue_mid[40]);
  
  // compare the histogram with the histogram from the bottom
  uint32_t sum_left = multiply(hue, intensity, histogram_hue_left, histogram_intensity_left);
  uint32_t sum_mid = multiply(hue, intensity, histogram_hue_mid, histogram_intensity_mid);
  uint32_t sum_right = multiply(hue, intensity, histogram_hue_right, histogram_intensity_right); 

  // choose the biggest relationship and set the strategy
  uint32_t biggest_direction = sum_left;
  printf("the sum of three different direcction is %d,%d,%d\n", sum_left, sum_mid, sum_right);
  *strategy = 0;
  if (sum_mid>biggest_direction){
    biggest_direction = sum_mid;
    *strategy = 1;
  }
  if (sum_right > biggest_direction){
    biggest_direction = sum_right;
    *strategy = 2;
  }
    free(array);
  printf("the current strategy is %d\n", *strategy);
  
  return cnt;
}

/* this function is to calculate the color histogram from the image of the bottom camera
 * it will send the histogram back with a pointer of an array
 */
void histogram_bottom(struct image_t *img, uint16_t *hue, uint16_t *intensity)
{
  
  //create two variables to save the RGB image and the HSI image
  uint8_t *array = (uint8_t *)malloc(2 * img->w * img->h * sizeof(uint8_t));

  // get the HSI from the YUV image
  turn_yuv_hsi(img,array);

  // get the histogram of the bottom camera according to the HSI message
  for (uint16_t y = 0; y < img->w; y++) {
      for (uint16_t x = 0; x < img->h; x ++) {
      	uint8_t hue_num = array[y * 2 * img->w + 2 * x];
        uint8_t intensity_num =  array[y * 2 * img->w + 2 * x + 1];
        hue[hue_num] += 1;
        intensity[intensity_num] += 1;
      }
  }
  printf("bottom %d\n",hue[40]);
  free(array);
}


void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        0, 0, local_filters[1].color_count, 1);
    local_filters[1].updated = false;
  }
}

// the function to turn the rgb image to hsi 
void turn_yuv_hsi(struct image_t *img, uint8_t *buf_HSI){

  // get the buffer from the YUV and hsi image
  uint8_t *buffer = img->buf;

  // Go through all the pixels in the YUV image
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

      // draw the image to RGB
 
      uint8_t r = *yp + 1.44*(*vp);
      uint8_t g = *yp - 0.39*(*up) - 0.58*(*vp);
      uint8_t b = *yp + 2.03*(*up);
      
      // draw the image to HSI
      double numerator = 0.5 * ((r - g) + (r - b));
      double denominator = sqrt((r - g) * (r - g) + (r - b) * (g - b));
      double theta = acos(numerator / denominator);
      double pi = M_PI;
      if (b<=g){
        buf_HSI[y * 2 * img->w + 2 * x] = theta*180/pi;
      }
      else{
        buf_HSI[y * 2 * img->w + 2 * x] = 360 - theta*180/pi;
      }
      buf_HSI[y * 2 * img->w + 2 * x + 1] = (r+g+b)/3;
    }
  }

}

// calculate the relationship parameter between two histogram
uint32_t multiply(uint16_t *hue_bot, uint16_t *intensity_bot, uint16_t *hue, uint16_t *intensity){
  uint32_t sum_hue = 0;
  uint32_t sum_intensity = 0;
  for (uint16_t y = 0; y < 256; y++) {
    sum_hue = sum_hue + hue_bot[y]*hue[y];
    sum_intensity = sum_intensity + intensity_bot[y]*intensity[y]; 
  }
  uint32_t sum = (sum_hue + sum_intensity)/2;
  return sum;
}
