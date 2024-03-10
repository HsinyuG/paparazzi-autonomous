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
#include "modules/computer_vision/cv_detect_edge.h"
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
#if EDGE_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef EDGE_DETECTOR_FPS1
#define EDGE_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef EDGE_DETECTOR_FPS2
#define EDGE_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
float sigma1 = 0.6;
float tlow1 = 0.5;
float thigh1 = 0.8;

float sigma2 = 0.6;
float tlow2 = 0.5;
float thigh2 = 0.8;

bool cod_draw1 = false;
bool cod_draw2 = false;

// define global variables
struct edge_t {
  // int32_t x_c;
  // int32_t y_c;
  uint16_t higest_column_idx;
  uint16_t first_edge_height;
  bool updated;
};
struct edge_t global_filters[2];

// Function
void find_edge(struct edge_t *local_filter_ptr, struct image_t *img, bool draw, float sigma, float tlow, float thigh);

unsigned char * YUV2Gray(struct image_t *img);

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *edge_detector(struct image_t *img, uint8_t filter)
{
  float sigma, tlow, thigh;
  bool draw;
  // printf("filter: %u\n", filter);
  switch (filter){
    case 1: // front camera
      sigma = sigma1;
      tlow = tlow1;
      thigh = thigh1;
      draw = cod_draw1;
      break;
    case 2:
      sigma = sigma2;
      tlow = tlow2;
      thigh = thigh2;
      draw = cod_draw2;
      break;
    default:
      return img;
  };

   // Filter and find centroid
  struct edge_t local_filter;
  find_edge(&local_filter, img, draw, sigma, tlow, thigh);

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].higest_column_idx = local_filter.higest_column_idx;
  global_filters[filter-1].first_edge_height = local_filter.first_edge_height;
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *edge_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *edge_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return edge_detector(img, 1);
}

struct image_t *edge_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *edge_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return edge_detector(img, 2);
}

void edge_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct edge_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef EDGE_DETECTOR_CAMERA1
#ifdef EDGE_DETECTOR_SIGMA1
  sigma1 = EDGE_DETECTOR_SIGMA1;
  tlow1 = EDGE_DETECTOR_TLOW1;
  thigh1 = EDGE_DETECTOR_THIGH1;
  
#endif
#ifdef EDGE_DETECTOR_DRAW1
  cod_draw1 = EDGE_DETECTOR_DRAW1;
#endif

  cv_add_to_device(&EDGE_DETECTOR_CAMERA1, edge_detector1, EDGE_DETECTOR_FPS1, 0);
#endif

#ifdef EDGE_DETECTOR_CAMERA2
#ifdef EDGE_DETECTOR_LUM_MIN2
  sigma2 = EDGE_DETECTOR_SIGMA2;
  tlow2 = EDGE_DETECTOR_TLOW2;
  thigh2 = EDGE_DETECTOR_THIGH2;

#endif
#ifdef EDGE_DETECTOR_DRAW2
  cod_draw2 = EDGE_DETECTOR_DRAW2;
#endif

  cv_add_to_device(&EDGE_DETECTOR_CAMERA2, edge_detector2, EDGE_DETECTOR_FPS2, 1);
#endif
}

/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
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
      if ( edge[y * img->w + x] < 100) {
        if (draw){
          *yp = 255;  // make pixel brighter in image
        }
      }
    }
  }

  // find the lowest edge for each column
  uint32_t edge_array_length = rows * cols;

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
    // printf("x: %u, ", current_x);
    // printf("y: %u, ", current_y);
    // printf("i: %u, ", current_index);
    // printf("edge: %u; ", edge[current_index]);
    if ((first_edge_x_each_row[current_y]==cols+1) && !edge[current_index]) {
      first_edge_x_each_row[current_y] = current_x;
    }
  }
  
  // find the column with highest free space from bottom
  uint16_t row_with_largest_space = rows/2;
  int16_t largest_first_edge_x = -1; // max x is < 255 so okay
  printf("cols: %d,", cols);

  // median filtering, find the largest space
  for (uint32_t i=1; i<rows-1; i++) {
    first_edge_x_each_row_filtered[i-1] = (first_edge_x_each_row[i-1] + first_edge_x_each_row[i] + first_edge_x_each_row[i+1])/3;
    // printf("%u: %u, ", i-1, first_edge_x_each_row_filtered[i-1]);
    if (first_edge_x_each_row_filtered[i-1] > largest_first_edge_x) {
      largest_first_edge_x = first_edge_x_each_row_filtered[i-1];
      row_with_largest_space = i;
    }
  } 
  if (largest_first_edge_x == -1) {largest_first_edge_x = cols;}
  // TODO: sliding window
  // buffer[least_first_edge_y * 2 * cols + 2 * highest_column_x - 2] = -37.52415; // U
  // buffer[least_first_edge_y * 2 * cols + 2 * highest_column_x] = 157.27575;     // V
  // buffer[least_first_edge_y * 2 * cols + 2 * highest_column_x + 1] = 76.245;    // Y
  printf("free direction: %u, ", row_with_largest_space);
  printf("in 0 - %u\n", rows);
  free(first_edge_x_each_row);
  free(first_edge_x_each_row_filtered);
  local_filter_ptr->first_edge_height = largest_first_edge_x;
  local_filter_ptr->higest_column_idx = row_with_largest_space;
  return;
}

void edge_detector_periodic(void)
{
  static struct edge_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct edge_t));
  pthread_mutex_unlock(&mutex);

  
  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, 0, 0,
        0, 0, local_filters[0].first_edge_height, local_filters[0].higest_column_idx); // second last element is the the free space height, int32_t, range (0,240)
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, 0, 0,
        0, 0, local_filters[1].first_edge_height, local_filters[1].higest_column_idx);
    local_filters[1].updated = false;
  }
  
}
