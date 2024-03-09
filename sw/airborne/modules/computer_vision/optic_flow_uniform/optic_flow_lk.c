/*
 * Copyright (C) 2014 G. de Croon
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2016 Hrvoje Brezak <hrvoje.brezak@gmail.com>
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
 * @file modules/computer_vision/lib/vision/lucas_kanade.c
 * @brief efficient fixed-point optical-flow calculation
 *
 * - Initial fixed-point C implementation by G. de Croon
 * - Algorithm: Lucas-Kanade by Yves Bouguet
 * - Publication: http://robots.stanford.edu/cs223b04/algo_tracking.pdf
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "optic_flow_lk.h"


/**
 * @file lucas_kanade.c
 *
 * Compute the optical flow of several points using the pyramidal Lucas-Kanade algorithm by Yves Bouguet
 * The initial fixed-point implementation is done by G. de Croon and is adapted by
 * Freek van Tienen for the implementation in Paparazzi.
 * Pyramids implementation and related development done by Hrvoje Brezak.
 * @param[in] *new_img The newest grayscale image (TODO: fix YUV422 support)
 * @param[in] *old_img The old grayscale image (TODO: fix YUV422 support)
 * @param[in] *points Points to start tracking from
 * @param[in,out] points_cnt The amount of points and it returns the amount of points tracked
 * @param[in] half_window_size Half the window size (in both x and y direction) to search inside
 * @param[in] subpixel_factor The subpixel factor which calculations should be based on
 * @param[in] max_iterations Maximum amount of iterations to find the new point
 * @param[in] step_threshold The threshold of additional subpixel flow at which the iterations should stop
 * @param[in] max_points The maximum amount of points to track, we skip x points and then take a point.
 * @param[in] pyramid_level Level of pyramid used in computation (0 == no pyramids used)
 * @param[in] keep_bad_points Do not filter out bad points. The error field will be set accordingly.
 * @return The vectors from the original *points in subpixels
 *
 * Pyramidal implementation of Lucas-Kanade feature tracker.
 *
 * Uses input images to build pyramid of padded images.
 * <p>For every pyramid level:</p>
 * <p>  For all points:</p>
 * - (1) determine the subpixel neighborhood in the old image
 * - (2) get the x- and y- gradients
 * - (3) determine the 'G'-matrix [sum(Axx) sum(Axy); sum(Axy) sum(Ayy)], where sum is over the window
 * - (4) iterate over taking steps in the image to minimize the error:
 *   + [a] get the subpixel neighborhood in the new image
 *   + [b] determine the image difference between the two neighborhoods
 *   + [c] calculate the 'b'-vector
 *   + [d] calculate the additional flow step and possibly terminate the iteration
 * - (5) use calculated flow as initial flow estimation for next level of pyramid
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

struct flow_t *opticFlowLK(struct image_t *new_img, struct image_t *old_img, struct point_t *points,
                           uint16_t *points_cnt, uint16_t half_window_size,
                           uint16_t subpixel_factor, uint8_t max_iterations, uint8_t step_threshold, uint8_t max_points, uint8_t pyramid_level,
                           uint8_t keep_bad_points)
{

  // if no pyramids, use the old code:
  if (pyramid_level == 0) {
    // use the old code in this case:
    return opticFlowLK_flat(new_img, old_img, points, points_cnt, half_window_size, subpixel_factor, max_iterations,
                            step_threshold, max_points, keep_bad_points);
  }

  // Allocate some memory for returning the vectors
  struct flow_t *vectors = calloc(max_points, sizeof(struct flow_t));

  // Determine patch sizes and initialize neighborhoods
  uint16_t patch_size = 2 * half_window_size + 1;
  // TODO: Feature management shows that this threshold rejects corners maybe too often, maybe another formula could be chosen
  uint32_t error_threshold = (25 * 25) * (patch_size * patch_size);
  uint16_t padded_patch_size = patch_size + 2;
  uint16_t border_size = padded_patch_size / 2 + 2; // amount of padding added to images

  // Allocate memory for image pyramids
  struct image_t *pyramid_old = malloc(sizeof(struct image_t) * (pyramid_level + 1));
  struct image_t *pyramid_new = malloc(sizeof(struct image_t) * (pyramid_level + 1));

  // Build pyramid levels
  pyramid_build(old_img, pyramid_old, pyramid_level, border_size);
  pyramid_build(new_img, pyramid_new, pyramid_level, border_size);

  // Create the window images
  struct image_t window_I, window_J, window_DX, window_DY, window_diff;
  image_create(&window_I, padded_patch_size, padded_patch_size, IMAGE_GRAYSCALE);
  image_create(&window_J, patch_size, patch_size, IMAGE_GRAYSCALE);
  image_create(&window_DX, patch_size, patch_size, IMAGE_GRADIENT);
  image_create(&window_DY, patch_size, patch_size, IMAGE_GRADIENT);
  image_create(&window_diff, patch_size, patch_size, IMAGE_GRADIENT);

  // Iterate through pyramid levels
  for (int8_t LVL = pyramid_level; LVL != -1; LVL--) {
    uint16_t points_orig = *points_cnt;
    *points_cnt = 0;
    uint16_t new_p = 0;

    // Calculate the amount of points to skip
    float skip_points = (points_orig > max_points) ? (float)points_orig / max_points : 1;

    // Go through all points
    for (uint16_t i = 0; i < max_points && i < points_orig; i++) {
      uint16_t p = i * skip_points;

      if (LVL == pyramid_level) {
        // Convert point position on original image to a subpixel coordinate on the top pyramid level
        vectors[new_p].pos.x = (points[p].x * subpixel_factor) >> pyramid_level;
        vectors[new_p].pos.y = (points[p].y * subpixel_factor) >> pyramid_level;
        vectors[new_p].flow_x = 0;
        vectors[new_p].flow_y = 0;

      } else {
        // (5) use calculated flow as initial flow estimation for next level of pyramid
        vectors[new_p].pos.x = vectors[p].pos.x << 1;
        vectors[new_p].pos.y = vectors[p].pos.y << 1;
        vectors[new_p].flow_x = vectors[p].flow_x << 1;
        vectors[new_p].flow_y = vectors[p].flow_y << 1;
      }

      // If the pixel is outside original image, do not track it
      if ((((int32_t) vectors[new_p].pos.x + vectors[new_p].flow_x) < 0)
          || ((vectors[new_p].pos.x + vectors[new_p].flow_x) > (uint32_t)((pyramid_new[LVL].w - 1 - 2 * border_size)*
              subpixel_factor))
          || (((int32_t) vectors[new_p].pos.y + vectors[new_p].flow_y) < 0)
          || ((vectors[new_p].pos.y + vectors[new_p].flow_y) > (uint32_t)((pyramid_new[LVL].h - 1 - 2 * border_size)*
              subpixel_factor))) {
        if (keep_bad_points) {
          vectors[new_p].error = LARGE_FLOW_ERROR;
          new_p++;
          (*points_cnt)++;
        }
        continue;
      }


      // (1) determine the subpixel neighborhood in the old image
      image_subpixel_window(&pyramid_old[LVL], &window_I, &vectors[new_p].pos, subpixel_factor, border_size);

      // (2) get the x- and y- gradients
      image_gradients(&window_I, &window_DX, &window_DY);

      // (3) determine the 'G'-matrix [sum(Axx) sum(Axy); sum(Axy) sum(Ayy)], where sum is over the window
      int32_t G[4];
      image_calculate_g(&window_DX, &window_DY, G);

      // calculate G's determinant in subpixel units:
      int32_t Det = (G[0] * G[3] - G[1] * G[2]);

      // Check if the determinant is bigger than 1
      if (Det < 1) {
        if (keep_bad_points) {
          vectors[new_p].error = LARGE_FLOW_ERROR;
          new_p++;
          (*points_cnt)++;
        }
        continue;
      }

      // (4) iterate over taking steps in the image to minimize the error:
      bool tracked = true;

      for (uint8_t it = max_iterations; it--;) {
        struct point_t new_point = { vectors[new_p].pos.x  + vectors[new_p].flow_x,
                 vectors[new_p].pos.y + vectors[new_p].flow_y,
                 0, 0, 0
        };

        // If the pixel is outside original image, do not track it
        if ((((int32_t)vectors[new_p].pos.x  + vectors[new_p].flow_x) < 0)
            || (new_point.x > (uint32_t)((pyramid_new[LVL].w - 1 - 2 * border_size)*subpixel_factor))
            || (((int32_t)vectors[new_p].pos.y  + vectors[new_p].flow_y) < 0)
            || (new_point.y > (uint32_t)((pyramid_new[LVL].h - 1 - 2 * border_size)*subpixel_factor))) {
          tracked = false;
          break;
        }

        //     [a] get the subpixel neighborhood in the new image
        image_subpixel_window(&pyramid_new[LVL], &window_J, &new_point, subpixel_factor, border_size);

        //     [b] determine the image difference between the two neighborhoods
        uint32_t error = image_difference(&window_I, &window_J, &window_diff);

        if (error > error_threshold && it < max_iterations / 2) {
          tracked = false;
          break;
        }

        int32_t b_x = image_multiply(&window_diff, &window_DX, NULL) / 255;
        int32_t b_y = image_multiply(&window_diff, &window_DY, NULL) / 255;


        //     [d] calculate the additional flow step and possibly terminate the iteration
        int16_t step_x = (((int64_t) G[3] * b_x - G[1] * b_y) * subpixel_factor) / Det;
        int16_t step_y = (((int64_t) G[0] * b_y - G[2] * b_x) * subpixel_factor) / Det;

        vectors[new_p].flow_x = vectors[new_p].flow_x + step_x;
        vectors[new_p].flow_y = vectors[new_p].flow_y + step_y;
        vectors[new_p].error = error;

        // Check if we exceeded the treshold CHANGED made this better for 0.03
        if ((abs(step_x) + abs(step_y)) < step_threshold) {
          break;
        }
      } // lucas kanade step iteration

      // If we tracked the point we update the index and the count
      if (tracked) {
        new_p++;
        (*points_cnt)++;
      } else if (keep_bad_points) {
        vectors[new_p].flow_x = 0;
        vectors[new_p].flow_y = 0;
        vectors[new_p].error = LARGE_FLOW_ERROR;
        new_p++;
        (*points_cnt)++;
      }
    } // go through all points

  } // LVL of pyramid

  // Free the images
  image_free(&window_I);
  image_free(&window_J);
  image_free(&window_DX);
  image_free(&window_DY);
  image_free(&window_diff);

  for (int8_t i = pyramid_level; i != -1; i--) {
    image_free(&pyramid_old[i]);
    image_free(&pyramid_new[i]);
  }
  pyramid_old = NULL;
  pyramid_new = NULL;

  // Return the vectors

  int image_width = new_img->w; 
  double flow_sum_left = 0.0, flow_sum_right = 0.0;
  for (uint16_t p = 0; p < *points->count; p++) {
    double magnitude = sqrt(vectors[p].flow_x * vectors[p].flow_x + vectors[p].flow_y * vectors[p].flow_y);
    if (points[p].x < image_width / 2) {
        // left
        flow_sum_left += magnitude;
    } else {
        // right
        flow_sum_right += magnitude;
    }
  }

  struct flow_t *flow_left_right = malloc(sizeof(struct flow_t));
    if (!flow_left_right) {
        // Handle memory allocation failure
        return NULL;
    }
    flow_left_right->mag.left = flow_sum_left; 
    flow_left_right->mag.right = flow_sum_right; 
  return flow_left_right
}


