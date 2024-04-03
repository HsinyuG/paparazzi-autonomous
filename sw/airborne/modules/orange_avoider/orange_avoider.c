/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#include "firmwares/rotorcraft/autopilot_firmware.h"

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);
static uint8_t chooseSelectedIncrementAvoidance(int16_t detect_result);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  MOVE_BACK
};

// define settings
float oa_color_count_frac = 0.18f;

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 1.f;          // heading angle increment [deg]
float maxDistance = 2.25;                // max waypoint displacement [m] 2.25
int16_t green_tracker_direction = ACTION_FORWARD;    //initial direction(ACTION_FORWARD)
int16_t green_detect_result = ACTION_FORWARD;
int32_t degree_to_rotate = 0;
uint32_t count_middle = 0;
uint32_t count_left = 0;
uint32_t count_right = 0;
uint32_t count_tree = 0;

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

bool random_rotate_avoid = true; // = true's reason: difficult to make sure the obstacle avoid is not triggerrd before out of bound when facing outside, and if keeps rotate deterministic it will face dead lock
bool compare_middle_left_right = false; // true --> feasible direction not only requrire middle > threshold, but also require middle > left and right
bool enable_bounds_detect = false; // true --> enable OUT_OF_BOUNDS state detection
bool slight_turn = true; // true --> enable uav to make slightly turn 

bool use_vel_control = true;
float forward_vel = 0.4;
float backward_vel = 0.1;

uint16_t count_move_back = 0;
uint16_t threshold_move_back = 2;
float proportion_move_back = 0.05f;

float proportion_predict = 0.5f;

float total_displacement = 0.0f;
float last_x = 0.0f;
float last_y = 0.0f;

float ang_vel = 3.14f;

uint16_t min_accel_cnt = 20;
uint16_t accel_cnt = 0;

float hysteresis_coeff = 1.2f;






/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static abi_event green_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  // green_detect_result = quality;
}

static void general_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t __attribute__((unused)) quality, int16_t  extra)
{
  green_detect_result = extra;
}

static void green_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t pixel_x, int16_t pixel_y,
                               int16_t pixel_width, int16_t pixel_height,
                               int32_t quality, int16_t extra)
{
  count_middle = pixel_x;
  count_left = pixel_y;
  count_right = pixel_width;
  // threshold_sideways = pixel_height;
  // threshold_middle = quality;
  count_tree = pixel_height;
  // tree_threshold_pixel, middle_threshold_pixel, sideways_threshold_pixel already included.

  green_detect_result = extra;
  // printf("green_detect_result = %d\n", green_detect_result);
}






/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void orange_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}
// SOMETHING CHANGJUN ADDED





/*
 * Initialisation function for the floor detection
 */
void green_tracker_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &green_detection_ev, green_detection_cb);

  #ifdef FORWARD_VEL
  maxDistance = MAX_DISTANCE;
  forward_vel = FORWARD_VEL;
  backward_vel = BACKWARD_VEL;
  threshold_move_back = THRESHOLD_MOVE_BACK;
  proportion_move_back = PROPORTION_MOVE_BACK;
  proportion_predict = PROPORTION_PREDICT;
  ang_vel = ANG_VEL;
  random_rotate_avoid = RANDOM_ROTATE_AVOID;
  compare_middle_left_right = COMPARE_MIDDLE_LEFT_RIGHT;
  enable_bounds_detect = ENABLE_BOUNDS_DETECT;
  slight_turn = SLIGHT_TURN;
  hysteresis_coeff = HYSTERESIS_COEFF;
  min_accel_cnt = MIN_ACCEL_CNT;
  #endif
}






/*
 * This is the state machine for the NAV mode. (WE DID NOT USE IN THE TEST!!!!!)
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 * The function contains the following 4 states:
 * 	1. Safe: Moving forward when there is not any obstacle in front of the drone
 *
 * 	2. Obstacle found: When the figure is out of the threshold, we switch the state to obstacle found, it will stop the drone.
 *
 * 	3. Search for safe heading: When the drone found the obstacle or go out of bound, it will find a safe direction.
 *
 * 	4. Out of bound: Use the number of the green pixels from the front camera to find if the drone is out of bounday.
 *
 */
void orange_avoider_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);

  // update our safe confidence using color threshold
  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

  switch (navigation_state){
    case SAFE:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }

      break;
    case OBSTACLE_FOUND:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      // randomly select new search direction
      chooseRandomIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      increase_nav_heading(heading_increment);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2){
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(heading_increment);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
    default:
      break;
  }
  return;
}






/*
 * This is the state machine for the GUIDED mode. (WE USE IN THE TEST!!!!!)
 * In this function, we give the command through the foward, backward and angular velocity. 
 * The function contains the following 4 states:
 * 	1. Safe: Moving forward when there is not any obstacle in front of the drone.
 * 		 Also the drone will make slight turning while moving forward if it think the right or left as a promising direction.
 *
 * 	2. Obstacle found: When the figure is out of the threshold, we switch the state to obstacle found, it will stop the drone.
 *
 * 	3. Search for safe heading: When the drone found the obstacle or go out of bound, it will find a safe direction.
 * 				    The safety threshold for heading is stricter than that for obstacle detection, preventing incautious drone detections. 
 *
 * 	4. Out of bound: Use the number of the green pixels from the front camera to find if the drone is out of bounday.
 *
 * 	5. Move back: Move Back a little when finding the direction, this is to avoid the collision while turning.
 */
void green_tracker_periodic(void)
{

  if(!autopilot_in_flight()){
    return;
  }
  if (autopilot_mode_auto2 == AP_MODE_GUIDED) use_vel_control = true;
  else use_vel_control = false; // if (autopilot_mode_auto2 == AP_MODE_NAV)

  float moveDistance = maxDistance;
  switch (navigation_state){
    case SAFE:
      accel_cnt ++;
      if ( ((green_detect_result != ACTION_FORWARD) && compare_middle_left_right) ||\
        ((green_detect_result == ACTION_LEFT || green_detect_result == ACTION_RIGHT) && !compare_middle_left_right) ||\
        count_tree > tree_threshold_pixel)
      { // it should be able to detect the boundary as well
        green_tracker_direction = green_detect_result;
        navigation_state = OBSTACLE_FOUND;
      } 
      else if ((green_detect_result == ACTION_FORWARD_LEFT ||\
                green_detect_result == ACTION_FORWARD_RIGHT) &&\
                slight_turn &&\
                accel_cnt > min_accel_cnt)
      {
        green_tracker_direction = green_detect_result;
        if (use_vel_control) {
           guidance_h_set_body_vel(forward_vel, 0); 
           chooseSelectedIncrementAvoidance(green_tracker_direction);
           guidance_h_set_heading_rate(heading_increment * ang_vel * 1.f);
           printf("this drone is making slight turning ----- ");
        }
        else {      
      	  chooseSelectedIncrementAvoidance( green_tracker_direction);
          increase_nav_heading(10.f * heading_increment);
          moveWaypointForward(WP_GOAL, 0.5f * moveDistance);
        }
      	
      	
      }
      else if (use_vel_control)
      {
        // Move waypoint forward
        guidance_h_set_body_vel(forward_vel, 0);
        guidance_h_set_heading_rate(0);
      }
      else
      {
        moveWaypointForward(WP_TRAJECTORY, proportion_predict * moveDistance); // 1.5f *  ; 0.75f + 
        if ( (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))) && enable_bounds_detect )
        { // only necessary if outside someone holding a green cloth
          chooseRandomIncrementAvoidance();
          // green_tracker_direction = green_detect_result;
          // stop
          moveWaypointForward(WP_GOAL, - proportion_move_back * moveDistance);
          moveWaypointForward(WP_TRAJECTORY, - proportion_move_back * moveDistance);
          navigation_state = OUT_OF_BOUNDS;
        }  
        else 
        {
          moveWaypointForward(WP_GOAL, 5.f * moveDistance);
        }
      }  

      break;
    case OBSTACLE_FOUND:
      // stop
      if (use_vel_control)
      {
        guidance_h_set_body_vel(0, 0);
      }
      else
      {
        waypoint_move_here_2d(WP_GOAL);
        waypoint_move_here_2d(WP_TRAJECTORY);
      }

      // Obstacle_found then change yaw velocity:
      if (!random_rotate_avoid) {
        switch (green_tracker_direction)
        {
          case ACTION_LEFT:
            heading_increment = 1.0f; //10.0f;
            break;
          case ACTION_RIGHT:
            heading_increment = -1.0f; //-10.0f;
            break;
          case ACTION_FORWARD_LEFT:
            heading_increment = 0.5f; //5.0f;
            break;
          case ACTION_FORWARD_RIGHT:
            heading_increment = -0.5f; //-5.0f;
            break;
          default:
            break;
        }
      }
      else {
        // randomly select new search direction
        chooseRandomIncrementAvoidance();
      }
      
      navigation_state = MOVE_BACK;

      break;
    case SEARCH_FOR_SAFE_HEADING: // continue on certain direction to turn, to prevent conflict direction in OBSTACLE_FOUND after OUT_OF_BOUNDS
      accel_cnt = 0;
      if (use_vel_control) {
        guidance_h_set_body_vel(0, 0);
        guidance_h_set_heading_rate(heading_increment * ang_vel);
      }
      else {
        increase_nav_heading(10.f * heading_increment);
      }
      // make sure we have a couple of good readings before declaring the way safe
      // if (obstacle_free_confidence >= 2){
      //   navigation_state = SAFE;
      if (!slight_turn){
      	if (  ((hysteresis_coeff * count_tree < tree_threshold_pixel) || !detect_tree) &&\
              (count_middle > hysteresis_coeff * middle_threshold_pixel) &&\
              (count_left > hysteresis_coeff * sideways_threshold_pixel) &&\
              (count_right > hysteresis_coeff * sideways_threshold_pixel) &&\
              (((count_middle > hysteresis_coeff * count_left) && (count_middle > hysteresis_coeff * count_right)) || !compare_middle_left_right))
      	{
        	navigation_state = SAFE;
      	}
      }
      else{
        if(((hysteresis_coeff * count_tree < tree_threshold_pixel) || !detect_tree) &&\
           (count_middle > hysteresis_coeff * middle_threshold_pixel) &&\
           (count_left > hysteresis_coeff * sideways_threshold_pixel) &&\
           (count_right > hysteresis_coeff * sideways_threshold_pixel) &&\
           green_detect_result == ACTION_FORWARD) // shall we use hysteresis here again?
        {
          navigation_state = SAFE;
        }
      }
      break;
    case OUT_OF_BOUNDS: // use the detect result to determine the changing direction, to avoid conflic direction with obstacle avoid? No, will cause deadlock --> obstacle avoid also random direction
      increase_nav_heading(10.f * heading_increment);
      // moveWaypointForward(WP_TRAJECTORY, 1.5f); // has bug: max distance + 0.75 out of boundary, but this one still in, so immediately jump out
      moveWaypointForward(WP_TRAJECTORY, 0.75f + moveDistance);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(10.f * heading_increment);

        // reset safe counter
        // obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
    case MOVE_BACK: // not considering move back and out of bounds, because we come from that direction
        if (use_vel_control)
        {
          guidance_h_set_body_vel(-backward_vel, 0);
        }
        else
        {
          moveWaypointForward(WP_GOAL, - proportion_move_back * moveDistance);
          moveWaypointForward(WP_TRAJECTORY, - proportion_move_back * moveDistance);
        }
        if (count_move_back >= threshold_move_back) {
          navigation_state = SEARCH_FOR_SAFE_HEADING;
          if (use_vel_control)
          {
            guidance_h_set_body_vel(0, 0);
          }
          else
          {
            waypoint_move_here_2d(WP_GOAL);
            waypoint_move_here_2d(WP_TRAJECTORY);
          }

          count_move_back = 0;
        }
        count_move_back ++;

      break;
    default:
      break;
  }
  /**
  float now_ts = (float)get_sys_time_usec() / 100000.f;
  update_odometry();
  // debug message
  printf("L = %d ----- R = %d ----- THS = %d ----- M = %d ----- THM = %d ----- ", \
    count_left, count_right, threshold_sideways, count_middle, threshold_middle);
  **/
  printf("mode: %u ----- ", autopilot_mode_auto2);
  printf("detector: %u ----- ", detection_mode);
  printf("state: %d ----- ", navigation_state);
  printf("detection: %d ----- \n", green_detect_result);
  // printf("disp = %f at %d\n", total_displacement, now_ts);
  // printf("deterministic cmd: %d\n", green_tracker_direction);
  
  return;
  
}






/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  // VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}






/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}






/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  // VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                // POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                // stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}





/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  // VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                // POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}





/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 1.f;
    // VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -1.f;
    // VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}





/*
 * Sets the variable 'heading_increment' according to the direction.
 * It will choose the direction with more green pixels.
 */
uint8_t chooseSelectedIncrementAvoidance(int16_t detect_result)
{
  // Randomly choose CW or CCW avoiding direction
  if (detect_result == ACTION_FORWARD_RIGHT) {
    heading_increment = 1.f;
    // VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -1.f;
    // VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}


void update_odometry(void)
{
  float x = stateGetPositionEnu_i()->x; // mm
  float y = stateGetPositionEnu_i()->y;
  total_displacement += sqrt((x - last_x) * (x - last_x) + (y - last_y) * (y - last_y)) / 100.f;
  last_x = x;
  last_y = y;
}
