/**
 * @file configuration.hpp
 * @brief a general file to configure the project
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_CONFIGURATION__CONFIGURATION_HPP_
#define PSAF_CONFIGURATION__CONFIGURATION_HPP_

#include <string>
#include <vector>

/**
 * To enable additional printouts
 */
#define DEBUG false

/**
 * To force some tests to pass. This can be helpful during development. Turn this
 * off for the finished code
 */
#define FORCE_TEST_PASS true

/**
* This header file is used to set certain constants like topic and node names
* Changing the topic names is not advised. Please make sure to set the PSAF 1 flag above to match
* the* the current car. True for PSAF 1 and false for PSAF 2.
*/


/**
* Define realsense topics
* The number of camera streams. Note: Even if only one realsense camera is mounted to the car,
* the different camera streams are seen as separate cameras. If you want more cameras
* simply increase the NBR_OF_CAMS and add the topic name to the vector
*/
#define NBR_OF_CAMS  2
#define CAM_TOPICS {"color/image_raw", "depth/image_rect_raw"}
#define NBR_OF_CAMS_RGB 1                   // Number of cameras for lane detection
#define CAM_TOPIC_RGB {"color/image_raw"}   // Camera topic for lane detection

/**
* Define the uc_bridge topic names
* Note: The PSAF 1 cars usually only have 3 ultrasonic sensors. For the PSAF 2 car
* comment out the current NBR_OF_US_SENSORS and US_TOPICS definitions and comment in
* the others.
*/
#define BUTTON_TOPIC  "uc_bridge/button"
#define SET_SPEED_FORWARD_TOPIC "uc_bridge/set_motor_level_forward"
#define SET_SPEED_BACKWARD_TOPIC  "uc_bridge/set_motor_level_backward"
#define GET_SPEED_TOPIC  "uc_bridge/get_speed"
#define SET_STEERING_TOPIC  "uc_bridge/set_steering"
#define GET_STEERING_TOPIC  "uc_bridge/get_steering"
#define SET_LIGHT_TOPIC  "uc_bridge/light"
#define MANUAL_SIGNALS_TOPIC  "uc_bridge/manual_signals"

#define NBR_OF_US_SENSORS  3
#define US_TOPICS {"uc_bridge/us_front_center", "uc_bridge/us_mid_right", "uc_bridge/us_mid_left"}

#define GET_PUSHBUTTONS_TOPIC "/pb_data"

#endif  // PSAF_CONFIGURATION__CONFIGURATION_HPP_
