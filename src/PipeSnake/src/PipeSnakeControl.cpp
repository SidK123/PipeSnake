  #ifdef __linux__
  #include <unistd.h>
  #include <fcntl.h>
  #include <termios.h>
  #elif defined(_WIN32) || defined(_WIN64)
  #include <conio.h>
  #endif

  #include <stdlib.h>
  #include <stdio.h>
  #include "dynamixel_sdk.h"                                   // Uses DYNAMIXEL SDK library
  #include "ros/ros.h"

  #define drive_mode_addr          10
  #define operating_mode_addr      11
  #define secondary_id_addr        12
  #define torque_enable_addr       64
  #define velocity_p_gain_addr     78
  #define goal_position_addr       116