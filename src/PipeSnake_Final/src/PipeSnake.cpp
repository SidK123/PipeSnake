  #ifdef __linux__
  #include <unistd.h>
  #include <fcntl.h>
  #include <termios.h>
  #elif defined(_WIN32) || defined(_WIN64)
  #include <conio.h>
  #endif

  #include <stdlib.h>
  #include <stdio.h>
  //#include <ros/ros.h>
  #include "dynamixel_sdk/dynamixel_sdk.h"                                   // Uses DYNAMIXEL SDK library
  #include <ros/ros.h>

  //Defning the device name (on physical computer) and the baud rate of the communications.
  #define dxl1                     1
  #define dxl2                     2
  #define dxl3                     3
  #define dxl4                     4
  #define dxl5                     5
  #define dxl6                     6
  #define dxl7                     7
  #define dxl8                     8
  #define dxl9                     9
  #define dxl10                    10
  #define dxl11                    11
  #define dxl12                    12
  #define device_name              "/dev/ttyUSB0"
  #define baudrate                 57600
  #define protocol_version         2.0
  
  //Define some of the more important addresses that we need to keep track of in our code.
  #define drive_mode_addr          10
  #define operating_mode_addr      11
  #define secondary_id_addr        12
  #define torque_enable_addr       64
  #define velocity_p_gain_addr     78
  #define goal_current_addr        102
  #define goal_velocity_addr       104
  #define goal_position_addr       116
  #define present_velocity_addr    128
  #define present_position_addr    132

  //Define some of the important values and what they mean.
  #define torque_enable            1
  #define velocity_mode            1
  #define current_pos_ctrl_mode    5
  #define torque_disable           0
  #define moving_status_threshold  20
  #define position_min             0
  #define position_max             4095
  #define esc_ascii_value          0x1b

  //The Dynamixel PortHandler is what is used to communicate with the Dynamixel and deals with opening, closing ports and setting the baudrates.
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(device_name);

  //The Dynamixel PacketHandler is what is used to write to the addresses on the actual Dynamixel and change the values of the different registers on the device.
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);


  int getch(){
    #ifdef __linux__
      struct termios oldt, newt;
      int ch;
      tcgetattr(STDIN_FILENO, &oldt);
      newt = oldt;
      newt.c_lflag &= ~(ICANON | ECHO);
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);
      ch = getchar();
      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
      return ch;
    #elif defined(_WIN32) || defined(_WIN64)
      return _getch();
    #endif
  }
  
  int write1Byte(uint8_t ID, uint16_t address, uint8_t data, uint8_t *error){
    return packetHandler->write1ByteTxRx(portHandler, ID, address, data, error);
  }

  int read1Byte(uint8_t ID, uint16_t address, uint8_t *data, uint8_t *error){
    return packetHandler->read1ByteTxRx(portHandler, ID, address, data, error); 
  }

  int write2Byte(uint8_t ID, uint16_t address, uint16_t data, uint8_t *error){
    return packetHandler->write2ByteTxRx(portHandler, ID, address, data, error);
  }

  int read2Byte(uint8_t ID, uint16_t address, uint16_t *data, uint8_t *error){
    return packetHandler->read2ByteTxRx(portHandler, ID, address, data, error);
  }

  int write4Byte(uint8_t ID, uint16_t address, uint32_t data, uint8_t *error){
    return packetHandler->write4ByteTxRx(portHandler, ID, address, data, error);
  }

  int read4Byte(uint8_t ID, uint16_t address, uint32_t *data, uint8_t *error){
    return packetHandler->read4ByteTxRx(portHandler, ID, address, data, error);
  }

  uint32_t readPresentPosition(uint8_t ID, uint8_t *error){
    uint32_t presentPosition = 0;
    int dxl_comm_result = read4Byte(ID, 132, &presentPosition, error);
    return presentPosition;
  }

  int32_t readPresentVelocity(uint8_t ID, uint8_t *error){
    int32_t presentVelocity = 0;
    int dxl_comm_result = read4Byte(ID, 132, (uint32_t*)&presentVelocity, error);
    return presentVelocity;
  }

  int32_t readGoalVelocity(uint8_t ID, uint8_t *error){
    int32_t goalVelocity = 0;
    int dxl_comm_result = read4Byte(ID, 104, (uint32_t*)&goalVelocity, error);
    return goalVelocity;
  }

  void torqueDisable(uint8_t ID, uint8_t *error){
    int dxl_comm_result = write1Byte(ID, torque_enable_addr, torque_disable, error);
    if(dxl_comm_result != COMM_SUCCESS || *error != 0){
          printf("Communication failed!\n");
    }
    else{
          printf("Torque disabled.\n");
    }
  }

  void torqueEnable(uint8_t ID, uint8_t *error){
    int dxl_comm_result = write1Byte(ID, torque_enable_addr, torque_enable, error);
    if(dxl_comm_result != COMM_SUCCESS || *error != 0){
      printf("Communication failed.\n");
    }
    else{
      printf("Torque enabled.\n");
    }
  }

  void torqueEnableRoll(uint8_t ID, uint8_t *error){
    int dxl_comm_result = write1Byte(ID, 512, torque_enable, error);
    if(dxl_comm_result != COMM_SUCCESS || *error != 0){
      printf("Communication failed.\n");
    }
    else{
      printf("Torque enabled.\n");
    }
  }

  void torqueDisableRoll(uint8_t ID, uint8_t *error){
    int dxl_comm_result = write1Byte(ID, 512, torque_disable, error);
    if(dxl_comm_result != COMM_SUCCESS || *error != 0){
      printf("Communication failed.\n");
    }
    else{
      printf("Torque disabled.\n");
    }
  }

  int setOpMode(uint8_t ID, uint8_t mode, uint8_t *error){
    return write1Byte(ID, operating_mode_addr, mode, error);
  }

  void setVelocityControl(uint8_t ID, uint8_t *error){
    int dxl_comm_result = setOpMode(ID, velocity_mode, error);
    if(dxl_comm_result != COMM_SUCCESS || *error != 0){
      printf("Communication failed.\n");
    }
    else{
      printf("Velocity control set.\n");
    }
  }

  void setCurrentBasedPosControl(uint8_t ID, uint8_t *error){
    int dxl_comm_result = setOpMode(ID, current_pos_ctrl_mode, error);
    if(dxl_comm_result != COMM_SUCCESS || *error != 0){
      printf("Communication failed.\n");
    }
    else{
      printf("Current based position control set.\n");
    }
  }

  void setGoalVelocity(uint8_t ID, uint32_t vel, uint8_t *error){
    int dxl_comm_result = write4Byte(ID, goal_velocity_addr, vel, error);
    if(dxl_comm_result != COMM_SUCCESS || *error != 0){
        printf("Communication failed.\n");
      }
      else{
        printf("Goal Velocity set.\n");
      }
  }

  void setGoalPosition(uint8_t ID, uint32_t pos, uint8_t *error){
    int dxl_comm_result = write4Byte(ID, goal_position_addr, pos, error);
    if(dxl_comm_result != COMM_SUCCESS || *error != 0){
      printf("Communication failed.\n");
    }
    else{
      printf("Goal Position set.\n");
    }
  }

  void setGoalPositionRoll(uint8_t ID, uint32_t pos, uint8_t *error){
    int dxl_comm_result = write4Byte(ID, 564, pos, error);
    if(dxl_comm_result != COMM_SUCCESS || *error != 0){
      printf("Communication failed.\n");
    }
    else{
      printf("Goal Position set.\n");
    }
  }

  void setGoalCurrent(uint8_t ID, uint32_t curr, uint8_t *error){
    int dxl_comm_result = write2Byte(ID, goal_current_addr, curr, error);
    if(dxl_comm_result != COMM_SUCCESS || *error != 0){
      printf("Communication failed.\n");
    }
    else{
      printf("Goal current set.\n");
    }
  }

  void rotate90(uint8_t ID, uint8_t *error){
    int dxl_comm_result = write4Byte(ID, 564, -243856, error);
    if(dxl_comm_result != COMM_SUCCESS || *error != 0){
      printf("Communication failed.\n");
    }
    else{
      printf("Rotated 90 degrees.\n");
    }
  }

  void rotate90Opp(uint8_t ID, uint8_t *error){
    int dxl_comm_result = write4Byte(ID, 564, 0, error);
    if(dxl_comm_result != COMM_SUCCESS || *error != 0){
      printf("Communication failed.\n");
    }
    else{
      printf("Rotated 90 degrees in opposite direction.\n");
    }

  }

  int readPresentPosition(uint8_t ID, uint32_t *pos, uint8_t *error){
    return read4Byte(ID, goal_position_addr, pos, error);
  }

  void openPort(){
    if((portHandler->openPort())){
      printf("Succeeded to open the port!\n");
    }
    else{
      printf("Failed to open the port\n");
      printf("Press any key to terminate...\n");
      getch();
    }
  }

  void closePort(){
    return (portHandler->closePort());
  }

  void setBaudRate(int baud_rate){
    if(portHandler->setBaudRate(baud_rate)){
      printf("Succeeded to change the baudrate!\n");
    }
    else{
      printf("Failed to change the baudrate.\n");
      printf("Press any to terminate...\n");
      getch();
    }
  }

  #include "sensor_msgs/Joy.h"

  void initializePipeSnake(){
    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;              // Communication result
    bool dxl_addparam_result = false;                // addParam result
    bool dxl_getdata_result = false;                 // GetParam result
    int dxl_goal_position[2] = {position_min, position_max};         // Goal position

    uint8_t dxl_error = 0;                           // Dynamixel error
    uint8_t dxl_led_value[2] = {0x00, 0xFF};         // Dynamixel LED value for write
    uint8_t param_goal_position[4];
    int32_t dxl1_present_position = 0;               // Present position
    uint8_t dxl2_led_value_read;                     // Dynamixel LED value for read

    int wheel_motors[6] = {3, 4, 5, 6, 9, 11};
    int joint_actuators[4] = {2, 8, 10, 12};

    /*
    Wheel positions:
    Wheel 3: Upper left.
    Wheel 4: Lower right of left section.
    Wheel 5: Lower left of left section.
    Wheel 6: Lower left of right section.
    Wheel 9: Upper right.
    Wheel 11: Lower right of right section.
    */

    /*
    Joint actuator positions:
    Joint 12: 1268 (Left up bend)
    Joint 8: 433 (Left connected to roll coupler)
    Joint 10: 2716   (Right Up bend)
    Joint 2: 785    (Right connected to roll coupler)
    */

    openPort();
    setBaudRate(baudrate);

    //rotate90(1, &dxl_error);
    torqueEnableRoll(1, &dxl_error);
    setGoalPosition(1, 100, &dxl_error);

    for(int i = 0; i < 6; i++){
      torqueEnable(wheel_motors[i], &dxl_error);
    }

    setGoalVelocity(5, -50, &dxl_error);
    setGoalVelocity(4, -50, &dxl_error);
    setGoalVelocity(3, +50, &dxl_error);
    setGoalVelocity(6, 50, &dxl_error);
    setGoalVelocity(9, -50, &dxl_error);
    setGoalVelocity(11, 50, &dxl_error);

    for(int i = 0; i < 4; i++){
      setCurrentBasedPosControl(joint_actuators[i], &dxl_error);
      setGoalCurrent(joint_actuators[i], 75, &dxl_error);
    }

    int goalPositions[4] = {1626, 2722, 2933, 2933};

    torqueEnable(12, &dxl_error);
    setGoalPosition(12, 1268, &dxl_error); //Decreasing this expands it, makes it closer to 180 degrees, increasing it contracts it.

    torqueEnable(8, &dxl_error);
    setGoalPosition(8, 433, &dxl_error); //Decreasing this contracts it, makes it closer to 0 degrees.

    torqueEnable(10, &dxl_error);
    setGoalPosition(10, 2716, &dxl_error); //Decreasing this expands it, makes it closer to 180 degrees, increasing it contracts it.
    
    torqueEnable(2, &dxl_error);
    setGoalPosition(2, 785, &dxl_error); //Decreasing this contracts it, makes it closer to 0 degrees.
    
    /*for(int j = 0; j < 1000; j+=10){
    for(int i = 0; i < 4; i++){
      goalPositions[i] = goalPositions[i] + 1;
      setGoalPosition(joint_actuators[i], goalPositions[i], &dxl_error);
    }
    }*/
    printf("All joint actuators and motors have been initialized.");
  }

  void JoyStickCallback(const sensor_msgs::Joy::ConstPtr& msg){
    uint8_t dxl_error = 0;
    if(msg->axes[1] > 0.1 || msg->axes[1] < -0.1){
      printf("Slowdown or speedup!\n");
      setGoalVelocity(3, 260*(msg->axes[1]), &dxl_error);    
      setGoalVelocity(4, -260*(msg->axes[1]), &dxl_error);  
      setGoalVelocity(5, -260*(msg->axes[1]), &dxl_error);    
      setGoalVelocity(6, 260*(msg->axes[1]), &dxl_error);    
      setGoalVelocity(9, -260*(msg->axes[1]), &dxl_error);    
      setGoalVelocity(11, 260*(msg->axes[1]), &dxl_error);    
    }
    else if(msg->buttons[0] == 1){
      setGoalPosition(10, 2716-100, &dxl_error); //Expands the joint so that it is able to rotate.
      setGoalPosition(1, 300, &dxl_error); //Change 300 to whatever the rotated value is
    }
    else if(msg->buttons[3] == 1){
      setGoalPosition(1, 100, &dxl_error); //Change 100 to whatever the original value is
      setGoalPosition(10, 2716, &dxl_error); //Contracts the joint to its original position
    }
    else if(msg->axes[3] > 0.1 || msg->axes[3] < -0.1){
      printf("Contracting and expanding!\n");
      setGoalPosition(12, 1268-200*(msg->axes[3]), &dxl_error);        
      setGoalPosition(10,  2716-200*(msg->axes[3]), &dxl_error);     
      setGoalPosition(8, 433+200*(msg->axes[3]), &dxl_error);        
      setGoalPosition(2,  785+200*(msg->axes[3]), &dxl_error);                   
    }
    else if(msg->buttons[1] == 1){
      printf("A has been pressed!\n");
      torqueDisable(3, &dxl_error);
      torqueDisable(4, &dxl_error);
      torqueDisable(5, &dxl_error);
      torqueDisable(6, &dxl_error);
      torqueDisable(9, &dxl_error);
      torqueDisable(11, &dxl_error);
      torqueDisable(2, &dxl_error);
      torqueDisable(8, &dxl_error);
      torqueDisable(10, &dxl_error);
      torqueDisable(12, &dxl_error);
      torqueDisableRoll(1, &dxl_error);
      closePort();
    }
  }


  int main(int argc, char **argv){
    initializePipeSnake();
    ros::init(argc, argv, "Joy_Listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/joy", 1000, JoyStickCallback);
    ros::spin();
    return 0;
  }