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
  // #include "ros/ros.h"

  //Defning the device name (on physical computer) and the baud rate of the communications.
  #define dxl_id                   3
  #define device_name              "/dev/ttyUSB0"
  #define baudrate                 57600
  #define protocol_version         2.0
  
  //Define some of the more important addresses that we need to keep track of in our code.
  #define drive_mode_addr          10
  #define operating_mode_addr      11
  #define secondary_id_addr        12
  #define torque_enable_addr       64
  #define velocity_p_gain_addr     78
  #define present_position_addr    132
  #define goal_current_addr        102
  #define goal_position_addr       116

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

  int torqueDisable(uint8_t ID, uint8_t *error){
    return write1Byte(ID, torque_enable_addr, torque_disable, error);
  }

  int torqueEnable(uint8_t ID, uint8_t *error){
    return write1Byte(ID, torque_enable_addr, torque_enable, error);
  }

  int setOpMode(uint8_t ID, uint8_t mode, uint8_t *error){
    return write1Byte(ID, operating_mode_addr, mode, error);
  }

  int setGoalPosition(uint8_t ID, uint32_t pos, uint8_t *error){
    return write4Byte(ID, goal_position_addr, pos, error);
  }

  int readPresentPosition(uint8_t ID, uint32_t *pos, uint8_t *error){
    return read4Byte(ID, present_position_addr, pos, error);
  }

  bool openPort(){
    return (portHandler->openPort());
  }

  void closePort(){
    return (portHandler->closePort());
  }

  bool setBaudRate(int baud_rate){
    return portHandler->setBaudRate(baud_rate);
  }

  int main()
  {
    //GroupBulkWrite function instance
    dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

    //GroupBulkRead function instance
    dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

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

    if(openPort()){
      printf("Succeeded to open the port!\n");
    }
    else{
      printf("Failed to open the port\n");
      printf("Press any key to terminate...\n");
      getch();
      return 0;
    }

    if(setBaudRate(baudrate)){
      printf("Succeeded to change the baudrate!\n");
    }
    else{
      printf("Failed to change the baudrate.\n");
      printf("Press any to terminate...\n");
      getch();
      return 0;
    }

    dxl_comm_result = torqueEnable(dxl_id, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS || dxl_error != 0){
      printf("Communication failed!\n");
      printf("%d", dxl_comm_result);
      printf("\n");
      printf("%d", dxl_error);
    }
    else{
      printf("Torque has been enabled.\n");
    }
    int goal_position = position_min;
    while(1){
      printf("Press any key to continue! (or press ESC to quit!)\n");
      if(getch() == esc_ascii_value){
        break;
      }
      else{
        dxl_comm_result = setGoalPosition(dxl_id, goal_position, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS || dxl_error != 0){
          printf("Communication failed!\n");
        }
        else{
          printf("Goal position succesfully set!\n");
          do{
            dxl_comm_result = readPresentPosition(dxl_id, (uint32_t*)&(dxl1_present_position), &dxl_error);
            if(dxl_comm_result != COMM_SUCCESS || dxl_error != 0){
              printf("Communication error!\n");
            }
            else{
              printf("Goal position:%03d, Present position:%03d\n", goal_position, dxl1_present_position);
            }
          }while(abs(goal_position-dxl1_present_position)>moving_status_threshold);
	}
          if(goal_position == position_min){
            goal_position = position_max;
          }
          else{
            goal_position = position_min;
          }
        }  
    }
    dxl_comm_result = torqueDisable(dxl_id, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS || dxl_error != 0){
      printf("Communication failed!\n");
    }
    else{
      printf("Torque disabled.\n");
    }

    closePort();
    return 0;
  }
