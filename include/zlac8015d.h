#ifndef ZLAC8015D
#define ZLAC8015D

#include <string>
#include <iostream>
#include <chrono>
#include <cstring> // for using memcpy


// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"
#include "Comm/crc_check.h"

struct MOT_DATA{
    int32_t encoder_L = 0;
    int32_t encoder_R = 0;
    double rpm_L = 0;
    double rpm_R = 0;
};

class ZLAC{
protected:
	//  hendler //
    bool PRINT_DEBUG_MSG = false;

    uint8_t hex_cmd[15] = {0};
    uint8_t receive_hex[15] = {0};
    
    struct MOT_DATA ZLAC_STAT;
    
    //  frame shape
    //  |driver_id|function_code|command|data|CRC|

    // driver id//
    uint8_t ID = 0x00;
    //  modbus function code    //
    const uint8_t READ = 0x03;
    const uint8_t WRITE = 0x06;
    const uint8_t MULTI_WRITE = 0x10;

    //  driver init commands    //
    const uint8_t CONTROL_WORD = 0X0E;  //0x200E
    const uint8_t MOTOR_ENABLE[2] = {0x00, 0X08};
    const uint8_t MOTOR_DISABLE[2] = {0x00, 0X07};
    //  driver control setting command  //
    const uint8_t CONTROL_MODE = 0X0D;  //0x200D
    const uint8_t VEL_MODE[2] = {0x00, 0X03};
    //  commands from vel mode  //
    const uint8_t SET_L_RPM = 0X88;     //0x2088
    const uint8_t SET_R_RPM = 0X89;     //0x2089
    const uint8_t GET_RPM = 0XAB;       //0x20AB
    const uint8_t GET_ENCODER_PULSE = 0xA6;   //0x20A6
    
    const uint8_t SET_L_ACC_TIME[2] = {0x20, 0X80};
    const uint8_t SET_R_ACC_TIME[2] = {0x20, 0X81};
    const uint8_t SET_L_DECC_TIME[2] = {0x20, 0X82};
    const uint8_t SET_R_DECC_TIME[2] = {0x20, 0X83};
    
    const uint8_t INITIAL_L_VEL[2] = {0X20, 0X43};
    const uint8_t INITIAL_R_VEL[2] = {0X20, 0X73};

    const uint8_t MAX_SPEED = 0X08;

    /**
     * @brief calculates the crc and stores it in the hex_cmd array, so there is no return value
     */
    void calculate_crc(uint8_t length);

    /**
     * @brief reads from the serial port and saves the string into the receive_hex array
     * @param num_bytes how many bytes to read from the buffer
     * @return return 0 when OK, 1 if crc error
     */
    uint8_t read_hex(uint8_t num_bytes);

    /**
     * @brief print the hex command for debugging
     */
    void print_hex_cmd() const;

    /**
     * @brief print received hex for debugging
     */
    void print_rcv_hex() const;

public:
    serial::Serial _serial;

    /**
     * @brief open serial port communication
     * @param port COM port eg. "/dev/ttyUSB0"
     * @param baudRate default baudrate is 115200
     * @param _ID Set the modbus ID of the motor driver in HEX, default 0x00
     */
    void begin(std::string port, int baudrate = 115200, uint8_t ID = 0x00);
    uint8_t set_vel_mode();

    /**
     * @param acc_time_ms acceleration time in ms eg. 500
     * @return 0 when OK. 1 if crc error
     */
    uint8_t set_acc_time(uint16_t acc_time_ms, std::string side);

    /**
     * @param decc_time_ms decceleration time in ms eg. 500
     * @return 0 when OK. 1 if crc error
     */
    uint8_t set_decc_time(uint16_t decc_time_ms, std::string side);

    /**
     * @return 0 when OK. 1 if crc error
     */
    uint8_t enable();

    /**
     * @brief when motor disabled wheel can spin freely but still can read the rpm
     * @return 0 when OK. 1 if crc error
     */
    uint8_t disable();

    /**
     * @param rpm
     * @param side
     * @return always 0
     */
    uint8_t set_single_rpm(int16_t rpm, std::string side);

    /**
     * @param rpm
     * @return always 0
     */
    uint8_t set_double_rpm(int16_t Lrpm, int16_t Rrpm);

    /**
     * @return request wheel rpm
     */
    MOT_DATA get_rpm();

    /**
     * @return request some encoder pulse count
     */
    MOT_DATA get_position();

    /**
     * @param port
     * @param baudrate
     * @param ID
     * @param DEBUG_MSG_SET
     * @return motor init process
     */
    uint8_t init(std::string port, int baudrate, uint8_t ID, bool DEBUG_MSG_SET);

    /**
     * @return motor terminate process
     */
    uint8_t terminate();

    /**
     * @return Error feedback,
     *          0000h: no error;
     *          0001h: over-voltage;
     *          0002h: under-voltage;
     *          0004h: over-current;
     *          0008h: overload;
     *          0010h: current is out of tolerance;
     *          0020h: encoder is out of tolerance;
     *          0040h: speed is out of tolerance;
     *          0080h: reference voltage error;
     *          0100h: EEPROM read and write error;
     *          0200h: Hall error;
     *          0400h: motor temperature is too high.
     */
    uint16_t get_error();

    /**
     * @return The initial speed when motor on begins.
     */
    uint8_t set_initial_vel(uint16_t rpm, std::string side);

    /**
     * @return Max operating speed of motor. (RPM)
     */
    uint8_t max_speed(uint16_t rpm);
    
    void sleep(unsigned long milliseconds);

    int32_t readInt32FromArray(const unsigned char* target_array, int startIndex);
    int16_t readInt16FromArray(const unsigned char* target_array, int startIndex);
};

#endif