#include "zlac8015d.h"

////////////////  MOT PARAMETER SETTING PART  ////////////////
void ZLAC::begin(std::string port, int baudrate, uint8_t ID){
    this->ID = ID;
    _serial.setPort(port);
    _serial.setBaudrate(baudrate);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    _serial.setTimeout(timeout);
    _serial.open();
    _serial.flushInput();
    printf("%d: SERIAL OK!", ID);
}

uint8_t ZLAC::enable(){ //motor enable command 0x0103/0x200E/0x0008/CRC
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = 0x20;
    hex_cmd[3] = CONTROL_WORD;
    hex_cmd[4] = MOTOR_ENABLE[0];
    hex_cmd[5] = MOTOR_ENABLE[1];
    calculate_crc(8);
    _serial.write(hex_cmd, 8);
    if (read_hex(8)){
        printf("\nMOTOR_ENABLE_ERR\n");
        return 1;
    } 
    return 0;
}

uint8_t ZLAC::disable(){ //motor stop command 0x0103/0x200E/0x0007/CRC
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = 0x20;
    hex_cmd[3] = CONTROL_WORD;
    hex_cmd[4] = MOTOR_DISABLE[0];
    hex_cmd[5] = MOTOR_DISABLE[1];
    calculate_crc(8);
    _serial.write(hex_cmd, 8);
    if (read_hex(8)){
        printf("\nMOTOR_DISABLE_ERR\n");
        return 1;
    } 
    return 0;
}

uint8_t ZLAC::set_vel_mode(){
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = 0x20;
    hex_cmd[3] = CONTROL_MODE;
    hex_cmd[4] = VEL_MODE[0];
    hex_cmd[5] = VEL_MODE[1];
    calculate_crc(8);
    _serial.write(hex_cmd, 8);
    if (read_hex(8)){
        printf("\nERR_SET_VEL_MODE\n");
        return 1;
    }
    return 0;
}

uint8_t ZLAC::set_acc_time(uint16_t acc_time_ms, std::string side){
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    if (side == "LEFT"){
        hex_cmd[2] = SET_L_ACC_TIME[0];
        hex_cmd[3] = SET_L_ACC_TIME[1];
    }
    else if (side == "RIGHT"){
        hex_cmd[2] = SET_R_ACC_TIME[0];
        hex_cmd[3] = SET_R_ACC_TIME[1];
    }
    hex_cmd[4] = (acc_time_ms >> 8) & 0xFF;
    hex_cmd[5] = acc_time_ms & 0xFF;
    calculate_crc(8);
    _serial.write(hex_cmd, 8);
    if (read_hex(8)){
        printf("\nERR_SET_ACC_TIME\n");
        return 1;
    } 
    return 0;
}

uint8_t ZLAC::set_decc_time(uint16_t decc_time_ms, std::string side){
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    if (side == "LEFT"){
        hex_cmd[2] = SET_L_DECC_TIME[0];
        hex_cmd[3] = SET_L_DECC_TIME[1];
    }
    else if (side == "RIGHT"){
        hex_cmd[2] = SET_R_DECC_TIME[0];
        hex_cmd[3] = SET_R_DECC_TIME[1];
    }
    hex_cmd[4] = (decc_time_ms >> 8) & 0xFF;
    hex_cmd[5] = decc_time_ms & 0xFF;
    calculate_crc(8);
    _serial.write(hex_cmd, 8);
    if (read_hex(8)){
        printf("\nERR_SET_DECC_TIME\n");
        return 1;
    } 
    return 0;
}

uint8_t ZLAC::set_initial_vel(uint16_t rpm, std::string side){
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    if (side == "LEFT"){
        hex_cmd[2] = INITIAL_L_VEL[0];
        hex_cmd[3] = INITIAL_L_VEL[1];
    }
    else if (side == "RIGHT"){
        hex_cmd[2] = INITIAL_R_VEL[0];
        hex_cmd[3] = INITIAL_R_VEL[1];
    }
    hex_cmd[4] = (rpm >> 8) & 0xFF;
    hex_cmd[5] = rpm & 0xFF;
    calculate_crc(8);
    _serial.write(hex_cmd, 8);
    if (read_hex(8)){
        printf("\nERR_SET_INITAL_VEL\n");
        return 1;
    } 
    return 0;
}

uint8_t ZLAC::max_speed(uint16_t rpm){
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = 0x20;
    hex_cmd[3] = MAX_SPEED;
    hex_cmd[4] = ((rpm>>8)&0xFF);
    hex_cmd[5] = (rpm&0xFF);
    calculate_crc(8);
    _serial.write(hex_cmd, 8);
    if (read_hex(8)){
        printf("\nERR_SET_MAX_SPD\n");
        return 1;
    }
    return 0;
}

////////////////  MOTOR VEL CONTROL MODE FUNC ////////////////
uint8_t ZLAC::set_single_rpm(int16_t rpm, std::string side){
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = WRITE;
    hex_cmd[2] = 0x20;
    if (side == "LEFT") hex_cmd[3] = SET_L_RPM;
    else if (side == "RIGHT") {
        hex_cmd[3] = SET_R_RPM; 
        rpm = -rpm;}
    hex_cmd[4] = (rpm >> 8) & 0xFF;
    hex_cmd[5] = rpm & 0xFF;
    calculate_crc(8);
    _serial.write(hex_cmd, 8);
    if (read_hex(8)) {
        printf("\nERR_SET_S_RPM\n");
        return 1;
    }
    return 0;
}

uint8_t ZLAC::set_double_rpm(int16_t Lrpm, int16_t Rrpm){
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = MULTI_WRITE;
    hex_cmd[2] = 0x20;
    hex_cmd[3] = SET_L_RPM;
    //following 3byte is required when use this double RPM command
    hex_cmd[4] = 0x00; //high 8 bits of reg number
    hex_cmd[5] = 0x02; //low 8 bits of reg number
    hex_cmd[6] = 0x04; //number of bytes
    //data 0
    hex_cmd[7] = (Lrpm >> 8) & 0xFF;    
    hex_cmd[8] = Lrpm & 0xFF;
    //data 1
    Rrpm = -Rrpm;
    hex_cmd[9] = (Rrpm >> 8) & 0xFF;
    hex_cmd[10] = Rrpm & 0xFF;
    calculate_crc(13);
    _serial.write(hex_cmd, 13);
    if (read_hex(8)) {
        printf("\nERR_SET_D_RPM\n");
        return 1;
    }
    return 0;
}

MOT_DATA ZLAC::get_rpm(){
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = READ;
    hex_cmd[2] = 0x20;
    hex_cmd[3] = GET_RPM;
    hex_cmd[4] = 0x00;
    hex_cmd[5] = 0x04;
    calculate_crc(8);
    _serial.write(hex_cmd, 8);
    if (read_hex(13)){
        printf("\nERR_GET_RPM\n");
        return ZLAC_STAT;
    }
    else{
        ZLAC_STAT.rpm_L = double(readInt16FromArray(receive_hex, 3))/10.0;    //(unit 0.1 RPM)
        ZLAC_STAT.rpm_R = -double(readInt16FromArray(receive_hex, 5))/10.0; 
        //printf("\n\nRPML:%lf|RPMR:%lf|",ZLAC_STAT.rpm_L, ZLAC_STAT.rpm_R);
        return ZLAC_STAT;
    }
}

MOT_DATA ZLAC::get_position(){  //0x0103|0x20A6|0x0005|CRC can rcv LR motor encoder count
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = ID;
    hex_cmd[1] = READ;
    hex_cmd[2] = 0x20;
    hex_cmd[3] = GET_ENCODER_PULSE;
    hex_cmd[4] = 0x00;
    hex_cmd[5] = 0x05;
    calculate_crc(8);
    _serial.write(hex_cmd, 8);
    if (read_hex(15)){
        printf("\nERR_GET_POS\n");
        return ZLAC_STAT;
    }
    else{
        ZLAC_STAT.encoder_L = readInt32FromArray(receive_hex, 5);
        ZLAC_STAT.encoder_R = -readInt32FromArray(receive_hex, 9); 
        return ZLAC_STAT;
    }
}

//////////////// HELPER Functions ////////////////
uint8_t ZLAC::init(std::string port, int baudrate, uint8_t ID, bool DEBUG_MSG_SET){
    PRINT_DEBUG_MSG = DEBUG_MSG_SET;
    printf("===serial begin===\n");
    begin(port, baudrate, ID);
    printf("===set_vel_mode===\n");
    set_vel_mode();
    printf("===enable===\n");
    enable();
    return set_double_rpm(0, 0);
}

uint8_t ZLAC::terminate(){
    set_double_rpm(0, 0);
    printf("===disable===\n");
    return disable();
}

void ZLAC::sleep(unsigned long milliseconds){
#ifdef _WIN32
    Sleep(milliseconds); // 100 ms
#else
    usleep(milliseconds * 1000); // 100 ms
#endif
}

void ZLAC::calculate_crc(uint8_t length){
    // calculate crc and append to hex cmd
    unsigned short result = crc16(hex_cmd, length - 2);
    hex_cmd[length - 2] = result & 0xFF;
    hex_cmd[length - 1] = (result >> 8) & 0xFF;
}

uint8_t ZLAC::read_hex(uint8_t num_bytes){  //crc err check (err = 1)
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    _serial.setTimeout(timeout);
    std::string line = _serial.read(num_bytes);
    // convert string to hex
    if(PRINT_DEBUG_MSG) printf("\nRCV_HEX:");
    for (uint8_t i = 0; i < uint8_t(line.size()); i++){
        receive_hex[i] = uint8_t(line[i]);
        if(PRINT_DEBUG_MSG) printf("|%02x|", receive_hex[i]);
    }
    if(PRINT_DEBUG_MSG) printf("\n");
    // crc check of received data
    if (crc16(receive_hex, num_bytes) != 0){
        printf("\nRCV_CRC_ERR\n");
        return 1;
    }
    return 0;
}

void ZLAC::print_hex_cmd() const{
    // print
    for (int i = 0; i < 8; i++){
        printf("%d, %02x\n", i, hex_cmd[i]);
    }
}

void ZLAC::print_rcv_hex() const{
    // print
    for (int i = 0; i < 8; i++){
        printf("rec: %d, %02x\n", i, receive_hex[i]);
    }
}

int32_t ZLAC::readInt32FromArray(const unsigned char* target_array, int startIndex) {
    // read int32 val from target_array
    int32_t value = 
        (target_array[startIndex] << 24) |
        (target_array[startIndex + 1] << 16) |
        (target_array[startIndex + 2] << 8) |
        (target_array[startIndex + 3]);
    // convert neg repression
    if (value & 0x80000000) {
        value = -(0xFFFFFFFF - value + 1);
    }
    return value;
}

int16_t ZLAC::readInt16FromArray(const unsigned char* target_array, int startIndex) {
    int16_t value = 
        (target_array[startIndex] << 8) |
        (target_array[startIndex + 1]);
    if (value & 0x8000) { // Check if the value is negative
        value = -(0xFFFF - value + 1); // Convert using 2's complement
    }
    return value;
}


/*
uint16_t ZLAC::get_error(){
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = 0x03;
    hex_cmd[1] = 0x03;
    hex_cmd[2] = 0x20;
    hex_cmd[3] = 0x00;
    hex_cmd[4] = 0x01;
    calculate_crc();
    _serial.write(hex_cmd, 7);
    printf("serial.write :");
    for (int i = 0; i < 7; ++i)
    {
        printf("%02X ", hex_cmd[i]);
    }

    printf("\nresponse:\n");
    if (read_hex(7))
    {
        printf("\nNo response.... Nah\n");
        return 1;
    }
    return 0;
    // return receive_hex[12] + (receive_hex[11] << 8);
}
*/
