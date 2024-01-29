#include "zlac8015d.h"

int main(){
    ZLAC mot;
    struct MOT_DATA motorstat;
    mot.init("/dev/ttyUSB0", 115200, 0x01, true);
    
    mot.set_double_rpm(0, 0);
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_position();
    printf("\nL:%lf|R:%lf\n", motorstat.rpm_L, motorstat.rpm_R);
    printf("\nLp:%d|Rp:%d\n", motorstat.encoder_L, motorstat.encoder_R);
        mot.sleep(1000);

    printf("===set_rpm===\n");
    mot.set_double_rpm(30, -30);   
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_position();
    printf("\nL:%lf|R:%lf\n", motorstat.rpm_L, motorstat.rpm_R);
    printf("\nLp:%d|Rp:%d\n", motorstat.encoder_L, motorstat.encoder_R);
        mot.sleep(1000);

    printf("===set_rpm===\n");
    mot.set_double_rpm(80, -80);   
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_position();
    printf("\nL:%lf|R:%lf\n", motorstat.rpm_L, motorstat.rpm_R);
    printf("\nLp:%d|Rp:%d\n", motorstat.encoder_L, motorstat.encoder_R);
        mot.sleep(1000);

    mot.set_double_rpm(0, 0); 
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_rpm();
    motorstat = mot.get_position();
    printf("\nL:%lf|R:%lf\n", motorstat.rpm_L, motorstat.rpm_R);
    printf("\nLp:%d|Rp:%d\n", motorstat.encoder_L, motorstat.encoder_R);

    printf("===disable===\n");
    mot.terminate();
    // motorL.disable();
}
