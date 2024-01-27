#include "zlac8015d.h"

int main(){
    ZLAC mot;
    struct MOT_DATA motorstat;
    printf("===begin===\n");
    mot.begin("/dev/ttyUSB0", 115200, 0x01);
    printf("===set_vel_mode===\n");
    mot.set_vel_mode();
    printf("===enable===\n");
    mot.enable();

    mot.set_double_rpm(0, 0);
    //I dunno why I have to use this 3 times to get normal RPM
    //maybe need some delay? message sending timing?// anyway it works 〜(￣▽￣〜)
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
    mot.disable();
    // motorL.disable();
}
