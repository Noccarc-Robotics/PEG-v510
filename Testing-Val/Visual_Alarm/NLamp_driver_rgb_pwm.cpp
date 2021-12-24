/********************************************************************
** @file     nlamp_us.cpp
** @brief    Notification Lamp Interface
** @author   Copyright (C) 2021  Noccarc Robotics
** @author   Nipun Pal <nipun.k@noccarc.com>
** @version  v1.03
** @modified 24/12/21  Added Nlamp support as per IEC standard
**
********************************************************************/
#include "nlamp.hpp"

/*****************************************************/
/*****************************************************
Setup details :
Noccarc Carrier + Noccarc SoM board.
1 x Notification Lamp Board
No DTS changes required , driven by sysfs pwm interface.

Platform:
OpenSTLinux Package : 3.0.0.
Kernel Version      : 5.10.10
Compile command     : ${CXX} nlamp.cpp -o nlamp

Hardware connections:

Led   |    Timer Channel

Red        pwmchip4(ch0)
Green      pwmchip8(ch2)
Blue       pwmchip0(ch0)

*****************************************************/
/*****************************************************/

void disable_all_channels() {

    // Disable all the channels

    if (hp) {

        system("echo 0 > /sys/class/pwm/pwmchip4/pwm0/enable "); // Red
        system("echo 0 > /sys/class/pwm/pwmchip4/unexport ");
        hp=FALSE;

    } else if(mp) {

        system("echo 0 > /sys/class/pwm/pwmchip8/pwm2/enable "); // Green
        system("echo 2 > /sys/class/pwm/pwmchip8/unexport ");

        system("echo 0 > /sys/class/pwm/pwmchip4/pwm0/enable "); // Red
        system("echo 0 > /sys/class/pwm/pwmchip4/unexport ");

        mp=FALSE;

    } else if(lp) {

        system("echo 0 > /sys/class/pwm/pwmchip8/pwm2/enable "); // Green
        system("echo 2 > /sys/class/pwm/pwmchip8/unexport ");

        system("echo 0 > /sys/class/pwm/pwmchip4/pwm0/enable "); // Red
        system("echo 0 > /sys/class/pwm/pwmchip4/unexport ");

        lp=FALSE;
    }

}

// Enable High Priority Alarm
void enable_HP() {

        int i=0,j=0;

        // Enable red channel
        system("echo 0 > /sys/class/pwm/pwmchip4/export");
        system("echo 0 > /sys/class/pwm/pwmchip4/pwm0/duty_cycle");
        system("echo 1000000 > /sys/class/pwm/pwmchip4/pwm0/period");

        system("echo 1 > /sys/class/pwm/pwmchip4/pwm0/enable");

        for(j=0;j<HP_PULSE_COUNT;j++) {

            for(i=0;i<=MAX_PULSE_AMPLITUDE;i=i+HP_TR_STEP_SIZE) {

                snprintf(buf, sizeof(buf), "echo %d > /sys/class/pwm/pwmchip4/pwm0/duty_cycle",i);
                system(buf);
                memset(buf, 0, sizeof(buf));

            }

            usleep(HP_PULSE_ON_TIME);

            for(i=MAX_PULSE_AMPLITUDE;i>=0;i=i-HP_TR_STEP_SIZE) {

                snprintf(buf, sizeof(buf), "echo %d > /sys/class/pwm/pwmchip4/pwm0/duty_cycle",i);
                system(buf);
                memset(buf, 0, sizeof(buf));

            }

            usleep(HP_PULSE_OFF_TIME);
        }

        system("echo 0 > /sys/class/pwm/pwmchip4/pwm0/enable");

}


// Enable Medium Priority Alarm
// cyan = red + green
void enable_MP() {


        int i=0,j=0;

        // Enable red + green channel
        system("echo 0 > /sys/class/pwm/pwmchip4/export");
        system("echo 0 > /sys/class/pwm/pwmchip4/pwm0/duty_cycle");
        system("echo 1000000 > /sys/class/pwm/pwmchip4/pwm0/period");

        system("echo 2 > /sys/class/pwm/pwmchip8/export");
        system("echo 0 > /sys/class/pwm/pwmchip8/pwm2/duty_cycle");
        system("echo 1000000 > /sys/class/pwm/pwmchip8/pwm2/period");

        system("echo 1 > /sys/class/pwm/pwmchip4/pwm0/enable");
        system("echo 1 > /sys/class/pwm/pwmchip8/pwm2/enable");

        for(j=0;j<MP_PULSE_COUNT;j++) {

            for(i=0;i<=MAX_PULSE_AMPLITUDE;i=i+MP_TR_STEP_SIZE) {

                snprintf(buf, sizeof(buf), "echo %d > /sys/class/pwm/pwmchip4/pwm0/duty_cycle; echo %d > /sys/class/pwm/pwmchip8/pwm2/duty_cycle",i,i);
                system(buf);
                memset(buf, 0, sizeof(buf));

            }

            usleep(MP_PULSE_ON_TIME);

            for(i=MAX_PULSE_AMPLITUDE;i>=0;i=i-MP_TR_STEP_SIZE) {

                snprintf(buf, sizeof(buf), "echo %d > /sys/class/pwm/pwmchip4/pwm0/duty_cycle; echo %d > /sys/class/pwm/pwmchip8/pwm2/duty_cycle",i,i);
                system(buf);
                memset(buf, 0, sizeof(buf));

            }

            usleep(MP_PULSE_OFF_TIME);
        }

        system("echo 0 > /sys/class/pwm/pwmchip4/pwm0/enable");
        system("echo 0 > /sys/class/pwm/pwmchip8/pwm2/enable");

}


// Enable Low Priority Alarm
// cyan = red + green
void enable_LP() {

        int i=0;

        // Enable red + green channel
        system("echo 0 > /sys/class/pwm/pwmchip4/export");
        system("echo 0 > /sys/class/pwm/pwmchip4/pwm0/duty_cycle");
        system("echo 1000000 > /sys/class/pwm/pwmchip4/pwm0/period");


        system("echo 2 > /sys/class/pwm/pwmchip8/export");
        system("echo 0 > /sys/class/pwm/pwmchip8/pwm2/duty_cycle");
        system("echo 1000000 > /sys/class/pwm/pwmchip8/pwm2/period");

        system("echo 1 > /sys/class/pwm/pwmchip4/pwm0/enable");
        system("echo 1 > /sys/class/pwm/pwmchip8/pwm2/enable");


        for(i=0;i<=MAX_PULSE_AMPLITUDE;i=i+LP_TR_STEP_SIZE) {

            snprintf(buf, sizeof(buf), "echo %d > /sys/class/pwm/pwmchip4/pwm0/duty_cycle; echo %d > /sys/class/pwm/pwmchip8/pwm2/duty_cycle",i,i);
            system(buf);
            memset(buf, 0, sizeof(buf));

        }

        usleep(LP_PULSE_ON_TIME);

        for(i=MAX_PULSE_AMPLITUDE;i>=0;i=i-LP_TF_STEP_SIZE) {

            snprintf(buf, sizeof(buf), "echo %d > /sys/class/pwm/pwmchip4/pwm0/duty_cycle; echo %d > /sys/class/pwm/pwmchip8/pwm2/duty_cycle",i,i);
            system(buf);
            memset(buf, 0, sizeof(buf));

        }

        system("echo 0 > /sys/class/pwm/pwmchip4/pwm0/enable");
        system("echo 0 > /sys/class/pwm/pwmchip8/pwm2/enable");

}


int main() {

        uint8_t data=0;

        while (1) {

         std::cout << "Enter mode\n1.HP\n2.MP\n3.LP\n4.Disable\n5.Exit" << std::endl;
         std::cin >> data;
         std::cout << "You chose : " << data << std::endl;

         switch(data) {

            case '1': disable_all_channels();
                      enable_HP(); hp = TRUE;
                      break;
            case '2': disable_all_channels();
                      enable_MP(); mp = TRUE;
                      break;
            case '3': disable_all_channels();
                      enable_LP(); lp = TRUE;
                      break;
            case '4': lp=mp=hp=TRUE;
                      std::cout << "In case 4" << std::endl;disable_all_channels();
                      break;
            case '5': exit(0);
                      break;
            default:
                      std::cout << "Invalid Input" << std::endl;
                      break;

         }

    }

    return 0;

}
