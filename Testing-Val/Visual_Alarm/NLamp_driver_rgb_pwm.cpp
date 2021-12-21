#include <iostream>
#include <unistd.h>
#include <stdlib.h>

/*****************************************************/
/*****************************************************
Setup details :
Noccarc Carrier + Noccarc SoM board.
1 x Notification Lamp Board
No DTS changes required , driven by libgpiod interface.

Platform:
OpenSTLinux Package : 3.0.0.
Kernel Version      : 5.10.10
Compile command     : ${CXX} nlamp.cpp -o nlamp -pthread

Hardware connections:

Led   |    Timer Channel

Red        pwmchip4(ch0)
Green      pwmchip8(ch2)
Blue       pwmchip0(ch0)

*****************************************************/
/*****************************************************/


// Flags for flow control
bool lp = 0;
bool mp = 0;
bool hp = 0;


void disable_all_channels() {

    // Disable all the channels

    if(hp) {

            system("echo 0 > /sys/class/pwm/pwmchip4/pwm0/enable "); // Red
            system("echo 0 > /sys/class/pwm/pwmchip4/unexport ");
            hp=0;

    } else if (mp) {

            system("echo 0 > /sys/class/pwm/pwmchip8/pwm2/enable "); // Green
            system("echo 2 > /sys/class/pwm/pwmchip8/unexport ");

            system("echo 0 > /sys/class/pwm/pwmchip4/pwm0/enable "); // Red
            system("echo 0 > /sys/class/pwm/pwmchip4/unexport ");
            mp=0;

    } else if (lp) {

            system("echo 0 > /sys/class/pwm/pwmchip0/pwm0/enable "); // Blue
            system("echo 0 > /sys/class/pwm/pwmchip0/unexport ");
            lp=0;

    }

}

// Enable High Priority Alarm
void enable_HP() {

        system("echo 0 > /sys/class/pwm/pwmchip4/export");
        system("echo 100000000 > /sys/class/pwm/pwmchip4/pwm0/period");
        system("echo 60000000 > /sys/class/pwm/pwmchip4/pwm0/duty_cycle");
        system("echo 1 > /sys/class/pwm/pwmchip4/pwm0/enable");

}

// Enable Medium Priority Alarm
void enable_MP() {

        system("echo 0 > /sys/class/pwm/pwmchip4/export");
        system("echo 100000000 > /sys/class/pwm/pwmchip4/pwm0/period");
        system("echo 60000000 > /sys/class/pwm/pwmchip4/pwm0/duty_cycle");

        system("echo 2 > /sys/class/pwm/pwmchip8/export");
        system("echo 100000000 > /sys/class/pwm/pwmchip8/pwm2/period");
        system("echo 60000000 > /sys/class/pwm/pwmchip8/pwm2/duty_cycle");


        system("echo 1 > /sys/class/pwm/pwmchip4/pwm0/enable");
        system("echo 1 > /sys/class/pwm/pwmchip8/pwm2/enable");

}

// Enable Low Priority Alarm
void enable_LP() {

        system("echo 0 > /sys/class/pwm/pwmchip0/export");
        system("echo 100000000 > /sys/class/pwm/pwmchip0/pwm0/period");
        system("echo 60000000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle");
        system("echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable");

}

int main() {

        uint8_t data=0;

        while (1) {

            std::cout << "Enter value :\n1.HP\n2.MP\n3.LP\n4.Disable\n5.Exit" << std::endl;
            std::cin >> data;

            switch(data) {

                case '1':
                            disable_all_channels();enable_HP();hp=1;
                            break;
                case '2':
                            disable_all_channels();enable_MP();mp=1;
                            break;
                case '3':
                            disable_all_channels();enable_LP();lp=1;
                            break;

                case '4':
                            disable_all_channels();
                            break;
                case '5':
                            exit(0);
                            break;
                default: break;

            }

        }

        return 0;

}
