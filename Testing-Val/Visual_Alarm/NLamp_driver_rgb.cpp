#include <iostream>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <stdlib.h>

/*****************************************************/
/*****************************************************
Setup details :
VisionSoM-STM32MP157A-carrier board + SoM board.
1 x Notification Lamp Board
5 x Jumper Wires
No DTS changes required , driven by libgpiod interface.

Platform:
OpenSTLinux Package : 3.0.0.
Kernel Version      : 5.10.10

Hardware connections:

Led   |    Port    | Pin Number

Red        PB6       38
Green      PB1       31
Blue       PC3       35

*****************************************************/
/*****************************************************/

// Enable / Disable Debug Prints
#define DEBUG 1


// Global variables for synchronization
uint8_t             g_break     = 0;
uint8_t             g_hp        = 0;
uint8_t             g_lp        = 0;
uint8_t             g_mp        = 0;

// Mutex locks for various variables
std::mutex data_lock;
std::mutex break_lock;

void disable_all_channels() {

    // Disable all the channel , by pulling down gpio low.

    system("gpioset gpiochip1 6=0 "); // PB6 - Pin 38
    system("gpioset gpiochip1 1=0 "); // PB1 - Pin 31
    system("gpioset gpiochip2 3=0 "); // PC3 - Pin 35

}

// Enable High Priority Alarm
void enable_HP() {

#if DEBUG
    printf("Enable HP Notification..!!\n");
#endif

    while(!g_break) {

#if DEBUG
        printf("gbreak from HP= %d\n", g_break);
#endif
        system("gpioset gpiochip1 6=1 "); //PB6 - High - Red
        usleep(500000);
        system("gpioset gpiochip1 6=0 "); //PB6 - Low -Red
        usleep(500000);

    }

}

// Enable Medium Priority Alarm
void enable_MP() {

#if DEBUG
    printf("Enable MP Notification..!!\n");
#endif

    while(!g_break) {

#if DEBUG
        printf("gbreak from MP= %d\n", g_break);
#endif
        system("gpioset gpiochip1 6=1 && gpioset gpiochip1 1=1 "); //PB6 && PB1 - High
        sleep(2);
        system("gpioset gpiochip1 6=0 && gpioset gpiochip1 1=0 "); //PB6 && PB1 - Low
        sleep(3);

    }
}

// Enable Low Priority Alarm
void enable_LP() {

#if DEBUG
        printf("Enable LP Notification..!!\n");
#endif

    while(!g_break) {

#if DEBUG
        printf("gbreak from LP= %d\n", g_break);
#endif
        system("gpioset gpiochip1 6=1 && gpioset gpiochip1 1=1 "); //PB6 && PB1 - High
        sleep(1);
        system("gpioset gpiochip1 6=0 && gpioset gpiochip1 1=0 "); //PB6 && PB1 - Low
        sleep(7);

    }

}

// Disable all notifications
void stop() {

#if DEBUG
        printf("Disabling the lamp...!!!\n");
        printf("gbreak from stop= %d\n", g_break);
#endif
        disable_all_channels();
        break_lock.lock();g_break=0;break_lock.unlock();

}

// Notification Lamp thread Callback
void lamp()
{
    while(1) {

        if(g_hp && !g_break) {

            data_lock.lock();g_hp=0;data_lock.unlock();
            disable_all_channels();
            enable_HP();
        }


        else if(g_mp && !g_break) {

            data_lock.lock();g_hp=0;data_lock.unlock();
            disable_all_channels();
            enable_MP();
        }


        else if(g_lp && !g_break) {

            data_lock.lock();g_hp=0;data_lock.unlock();
            disable_all_channels();
            enable_LP();
        }

        else if (g_break) {

            stop();
        }
    }

}

int main() {

		// Spawn notification Lamp thread.
        std::thread N_Lamp (lamp);

        while (1) {

            std::cout << "Enter value to see the alarm in main thread : 1- HP , 2-MP, 3-LP, 4-STP, 5-Exit: " << std::endl;
            std::cin >> data;

            switch(data) {

                case 1: // Setting flag for firing HP alarm by N_Lamp thread
                        data_lock.lock();g_hp=1;data_lock.unlock();
                        break_lock.lock(); g_break=0; break_lock.unlock();
                        break;
                case 2: // Setting flag for firing MP alarm by N_Lamp thread
                        data_lock.lock();g_mp=1;data_lock.unlock();
                        break_lock.lock(); g_break=0; break_lock.unlock();
                        break;
                case 3: // Setting flag for firing LP alarm by N_Lamp thread
                        data_lock.lock();g_lp=1;data_lock.unlock();
                        break_lock.lock(); g_break=0; break_lock.unlock();
                        break;
                case 4: // Setting flag for stopping alarm by N_Lamp thread
                        break_lock.lock(); g_break=1; break_lock.unlock();
                        break;
                case 5: // Exit and Cleanup
                        N_Lamp.join();
                        exit(0);
                        break;
                default: break;

            }

        }

        return 0;

}