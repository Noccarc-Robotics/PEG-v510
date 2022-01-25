#include<iostream>
#include<string>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <cerrno> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <cstring>


//char uni_buff[] = "Hello-God\n";

enum Result : uint8_t {

    SUCCESS = 1,
    FAILURE = 0
};


/**********************************************************************************************************************/
//
//                                    AT Commands BUffer Init list
//
/*********************************************************************************************************************/


//Response Buffer
static char rbuf[255];


class QModem {

    public:
        QModem();
        ~QModem();

        int openPort();
        int sendCommand(const char *command);
        int readPort();
        int closePort();
        int set_interface_attribs (int fd, int speed, int parity);
        void set_blocking (int fd, int should_block);

    private:
        std::string pathSerial;

        //filedescriptors
        int serial_port;
        struct termios tty;
};