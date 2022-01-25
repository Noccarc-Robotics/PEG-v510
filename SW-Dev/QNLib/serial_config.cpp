#include<iostream>
#include "serial_config.hpp"

QModem::QModem()
{
    serial_port = 0;
    pathSerial = "/dev/ttyUSB2";
}

QModem::~QModem()
{
}

int QModem::openPort()
{
    std::cout << "[QModem]: OpenPort() started with the following parameter : pathSerial = " << pathSerial << ", baudrate = 115200" << std::endl;

    serial_port = open(pathSerial.c_str(), O_RDWR| O_NOCTTY | O_SYNC);

    // Check for errors
    if (serial_port < 0) {

       std::cout << "[QModem] OpenPort() : [ERROR]: " << std::strerror(errno) << std::endl;
       return FAILURE;

    } else {

        std::cout << "[QModem] OpenPort() : [SUCCESS] Port opened successfully." << std::endl;
        set_interface_attribs (serial_port, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking (serial_port, 0);      // set no blocking

        return SUCCESS;

    }

}

int QModem::sendCommand(const char *command)
{

    std::cout << "[QModem][Write] Command to be sent:  = " << command << std::endl;

    int bytesWritten = write (serial_port, command, sizeof(command));           // send 7 character greeting
    usleep ((sizeof(command) + 25) * 100); // sleep enough to transmit the 7 plus
                                        // receive 25:  approx 100 uS per char transmit

    //SENDING FINISHED
    std::cout << "[QModem][Write] Finished Successfully" << std::endl;

    if(bytesWritten > 0)
        return SUCCESS;
    else
        return FAILURE;
}

int QModem::readPort(){

    memset(rbuf, 0, sizeof(rbuf));

    std::cout << "[QModem][Read] started reading port = " << std::endl;

    sleep(0.5);
    int bytesread = read(serial_port, &rbuf, sizeof(rbuf));

    std::cout << "[QModem][Read] Finished : Number of bytes read = " << bytesread << std::endl;
    std::cout << "Response : " << rbuf << std::endl;

    if(bytesread > 0)
        return SUCCESS;
    else
        return FAILURE;

}


int QModem::closePort()
{
    std::cout << "[QModem][Close] Closing Port..." << std::endl;
    close(serial_port);
    return SUCCESS;
}

int
QModem::set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                //error_message ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void
QModem::set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
               // error_message ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0) {

            std::cout << "Failed to set" << std::endl;
        }
              //  error_message ("error %d setting term attributes", errno);
}