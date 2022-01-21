#include "serial_config.hpp"

QModem Channel;

int main() {

    Channel.openPort();
    Channel.sendPort();
    Channel.closePort();
    return 0;
}