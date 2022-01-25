#include "serial_config.hpp"
#include "at_commands.hpp"

QModem Channel;

int main() {

    int openstatus = Channel.openPort();

    if(openstatus == SUCCESS) {

        for(int i=0;i<Modem_Init.size();i++) {

            int response = Channel.sendCommand(Modem_Init[i]);

            std::cout << "[QModem] : Command Sent Status : " << response << std::endl;

            if(response == SUCCESS) {

                int ops = Channel.readPort();

                if(ops == SUCCESS) {

                    std::cout << "[QModem] Proceed to next command : "  << rbuf << std::endl;

                    while(strstr(rbuf, "ERROR")) {

                        std::cout << "[QModem] Error is present" << std::endl;
                        response = Channel.sendCommand(Modem_Init[2]);
                        ops = Channel.readPort();

                    }

                    std::cout << "[QModem] Command Response Good" << std::endl;

                }
            }

        }

    Channel.closePort();

    }
    return 0;
}