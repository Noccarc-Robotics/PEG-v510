#include <vector>


//Init command list
static const char cmd1[]  = "AT\r\n";
static const char cmd2[]  = "ATI\r\n";
static const char cmd3[]  = "ATI\r\n";
static const char cmd4[]  = "ATV1\r\n";
static const char cmd5[]  = "ATE1\r\n";
static const char cmd6[]  = "AT+CMEE=2\r\n";
static const char cmd7[]  = "ATI\r\n";
static const char cmd8[]  = "AT+GSN\r\n";
static const char cmd9[] = "AT+CPIN?\r\n";
static const char cmd10[] = "AT+CIMI\r\n";
static const char cmd11[] = "AT+QCCID\r\n";
static const char cmd12[] = "AT+CSQ\r\n";;
static const char cmd13[] = "AT+COPS?\r\n";

std::vector <const char *> Modem_Init { cmd1, cmd2, cmd3, cmd4, cmd5, cmd6,
                                        cmd7, cmd8, cmd9, cmd10, cmd11, cmd12,
                                        cmd13
                                      };