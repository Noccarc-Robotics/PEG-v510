#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <chrono>
#include "CppLinuxSerial/SerialPort.hpp"

constexpr uint8_t HEADER_ = 0x23;
constexpr uint8_t EOF_    = 0x40;

using namespace mn::CppLinuxSerial;
using std::chrono::operator""ms;
std::mutex serial_mutex_r;
std::mutex serial_mutex_w;

std::string readData;


int32_t CalculateData(const std::string& readData) {
	
	uint32_t data = 0;
	
	if((static_cast<uint8_t>(readData[6]))!=0xF5) {
	
		
		data |= (static_cast<uint8_t>(readData[6]) << 24);
	}
	
	if ((static_cast<uint8_t>(readData[7]))!=0xF5) {
		
		data |= (static_cast<uint8_t>(readData[7]) << 16);
	}
		
	if (static_cast<uint8_t>(readData[8])!=0xF5) {
			
			
		data |= (static_cast<uint8_t>(readData[8]) << 8);
		
	}
	
	data |= static_cast<uint8_t>(readData[9]);
	data = data & ~(0xFFFFF000);
	
	return data;
}

// Callback function for the Read Thread.
void SerialRead_MB() {

    serial_mutex_r.lock();
    // Create serial port object and open serial port
    SerialPort serialPort("/dev/ttyACM7", BaudRate::B_9600);

    serialPort.SetTimeout(-1); // Block when reading until any data is received
    serialPort.Open();

    // Write some ASCII datae
    // serialPort0.Write("Hello");

    // Read some data back
    while(1) {

        //std::string readData;
        serialPort.Read(readData);

	//===================================== Header =========================================//
	
	if(readData[0]==HEADER_ && readData[10]==EOF_) {
		
		std::cout << "SOF: " << std::hex << static_cast<int8_t>(readData[0]) << 
		" EOF: " << std::hex << static_cast<int8_t>(readData[10]) << std::endl;
		
		//================================ ID ============================================//	
			
		if(readData[1]==0x47) {
			
			uint8_t modeID = static_cast<uint8_t>(readData[3]);
			
			switch(static_cast<uint8_t>(modeID)) {
			
				case 0x31: std::cout << "Mode: PC-SIMV" << std::endl;break;
				case 0x32: std::cout << "Mode: PC-CMV" << std::endl;break;
				case 0x33: std::cout << "Mode: PC-AC" << std::endl;break;
				default:   std::cout << "Invalid Mode-ID" << std::endl;break;
			}
			
				
		} else {
		
			std::cout << "Invalid Mode packet format." << std::endl;
		
		}
		
		//================== Param ID scan and value extraction========================
		uint8_t paramid = static_cast<uint8_t>(readData[5]);
		
		switch(paramid) {
		
			case 0x21: std::cout << "Param: Volume= " << std::setbase(10) << CalculateData(readData) << " ml" << std::endl;break;
			case 0x22: std::cout << "Param: Pressure= " << std::setbase(10) << CalculateData(readData) << " Bar" << std::endl;break;
			case 0x23: std::cout << "Param: Flow= " << std::setbase(10) << CalculateData(readData) << " lpm" << std::endl;break;
			default:   break;
		}
	
	  } 
	  
	  //std::this_thread::sleep_for(150ms);

    }

	// Close the serial port
	serialPort.Close();
	serial_mutex_w.unlock();
	//std::this_thread::yield();


}


void SerialWrite_MB() {

					
	serial_mutex_w.lock();
    	// Create serial port object and open serial port
    	SerialPort serialPort0("/dev/ttyACM8", BaudRate::B_9600);

    	serialPort0.SetTimeout(-1); // Block when reading until any data is received
    	serialPort0.Open();

	while(1) {
	
		//std::cout << "============ Hello from write thread =============" << std::endl;
		//std::this_thread::sleep_for(150ms);
		//std::this_thread::yield();;
    		serialPort0.Write(readData);
    		//std::this_thread::sleep_for(150ms);
		
	}
	
	serialPort0.Close();
	serial_mutex_w.unlock();

}



int main(int argc, char **argv)
{
	std::thread UART_Read(SerialRead_MB);
	std::thread UART_Write(SerialWrite_MB);
	
	while(1) {
	
		UART_Read.join();
		UART_Write.join();
	}
	
}
