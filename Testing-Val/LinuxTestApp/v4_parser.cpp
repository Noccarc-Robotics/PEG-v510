#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include "CppLinuxSerial/SerialPort.hpp"

constexpr uint8_t HEADER_ = 0x23;
constexpr uint8_t EOF_    = 0x40;
uint8_t g_sendbuff_tmlt[11]={0x23,0x47,0xF5,0x00,0xF5,0x00,0xF5,0x00,0x00,0x00,0x40};


using namespace mn::CppLinuxSerial;
using std::chrono::operator""ms;
std::mutex serial_mutex_r;
std::mutex serial_mutex_w;
std::mutex ui_data;
std::string readData;


struct ui_data_dummy {

	uint8_t I_data;
	uint8_t E_data;
	uint8_t Tapnea_data;
	uint8_t PETime_data;
	uint8_t Pplat_data;
	uint8_t Vt_data;
	uint8_t RR_data;
	uint8_t FiO2;
	uint8_t FTrigg_data;
	uint8_t PTrigg_data;
	uint8_t EInspP_data;
	uint8_t PS_above_PEEP_data;
	uint8_t Trigg_type_data;
	uint8_t Phigh_data;
	uint8_t Vt_Low_data;
	uint8_t Vt_High_data;
	uint8_t RR_low_data;
	uint8_t RR_high_data;
	uint8_t PEEP_low_data;
	uint8_t PEEP_high_data;
	uint8_t PExptime_data;
	uint8_t deltaPEEP_data;
	uint8_t Thigh_data;
	uint8_t Tpeep_data;
	uint8_t TinspRise_data;
	uint8_t Insp_Cycle_off_data;
	uint8_t Ps_abv_Phigh_data;
	uint8_t flow_rate_data;
	
}; 


ui_data_dummy ui_data_1;


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


void Packetize_and_Send(uint32_t param, uint8_t id, SerialPort& serialPort0) {

	uint8_t packet[11]={0x23,0x47,0xF5,0x00,0xF5,0x00,0xF5,0x00,0x00,0x00,0x40};
	
	packet[5]=id;
	
	if((param&(0x00FF0000))==0) {
	
		packet[7]=0xF5;
		
	} else {
	
		packet[7]= static_cast<uint8_t>((param&(0x00FF0000) >> 16));
	
	}
	
	packet[8]=static_cast<uint8_t>((param&(0x0000FF00) >> 8));
	packet[9]=static_cast<uint8_t>((param&(0x000000FF)));
	
	std::ostringstream convert;
	
	for (int a = 0; a < 11; a++) {
	
		convert << packet[a];
	}

	std::string packet_string = convert.str();
	
	serialPort0.Write(packet_string);	
	
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
				case 0x90: break;// Start_Ventilation()
				case 0x91: break;// Stand_by()
				case 0x92: break;// Pause_Mode();
				case 0x93: break;// Insp_Pause();
				case 0x94: break;// Exp_Pause();
				case 0x95: break;// Respond_to_ACK();
				case 0x96: break; // Respond_to_NACK();
				default:   std::cout << "Invalid Mode-ID" << std::endl;break;
			}
			
				
		} else {
		
			std::cout << "Invalid Mode packet format." << std::endl;
		
		}
		
		//================== Param ID scan and value extraction========================
		uint8_t paramid = static_cast<uint8_t>(readData[5]);
		//std::cout << "Param ID: " << paramid << std::endl;
		
		switch(paramid) {
		
			case 0x21: std::cout << "Param: Volume= " << std::setbase(10) << CalculateData(readData) << " ml" << std::endl;break;
			case 0x22: std::cout << "Param: Pressure= " << std::setbase(10) << CalculateData(readData) << " Bar" << std::endl;break;
			case 0x23: std::cout << "Param: Pressure= " << std::setbase(10) << CalculateData(readData) << " lpm" << std::endl;break;
			case 0x24: std::cout << "Param: FiO2= " << std::setbase(10) << CalculateData(readData) << " %" << std::endl;break;
			case 0x25: std::cout << "Param: RR= " << std::setbase(10) << CalculateData(readData) << " BPM" << std::endl;break;
			case 0x26: std::cout << "Param: I= " << std::setbase(10) << CalculateData(readData) << " " << std::endl;break;
			case 0x27: std::cout << "Param: E= " << std::setbase(10) << CalculateData(readData) << " " << std::endl;break;
			case 0x28: std::cout << "Param: PEEP= " << std::setbase(10) << CalculateData(readData) << " Bar" << std::endl;break;
			case 0x29: std::cout << "Param: Flow Rate= " << std::setbase(10) << CalculateData(readData) << " lpm" << std::endl;break;
			case 0x2A: std::cout << "Param: Pplat= " << std::setbase(10) << CalculateData(readData) << " Bar" << std::endl;break;
			case 0x2B: std::cout << "Param: Vte= " << std::setbase(10) << CalculateData(readData) << " ml" << std::endl;break;
			case 0x2C: std::cout << "Param: Vti= " << std::setbase(10) << CalculateData(readData) << " ml" << std::endl;break;
			case 0x2D: std::cout << "Param: MVe= " << std::setbase(10) << CalculateData(readData) << " ml" << std::endl;break;
			case 0x2E: std::cout << "Param: MVi= " << std::setbase(10) << CalculateData(readData) << " ml" << std::endl;break;
			case 0x2F: std::cout << "Param: Ti= " << std::setbase(10) << CalculateData(readData) << " s" << std::endl;break;
			case 0x30: std::cout << "Param: Te= " << std::setbase(10) << CalculateData(readData) << " s" << std::endl;break;
			case 0x31: std::cout << "Param: Cstat= " << std::setbase(10) << CalculateData(readData) << " ml/cm H2O" << std::endl;break;
			case 0x32: std::cout << "Param: Cdyn= " << std::setbase(10) << CalculateData(readData) << " ml/cm H2O" << std::endl;break;
			case 0x33: std::cout << "Param: Leak Volume= " << std::setbase(10) << CalculateData(readData) << " lpm" << std::endl;break;
			case 0x34: std::cout << "Param: Resistance= " << std::setbase(10) << CalculateData(readData) << " cm-H2O lps" << std::endl;break;
			case 0x35: std::cout << "Param: Alarms= " << std::setbase(10) << CalculateData(readData) << " " << std::endl;break;
			case 0x36: std::cout << "Param: Battery= " << std::setbase(10) << CalculateData(readData) << " %" << std::endl;break;
			default: break;
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

		//memcpy (dest_struct, source_struct, sizeof (*dest_struct)); <----------------------------memcpy ui strcut to local struct
		//std::cout << "============ Hello from write thread =============" << std::endl;
		//std::this_thread::sleep_for(150ms);
		//std::this_thread::yield();;
		
		/*sendbuff[3]=0x31;
		sendbuff[5]=0x21;
		sendbuff[7]=0xF5;
		sendbuff[8]=0x04;
		sendbuff[9]=0xB0;*/
		
		Packetize_and_Send(ui_data_1.I_data,0x51,serialPort0);
		Packetize_and_Send(ui_data_1.E_data,0x52,serialPort0);
		Packetize_and_Send(ui_data_1.Tapnea_data,0x53,serialPort0);
		Packetize_and_Send(ui_data_1.PETime_data,0x54,serialPort0);
		Packetize_and_Send(ui_data_1.Pplat_data,0x56,serialPort0);
		Packetize_and_Send(ui_data_1.Vt_data,0x57,serialPort0);
		Packetize_and_Send(ui_data_1.RR_data,0x58,serialPort0);
		Packetize_and_Send(ui_data_1.FiO2,0x59,serialPort0);
		Packetize_and_Send(ui_data_1.FTrigg_data,0x5A,serialPort0);
		Packetize_and_Send(ui_data_1.PTrigg_data,0x5B,serialPort0);
		Packetize_and_Send(ui_data_1.EInspP_data,0x5C,serialPort0);
		Packetize_and_Send(ui_data_1.PS_above_PEEP_data,0x5D,serialPort0);
		Packetize_and_Send(ui_data_1.Trigg_type_data,0x5E,serialPort0);
		Packetize_and_Send(ui_data_1.Phigh_data,0x5F,serialPort0);
		Packetize_and_Send(ui_data_1.Vt_Low_data,0x60,serialPort0);
		Packetize_and_Send(ui_data_1.Vt_High_data,0x61,serialPort0);
		Packetize_and_Send(ui_data_1.RR_low_data,0x62,serialPort0);
		Packetize_and_Send(ui_data_1.RR_high_data,0x63,serialPort0);
		Packetize_and_Send(ui_data_1.PEEP_low_data,0x64,serialPort0);
		Packetize_and_Send(ui_data_1.PEEP_high_data,0x65,serialPort0);
		Packetize_and_Send(ui_data_1.PExptime_data,0x66,serialPort0);
		Packetize_and_Send(ui_data_1.deltaPEEP_data,0x67,serialPort0);
		Packetize_and_Send(ui_data_1.Thigh_data,0x68,serialPort0);
		Packetize_and_Send(ui_data_1.Tpeep_data,0x69,serialPort0);
		Packetize_and_Send(ui_data_1.TinspRise_data,0x6A,serialPort0);
		Packetize_and_Send(ui_data_1.Insp_Cycle_off_data,0x6B,serialPort0);
		Packetize_and_Send(ui_data_1.Ps_abv_Phigh_data,0x6C,serialPort0);
		Packetize_and_Send(ui_data_1.flow_rate_data,0x6D,serialPort0);
		
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
