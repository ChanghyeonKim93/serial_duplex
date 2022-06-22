#include <iostream>
#include "serial_comm_linux.h"

int main(void) {
	std::string serial_name = "/dev/ttyACM0";
	int baud_rate = 921600;

	try{
		std::shared_ptr<SerialCommunicatorLinux> serial_com;
		serial_com = std::make_shared<SerialCommunicatorLinux>(serial_name, baud_rate);
	}
	catch (std::exception& e){
		std::cout << "error : " << e.what() << std::endl;
	}
	return 0;
}