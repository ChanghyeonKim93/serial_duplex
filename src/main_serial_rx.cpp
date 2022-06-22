#include <iostream>
#include "serial_comm.h"

int main(void) {
	std::string serial_name = "/dev/ttyACM0";
	int message_length = 19;

	try{
		std::shared_ptr<SerialCommunicator> serial_com;
		serial_com = std::make_shared<SerialCommunicator>(serial_name, message_length);
		serial_com->runThread();
	}
	catch (std::exception& e){
		std::cout << "error : " << e.what() << std::endl;
	}
	return 0;
}