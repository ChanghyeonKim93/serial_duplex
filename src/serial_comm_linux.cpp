#include "serial_comm_linux.h"

SerialCommunicatorLinux::SerialCommunicatorLinux(const std::string& portname, const int& baud_rate){
    portname_ = portname;
    STX_      = '$';
    ETX_      = '%';
    
    // initialize
    seq_recv_  = 0;
    seq_send_  = 0;

    stack_len_ = 0;
    
    // Check whether this baud rate is valid
    if(     baud_rate == 57600)   BAUD_RATE_ = B57600;
    else if(baud_rate == 115200)  BAUD_RATE_ = B115200;
    else if(baud_rate == 230400)  BAUD_RATE_ = B230400;
    else if(baud_rate == 460800)  BAUD_RATE_ = B460800;
    else if(baud_rate == 500000)  BAUD_RATE_ = B500000;
    else if(baud_rate == 576000)  BAUD_RATE_ = B576000;
    else if(baud_rate == 921600)  BAUD_RATE_ = B921600;
    else if(baud_rate == 1000000) BAUD_RATE_ = B1000000;
    else if(baud_rate == 1152000) BAUD_RATE_ = B1152000;
    else if(baud_rate == 1500000) BAUD_RATE_ = B1500000;
    else if(baud_rate == 2000000) BAUD_RATE_ = B2000000;
    else if(baud_rate == 2500000) BAUD_RATE_ = B2500000;
    else if(baud_rate == 3000000) BAUD_RATE_ = B3000000;
    else if(baud_rate == 3500000) BAUD_RATE_ = B3500000;
    else if(baud_rate == 4000000) BAUD_RATE_ = B4000000;
    else std::runtime_error("Unsupported Baudrate...");
    // In 'termios-baud.h',
    // #define  B57600    0010001
    // #define  B115200   0010002
    // #define  B230400   0010003
    // #define  B460800   0010004
    // #define  B500000   0010005
    // #define  B576000   0010006
    // #define  B921600   0010007
    // #define  B1000000  0010010
    // #define  B1152000  0010011
    // #define  B1500000  0010012
    // #define  B2000000  0010013
    // #define  B2500000  0010014
    // #define  B3000000  0010015
    // #define  B3500000  0010016
    // #define  B4000000  0010017
    // #define __MAX_BAUD B4000000

    double bytes_per_second = (double)baud_rate/(10.0);
    double packet_per_second = bytes_per_second/27.0; // 3413. Hz
    double time_per_packet = 1./packet_per_second;

    std::cout << "bytes_per_second: "  << bytes_per_second << std::endl;
    std::cout << "packet_per_second: " << packet_per_second << std::endl;
    std::cout << "time_per_packet: "   << time_per_packet*1000.0 << " [ms]" << std::endl; // 0.29 ms

    // Open serial port
    this->openSerialPort();
    
    // run TX & RX threads
    this->runThreadRX();
    this->runThreadTX();

    printf("[SerialComm] Portname is {%s}, message length: {%d}\n", portname_.c_str(), MSG_LEN_);
};

// deconstructor
SerialCommunicatorLinux::~SerialCommunicatorLinux() {
    if(thread_rx_.joinable()) this->thread_rx_.join();
    if(thread_tx_.joinable()) this->thread_tx_.join();
};

void SerialCommunicatorLinux::runThreadRX(){ 
    this->thread_rx_ = std::thread(&SerialCommunicatorLinux::processRX, this); 
};

void SerialCommunicatorLinux::runThreadTX(){
    this->thread_tx_ = std::thread(&SerialCommunicatorLinux::processTX, this); 
};


void SerialCommunicatorLinux::openSerialPort(){
    printf("Opening the serial port...\n");
    fd_ = open(portname_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ == -1) {
        throw std::runtime_error("Error - no serial port '" + portname_ + "' is opened! Check the serial device.\n");
    }
    else {
        printf("{%s} is opened.\n",portname_.c_str());
    }
};


int SerialCommunicatorLinux::read_withChecksum(uint8_t* msg){
    int len_packet = 0;
    int len_read = read(fd_, buf_, BUF_SIZE);
    for(int i = 0; i < len_read; ++i) {
        if(buf_[i] == ETX_){ // 'ETX', data part : serial_stack_[1] ~ serial_stack_[MSG_LEN_]
            if(serial_stack_[0] == STX_) { // 'STX'
                MSG_LEN_ = (int)serial_stack_[1];
                uint8_t check_sum = stringChecksum(serial_stack_, 2, MSG_LEN_+1);
                if(serial_stack_[MSG_LEN_+2] == check_sum) { // 'Checksum test'                       
                    for(int j = 0; j < MSG_LEN_;++j) *(msg+j) = serial_stack_[j+2];
                    len_packet = MSG_LEN_;
                }
            }
            stack_len_ = 0;
        }
        else serial_stack_[stack_len_++] = buf_[i];
    }
    return len_packet;
};

void SerialCommunicatorLinux::send_withChecksum(const uint8_t* data, int len){
    uint8_t crc = stringChecksum(data, 0, len-1);

    uint8_t len_c = (uint8_t)len;
    write(fd_,&STX_,  1);
    write(fd_,&len_c, 1);
    write(fd_,data,   len);
    write(fd_,&crc,   1);
    write(fd_,&ETX_,  1);
};


void SerialCommunicatorLinux::processRX(){
    // newtio_rx initialization.
    memset(&this->newtio_, 0, sizeof(this->newtio_));
    newtio_.c_cflag     = BAUD_RATE_ | CS8 | CLOCAL | CREAD;
    newtio_.c_oflag     = 0;
    newtio_.c_lflag     = 0;
    newtio_.c_cc[VTIME] = 0;
    newtio_.c_cc[VMIN]  = 1;

    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &newtio_);

    // poll preparation
    poll_events_.fd = fd_;
    poll_events_.events = POLLIN | POLLERR; // if received or if there is error.
    poll_events_.revents = 0;
    
    while(true) { // polling stage. (when some data is received.)
        poll_state_ = poll(                 //poll() call. event check.
            (struct pollfd *)&poll_events_, // event registration
            1,                             // the number of pollfd which we want to check.
            1000);                         // time out [ms]

        if (poll_state_ > 0) {
            if (poll_events_.revents & POLLIN) { 
                // Read serial RX buffer
                uint8_t packet[1024];
                int len_packet = read_withChecksum(packet);
                if(len_packet > 0) {
                    // Variables
                    USHORT_UNION acc[3];
                    USHORT_UNION gyro[3];
                    USHORT_UNION mag[3];
                    USHORT_UNION sec;
                    UINT_UNION   usec;

                    acc[0].bytes_[0] = packet[1];    acc[0].bytes_[1] = packet[0];
                    acc[1].bytes_[0] = packet[3];    acc[1].bytes_[1] = packet[2];
                    acc[2].bytes_[0] = packet[5];    acc[2].bytes_[1] = packet[4];

                    gyro[0].bytes_[0] = packet[1+6]; gyro[0].bytes_[1] = packet[0+6];
                    gyro[1].bytes_[0] = packet[3+6]; gyro[1].bytes_[1] = packet[2+6];
                    gyro[2].bytes_[0] = packet[5+6]; gyro[2].bytes_[1] = packet[4+6];
                    
                    mag[0].bytes_[0] = packet[1+12]; mag[0].bytes_[1] = packet[0+12];
                    mag[1].bytes_[0] = packet[3+12]; mag[1].bytes_[1] = packet[2+12];
                    mag[2].bytes_[0] = packet[5+12]; mag[2].bytes_[1] = packet[4+12];
                    
                    sec.bytes_[0] = packet[18]; sec.bytes_[1] = packet[19];
                    usec.bytes_[0] = packet[20]; usec.bytes_[1] = packet[21];
                    usec.bytes_[2] = packet[22]; usec.bytes_[3] = packet[23];

                    double t_now   = ((double)sec.ushort_ + (double)usec.uint_/1000000.0);
                    ++seq_recv_;

                    std::cout << "seq:" << seq_recv_ <<", msglen:" << MSG_LEN_ <<", time:" 
                    << t_now << " / "  << (short)acc[0].ushort_ <<"," << (short)acc[1].ushort_ << "," << (short)acc[2].ushort_ 
                    << " / " << (short)gyro[0].ushort_ << "," << (short)gyro[1].ushort_ << "," << (short)gyro[2].ushort_
                    << " / " << (short)mag[0].ushort_ << "," << (short)mag[1].ushort_ << "," << (short)mag[2].ushort_ << std::endl;
                }
            }
            if (poll_events_.revents & POLLERR) {// The connection is destructed.
                printf("SerialCommunicatorLinux ERROR - There is communication error. program exit.");
                break;
            }
        }
        else if (poll_state_ == 0) { //timeout.
            printf("SerialCommunicatorLinux - TIMEOUT 1000 ms\n");
        }
        else if(poll_state_ < 0) { // Error occurs.
            //printf("ERROR-critical error occurs. Program is shutdown.\n");
            //return -1;
            continue;
        }
    } // END WHILE
};

void SerialCommunicatorLinux::processTX(){
    seq_send_ = 0;
    while(true){
        ++seq_send_;
        uint8_t data[15]={'a','b','c','d','e','a','b','c','d','e','a','b','c','d','e'};
        send_withChecksum(data, 15);
        std::cout << " TX seq.:" << seq_send_ << std::endl;
        std::this_thread::sleep_for(50ms);
    }
};

uint8_t SerialCommunicatorLinux::stringChecksum(const uint8_t* s, int idx_start, int idx_end) {
    uint8_t c = 0;
    for(int i = idx_start; i <= idx_end; i++) c ^= s[i];
    return c;
};
