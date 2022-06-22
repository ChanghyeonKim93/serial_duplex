#ifndef _SERIAL_COMM_H_
#define _SERIAL_COMM_H_

#define BUF_SIZE 1024

#include <iostream>
#include <time.h>
#include <thread>
#include <string>
#include <cstring>
#include <chrono>

// serial
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <sys/time.h>

using namespace std::chrono_literals;

class SerialCommunicatorLinux {
public:
    SerialCommunicatorLinux(){};
    SerialCommunicatorLinux(const std::string& portname, const int& baud_rate){
        portname_ = portname;
        STX_ = '$';
        ETX_ = '%';
        
        // initialize
        seq_recv_  = 0;
        seq_send_  = 0;
        
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

        openSerialPort();
        
        this->runThreadRX();
        this->runThreadTX();

        printf("[SerialComm] Portname is {%s}, message length: {%d}\n", portname_.c_str(), MSG_LEN_);
    };

    // deconstructor
    ~SerialCommunicatorLinux() {
        if(thread_rx_.joinable()) this->thread_rx_.join();
        if(thread_tx_.joinable()) this->thread_tx_.join();
    };

    void runThreadRX(){
        this->thread_rx_ = std::thread(&SerialCommunicatorLinux::processRX, this);
    };

    void runThreadTX(){
        this->thread_tx_ = std::thread(&SerialCommunicatorLinux::processTX, this);
    }


private:
    void read_withChecksum(char* msg){
       
    };

    void send_withChecksum(const char* data, int len){
        char crc = stringChecksum(data,0,len-1);

        char len_c = (char)len;
        write(fd_,&STX_,  1);
        write(fd_,&len_c, 1);
        write(fd_,data,   len);
        write(fd_,&crc,   1);
        write(fd_,&ETX_,  1);
    };

    void openSerialPort(){
        printf("Opening the serial port...\n");
        fd_ = open(portname_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ == -1) {
            throw std::runtime_error("Error - no serial port '" + portname_ + "' is opened! Check the serial device.\n");
        }
        else {
            printf("{%s} is opened.\n",portname_.c_str());
        }
    };

    void processRX(){
        // newtio_rx initialization.
        memset(&this->newtio_rx_, 0, sizeof(this->newtio_rx_));
        newtio_rx_.c_cflag     = BAUD_RATE_ | CS8 | CLOCAL | CREAD;
        newtio_rx_.c_oflag     = 0;
        newtio_rx_.c_lflag     = 0;
        newtio_rx_.c_cc[VTIME] = 0;
        newtio_rx_.c_cc[VMIN]  = 1;

        tcflush(fd_, TCIFLUSH);
        tcsetattr(fd_, TCSANOW, &newtio_rx_);

        // poll preparation
        poll_events_.fd = fd_;
        poll_events_.events = POLLIN | POLLERR; // if received or if there is error.
        poll_events_.revents = 0;
        
        stack_len_ = 0;
        while(true) {
            // polling stage. (when some data is received.)
            poll_state_ = poll(                 //poll() call. event check.
                (struct pollfd *)&poll_events_, // event registration
                1,                             // the number of pollfd which we want to check.
                500);                          // time out [ms]

            if (poll_state_ > 0) {
                if (poll_events_.revents & POLLIN) { // event is receiving data.
                    //#define POLLIN   0x0001 // 읽을 데이터가 있다.
                    //#define POLLPRI  0x0002 // 긴급한 읽을 데이타가 있다.
                    //#define POLLOUT  0x0004 // 쓰기가 봉쇄(block)가 아니다.
                    //#define POLLERR  0x0008 // 에러발생
                    //#define POLLHUP  0x0010 // 연결이 끊겼음
                    //#define POLLNVAL 0x0020 // 파일지시자가 열리지 않은 것 같은, Invalid request (잘못된 요청)

                    // Read serial RX buffer
                    // char msg_recv[1024];
                    // this->read_withChecksum(msg_recv);
                    int len_read = read(fd_, buf_, BUF_SIZE);
                    for(int i = 0; i < len_read; ++i) {
                        if(buf_[i] == ETX_){ // Test 1: ETX
                            // data part : serial_stack_[1] ~ serial_stack_[MSG_LEN_]
                            if(serial_stack_[0] == STX_) { // Test 2: STX
                                MSG_LEN_ = (int)serial_stack_[1];
                                char check_sum = stringChecksum(serial_stack_, 2, MSG_LEN_+1);
                                if(serial_stack_[MSG_LEN_+2] == check_sum) { // Test 3: checksum test.
                                    ++seq_recv_;                                   
                                    short acc[3]; 
                                    short gyro[3];
                                    short mag[3];
                                    double t_now = 0;
                                    
                                    for(int k = 0; k < 3; ++k){
                                        int k6 = 6*k + 2;
                                        acc[k]  = decode2BytesToShort(serial_stack_[k6],   serial_stack_[1+k6]);
                                        gyro[k] = decode2BytesToShort(serial_stack_[2+k6], serial_stack_[3+k6]);
                                        mag[k]  = decode2BytesToShort(serial_stack_[4+k6], serial_stack_[5+k6]);
                                    }
                                    t_now   = decodeTime(serial_stack_[20],serial_stack_[21],
                                                         serial_stack_[22],serial_stack_[23],serial_stack_[24],serial_stack_[25]);
                                    std::cout << "seq:" << seq_recv_ <<", msglen:" << MSG_LEN_ <<", time:" 
                                    << t_now << " / "  << acc[0] <<"," <<acc[1] <<"," << acc[2] 
                                    << " / " << gyro[0] << "," << gyro[1] << "," << gyro[2]
                                    << " / " << mag[0] << "," << mag[1] << "," << mag[2] << std::endl;
                                }
                            }
                            stack_len_ = 0;
                        }
                        else serial_stack_[stack_len_++] = buf_[i];
                    }
                }
                if (poll_events_.revents & POLLERR) {// The connection is destructed.
                    printf("[SerialComm] ERROR - There is communication error. program exit.\n");
                    break;
                }
            }
            else if (poll_state_ == 0) { //timeout.
                printf("[SerialComm] TIMEOUT-500 [ms].\n");
            }
            else if(poll_state_ < 0) { // Error occurs.
                //printf("ERROR-critical error occurs. Program is shutdown.\n");
                //return -1;
                continue;
            }
        } // END WHILE
    };

    void processTX(){
        int cnt = 0;
        while(true){
            char data[15]={'a','b','c','d','e','a','b','c','d','e','a','b','c','d','e'};
            send_withChecksum(data, 15);
            std::cout << " TX OK!:"<< ++cnt << std::endl;
            std::this_thread::sleep_for(50ms);
        }
    };

private:
    char stringChecksum(const char* s, int idx_start, int idx_end) {
        char c = 0;
        for(int i = idx_start; i <= idx_end; i++) c ^= s[i];
        return c;
    };

private:
    std::string portname_;
    int BAUD_RATE_;

    int fd_; // file descriptor...
    
    struct termios newtio_rx_;
    struct termios newtio_tx_;

    struct pollfd poll_events_;
    int poll_state_;
    char buf_[BUF_SIZE];
    
    int MSG_LEN_;
    char STX_;
    char ETX_;

    int stack_len_;
    char serial_stack_[BUF_SIZE];

private:
    uint64_t seq_recv_;
    uint64_t seq_send_;

private:
    std::thread thread_rx_;
    std::thread thread_tx_;

private:

    inline short decode2BytesToShort(char h_byte, char l_byte){
        return (short) (((unsigned char)h_byte << 8 )| (unsigned char)l_byte);
    };

    double decodeTime(char sec_l, char sec_h, char usec0, char usec1, char usec2, char usec3){
        unsigned short sec 
            = (unsigned short) ( (unsigned char)sec_h << 8 | (unsigned char)sec_l);
        unsigned int usec  
            = (unsigned int)   ( (unsigned char)usec3 << 24 | (unsigned char)usec2 << 16 | (unsigned char)usec1 << 8 | (unsigned char)usec0 );
        return ((double)sec + (double)usec/1000000.0);
    };
};


#endif