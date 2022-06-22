#ifndef _SERIAL_COMM_H_
#define _SERIAL_COMM_H_

#define BUF_SIZE 1024

#include <iostream>
#include <time.h>
#include <thread>
#include <string>
#include <cstring>

// serial
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <sys/time.h>

class SerialCommunicator {
public:
    SerialCommunicator(){};
    SerialCommunicator(const std::string& portname, const int& msg_len){
        portname_ = portname;
        MSG_LEN_  = msg_len;
        STX_ = '$';
        ETX_ = '%';
        CTX_ = '*';

        printf("[SerialComm] Portname is {%s}, message length: {%d}\n", portname_.c_str(), MSG_LEN_);
    };
    // deconstructor
    ~SerialCommunicator() {
        this->thread_.join();
    }

    void runThread(){
        this->thread_ = std::thread(&SerialCommunicator::process, this);
    };


private:
    void process(){
        uint64_t seq = 0;
        printf("Process runs.\n");
        fd_ = open(portname_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ == -1) {
            printf("Error - no serial port {%s} is opened! Check the serial device.\n", portname_.c_str());
            return;
        }
        else printf("{%s} is opened.\n",portname_.c_str());


        // newtio_rx initialization.
        memset(&this->newtio_rx_, 0, sizeof(this->newtio_rx_));
        newtio_rx_.c_cflag     = B460800 | CS8 | CLOCAL | CREAD;
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
        int cnt = 0;
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
                    int len_read = read(fd_, buf_, BUF_SIZE);
                    
                    for(int i = 0; i < len_read; ++i) {
                        if(buf_[i] == ETX_){ // Test 1: ETX
                            // data part : serial_stack_[1] ~ serial_stack_[MSG_LEN_]
                            if(serial_stack_[0] == STX_ 
                            && serial_stack_[MSG_LEN_+1] == CTX_) { // Test 2: STX and CTX
                                char check_sum = stringChecksum(serial_stack_, 1, MSG_LEN_);
                                if(serial_stack_[MSG_LEN_+2] == check_sum) { // Test 3: checksum test.
                                    ++seq;                                   
                                    short acc[3]; 
                                    short gyro[3];
                                    double t_now = 0;
                                    
                                    acc[0]  = decode2BytesToShort(serial_stack_[1],serial_stack_[2]);
                                    acc[1]  = decode2BytesToShort(serial_stack_[3],serial_stack_[4]);
                                    acc[2]  = decode2BytesToShort(serial_stack_[5],serial_stack_[6]);
                                    gyro[0] = decode2BytesToShort(serial_stack_[7],serial_stack_[8]);
                                    gyro[1] = decode2BytesToShort(serial_stack_[9],serial_stack_[10]);
                                    gyro[2] = decode2BytesToShort(serial_stack_[11],serial_stack_[12]);
                                    t_now   = decodeTime(serial_stack_[13],serial_stack_[14],
                                                         serial_stack_[15],serial_stack_[16],serial_stack_[17],serial_stack_[18]);
                                    std::cout << "seq:" << seq <<", time:" 
                                    << t_now << " / "  << acc[0] <<"," <<acc[1] <<"," << acc[2] 
                                    << " / " << gyro[0] << "," << gyro[1] << "," << gyro[2] << std::endl;
                                }
                            }
                            stack_len_ = 0;
                            ++cnt;
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

private:

    char stringChecksum(char* s, int idx_start, int idx_end) {
        char c = 0;
        for(int i = idx_start; i <= idx_end; i++) c ^= s[i];
        return c;
    };

private:
    std::string portname_;

    int fd_;
    struct termios newtio_rx_;
    struct pollfd poll_events_;
    int poll_state_;
    char buf_[BUF_SIZE];
    
    int MSG_LEN_;
    char STX_;
    char ETX_;
    char CTX_;

    int stack_len_;
    char serial_stack_[BUF_SIZE];

private:
    std::thread thread_;

private:

    inline short decode2BytesToShort(char h_byte, char l_byte){
        return (short) ( (unsigned char)h_byte << 8 | (unsigned char)l_byte);
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