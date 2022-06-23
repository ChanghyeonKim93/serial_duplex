#ifndef _SERIAL_COMM_H_
#define _SERIAL_COMM_H_

#define BUF_SIZE 2048

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

#include "struct.h"

using namespace std::chrono_literals;

class SerialCommunicatorLinux {
public:
    SerialCommunicatorLinux(const std::string& portname, const int& baud_rate);
    ~SerialCommunicatorLinux();

private:
    // initialize functions
    void openSerialPort();

    // Thread functions
    void runThreadRX();
    void runThreadTX();

    void processRX();
    void processTX();

    // Read and write functions
    int  read_withChecksum(uint8_t* msg);
    void send_withChecksum(const uint8_t* data, int len);
    
private:
    // Checksum functions
    uint8_t stringChecksum(const uint8_t* s, int idx_start, int idx_end);

private:
    std::string portname_;
    int BAUD_RATE_;

    int fd_; // file descriptor...
    
    struct termios newtio_;

    struct pollfd poll_events_;
    // event is receiving data.
    //#define POLLIN   0x0001 // 읽을 데이터가 있다.
    //#define POLLPRI  0x0002 // 긴급한 읽을 데이타가 있다.
    //#define POLLOUT  0x0004 // 쓰기가 봉쇄(block)가 아니다.
    //#define POLLERR  0x0008 // 에러발생
    //#define POLLHUP  0x0010 // 연결이 끊겼음
    //#define POLLNVAL 0x0020 // 파일지시자가 열리지 않은 것 같은, Invalid request (잘못된 요청)

    int  poll_state_;
    uint8_t buf_[BUF_SIZE];
    
    int  MSG_LEN_;
    uint8_t STX_;
    uint8_t ETX_;

    int  stack_len_;
    uint8_t serial_stack_[BUF_SIZE];

private:
    uint64_t seq_recv_;
    uint64_t seq_send_;

private:
    std::thread thread_rx_;
    std::thread thread_tx_;
};


#endif