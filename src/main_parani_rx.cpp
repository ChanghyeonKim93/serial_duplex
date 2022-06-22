// MAIN_RX.CPP

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/poll.h>

#include <sys/time.h>
#include <queue>

#define PI 3.141592

int main(void)
{
    int fd_parani2;
    struct termios newtio_rx;
    struct pollfd poll_events; // event struct ( interrupt )
    int poll_state;

    int num_byte;
    char buf[1024];

    printf("This is PARANI2 (RX) node.\n");
    fd_parani2 = open("/dev/PARANI2", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_parani2 == -1)
    {
        printf("Error - no serial port /dev/PARANI2 is opened! Check the serial device.\n");
        return -1;
    }
    else
        printf("/dev/PARANI2 is opened.\n");
    memset(&newtio_rx, 0, sizeof(newtio_rx));
    newtio_rx.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    newtio_rx.c_oflag = 0;
    newtio_rx.c_lflag = 0;
    newtio_rx.c_cc[VTIME] = 0;
    newtio_rx.c_cc[VMIN] = 1;

    tcflush(fd_parani2, TCIFLUSH);
    tcsetattr(fd_parani2, TCSANOW, &newtio_rx);

    // poll preparation
    poll_events.fd = fd_parani2;
    poll_events.events = POLLIN | POLLERR; // if received or if there is error.
    poll_events.revents = 0;

    // read
    int cnt = 0;
    std::queue<char> data_queue;
    while (1) {
        // polling stage. (when some data is received.)
        poll_state = poll(                 //poll() call. event check.
            (struct pollfd *)&poll_events, // event registration
            1,                             // the number of pollfd which we want to check.
            500);                          // time out [ms]

        if (poll_state > 0) {
            if (poll_events.revents & POLLIN) { // event is receiving data.
                //#define POLLIN 0x0001 // 읽을 데이터가 있다.
                //#define POLLPRI 0x0002 // 긴급한 읽을 데이타가 있다.
                //#define POLLOUT 0x0004 // 쓰기가 봉쇄(block)가 아니다.
                //#define POLLERR 0x0008 // 에러발생
                //#define POLLHUP 0x0010 // 연결이 끊겼음
                //#define POLLNVAL 0x0020 // 파일지시자가 열리지 않은 것 같은, Invalid request (잘못된 요청)
                num_byte = read(fd_parani2, buf, 1024);
                printf("Data received - cnt : %d, %d %s\n", cnt, num_byte, buf);

                for (int i = 0; i < num_byte; i++) {
                    data_queue.push(buf[i]);
                    buf[i] = '\0'; // initialize all buffer.
                }
                num_byte = 0;
                cnt++;

                printf("Queue size : %d\n", data_queue.size());
            }
            if (poll_events.revents & POLLERR) {// The connection is destructed.
                printf("ERROR - There is communication error. program exit.\n");
                break;
            }
        }
        else if (poll_state == 0) { //timeout.
            printf("TIMEOUT-500 [ms].\n");
        }
        else if(poll_state < 0) { // Error occurs.
            printf("ERROR-critical error occurs. Program is shutdown.\n");
            return -1;
        }

        // Decode 
        if (data_queue.size() > 100) {
            std::vector<char> data_vector;
            for (int i = 0; i < data_queue.size(); i++) {
                if (data_queue.front() != '(') {
                    data_queue.pop();
                }

                else
                {
                    data_queue.pop();
                    while(data_queue.front() != ')')
                    {
                        data_vector.push_back(data_queue.front());
                        data_queue.pop();
                    }
                    printf("vector size: %ld\n",data_vector.size());
                    for(int k = 0; k < data_vector.size(); k++)
                        printf("%c",data_vector[k]);
                    printf("\n");
                    break;
                }
            }
        }
        // usleep(1000 * 10);
    }

    close(fd_parani2);
    return 0;
}