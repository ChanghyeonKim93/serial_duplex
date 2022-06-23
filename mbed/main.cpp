/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"
#include "serial_comm_mbed.h"

#include "motor_pwm.h"

#include "mpu9250.h" // IMU, https://os.mbed.com/users/manitou/code/mpu9250//file/0158e4d78423/main.cpp/
#include <cstdint>

// Parameter settings
#define BAUD_RATE 921600
#define I2C_FREQ  400000 // 400 kHz

// IMU interrupt pin
#define PIN_IMU_INTTERRUPT   PB_0 //int 6

// Camera trigger pin
#define PIN_CAMERA_TRIGGER   PB_1//int 6

// PWM pin
#define PIN_PWM0             PA_1  // PWM timer 1, channel 1
#define PIN_PWM1             PA_3  // PWM timer 1, channel 2
#define PIN_PWM2             PA_10 // PWM timer 1, channel 3
#define PIN_PWM3             PA_11 // PWM timer 1, channel 4

// MCU Status to be sent to the computer
#define STATE_IMU      0b0001
#define STATE_CAMERA   0b0010

typedef union USHORT_UNION_{
    uint16_t ushort_;
    uint8_t  bytes_[2];
} USHORT_UNION;

typedef union UINT_UNION_{
    uint32_t uint_;
    uint8_t  bytes_[4];
} UINT_UNION;

typedef union FLOAT_UNION_{
    float float_;
    uint8_t bytes_[4];   
} FLOAT_UNION;

// Serial
SerialCommunicatorMbed serial(BAUD_RATE);

uint8_t send_buffer[256];
uint8_t recv_buffer[256];

// IMU
MPU9250 mpu9250;
InterruptIn int_imu(PIN_IMU_INTTERRUPT);

// PWM
MotorPwm pwm0(PIN_PWM0);
MotorPwm pwm1(PIN_PWM1);
// MotorPwm pwm2(PIN_PWM2);
// MotorPwm pwm3(PIN_PWM3);

volatile float duty[4] = {0.1, 0.3, 0.5, 0.7};
volatile float duty_step[4] = {0.005,0.005,0.005,0.005};

void workISRUserContext_readSerial(){
    serial.read_withChecksum(recv_buffer);
};

// Set event queue and worker thread
Thread thread_poll(osPriorityRealtime); // polling all interrupt signals. https://os.mbed.com/docs/mbed-os/v6.10/apis/thread.html
EventQueue equeue(128 * EVENTS_EVENT_SIZE);

volatile bool FLAG_IMU_NEWDATA  = false;
volatile bool FLAG_DO_DATA_SEND = false;

void workISRUserContext_IMU(){ // this code will run in the user context.
    mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS);  //? need this with ISR
    mpu9250.readAccelData(accelCount, accel_raw);   // Read the x/y/z adc values
    mpu9250.readGyroData(gyroCount, gyro_raw);      // Read the x/y/z adc values
    mpu9250.readMagData(magCount, mag_raw);                  // Read the x/y/z adc values
};

void flagISR_IMU(){
    FLAG_IMU_NEWDATA = true;
    equeue.call(workISRUserContext_IMU); 
}; // ISR function for IMU

// Timer to know how much time elapses.
Timer timer;


int main()
{
    // Timer starts.
    timer.start();
    uint64_t us_curr;
    USHORT_UNION tsec;
    UINT_UNION   tusec;

    // Start the event queue
    thread_poll.start(callback(&equeue, &EventQueue::dispatch_forever));
    serial.send_withoutChecksum("Starting event queue in context...",50);
    
    
    // IMU initialization (I2C Setup)
    int_imu.rise(flagISR_IMU);
    i2c.frequency(I2C_FREQ);  // use fast (400 kHz) I2C   
    uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

    if (whoami == 0x71) { // WHO_AM_I should always be 0x71
        serial.send_withoutChecksum("MPU9250 is online...\n\r",22);
        mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
        mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
        wait_us(1000000);
        mpu9250.initMPU9250();
        serial.send_withoutChecksum("MPU9250 initialized for active data mode....\n\r",100); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        mpu9250.initAK8963(magCalibration);
        serial.send_withoutChecksum("AK8963 initialized for active data mode....\n\r",100); // Initialize device for active mode read of magnetometer
        // serial.send_withoutChecksum("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
        // serial.send_withoutChecksum("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
        if(Mscale == 0) serial.send_withoutChecksum("Magnetometer resolution = 14  bits\n\r",100);
        if(Mscale == 1) serial.send_withoutChecksum("Magnetometer resolution = 16  bits\n\r",100);
        if(Mmode == 2)  serial.send_withoutChecksum("Magnetometer ODR = 8 Hz\n\r",100);
        if(Mmode == 6)  serial.send_withoutChecksum("Magnetometer ODR = 100 Hz\n\r",100);
        wait_us(1000000);
    } else {
        serial.send_withoutChecksum("Could not connect to MPU9250: \n\r",50);
        while(1) ; // Loop forever if communication doesn't happen
    }

    // Loop
    while (true) {
        std::chrono::microseconds tnow = timer.elapsed_time();       

        // Current time
        us_curr      = timer.elapsed_time().count();
        tsec.ushort_ = (uint16_t)(us_curr/1000000);
        tusec.uint_  = (uint32_t)(us_curr-((uint32_t)tsec.ushort_)*1000000);
        
        // Write data if IMU data received
        if(FLAG_IMU_NEWDATA){
            FLAG_IMU_NEWDATA = false;

            // fill out send buffer
            for(int k = 0; k < 6; ++k){
                send_buffer[k]    = accel_raw[k]; // High byte first.
                send_buffer[k+6]  = gyro_raw[k];  // High byte first.
                send_buffer[k+12] = mag_raw[k];   // High byte first.
            }

            send_buffer[18]  = tsec.bytes_[0];  // time (second part, low)
            send_buffer[19]  = tsec.bytes_[1];  // time (second part, high)
            
            send_buffer[20]  = tusec.bytes_[0]; // time (microsecond part, lowest)
            send_buffer[21]  = tusec.bytes_[1]; // time (microsecond part, low)
            send_buffer[22]  = tusec.bytes_[2]; // time (microsecond part, high)
            send_buffer[23]  = tusec.bytes_[3]; // time (microsecond part, highest)

            // STATE
            // unsigned char STATE_MCU_TO_PC = 0;
            // STATE_MCU_TO_PC |= STATE_IMU;        
            // send_buffer[18]  = STATE_MCU_TO_PC;

            serial.send_withChecksum(send_buffer, 24);

            pwm0.setDuty(duty[0]);
            pwm1.setDuty(duty[1]);
            // pwm2.setDuty(duty[2]);
            // pwm3.setDuty(duty[3]);
            duty[0] += duty_step[0];
            duty[1] += duty_step[1];
            // duty[2] += duty_step[2];
            // duty[3] += duty_step[3];
            if(duty[0] > 1 || duty[0] < 0) duty_step[0] = -duty_step[0];
            if(duty[1] > 1 || duty[1] < 0) duty_step[1] = -duty_step[1];
            // if(duty[2] > 1 || duty[2] < 0) duty_step[2] = -duty_step[2];
            // if(duty[3] > 1 || duty[3] < 0) duty_step[3] = -duty_step[3];
        }
        
        // Read data if data exists.
        if(serial.readable()){
            // [RECV ETHERNET] data from PC
            // int rcount = pc.
            // if(rcount > 0){ // if there is a signal, (control signal...?)
            equeue.call(workISRUserContext_readSerial);
        }
                
    }
}
