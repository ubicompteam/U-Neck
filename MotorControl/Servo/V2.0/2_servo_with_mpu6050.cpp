#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <stdint.h> // uint8_t, int16_t 등의 자료형 사용
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h> // clock_gettime 및 CLOCK_MONOTONIC

#include "JHPWMPCA9685.h" // PCA9685 라이브러리

#define MPU6050_ADDR 0x68
#define PCA9685_FREQ 60
#define SERVO_MIN 120
#define SERVO_MAX 720
#define SERVO_NEUTRAL 90 // 중립값

// I2C 설정
int fd;
int openI2C(const char* device, int addr) {
    fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("Failed to open I2C device");
        exit(1);
    }
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        perror("Failed to set I2C address");
        close(fd);
        exit(1);
    }
    return fd;
}

void writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    write(fd, buffer, 2);
}

void readRegister(uint8_t reg, uint8_t* buffer, size_t length) {
    write(fd, &reg, 1);
    read(fd, buffer, length);
}

// IMU 데이터 읽기
void readIMUData(int16_t* acc) {
    uint8_t buffer[6];
    readRegister(0x3B, buffer, 6); // Accelerometer data
    for (int i = 0; i < 3; ++i) {
        acc[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
    }
}

// 각도 계산 (Roll/Pitch)
void calculateAngles(int16_t* acc, float* roll, float* pitch) {
    *roll = atan2(acc[1], acc[2]) * 180.0 / M_PI;
    *pitch = atan2(-acc[0], sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * 180.0 / M_PI;
}

// 각도 -> PWM 변환
int angleToPWM(float angle) {
    return (int)((angle / 180.0) * (SERVO_MAX - SERVO_MIN) + SERVO_MIN);
}

// 메인 프로그램
int main() {
    // PCA9685 초기화
    PCA9685* pca9685 = new PCA9685();
    if (pca9685->openPCA9685() < 0) {
        printf("Error opening PCA9685\n");
        return -1;
    }
    pca9685->setAllPWM(0, 0);
    pca9685->reset();
    pca9685->setPWMFrequency(PCA9685_FREQ);

    // MPU6050 초기화
    openI2C("/dev/i2c-0", MPU6050_ADDR);
    writeRegister(0x6B, 0x00); // Wake up
    writeRegister(0x1B, 0x00); // Gyro config
    writeRegister(0x1C, 0x00); // Accel config

    // 초기값 설정
    float roll_neutral = SERVO_NEUTRAL;  // 기본 Roll 각도 (90도)
    float pitch_neutral = SERVO_NEUTRAL; // 기본 Pitch 각도 (90도)

    while (1) {
        // IMU 데이터 읽기
        int16_t acc[3];
        readIMUData(acc);

        // Roll, Pitch 계산
        float roll, pitch;
        calculateAngles(acc, &roll, &pitch);

        // 변화량 계산 (ΔRoll, ΔPitch)
        float delta_roll = roll;   // 초기값이 0도이므로 변화량은 Roll 자체
        float delta_pitch = pitch; // 초기값이 0도이므로 변화량은 Pitch 자체

        // Roll, Pitch에 따른 PWM 값 계산
        int roll_pwm = angleToPWM(roll_neutral - delta_roll);
        int pitch_pwm = angleToPWM(pitch_neutral - delta_pitch);

        // 서보모터 제어
        pca9685->setPWM(0, 0, roll_pwm);  // Roll 축 서보모터
        pca9685->setPWM(1, 0, pitch_pwm); // Pitch 축 서보모터

        // 디버깅 출력
        printf("Roll: %.2f, Pitch: %.2f, Roll PWM: %d, Pitch PWM: %d\n", roll, pitch, roll_pwm, pitch_pwm);

        // 주기 10ms
        usleep(10000);
    }

    pca9685->closePCA9685();
    close(fd);
    return 0;
}
