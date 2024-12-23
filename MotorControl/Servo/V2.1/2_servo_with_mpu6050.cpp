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
void readIMUData(int16_t* acc, int16_t* gyro) {
    uint8_t buffer[6];
    readRegister(0x3B, buffer, 6); // Accelerometer data
    for (int i = 0; i < 3; ++i) {
        acc[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
    }

    readRegister(0x43, buffer, 6); // Gyroscope data
    for (int i = 0; i < 3; ++i) {
        gyro[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
    }
}

// 각도 계산 (Roll/Pitch)
void calculateAngles(int16_t* acc, float* roll, float* pitch) {
    *roll = atan2(acc[1], acc[2]) * 180.0 / M_PI;
    *pitch = atan2(-acc[0], sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * 180.0 / M_PI;
}

// 칼만 필터 구조체
typedef struct {
    float angle;       // 추정된 각도 (롤 또는 피치)
    float rate;        // 각속도 (자이로에서 읽은 값)
    float P;           // 오차 공분산
    float K;           // 칼만 이득
    float Q;           // 프로세스 잡음
    float R;           // 측정 잡음
} KalmanFilter;

// 칼만 필터 초기화
void KalmanFilter_Init(KalmanFilter* kf, float Q, float R) {
    kf->angle = 0.0f;
    kf->rate = 0.0f;
    kf->P = 1.0f;  // 초기 오차 공분산
    kf->Q = Q;
    kf->R = R;
}

// 칼만 필터 업데이트
float KalmanFilter_Update(KalmanFilter* kf, float newAngle, float newRate, float dt) {
    // 예측 단계
    kf->angle += dt * kf->rate;  // 각속도를 바탕으로 각도 예측
    kf->P += kf->Q;

    // 갱신 단계
    kf->K = kf->P / (kf->P + kf->R);  // 칼만 이득 계산
    kf->angle += kf->K * (newAngle - kf->angle);  // 각도 보정
    kf->P = (1 - kf->K) * kf->P;  // 오차 공분산 갱신

    // 각속도 업데이트 (자이로스코프에서 얻은 각속도를 사용)
    kf->rate = newRate;

    return kf->angle;
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

    // 칼만 필터 초기화
    KalmanFilter rollFilter, pitchFilter;
    KalmanFilter_Init(&rollFilter, 0.001f, 0.03f); // 프로세스 잡음, 측정 잡음 설정
    KalmanFilter_Init(&pitchFilter, 0.001f, 0.03f);

    // 초기값 설정
    float roll_neutral = SERVO_NEUTRAL;  // 기본 Roll 각도 (90도)
    float pitch_neutral = SERVO_NEUTRAL; // 기본 Pitch 각도 (90도)

    while (1) {
        // IMU 데이터 읽기
        int16_t acc[3], gyro[3];
        readIMUData(acc, gyro); // 가속도계 및 자이로스코프 데이터

        // 가속도계와 자이로스코프 데이터를 사용하여 롤, 피치 각도 계산
        float roll_acc = atan2(acc[1], acc[2]) * 180.0 / M_PI;
        float pitch_acc = atan2(-acc[0], sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * 180.0 / M_PI;
        
        float roll_rate = gyro[0] / 131.0;  // 자이로스코프 각속도 (MPU6050의 131.0으로 나눔)
        float pitch_rate = gyro[1] / 131.0;

        // 칼만 필터로 각도 추정
        float roll = KalmanFilter_Update(&rollFilter, roll_acc, roll_rate, 0.01);
        float pitch = KalmanFilter_Update(&pitchFilter, pitch_acc, pitch_rate, 0.01);

        // 변화량 계산 (ΔRoll, ΔPitch)
        float delta_roll = roll;   
        float delta_pitch = pitch; 

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
