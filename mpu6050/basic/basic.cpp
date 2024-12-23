#include <iostream>
#include <iomanip>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

#define MPU6050_ADDR 0x68

void writeRegister(int fd, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(fd, buffer, 2) != 2) {
        perror("Failed to write to the i2c bus");
        exit(1);
    }
}

void readRegister(int fd, uint8_t reg, uint8_t* buffer, size_t length) {
    if (write(fd, &reg, 1) != 1) {
        perror("Failed to set register for reading");
        exit(1);
    }
    if (read(fd, buffer, length) != static_cast<int>(length)) {
        perror("Failed to read from the i2c bus");
        exit(1);
    }
}

void initializeMPU6050(int fd) {
    // Wake up MPU6050 by writing 0 to the power management register (Register 107)
    writeRegister(fd, 107, 0);

    // Configure gyro to default range (±250°/s)
    writeRegister(fd, 27, 0);

    // Configure accelerometer to default range (±2g)
    writeRegister(fd, 28, 0);

    std::cout << "MPU6050 initialized successfully!" << std::endl;
}

int main() {
    const char* i2c_device = "/dev/i2c-0";
    int fd = open(i2c_device, O_RDWR);
    if (fd < 0) {
        perror("Failed to open the i2c bus");
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(fd);
        return 1;
    }

    // Initialize MPU6050
    initializeMPU6050(fd);

    while (true) {
        int16_t acc_raw[3] = {0}, gyro_raw[3] = {0};
        uint8_t buffer[6];

        // Read accelerometer data
        readRegister(fd, 59, buffer, 6);
        for (size_t i = 0; i < 3; ++i) {
            acc_raw[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
        }

        // Read gyroscope data
        readRegister(fd, 67, buffer, 6);
        for (size_t i = 0; i < 3; ++i) {
            gyro_raw[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
        }

        // Print results
        std::ostringstream output;
        output << "ACC X: " << acc_raw[0] << ", Y: " << acc_raw[1] << ", Z: " << acc_raw[2]
               << "   GYRO X: " << gyro_raw[0] << ", Y: " << gyro_raw[1] << ", Z: " << gyro_raw[2];
        std::cout << output.str() << std::endl;

        usleep(100000); // Delay for 100 ms
    }

    close(fd);
    return 0;
}
