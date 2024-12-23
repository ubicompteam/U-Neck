#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include "JHPWMPCA9685.h"

// Calibrated for a Robot Geek RGS-13 Servo
int servoMin = 120; // Minimum pulse length
int servoMax = 720; // Maximum pulse length

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO | ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

// Map an integer from one coordinate system to another
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main() {
    PCA9685 *pca9685 = new PCA9685();
    int err = pca9685->openPCA9685();
    if (err < 0) {
        printf("Error: %d\n", pca9685->error);
    } else {
        printf("PCA9685 Device Address: 0x%02X\n", pca9685->kI2CAddress);
        pca9685->setAllPWM(0, 0);
        pca9685->reset();
        pca9685->setPWMFrequency(60); // Set frequency to 60 Hz
        printf("Press ESC key to exit\n");

        while (pca9685->error >= 0 && getkey() != 27) {
            // Move to 120 degrees
            int position120 = map(120, 0, 180, servoMin, servoMax);
            pca9685->setPWM(0, 0, position120);
            printf("Servo moved to 120 degrees\n");
            sleep(2);

            // Move back to 0 degrees
            int position0 = map(0, 0, 180, servoMin, servoMax);
            pca9685->setPWM(0, 0, position0);
            printf("Servo moved back to 0 degrees\n");
            sleep(2);
        }

        // Stop the servo when exiting
        pca9685->setPWM(0, 0, 0);
        sleep(1);
    }
    pca9685->closePCA9685();
}

