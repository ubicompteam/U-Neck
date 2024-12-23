# servo

## 사용법
- `sudo apt-get install libi2c-dev`
- `sudo g++ servoExample.cpp JHPWMPCA9685.cpp -o main -li2c`

## using PCA9685
- https://github.com/jetsonhacks/JHPWMDriver
- 위 깃허브가 옛날꺼라 라이브러리 수정하여 사용

## V1.0 : basic servo control
### 코드 심층 해석
- https://kksp12y.tistory.com/77

## V1.1 : 2 servo control

## V2.0 : 2 servo cpntrol with mpu6050
- `sudo g++ 2_servo_with_mpu6050.cpp JHPWMPCA9685.cpp -o main -li2c -lrt`


## V2.1 : 2 servo cpntrol with mpu6050, 칼만필터
- `sudo g++ 2_servo_with_mpu6050.cpp JHPWMPCA9685.cpp -o main -li2c -lrt`
