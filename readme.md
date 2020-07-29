# ESP32_IMU_Module

```
/*
Version:		V1.1
Author:			Vincent
Create Date:	2020/7/22
Note：
	v1.1:Added the IMU documentation and PCB diagram.
*/
```

![main](md_pic/show.gif)

*The following two project were briefly modified.*

[PyTeapot-Quaternion-Euler-cube-rotation](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation)

[i2cdevlib](https://github.com/jrowberg/i2cdevlib)

[YouTube:6 Axis IMU Sensor with ESP32](https://www.youtube.com/watch?v=PJSM8G_pLW8&feature=youtu.be)

[toc]

# Overview

[Makerfabs home page](https://www.makerfabs.com/)

[Makerfabs Wiki](https://makerfabs.com/wiki/index.php?title=Main_Page)

At present, it is only a technical verification of IMU, and there may be a real project in the future. The hardware is in develop.

Through the socket to achieve wifi transmission euler Angle, and display on the PC side of the Python program.To facilitate testing, I packaged a Win10 64-bit EXE program that can be accessed by changing the IP in the ESP32 program.Otherwise, you need to install a dependent library like OpenGL.



# ESP32 IMU Module V1.1

The Makerfabs IMU Module features the 6-axis MPU-6050 MEMS sensor from InvenSense. Each of these 6DoF IMU feature an ESP32 with a MPU-6050 which contains a 3-axis gyroscope as well as a 3-axis accelerometer. The MPU-6050 uses 16-bit analog-to-digital converters (ADCs) for digitizing 6 axes. By combining a MEMS 3-axis gyroscope and a 3-axis accelerometer on the same silicon die together with an onboard Digital Motion Processor™ (DMP™) .It can be used as a helicopter/quadcopter.

## Feature

- Integrated ESP32 2.4G WiFi and Bluetooth
- I2C Digital-output of 6-axis MotionFusion data in rotation matrix, quaternion, Euler Angle, or raw data format
- Input Voltage: 3.7V Battery or 5.0V MicroUSB 
- Tri-Axis angular rate sensor (gyro) with a sensitivity up to 131 LSBs/dps and a full-scale range of ±250, ±500, ±1000, and ±2000dps
- Tri-Axis accelerometer with a programmable full scale range of ±2g, ±4g, ±8g and ±16g
- Digital Motion Processing™ (DMP™) engine offloads complex MotionFusion, sensor timing synchronization and gesture detection
- Embedded algorithms for run-time bias and compass calibration. No user intervention required
- Digital-output temperature sensor
- Size: max 100mmX100mmX9mm

![front](md_pic/front.jpg)

![back](md_pic/back.jpg)



# Steps

## PC

- If you're on Windows 64-bit, you can run "/ESP32_IMU_Module/imu_show/dist/imu_show.exe".
- Other platform can run "/ESP32_IMU_Module/imu_show/imu_show.py".(Dependency libraries such as OpenGL need to be installed manually)
- After run program,wait seconds.
- A line of native IP is displayed.
- Leave the Python program window still and prepare ESP32.

## ESP32

- Open "/ESP32_IMU_Module/mpu6050_wifi/mpu6050_wifi.ino"
- Change IP ,your Wifi ssid and password.

```c++
const char *ssid = "Makerfabs";
const char *password = "20160704";
const char *host = "192.168.1.117"; 
```

- Select Tools/Board/BoardsManager.
- Search esp32.
- Install esp32 by Espressif Systems.

![install_esp32](md_pic/install_esp32.png)

- Select Toos/board/Esp32 Dev Modue.

![select_board](md_pic/select_board.png)

- Connect ESP32 to your PC via Micro USB .
- Select Port.
- And at last ,push upload.

## Show On PC

- Once ESP32 is ready, it usually restarts automatically, and the Python window displays the mock up image.
- If there is no restart, you need to press RST to reboot manually.

![pc_show](md_pic/pc_show.png)

