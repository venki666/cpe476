## RebelBot

We will modify the Pololu's Romi Chasis to our RebelBot.

### Components: 

1. [#3502 Romi Chassis Kit - Red (color of your choice) (Links to an external site.)](https://www.pololu.com/product/3502)
2. [#3542 Romi Encoder Pair Kit, 12 CPR, 3.5-18V (Links to an external site.)](https://www.pololu.com/product/3542)
3. [#3544 Romi 32U4 Control Board (Links to an external site.)](https://www.pololu.com/product/3544)
4. [#966 0.100" (2.54 mm) Breakaway Male Header: 2×40-Pin, Straight (Links to an external site.)](https://www.pololu.com/product/966)x 2Nos
5. [#965 0.100" (2.54 mm) Breakaway Male Header: 1×40-Pin (Links to an external site.)](https://www.pololu.com/product/965)
6. [Stackable 0.100″ Female Header with Extra 0.3″ Spacer: 2x20-Pin (Links to an external site.)](https://www.pololu.com/product/2749) x 2Nos
7. [#3560 Romi Chassis Expansion Plate - Black (Links to an external site.)](https://www.pololu.com/product/3560)x 2 Nos (optional)
8. [#1944 Aluminum Standoff: 1" Length, 2-56 Thread, M-F (4-Pack) x (Links to an external site.)](https://www.pololu.com/product/1944)2 Nos(optional)
9. [#1067 Machine Hex Nut: #2-56 (25-pack) (Links to an external site.)](https://www.pololu.com/product/1067)- (optional)
10. [#1955 Machine Screw: #2-56, 1/4″ Length, Phillips (25-pack) (Links to an external site.)](https://www.pololu.com/product/1955)- (optional)
11. [2800mAh High Capacity Batteries 1.2V NiMH (Links to an external site.)](https://www.amazon.com/POWEROWL-Rechargeable-Batteries-Capacity-Discharge/dp/B07M7JHBNC)
12. [HiQuick Rechargeable AA Battery Charger (Links to an external site.)](https://www.amazon.com/dp/B08C791XND)
13. [Raspberry Pi Camera Module (Links to an external site.)](https://www.amazon.com/seeed-studio-Raspberry-Megapixel-Compatible/dp/B07Z76QHBY)
14. [Raspberry Pi 4 Computer Model B 8GB (Links to an external site.)](https://www.amazon.com/Raspberry-Computer-Suitable-Building-Workstation/dp/B0899VXM8F/)
15. [SanDisk 32GB Extreme microSDHC UHS-I Memory Card (Links to an external site.)](https://www.amazon.com/dp/B06XWMQ81P)
16. [Raspberry Pi 4 Accessories Pack](https://www.amazon.com/Miuzei-Raspberry-HDMI-Micro-Aluminum-Included/dp/B07VX2WDHM)

### [Assembling the Romi Chassis](https://www.pololu.com/docs/0J68/4)

Assemble the Romi Chassis with the components using instructions in [Assembling the Romi Chassis](https://www.pololu.com/docs/0J68/4)

### Review the Romi 32U4 Control Board User Guide

User’s manual for the Pololu Romi 32U4 Control Board @ [Pololu Romi 32U4 Control Board User’s Guide](https://www.pololu.com/docs/0J69) 

### Review the Romi 32U4 Control Schematic

[Schematic diagram of the Romi 32U4 Control Board](https://www.pololu.com/file/0J1258/romi-32u4-control-board-schematic-diagram.pdf) 

### Install Arduino IDE (latest) in Linux (host system)

Arduino integrated development environment (IDE) software @ [Arduino Software](http://arduino.cc/en/Main/Software). Start a clean install. Do not us the App version, use the zip version to install.

### Install Arduino Support Libraries

1. Romi32U4 library - Use the one provided in the class github.
2.  Arduino library for interfacing with the LSM6DS33 accelerometer and gyro - [LSM6 library for Arduino](https://github.com/pololu/lsm6-arduino)
3. PID Library - [TimedPID](https://www.arduino.cc/reference/en/libraries/timedpid/)
4. Alternate PID -  [PID_v1](https://github.com/br3ttb/Arduino-PID-Library/)
5. Timer Library - 

### Test your Romi Control Board

Using built-in example codes test the Romi Controller Board - In Class demo 

