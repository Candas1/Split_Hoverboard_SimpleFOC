# Split Hoverboards with C++ SimpleFOC :-)
## this is just the beginning.. 
HallSensor class working and three led blinking while rotating the (slave) wheel.

#### This PlatformIO code is based on [Sideboard-Arduino](https://github.com/Candas1/Sideboard-Arduino) from Candas1 :-)

#### for choosing your hardware layout look here: [Hoverboard-Firmware-Hack-Gen2.x](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x)
/include/config.h :
```
// LAYOUT_2_X is used in defines.h
//#define LAYOUT_2_0	// https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2
//#define LAYOUT_2_1	// https://github.com/krisstakos/Hoverboard-Firmware-Hack-Gen2.1
//#define LAYOUT_2_2	// 2023/05/11 only MASTER and TEST_SPEED: motor is spinning but needs a push to startup :-/
//#define LAYOUT_2_4	// NOT READY !!! https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/3
#define LAYOUT_2_5	// NOT READY !!! https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/11
```

#### Big thanks to [ArduinoCore-GD32](https://github.com/CommunityGD32Cores/ArduinoCore-GD32) for making it possible to build a firmware based on [simplefoc](https://github.com/simplefoc) :-)

### How to use
* Install [Vscode](https://code.visualstudio.com/download)
* Install [Platformio](https://platformio.org/platformio-ide) from the website or from VSCODE's extensions
* Download the project or use git to clone this repository in VSCODE
* Select the GD32F103C8 or GD32F103C6 environment that matches with your chip in platformio's bottom bar
*  [Unlock](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/How-to-Unlock-MCU-Flash) the chip
* Press the 'PlatformIO:Upload' button (bottom left in vscode) to build and upload the firmwware

