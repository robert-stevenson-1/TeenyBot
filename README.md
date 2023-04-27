# TeenyBot - *A Small Wireless enable robot to mess around with!!* <!-- omit from toc -->

![TeensyBot Img](imgs/TeenyBot.jpg)

## Overview

Not much special about the project (for now).
The current aim of the project is to just create a small, simple robot that can be talked to over Wi-Fi to do whatever I want to do  with the robot.

The whole idea is that I can develop this robot and can add any extra feature that want to experiment with.
Essentially this robot (and this repo itself) serves as a small prototyping platform which I can use to scale to a larger robot platform.

## Table of Contents <!-- omit from toc -->

- [Overview](#overview)
- [TODO List](#todo-list)
  - [TODO: Firmware](#todo-firmware)
  - [TODO: Control App](#todo-control-app)
- [Hardware](#hardware)
- [Software](#software)
  - [Microcontroller firmware](#microcontroller-firmware)
  - [Control App](#control-app)

## TODO List

### TODO: Firmware

- [ ] Communication
  - [ ] Serial (Debug and Dev ONLY)
  - [ ] Wi-Fi Communication
  - [ ] BT Communication
- [ ] Robot Control
  - [ ] Forward and Backwards motor control
  - [ ] Turning
    - [ ] Blended turning
    - [ ] Stationary/on-the-spot rotation
- [ ] Close-loop speed control
- [ ] Orientation of the robot (**Hardware Dependant**)
- [ ] robot position (**Hardware Dependant**)
- [ ] Vision (**Hardware Dependant**)
  - [ ] LIDAR(**Hardware Dependant**)
  - [ ] Video (**MASSIVE Hardware Revision needed**)

### TODO: Control App

- [ ] Robot Communication
  - [ ] Wi-Fi Connection to robot
  - [ ] BT Connection to robot
  - [ ] Send commands
  - [ ] Receive data from the robot
- [ ] Robot Control
  - [ ] Drive robot Forward/Backwards
  - [ ] Rotate the robot
- [ ] Live Feedback
  - [ ] Status Monitor
    - [ ] Robot Speed
    - [ ] Direction (**Hardware Dependant**)
    - [ ] Postion (**Hardware Dependant**)
  - [ ] LIDAR(**Hardware Dependant**)
  - [ ] Video (**MASSIVE Hardware Revision needed**)

## Hardware

- [Chassis](https://thepihut.com/products/pololu-zumo-chassis-kit-no-motors?variant=42393113428163)
- [Motors](https://thepihut.com/products/micro-metal-geared-motor-w-encoder-6v-105rpm-150-1?variant=27740942929)
- ESP32 Dev board
  - [These are the ones I used](https://www.amazon.co.uk/dp/B08CCYWZN3/ref=twister_B07Y3VDYSJ?_encoding=UTF8&psc=1)
- [Motor Driver Boards - DRV8833](https://thepihut.com/products/adafruit-drv8833-dc-stepper-motor-driver-breakout-board)
  - [I used these one's](https://www.amazon.co.uk/HALJIA-DRV8833-Channel-Printer-Arduino/dp/B071SJ4T9M/ref=sxts_rp_s_1_0?content-id=amzn1.sym.07198d44-a16f-4503-b71e-3f4c67470a0f%3Aamzn1.sym.07198d44-a16f-4503-b71e-3f4c67470a0f&crid=24HN74SKBRO2I&cv_ct_cx=drv8833&keywords=drv8833&pd_rd_i=B071SJ4T9M&pd_rd_r=eeeaad8c-c59c-40fa-9264-070377be3bc0&pd_rd_w=bDeXo&pd_rd_wg=TLC7a&pf_rd_p=07198d44-a16f-4503-b71e-3f4c67470a0f&pf_rd_r=2JE906Z0MZB1D0X2SRXY&qid=1682629087&sbo=RZvfv%2F%2FHxDF%2BO5021pAnSA%3D%3D&sprefix=drv8833%2Caps%2C90&sr=1-1-1890b328-3a40-4864-baa0-a8eddba1bf6a)
- Power management:
  - Power Switch
  - Battery: Currently just 4xAA Batteries that the chassis holds
    - Plans to swap this with a rechargable solution
  - Charger: TBA
  - Voltage Regulator: The once I use are no longer listed but [these ones](https://www.amazon.co.uk/Yizhet-Efficiency-Regulator-Converter-Adjustable/dp/B0823P6PW6/ref=sr_1_2_sspa?keywords=voltage%2Bregulator&qid=1682629281&sprefix=volatage%2Bre%2Caps%2C101&sr=8-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1) should be fine for now
    - I Plan to change this for smaller option when I get to redesigning the rest of the power management of the robot.

## Software

The project has two software tools, the Microcontroller firmware and the Control app.

### Microcontroller firmware

The Microcontroller is the main brain of the robot and handles the communication with the Control app over a wireless connection.

The code is written using [platformIO](https://platformio.org/) in VSCode use the platformIO [Extention](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide) and project files can be found [here](Firmware/)

#### Libraries used <!-- omit from toc -->

- TBA

### Control App

TODO: Write overview and features once development starts

#### Packages used <!-- omit from toc -->

- TBA
