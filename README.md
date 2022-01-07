# JoesDriveV2
* Shared Code Base for Joe's Drive V2 using AdaFruit Feather CPUs and SND Boards
* This will support a complete PCB delivered solution to simplify and provide cost savings for future development.
* utilizes ESP32 and Feather based CPUs
* Simplified but adaptable Dome Controls
* Land, Sea and Air control schemas supporting Wifi, BT and more...
* This will contain all the code for my Joe's drive V2 setup.
https://github.com/jlvandusen/JoesDriveV2/blob/main/Photos/146278801_4037437846289766_7971786754029637150_n.jpg

## Hardware setup
This drive is everything including the kitchen sink, remote starter, easy to configure ESP32 to Move or Xbox controllers (no other hardware needed), Single PCB board with all needed connections


### ESP32 Huzzah
The ESP32 Huzzah Feather is the main brains of the drive, and it is responsible for handling the remote functions and reading the IMU sub-board from the serial terminals.

### 32u4 Proto M0
Currently utilizes a second Feather 32u4 M0 for offloading remaining serial ports for Dome specific controls over serial.


### trinket Pro Mini - IMU reader
IMU locks are the thing of the past

## New Features
The following are the new features that I have added into my drive to make it more usable.
Feature | Description
------- | -----------
v2 Flywheel | Very low profile (higher than the battery compartment - thus allowing for out of ball testing and weight disbursement.
Mp3 Qwiic/i2c trigger | Code that drives the custom Mp3 trigger for full sound controls via multi combinations
