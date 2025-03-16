This software is used as a tracker for Rockets, or other.
The basestation is the component you have that shows you how far away your asset is.
It runs on a Adafruit Feather M0 RFM98 LoRa board with an attached GPS and compass.

V1.0 original version of LoRa GPS basestation

v1.5 - bug fixes through v1.35 plus changes to add up to 15 channels

V2.0 - Added support for SD cards for data logging and storage of Last Know Good GPS data. Support for 1 wire EEPROM to store Last Known Good received GPS message. Allows changing the transmitter channel wirelessly. Scanning of channels to see which are in use. Calibrating the compass module. Large arrow support for tracking screen. Display of Maximum speed. Display of Maximum altitude. Display of battery status.

lora_gps_v2.ino is the file for the tracker. That runs on a Adafruit Feather 32u4 or equivalent with an attached GPS.
