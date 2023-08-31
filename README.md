# Final Degree Project about Embedded Systems development with MCU ESP32

## Note from the Author

Hi everyone!!

This is my Final Degree Project of Electronic Industry and Automatic Engineering at University of Almería (Spain).

This project is based on MQTT and TCP/IP connection to a online server, reading and sending several sensors' data like temperature, pressure or humidity...

These sensors are: MS5611, BME680, and CCS811.

I have programmed a ESP32-CAM too!! You can see the C main project code in my profile!

You can see the project of each sensor in my Github profile!

Year:		2021-2022

Email:		f.martinvillegas00@gmail.com

## How to use example

### Hardware Required

This project can be execute in every ESP32.

It needs a MS5611, BME680 and CCS811 sensor.

### Configure the project

* Open the project configuration menu (`idf.py menuconfig`) with ESP-IDF terminal.
* Configure Wi-Fi or Ethernet under "Example Connection Configuration" menu. Type here your SSID and your password.
* Go then to "Example Configuration" and type your MQTT broker URL.
* When using Make build system, set `Default serial port` under `Serial flasher config`.
* In the same ESP-IDF terminal type: (`idf.py -p COMx flash monitor`) to execute this project.
* Only enjoy it ;-) !!

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.
