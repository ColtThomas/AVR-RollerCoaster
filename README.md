# AVR-RollerCoaster
IOT project that collects accelerometer and GPS data to send to Flask server via LTE modem. Runs on an AVR-IOT Cellular Mini dev board.

# Dependencies

Below are the Arduino library dependencies that are needed:
* AVR-IOT-Cellular v1.3.1: General dependencies for the dev board
* AVR-IOT veml3328 v1.1.1: Used for dev board's color sensor (powered down for this project)
* AVR-IOT mcp9808 v1.1.3: Used for dev board's temperature sensor (also powered down)
* SparkFun 6DoF ISM330DHCX v1.0.4: Dependencies used for the accelerometer
* ArduinoJson v6.19.4: Data packed into JSON format before sending HTTP post request
* Adafruit GPS Library 1.7.0: Used to interface with Adafruit Ultimate GPS Featherwing

# Resources Used

Below are some useful resources I used to put together this software:
* [Microchip's GPS Tracker tutorial](https://iot.microchip.com/docs/arduino/examples/GPS%20Tracker/Intro) - I used this example to figure out how to utilize the GPS itself and send packages to a flask server that they provide
* [ISM330DHCX Library](https://github.com/sparkfun/SparkFun_6DoF_ISM330DHCX_Arduino_Library)
