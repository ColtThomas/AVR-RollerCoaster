# AVR-RollerCoaster
IOT project that collects accelerometer and GPS data to send to Flask server via LTE modem. Runs on an AVR-IOT Cellular Mini dev board.

# Hardware Used

* [Adafruit Ultimate GPS Featherwing](https://learn.adafruit.com/adafruit-ultimate-gps-featherwing) - I recommend using a coin cell battery for faster GPS lock
* [SparkFun 9DoF IMU Breakout - ISM330DHCX, MMC5983MA](https://www.sparkfun.com/products/19895) - NOTE: This includes both an accelerometer/gyroscope and magnetometer. You could alternatively use just the [6DoF IMU breakout board](https://www.sparkfun.com/products/19764) that only has the accelerometer/gyroscope.
* 32db High Gain Cirocomm 5cm Active GPS Antenna Ceramic Antenna. The model I used is an older model (510). [Here's a link to a newer model](http://www.cirocomm.com/en-global/products_ciro/detail/GBA-154C) This was helpful for acquiring a fast GPS lock. Without the active antenna and no cell battery, it took about 15-30 minutes to acquire a lock every powercycle. With this antenna and a battery it was almost instantaneous.

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
