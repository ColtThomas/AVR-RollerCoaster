#include <Adafruit_GPS.h>
#include <Arduino.h>
#include <http_client.h>
#include <led_ctrl.h>
#include <log.h>
#include <low_power.h>
#include <lte.h>
#include <mcp9808.h>
#include <veml3328.h>

// Accelerometer dependencies
#include "SparkFun_ISM330DHCX.h"
#include <Wire.h>
#include <ArduinoJson.h>

#define GPSSerial Serial2
#define HTTP_DOMAIN "xx.xxx.xxx.xxx"
#undef printf // Allows you to print floats

#define START_SAMPLING_FLAG (1 << 0)
#define SAMPLE_PERIOD 100 // milliseconds
#define GFORCE_DEFAULT_VALUE 1000
static volatile uint16_t event_flags = 0;

/**
 * @brief Connected refers to the LTE network.
 */
enum class State { NOT_CONNECTED, CONNECTED, CONNECTED_WITH_FIX,STANDBY,RECONNECT,UPDATE_GPS,SAMPLE };

/**
 * @brief Interface for the GPS.
 */
Adafruit_GPS GPS(&GPSSerial);
SparkFun_ISM330DHCX myISM; 

/**
 * @brief Due to no float string support, these variables are used with dtostrf
 * such that the float values for latitude and longitude are properly
 * converted before being sent to the server.
 */
static char latitude[16]  = "0";
static char longitude[16] = "0";
static char time[24]      = "0";

static sfe_ism_data_t accelData; 
static sfe_ism_data_t gyroData; 
static struct SamplePoint {
  char accelX[11];
  char accelY[11];
  char accelZ[11];
  char gyroX[11];
  char gyroY[11];
  char gyroZ[11];
} sample;

static struct Statistics {
  char gMax[11];
  char gMin[11];
  char vMax[11];
} stats;

StaticJsonDocument<512> doc;

// Live variable tracking
static double gForceMax = 1000.0;
static char  gForceMaxF[11]="";
static double gForceMin = 10000.0; // Set really high to initialize properly
static char  gForceMinF[11] ="";
static double deltaVelocity=0.0;
static double currVelocity=0.0;
static double maxVelocity=0.0;
static double netAccelPrior=0.0;

// What else do you want to collect?
// freefall duration (air time)
// max speed
// average speed

/**
 * @brief Keeps track of the state.
 */
static State state = State::NOT_CONNECTED;

/**
 * @brief Whether or not we've parsed one GPS entry. Prevents sending zeros
 * whilst having fix after boot.
 */
static bool has_parsed = false;

/**
 * @brief Starts the GPS modem and sets up the configuration.
 */
static void initializeGPS(void) {

    GPSSerial.swap(1);

    GPS.begin(9600);

    // Enable RMC & GGA output messages
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    // Set the update rate to 1Hz
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

    // Disable updates on antenna status
    GPS.sendCommand(PGCMD_NOANTENNA);

    delay(1000);

    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);
}

/**
 * @brief Sets up the HTTP client with the given domain
 */
static void initializeHTTP(void) {
    if (!HttpClient.configure(HTTP_DOMAIN, 8080, false)) {
        Log.info("Failed to configure HTTP client");
    } else {
        Log.info("Configured HTTP");
    }
}

static void prepareStats(void) {
  doc["type"] = "stat";
  doc["lat"] = latitude;
  doc["lon"] = longitude;
  doc["time"] = time;
  doc["gMax"] = stats.gMax;
  doc["gMin"] = stats.gMin;
  doc["vMax"] = maxVelocity;
}
/**
 * @brief Sends a payload with latitude, longitude and the timestamp.
 */
static void sendData(void) {

    Log.info("Sending Data...");
    char data[512] = "";

    prepareStats();

    serializeJson(doc,data);

#ifdef DEBUG_MODE
    sprintf(data,
            "{\"lat\":\"%s\",\"lon\":\"%s\",\"time\": \"%s\"}",
            latitude,
            longitude,
            time);
#endif

    HttpResponse response = HttpClient.post("/data", data);

    Log.infof("POST - status code: %u, data size: %u\r\n",
              response.status_code,
              response.data_size);

    if (response.status_code != 0) {
        String body = HttpClient.readBody(512);

        if (body != "") {
            Log.infof("Response: %s\r\n", body.c_str());
        }
    }
}

/**
 * @brief Connects to the network operator. Will block until connection is
 * achieved.
 */
static void connectToNetwork() {

    // If we already are connected, don't do anything
    if (!Lte.isConnected()) {
        while (!Lte.begin()) {}
        Log.infof("Connected to operator: %s\r\n", Lte.getOperator().c_str());
    }

    state = State::CONNECTED;
}

/**
 * @brief Checks for new GPS messages and parses the NMEA messages if any.
 */
static void parseGPSMessages() {

    // Read the incoming messages, needn't do anything with them yet as that is
    // taken care of by the newNMEAReceived() function.
    if (GPS.available()) {
        GPS.read();
    }

    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) {
            // If we fail to parse the NMEA, wait for the next one
            return;
        } else {

            Log.rawf("Timestamp: %d/%d/20%d %d:%d:%d GMT+0 \r\n",
                     GPS.day,
                     GPS.month,
                     GPS.year,
                     GPS.hour,
                     GPS.minute,
                     GPS.seconds);

            Log.rawf("Fix: %d, quality: %d\r\n", GPS.fix, GPS.fixquality);

            if (GPS.fix) {

                // Need to convert all floats to strings
                dtostrf(GPS.latitudeDegrees, 2, 4, latitude);
                dtostrf(GPS.longitudeDegrees, 2, 4, longitude);

                sprintf(time,
                        "%d/%d/20%d %d:%d:%d",
                        GPS.day,
                        GPS.month,
                        GPS.year,
                        GPS.hour,
                        GPS.minute,
                        GPS.seconds);

                Log.rawf("Location: %s N, %s E\r\n", latitude, longitude);
                Log.rawf("Satellites: %d\r\n", GPS.satellites);

                Log.rawf("\r\n");

                has_parsed = true;
            } else {
                Log.info("Waiting for GPS fix...");
            }
        }
    }
}

static void initializeAccel() {
  // Init the accelerometer/gyroscope
    if( !myISM.begin(Wire1) ){
      Log.info("Unable to connect to ISM330DHCX.");
      LedCtrl.on(Led::ERROR);
      while(1); // Just reset the board at this point and verify wiring
      
    }

    // Reset the device to default settings. This if helpful is you're doing multiple
    // uploads testing different settings. 
    myISM.deviceReset();

    // Wait for it to finish reseting
    while( !myISM.getDeviceReset() ){ 
      delay(1);
    } 

    Log.info("Reset.");
    Log.info("Applying settings.");
    delay(100);
    
    myISM.setDeviceConfig();
    myISM.setBlockDataUpdate();
    
    // Set the output data rate and precision of the accelerometer
    myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
    myISM.setAccelFullScale(ISM_8g); 

    // Set the output data rate and precision of the gyroscope
    myISM.setGyroDataRate(ISM_GY_ODR_104Hz);
    myISM.setGyroFullScale(ISM_500dps); 

    // Turn on the accelerometer's filter and apply settings. 
    myISM.setAccelFilterLP2();
    myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

    // Turn on the gyroscope's filter and apply settings. 
    myISM.setGyroFilterLP1();
    myISM.setGyroLP1Bandwidth(ISM_MEDIUM);  
}

void buttonInterrupt(void) {
    if (PORTD.INTFLAGS & PIN2_bm) {
        event_flags |= START_SAMPLING_FLAG;
        PORTD.INTFLAGS = PIN2_bm;
    }
}

void setup() {
    LedCtrl.begin();
    LedCtrl.startupCycle();

    Log.begin(115200);
    Log.info("Starting AVR-IoT Cellular Adafruit GPS example");

    // We configure the low power module for power down configuration, where
    // the LTE modem and the CPU will be powered down
    LowPower.configurePowerDown();

    // Make sure sensors are turned off
    Veml3328.begin();
    Mcp9808.begin();
    Veml3328.shutdown();
    Mcp9808.shutdown();

    initializeGPS();
    initializeHTTP();
    initializeAccel();  

    // SW0 Button
    pinConfigure(PIN_PD2, PIN_DIR_INPUT);
    attachInterrupt(PIN_PD2, buttonInterrupt, FALLING);

    // TODO: disconnect reset switch
    // pinConfigure(PIN_PF6, PIN_DIR_INPUT);
    // attachInterrupt(PIN_PF6, buttonInterrupt, FALLING);

}

void loop() {
    // Sample state only concerned with Accelerometer values
    if(state != State::SAMPLE) {
      parseGPSMessages();
    }
    

    switch (state) {
    // First connect to LTE
    case State::NOT_CONNECTED:
        connectToNetwork();
        break;

    // Next get a GPS fix
    case State::CONNECTED:
        if (!Lte.isConnected()) {
            state = State::NOT_CONNECTED;
        } else if (GPS.fix) {
            state = State::CONNECTED_WITH_FIX;

            // Decrease update rate once we have fix to save power
            GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
        }

        break;

    // Verify the accelerometer is ready
    case State::CONNECTED_WITH_FIX:
      if (!Lte.isConnected()) {
        state = State::NOT_CONNECTED;
      }
      else if(!GPS.fix) {
        state = State::CONNECTED;

        // Lost fix, set update rate back to 1 Hz
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
      } 
      else if(myISM.checkStatus() && Lte.isConnected() && GPS.fix) {
        LedCtrl.on(Led::CON);
        Log.info("All peripherals up");

        state = State::STANDBY;        
      }
      break;
    case State::STANDBY:
      if (!Lte.isConnected()) {
        state = State::RECONNECT;
        Log.info("Lost LTE connection. Reconnecting to network...");
        LedCtrl.off(Led::CON);
        break;
      }

      if(!GPS.fix) {
        Log.info("Lost GPS fix. Updating...");
        LedCtrl.off(Led::CON);

        // Lost fix, set update rate back to 1 Hz
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
      }      
      
      if(event_flags & START_SAMPLING_FLAG) {
        state = State::SAMPLE;
        Log.info("Sampling...");
        LedCtrl.on(Led::DATA);
        event_flags = 0;
      }
      break;

    case State::RECONNECT:
      connectToNetwork();
      LedCtrl.on(Led::CON);
      Log.info("LTE network reconnected.");
      state = State::STANDBY; 
      break;
    case State::UPDATE_GPS:
      if (GPS.fix) {
          state = State::STANDBY;
          LedCtrl.on(Led::CON);
          Log.info("GPS Fix reaquired.");

          // Decrease update rate once we have fix to save power
          GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
      }
      break;
    case State::SAMPLE:
      myISM.getAccel(&accelData);
      myISM.getGyro(&gyroData);

      // CALCULATION NOTES:
      /*
        The accelerometer will read a value of 1000.000 for 1 g of acceleration. This is useful for indicating orientation of the accelerometer.
        All other axis will read zero if horizontal to the ground. 
      */
      double tmp = 0;
      
      // tmp = abs(accelData.xData)+ abs(accelData.yData) + abs(accelData.zData); // incorrect
      tmp = sqrt(accelData.xData*accelData.xData + accelData.yData*accelData.yData+accelData.zData*accelData.zData); // Calculate the norm
      
      // Track records
      if(tmp > gForceMax) {
        gForceMax = tmp; 
        dtostrf(gForceMax,10,3,gForceMaxF);        
      }
      if(tmp < gForceMin) {
        gForceMin = tmp;
        dtostrf(gForceMin,10,3,gForceMinF);
      }
      
#ifdef DEBUG_MODE
      dtostrf(accelData.xData,10,3,sample.accelX);
      dtostrf(accelData.yData,10,3,sample.accelY);
      dtostrf(accelData.zData,10,3,sample.accelZ);
      dtostrf(gyroData.xData,10,3,sample.gyroX);
      dtostrf(gyroData.yData,10,3,sample.gyroY);
      dtostrf(gyroData.zData,10,3,sample.gyroZ);
      Log.info("Data samples:");
      Log.rawf("Accel: %s, %s, %s\r\n",sample.accelX,sample.accelY,sample.accelZ);
      Log.rawf("Gyro: %s, %s, %s\r\n",sample.gyroX, sample.gyroY, sample.gyroZ);

      Log.info("Records:");
      Log.rawf("Max %s\r\n",gForceMaxF);
      Log.rawf("Min %s\r\n",gForceMinF);
      Log.rawf("Temp %d\r\n",(int)tmp);
      Log.rawf("Max Int %d\r\n",(int)gForceMax);
      Log.rawf("Min Int %d\r\n",(int)gForceMin);
#endif
      // Velocity calculations - Given two data points we can use the change in acceleration
      // to derive the change in velocity as follows:
      // (delta accel * 9.81 m/s^2)/ delta seconds
      deltaVelocity = (tmp-1000.0) - (netAccelPrior-1000.0); // Get rid of acceleration due to gravity
      // Log.rawf("Velocity: %d\r\n",(int)deltaVelocity);
      // deltaVelocity = deltaVelocity/1000.0; // Convert from mg to g
      // Log.rawf("Velocity: %d\r\n",(int)deltaVelocity);
      // deltaVelocity = deltaVelocity*9.81; // convert from g to m/s^2
      // Log.rawf("Velocity: %d\r\n",(int)deltaVelocity);
      // deltaVelocity = deltaVelocity*SAMPLE_PERIOD/1000.0; // multiply by the sample period in seconds 
      // deltaVelocity = deltaVelocity/1000.0; // Convert from mg to g
      // Log.rawf("Velocity: %d\r\n",(int)deltaVelocity);
      // deltaVelocity = deltaVelocity + ((tmp - netAccelPrior)*9.81)/(SAMPLE_PERIOD/1000.0);
      // currVelocity = currVelocity + deltaVelocity;

      // Log.rawf("%d\r\n",currVelocity);
      netAccelPrior = tmp; // Save value for next calculation

      if(deltaVelocity > maxVelocity) {
        maxVelocity = deltaVelocity;
      }

      
      delay(SAMPLE_PERIOD);

      if(event_flags & START_SAMPLING_FLAG) {
        state = State::STANDBY;
        LedCtrl.off(Led::DATA);
        event_flags = 0;

        // save off stats
        dtostrf(gForceMax,10,3,stats.gMax);
        dtostrf(gForceMin,10,3,stats.gMin);

        sendData();

        // Reset stats for the next round
        gForceMax = 1000.0;
        gForceMin = 10000.0;
        deltaVelocity=0.0;
        netAccelPrior=0.0;
      }
      
      break;
    }
}
