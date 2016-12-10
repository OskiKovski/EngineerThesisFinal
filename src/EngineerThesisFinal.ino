#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <HMC5883L.h>
#include <QueueList.h>

#ifndef UNIT_TEST
#define DEBUG false

SoftwareSerial gpsSerial(2, 3); //tx, rx
SoftwareSerial wifiSerial(8, 9); // RX, TX

TinyGPS gps;
HMC5883L compass;

bool hasIP = false;

uint32_t baud = 9600;

const int leftMotorPinNumber = 4;
const int frontMotorPinNumber = 5;
const int rightMotorPinNumber = 6;
const int backMotorPinNumber = 7;

const int powerDiodePinNumber = 10;
const int wifiDiodePinNumber = 11;

float currentLatitude;
float currentLongitude;
float targetLatitude;
float targetLongitude;
float currentDestinationAngle;

QueueList<float> targetLatitudes;
QueueList<float> targetLongitudes;

String IP = "";
String port = "3333";

bool hasIPAddress(uint32_t);
void displayIPOnSerialMonitor();
int sendUDPCommandToWifi(String, const int, bool);
int connectWithWiFi(bool);
void getCompassReady(bool);
void getGpsData(bool, uint32_t);
void showCoordinate();
void getFirstDestinationCoordinatesFromWifi(bool);
void listenForNewDestinationCoordinatesOrNavigationCancelation(uint32_t, bool);
float getCurrentHeading(bool);
float getCurrentCourseAngle(float, float, float, float, bool);
void vibrateOneMotorOnce(int);
void vibrateAllMotorsOnceForGivenTime(uint32_t);
void vibrateTwoMotorsOnce(int, int);
void vibrateMotorsForFinish();
void vibrateTheProperDirectionMotorOnce(float);

void setup() {
  pinMode(powerDiodePinNumber, OUTPUT);
  pinMode(wifiDiodePinNumber, OUTPUT);
  pinMode(leftMotorPinNumber, OUTPUT);
  pinMode(frontMotorPinNumber, OUTPUT);
  pinMode(rightMotorPinNumber, OUTPUT);
  pinMode(backMotorPinNumber, OUTPUT);

  if (DEBUG) {
    Serial.begin(baud);
    Serial.print("SETUP!! @");
    Serial.println(baud);
  }

  digitalWrite(powerDiodePinNumber, HIGH);
  vibrateOneMotorOnce(frontMotorPinNumber);
  vibrateOneMotorOnce(leftMotorPinNumber);
  vibrateOneMotorOnce(rightMotorPinNumber);
  vibrateOneMotorOnce(backMotorPinNumber);

  wifiSerial.begin(baud);
  connectWithWiFi(DEBUG);
  gpsSerial.begin(baud);
  if (DEBUG) {
    Serial.println("GPS is Ready");
  }
  delay(1000);
  if (DEBUG) {
    Serial.println("System Ready..");
  }
  getCompassReady(DEBUG);
}

void loop() {
  getGpsData(DEBUG, 1000);
  getFirstDestinationCoordinatesFromWifi(DEBUG);
  targetLatitude = targetLatitudes.pop();
  targetLongitude = targetLongitudes.pop();
  while(true) {
    getGpsData(DEBUG, 1000);
    float distanceToTarget = gps.distance_between(currentLatitude,currentLongitude,targetLatitude,targetLongitude);
    if(DEBUG) {
      Serial.print("DISTANCE IN METERS=");
      Serial.println(distanceToTarget);
    }
    if(distanceToTarget <= 5) {
      if(targetLatitudes.isEmpty() && targetLongitudes.isEmpty()) {
        vibrateMotorsForFinish();
        break;
      }
      targetLatitude = targetLatitudes.pop();
      targetLongitude = targetLongitudes.pop();
    }
    currentDestinationAngle = getCurrentCourseAngle(currentLatitude, currentLongitude, targetLatitude,
                                                    targetLongitude, DEBUG);
    vibrateTheProperDirectionMotorOnce(currentDestinationAngle);
    listenForNewDestinationCoordinatesOrNavigationCancelation(10000, DEBUG);
  }
}

/**
  * Checks if connected ESP8266 got IP address
  * @param samplingTime time in which Arduino checks WiFi state
  * @return boolean value of IP existence
  */
bool hasIPAddress(uint32_t samplingTime) {
  uint32_t samplingStart = millis();
  while (samplingStart + samplingTime > millis()) {
    while (wifiSerial.available() > 0) {
      if (wifiSerial.find((char *) "WIFI GOT IP")) {
        return true;
      }
    }
  }
  digitalWrite(wifiDiodePinNumber, LOW);
  return false;
}

/**
  * Displays ESP8266 IP address and port on computer's serial monitor
  */
void displayIPOnSerialMonitor() {
  IP = "";
  char ch = 0;
  while (1) {
    wifiSerial.println("AT+CIFSR");
    while (wifiSerial.available() > 0) {
      if (wifiSerial.find((char *) "STAIP,")) {
        delay(1000);
        Serial.print("IP Address:");
        while (wifiSerial.available() > 0) {
          ch = wifiSerial.read();
          if (ch == '+') break;
          IP += ch;
        }
      }
      if (ch == '+') break;
    }
    if (ch == '+') break;
    delay(1000);
  }
  Serial.print(IP);
  Serial.print("Port:");
  Serial.println(port);
  delay(1000);
}

/**
  * Sends an UDP message to ESP8266 in order to control this module. Sends the command 5 times.
  * @param command data desired to send to ESP
  * @param timeout time given to ESP to retrieve answer
  * @param debug print to Serial window?(true = yes, false = no)
  * @return 0 if everything is fine, 1 when after 5 attempts ESP gives nothing
  */
int sendUDPCommandToWifi(String command, const int timeout, bool debug) {
  int sendingAttempts = 0;
  if (debug) {
    Serial.println(command);
  }
  while (sendingAttempts <= 5) {
    wifiSerial.println(command);
    while (wifiSerial.available() > 0) {
      if (wifiSerial.find((char *) "OK")) {
        sendingAttempts = 7;
      }
    }
    delay(timeout);
    sendingAttempts++;
  }
  if (sendingAttempts == 8) {
    if (debug) {
      Serial.println("OK");
    }
    delay(1000);
    return 0;
  } else {
    if (debug) {
      Serial.println("Error");
    }
    delay(1000);
    return 1;
  }
}

/**
  * Connects ESP8266 with WiFi
  * @param debug print to Serial window?(true = yes, false = no)
  * @return 0 when connected, 1 when not
  */
int connectWithWiFi(bool debug) {
  wifiSerial.begin(baud);
  if (sendUDPCommandToWifi("AT", 1000, DEBUG)) {
    if (debug) {
      Serial.println("Wifi probably turned off, aborting");
    }
    return 1;
  }
  sendUDPCommandToWifi("AT+RST", 5000, DEBUG);
  sendUDPCommandToWifi("AT+CWQAP", 1000, DEBUG);
  sendUDPCommandToWifi("AT+CWMODE=1", 1000, DEBUG);
  hasIP = hasIPAddress(5000);
  if (!hasIP) {
    Serial.println("Connecting WiFi....");
    if (!sendUDPCommandToWifi("AT+CWJAP=\"EngineerThesisTest\",\"TestPassword\"", 7000, DEBUG)) {
      if (debug) {
        Serial.println("WiFi Connected");
      }
    } else {
      if (debug) {
        Serial.println("WiFi probably turned off, aborting");
      }
      wifiSerial.end();
      return 1;
    }
  }
  digitalWrite(wifiDiodePinNumber, HIGH);
  if (debug) {
    displayIPOnSerialMonitor();
  }
  delay(2000);
  sendUDPCommandToWifi("AT+CIPMUX=1", 100, DEBUG);
  sendUDPCommandToWifi("AT+CIPSERVER=1," + port, 100, DEBUG);
  return 0;
}

/**
  * Gets magnetometer device (used here as compass) ready to work
  * @param debug print to Serial window?(true = yes, false = no)
  */
void getCompassReady(bool debug) {
  while (!compass.begin()) {
    if (debug) {
      Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    }
    delay(500);
  }
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
}

/**
  * Reads data acquired by GPS module
  * @param debug print to Serial window?(true = yes, false = no)
  * @param samplingTime time in which device listens to coming data
  * @return 0 when there is some correct data, 0 when not
  */
void getGpsData(bool debug, uint32_t samplingTime) {
  gpsSerial.begin(baud);
  uint32_t samplingStart = millis();
  while (samplingStart + samplingTime > millis()) {
    while (gpsSerial.available()) {
      int c = gpsSerial.read();
      if (gps.encode(c)) {
        unsigned long fix_age;
        gps.f_get_position(&currentLatitude, &currentLongitude, &fix_age);
        if (debug) {
          showCoordinate();
        }
      }
    }
  }
}

/**
  * Shows current (read from GPS module) coordinates on serial monitor
  */
void showCoordinate() {
  Serial.print("Latitude:");
  Serial.println(currentLatitude);
  Serial.print("Longitude:");
  Serial.println(currentLongitude);
}

/**
  * Blocking function that retrieves parsed data from Wifi and aligns its value to params
  * @param debug print to Serial window?(true = yes, false = no)
  */
void getFirstDestinationCoordinatesFromWifi(bool debug) {
  char c;
  String temp;
  wifiSerial.begin(baud);
  while (targetLatitudes.isEmpty() && targetLongitudes.isEmpty()) {
    while(wifiSerial.find((char *) "+IPD,")) {
      delay(1000);
      wifiSerial.find((char *) "lat/lng: (");
      c = wifiSerial.read();
      while (c != ',') {
        temp += c;
        c = wifiSerial.read();
      }
      targetLatitudes.push(temp.toFloat());
      if (debug) {
        Serial.println(temp.toFloat());
      }
      c = wifiSerial.read();
      temp = "";
      while (c != ')') {
        temp += c;
        c = wifiSerial.read();
      }
      targetLongitudes.push(temp.toFloat());
      if (debug) {
        Serial.println(temp.toFloat());
      }
    }
  }
}

/**
  * Listens for data from Wifi, parses it and aligns its value to params
  * @param samplingTime time in which device listens to coming data
  * @param debug print to Serial window?(true = yes, false = no)
  */
void listenForNewDestinationCoordinatesOrNavigationCancelation(uint32_t samplingTime, bool debug) {
  char c;
  String temp;
  wifiSerial.begin(baud);
  uint32_t samplingStart = millis();
  while (samplingStart + samplingTime > millis()) {
    while (wifiSerial.find((char *) "+IPD,")) {
      delay(1000);
      while (wifiSerial.find((char *) "lat/lng: (")) {
        c = wifiSerial.read();
        while (c != ',') {
          temp += c;
          c = wifiSerial.read();
        }
        targetLatitudes.push(temp.toFloat());
        if (debug) {
          Serial.println(temp);
        }
        c = wifiSerial.read();
        temp = "";
        while (c != ')') {
          temp += c;
          c = wifiSerial.read();
        }
        targetLongitudes.push(temp.toFloat());
        if (debug) {
          Serial.println(temp);
        }
      }
    }
    if(wifiSerial.find((char *) "CANCEL")) {
      while(!targetLongitudes.isEmpty() && !targetLatitudes.isEmpty()) {
        targetLatitudes.pop();
        targetLongitudes.pop();
      }
    }
  }
}

/**
  * Gets angle which device heads currently
  * @return angle between NS line and user's sight line in degrees
  * @param debug print to Serial window?(true = yes, false = no)
  */
float getCurrentHeading(bool debug) {
  Vector norm = compass.readNormalize();
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Warsaw / Poland declination angle is 5'41E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (5.0 + (41.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }
  // Convert to degrees
  float headingDegrees = heading * 180 / M_PI;

  if (debug) {
    Serial.print("YOUR CURRENT HEADING:");
    Serial.println(headingDegrees);
  }

  return headingDegrees;
}

/**
  * Gets the angle in which user should go to reach the destination point
  * @param currentLatitude Y of current point
  * @param currentLongitude X of current point
  * @param targetLatitude Y of target point
  * @param targetLongitude X of target point
  * @param debug print to Serial window?(true = yes, false = no)
  * @return angle of going direction in degrees
  */
float getCurrentCourseAngle(float currentLatitude, float currentLongitude, float targetLatitude, float targetLongitude,
                            bool debug) {
  float currentHeading = getCurrentHeading(debug);
  float currentBearing = gps.course_to(currentLatitude,currentLongitude,targetLatitude,targetLongitude);
  if(debug) {
    Serial.print("CURRENT BEARING (TO DESTINATION)=");
    Serial.println(currentBearing);
  }
  float currentCourseAngle = currentBearing - currentHeading;
  if (currentCourseAngle < 0) {
    currentCourseAngle += 360;
  }
  if (debug) {
    Serial.print("CURRENT COURSE ANGLE:");
    Serial.println(currentCourseAngle);
  }
  return currentCourseAngle;
}

/**
  * Vibrates one motor with indicated number
  * @param motorNumber number of motor which will vibrate
  */
void vibrateOneMotorOnce(int motorNumber) {
  digitalWrite(motorNumber, HIGH);
  delay(1000);
  digitalWrite(motorNumber, LOW);
}

/**
  * Vibrates all motors for given time
  * @param vibratingTime time in which all motors vibrates
  */
void vibrateAllMotorsOnceForGivenTime(uint32_t vibratingTime) {
  digitalWrite(frontMotorPinNumber, HIGH);
  digitalWrite(leftMotorPinNumber, HIGH);
  digitalWrite(rightMotorPinNumber, HIGH);
  digitalWrite(backMotorPinNumber, HIGH);
  delay(vibratingTime);
  digitalWrite(frontMotorPinNumber, LOW);
  digitalWrite(leftMotorPinNumber, LOW);
  digitalWrite(rightMotorPinNumber, LOW);
  digitalWrite(backMotorPinNumber, LOW);
}

/**
  * Vibrates two motors with indicated numbers
  * @param firstMotorNumber number of motor which will vibrate
  * @param secondMotorNumber number of motor which will vibrate
  */
void vibrateTwoMotorsOnce(int firstMotorNumber, int secondMotorNumber) {
  digitalWrite(firstMotorNumber, HIGH);
  digitalWrite(secondMotorNumber, HIGH);
  delay(1000);
  digitalWrite(firstMotorNumber, LOW);
  digitalWrite(secondMotorNumber, LOW);
}

/**
  * Vibrates end sequence (the same as football refree's whistle at the end of the game)
  */
void vibrateMotorsForFinish() {
  vibrateAllMotorsOnceForGivenTime(500);
  delay(500);
  vibrateAllMotorsOnceForGivenTime(500);
  delay(500);
  vibrateAllMotorsOnceForGivenTime(1000);
}

/**
  * Vibrates the motor (or motors) according to the destination's direction
  * @param destinationAngle angle of going direction in degrees
  */
void vibrateTheProperDirectionMotorOnce(float destinationAngle) {
  if ((0 <= destinationAngle && destinationAngle < 22.5) || (337.5 <= destinationAngle && destinationAngle < 360)) {
    vibrateOneMotorOnce(frontMotorPinNumber);
  } else if (destinationAngle >= 22.5  && destinationAngle < 67.5) {
    vibrateTwoMotorsOnce(frontMotorPinNumber, rightMotorPinNumber);
  } else if (destinationAngle >= 67.5 && destinationAngle < 112.5) {
    vibrateOneMotorOnce(rightMotorPinNumber);
  } else if (destinationAngle >= 112.5 && destinationAngle < 157.5) {
    vibrateTwoMotorsOnce(rightMotorPinNumber, backMotorPinNumber);
  } else if (destinationAngle >= 157.5 && destinationAngle < 202.5) {
    vibrateOneMotorOnce(backMotorPinNumber);
  } else if (destinationAngle >= 202.5 && destinationAngle < 247.5) {
    vibrateTwoMotorsOnce(backMotorPinNumber, leftMotorPinNumber);
  } else if (destinationAngle >= 247.5 && destinationAngle < 292.5) {
    vibrateOneMotorOnce(leftMotorPinNumber);
  } else if (destinationAngle >= 292.5 && destinationAngle < 337.5) {
    vibrateTwoMotorsOnce(leftMotorPinNumber, frontMotorPinNumber);
  } else {
    vibrateAllMotorsOnceForGivenTime(2000);
  }
}

#endif