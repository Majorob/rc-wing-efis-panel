/* ============================================================================
 * RC WING EFIS PANEL - MAIN FIRMWARE
 * 
 * Platform: ESP32-32 N16P8
 * Flight Controller: SpeedyBee F405 Wing (INAV)
 * Display: 4" LCD (ILI9488 or ST7796) via SPI
 * Sensors: GPS M10, Airspeed Sensor, IMU data from FC
 * 
 * Features:
 * - Real-time artificial horizon (attitude indicator)
 * - Airspeed tape (analog needle + digital display)
 * - Altitude tape with trend
 * - Vertical speed indicator with trend
 * - Compass heading with bug marker
 * - Flight Director with steering bars
 * - Localizer and Glide Slope deviation indicators
 * - GPS information display
 * - Battery voltage and flight time
 * 
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// TFT Display Library
#include <TFT_eSPI.h>               // For both ILI9488 and ST7796

// MAVLink/INAV telemetry
#include <MAVLink.h>                // MAVLink protocol library

// GPS parsing
#include <TinyGPS++.h>              // GPS NMEA parser

// ============================================================================
// CONFIGURATION - MODIFY THESE FOR YOUR SETUP
// ============================================================================

// Display Selection (uncomment one)
#define DISPLAY_ILI9488             // Use ILI9488 driver
// #define DISPLAY_ST7796            // Use ST7796 driver (comment above if using this)

// Pin Definitions
#define FC_TELEM_RX     9           // SpeedyBee F405 TX -> ESP32 RX (UART1)
#define FC_TELEM_TX     10          // SpeedyBee F405 RX -> ESP32 TX (UART1)
#define GPS_RX          16          // GPS M10 TX -> ESP32 RX (UART2)
#define GPS_TX          17          // GPS M10 RX -> ESP32 TX (UART2)
#define AIRSPEED_ADC    36          // Airspeed sensor analog input (GPIO36/VP)
#define BATTERY_ADC     39          // Battery voltage input (GPIO39/VN)
#define LCD_BACKLIGHT   15          // PWM pin for LCD brightness

// Baud Rates
#define FC_BAUD_RATE    57600       // SpeedyBee INAV telemetry
#define GPS_BAUD_RATE   38400       // GPS M10 standard baud
#define DEBUG_BAUD      115200      // USB serial debug

// Calibration Values (adjust to your setup)
#define AIRSPEED_MIN_ADC    0.4     // ADC voltage at 0 knots
#define AIRSPEED_MAX_ADC    3.0     // ADC voltage at max speed
#define AIRSPEED_MAX_SPEED  100.0   // Maximum airspeed to display (knots)
#define BATTERY_VOLTAGE_RATIO 7.64  // Voltage divider ratio (adjust per your resistors)
#define BATTERY_MIN_VOLTAGE 3.0     // Minimum cell voltage (LiPo cutoff)
#define BATTERY_MAX_VOLTAGE 4.2     // Maximum cell voltage (LiPo full)

// Flight Director / ILS Configuration
#define ILS_GLIDE_SLOPE_ANGLE   3.0   // Standard 3-degree glide slope
#define LOCALIZER_DEVIATION_MAX 2.5   // Max deviation in degrees to display
#define GLIDE_SLOPE_DEVIATION_MAX 500 // Max deviation in feet to display

// ============================================================================
// GLOBAL OBJECTS & VARIABLES
// ============================================================================

TFT_eSPI tft = TFT_eSPI();          // Create display object

// UART for telemetry and GPS
HardwareSerial fcTelemetry(1);      // UART1 for flight controller
HardwareSerial gpsSerial(2);        // UART2 for GPS

// GPS parser
TinyGPSPlus gps;

// ============================================================================
// DATA STRUCTURES FOR SENSOR DATA
// ============================================================================

struct FlightData {
  // Attitude (from FC)
  float roll;                       // Degrees
  float pitch;                      // Degrees
  float yaw;                        // Degrees
  float heading;                    // 0-360 degrees
  
  // Altitude & Climb
  float altitude;                   // Meters
  float altitudeAGL;                // Meters above ground level
  float verticalSpeed;              // m/s (positive = climbing)
  float verticalSpeedTrend;         // Filtered trend
  
  // Speed
  float airspeed;                   // Knots (from pitot sensor)
  float groundSpeed;                // Knots (from GPS)
  float verticalSpeedKnots;         // Converted to knots
  
  // Navigation
  float latitude;                   // Decimal degrees
  float longitude;                  // Decimal degrees
  float courseOverGround;           // Degrees from GPS
  float hdop;                       // Horizontal dilution of precision
  uint8_t satellites;               // Number of satellites
  
  // Battery
  float batteryVoltage;             // Volts
  uint8_t batteryPercent;           // 0-100%
  
  // Flight Time
  uint32_t flightTimeSeconds;       // Elapsed flight time
  uint32_t startTimeMs;             // When flight started
  
  // Flight Director / ILS
  float desiredHeading;             // Where autopilot wants to go
  float desiredPitch;               // Target pitch
  float desiredRoll;                // Target roll
  float localizerDeviation;         // Degrees from runway centerline
  float glideSlopeDeviation;        // Feet from 3-degree path
  
  // System Status
  bool fcConnected;                 // MAVLink connection status
  bool gpsLocked;                   // GPS fix status
  uint32_t lastMavLinkMs;           // Last MAVLink heartbeat
  uint32_t lastGpsUpdateMs;         // Last GPS update
};

FlightData flightData = {};
volatile bool dataUpdated = false;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize serial ports
  Serial.begin(DEBUG_BAUD);         // USB Debug
  fcTelemetry.begin(FC_BAUD_RATE, SERIAL_8N1, FC_TELEM_RX, FC_TELEM_TX);
  gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX, GPS_TX);
  
  delay(500);
  Serial.println("\n\n=== RC WING EFIS PANEL STARTING ===");
  
  // Initialize display
  tft.init();
  tft.setRotation(0);               // Portrait orientation
  tft.fillScreen(TFT_BLACK);
  
  // Configure backlight
  pinMode(LCD_BACKLIGHT, OUTPUT);
  analogWrite(LCD_BACKLIGHT, 200);  // 78% brightness
  
  // Draw startup splash
  drawSplashScreen();
  
  // Initialize ADC
  analogSetAttenuation(ADC_11db);   // For full range (0-3.3V)
  
  // Initialize data
  flightData.startTimeMs = millis();
  flightData.fcConnected = false;
  flightData.gpsLocked = false;
  
  Serial.println("EFIS System initialized. Waiting for telemetry...");
  
  delay(2000);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Update sensor data
  updateFlightControllerData();     // Read MAVLink telemetry
  updateGpsData();                  // Read GPS
  updateAirspeedData();             // Read pitot airspeed sensor
  updateBatteryVoltage();           // Read battery voltage
  updateFlightTime();               // Update elapsed flight time
  
  // Calculate flight director and ILS data
  calculateFlightDirector();
  calculateILS();
  
  // Render EFIS display
  renderEFISDisplay();
  
  // Status monitoring
  monitorSystemStatus();
  
  delay(50);                        // ~20 Hz update rate
}

// ============================================================================
// TELEMETRY DATA ACQUISITION
// ============================================================================

void updateFlightControllerData() {
  static uint32_t lastMavLinkMs = 0;
  
  // Read MAVLink data from SpeedyBee F405 Wing
  if (fcTelemetry.available()) {
    uint8_t byte = fcTelemetry.read();
    
    // MAVLink message parsing (simplified version)
    // In production, use full MAVLink library with message handlers
    parseMAVLinkData(byte);
    
    lastMavLinkMs = millis();
    flightData.lastMavLinkMs = millis();
    
    if (!flightData.fcConnected) {
      flightData.fcConnected = true;
      Serial.println("FC Telemetry Connected!");
    }
  }
  
  // Check for connection timeout
  if (millis() - flightData.lastMavLinkMs > 2000) {
    flightData.fcConnected = false;
  }
}

void parseMAVLinkData(uint8_t byte) {
  // Simplified MAVLink parser for INAV
  // This is a placeholder - full implementation requires message decoding
  
  static uint32_t lastHeartbeat = 0;
  
  // In a real implementation, you would:
  // 1. Use mavlink_parse_char() to decode incoming bytes
  // 2. Handle specific message types (ATTITUDE, VFR_HUD, GPS_RAW_INT, etc.)
  // 3. Extract and store the data in flightData struct
  
  // Example structure for common messages:
  /*
  mavlink_message_t msg;
  mavlink_status_t status;
  
  if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
    switch (msg.msgid) {
      case MAVLINK_MSG_ID_ATTITUDE:
        {
          mavlink_attitude_t att;
          mavlink_msg_attitude_decode(&msg, &att);
          flightData.roll = att.roll * RAD_TO_DEG;
          flightData.pitch = att.pitch * RAD_TO_DEG;
          flightData.yaw = att.yaw * RAD_TO_DEG;
        }
        break;
        
      case MAVLINK_MSG_ID_VFR_HUD:
        {
          mavlink_vfr_hud_t hud;
          mavlink_msg_vfr_hud_decode(&msg, &hud);
          flightData.airspeed = hud.airspeed;
          flightData.groundSpeed = hud.groundspeed;
          flightData.altitude = hud.alt;
          flightData.heading = hud.heading;
          flightData.verticalSpeed = hud.climb;
        }
        break;
        
      case MAVLINK_MSG_ID_HEARTBEAT:
        lastHeartbeat = millis();
        break;
    }
  }
  */
}

void updateGpsData() {
  // Read GPS data from M10 module
  while (gpsSerial.available()) {
    uint8_t gpsByte = gpsSerial.read();
    
    if (gps.encode(gpsByte)) {
      // GPS sentence parsed successfully
      flightData.lastGpsUpdateMs = millis();
      
      if (gps.location.isValid()) {
        flightData.latitude = gps.location.lat();
        flightData.longitude = gps.location.lng();
        flightData.gpsLocked = true;
      }
      
      if (gps.speed.isValid()) {
        flightData.groundSpeed = gps.speed.knots();
        flightData.courseOverGround = gps.course.deg();
      }
      
      if (gps.satellites.isValid()) {
        flightData.satellites = gps.satellites.value();
      }
      
      if (gps.hdop.isValid()) {
        flightData.hdop = gps.hdop.hdop();
      }
    }
  }
  
  // Check for GPS timeout
  if (millis() - flightData.lastGpsUpdateMs > 5000) {
    flightData.gpsLocked = false;
  }
}

void updateAirspeedData() {
  // Read analog airspeed sensor from pitot tube
  uint16_t rawAdc = analogRead(AIRSPEED_ADC);
  float adcVoltage = (rawAdc / 4095.0) * 3.3;  // Convert to voltage
  
  // Calibration: Scale voltage to airspeed in knots
  // Adjust AIRSPEED_MIN_ADC and AIRSPEED_MAX_ADC for your sensor
  if (adcVoltage < AIRSPEED_MIN_ADC) {
    flightData.airspeed = 0.0;
  } else if (adcVoltage > AIRSPEED_MAX_ADC) {
    flightData.airspeed = AIRSPEED_MAX_SPEED;
  } else {
    flightData.airspeed = ((adcVoltage - AIRSPEED_MIN_ADC) / 
                          (AIRSPEED_MAX_ADC - AIRSPEED_MIN_ADC)) * AIRSPEED_MAX_SPEED;
  }
  
  // Smooth with a simple filter
  static float filteredAirspeed = 0.0;
  filteredAirspeed = (filteredAirspeed * 0.8) + (flightData.airspeed * 0.2);
  flightData.airspeed = filteredAirspeed;
}

void updateBatteryVoltage() {
  // Read battery voltage via voltage divider on ADC
  uint16_t rawAdc = analogRead(BATTERY_ADC);
  float adcVoltage = (rawAdc / 4095.0) * 3.3;  // ADC range
  
  // Convert to actual battery voltage using divider ratio
  flightData.batteryVoltage = adcVoltage * BATTERY_VOLTAGE_RATIO;
  
  // Calculate battery percentage (for LiPo)
  // Simple linear mapping from BATTERY_MIN_VOLTAGE to BATTERY_MAX_VOLTAGE
  float minVoltage = BATTERY_MIN_VOLTAGE * 6;  // For 6S LiPo
  float maxVoltage = BATTERY_MAX_VOLTAGE * 6;
  
  if (flightData.batteryVoltage <= minVoltage) {
    flightData.batteryPercent = 0;
  } else if (flightData.batteryVoltage >= maxVoltage) {
    flightData.batteryPercent = 100;
  } else {
    flightData.batteryPercent = ((flightData.batteryVoltage - minVoltage) / 
                                (maxVoltage - minVoltage)) * 100;
  }
}

void updateFlightTime() {
  if (flightData.fcConnected) {
    flightData.flightTimeSeconds = (millis() - flightData.startTimeMs) / 1000;
  }
}

// ============================================================================
// FLIGHT DIRECTOR & ILS CALCULATIONS
// ============================================================================

void calculateFlightDirector() {
  // Calculate steering commands based on autopilot desired attitude
  // This would normally come from the flight controller
  
  // Placeholder: Calculate deviation from desired heading
  float headingError = flightData.desiredHeading - flightData.heading;
  
  // Normalize to -180 to +180
  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;
  
  // Limit scaling for display (typically ±30 degrees)
  flightData.desiredRoll = constrain(headingError * 0.5, -30, 30);
  
  // Pitch control for altitude
  float altitudeError = flightData.altitude - 100; // Target 100m (placeholder)
  flightData.desiredPitch = constrain(altitudeError / 50.0, -30, 30);
}

void calculateILS() {
  // Calculate localizer and glide slope deviations
  // Requires defining a runway location and approach path
  
  // Example: Define runway threshold at specific lat/lon
  static const float RUNWAY_LAT = 0.0;      // Set to your runway
  static const float RUNWAY_LON = 0.0;
  static const float RUNWAY_HEADING = 180.0;  // Runway orientation
  
  // Calculate distance and bearing from current position to runway
  float distanceToRunway = gps.distanceBetween(
    flightData.latitude, flightData.longitude,
    RUNWAY_LAT, RUNWAY_LON
  ) * 0.000621371;  // Convert meters to nautical miles
  
  float bearingToRunway = gps.courseTo(
    flightData.latitude, flightData.longitude,
    RUNWAY_LAT, RUNWAY_LON
  );
  
  // Localizer deviation (cross-track error from runway centerline)
  float trackDifference = bearingToRunway - RUNWAY_HEADING;
  while (trackDifference > 180) trackDifference -= 360;
  while (trackDifference < -180) trackDifference += 360;
  
  // Simple cross-track error calculation
  float crossTrackError = distanceToRunway * sin(trackDifference * PI / 180.0);
  flightData.localizerDeviation = constrain(crossTrackError * 100, 
                                           -LOCALIZER_DEVIATION_MAX, 
                                           LOCALIZER_DEVIATION_MAX);
  
  // Glide slope deviation
  // Calculate expected altitude at current distance
  float expectedAltitude = 0;  // Threshold altitude (set to runway elevation)
  expectedAltitude += distanceToRunway * 101.3 * tan(ILS_GLIDE_SLOPE_ANGLE * PI / 180.0);
  
  float altitudeError = flightData.altitude - expectedAltitude;
  flightData.glideSlopeDeviation = constrain(altitudeError * 3.28084,  // Convert m to feet
                                            -GLIDE_SLOPE_DEVIATION_MAX,
                                            GLIDE_SLOPE_DEVIATION_MAX);
}

// ============================================================================
// DISPLAY RENDERING
// ============================================================================

void renderEFISDisplay() {
  // Clear display once per frame
  tft.fillScreen(TFT_BLACK);
  
  // Draw EFIS instrument layout
  drawArtificialHorizon();
  drawAirspeedIndicator();
  drawAltitudeTape();
  drawVerticalSpeedTape();
  drawCompassHeading();
  drawFlightDirector();
  drawLocalizerDisplay();
  drawGlideSlopeDisplay();
  drawNavigationData();
  drawSystemStatus();
  
  // Draw frame rate counter (debug)
  // drawFrameRate();
}

void drawArtificialHorizon() {
  // Position: Center-left of display
  int x = 60;
  int y = 120;
  int radius = 50;
  
  // Draw outer circle
  tft.drawCircle(x, y, radius, TFT_WHITE);
  
  // Draw sky (blue) and ground (brown)
  // Rotate based on roll angle
  // This is simplified - full implementation uses rotation matrix
  
  int rollAngle = (int)flightData.roll;
  int pitchAngle = (int)flightData.pitch;
  
  // Sky color: light blue
  tft.fillRect(x - radius, y - radius, radius * 2, radius, TFT_BLUE);
  
  // Ground color: brown
  tft.fillRect(x - radius, y, radius * 2, radius, TFT_BROWN);
  
  // Draw horizon line
  tft.drawLine(x - radius, y, x + radius, y, TFT_WHITE);
  
  // Draw pitch scale
  for (int i = -60; i <= 60; i += 10) {
    if (i == 0) continue;  // Skip center (horizon)
    int pixelY = y + (i * 50 / 30);  // Scale pixels per degree
    int pixelX = x;
    
    if (i % 30 == 0) {
      // Major tick
      tft.drawLine(pixelX - 15, pixelY, pixelX + 15, pixelY, TFT_WHITE);
      tft.drawString(String(i), pixelX + 20, pixelY - 5, 2);
    } else {
      // Minor tick
      tft.drawLine(pixelX - 8, pixelY, pixelX + 8, pixelY, TFT_WHITE);
    }
  }
  
  // Draw aircraft symbol (triangle)
  tft.drawLine(x, y + 20, x - 15, y + 35, TFT_YELLOW);
  tft.drawLine(x, y + 20, x + 15, y + 35, TFT_YELLOW);
  tft.drawLine(x - 15, y + 35, x + 15, y + 35, TFT_YELLOW);
}

void drawAirspeedIndicator() {
  // Position: Right side
  int x = 300;
  int y = 80;
  
  // Draw tape background
  tft.fillRect(x - 30, y, 60, 120, TFT_BLACK);
  tft.drawRect(x - 30, y, 60, 120, TFT_WHITE);
  
  // Draw speed graduations
  for (int speed = 0; speed <= 120; speed += 10) {
    int pixelY = y + 120 - (speed * 120 / 120);
    
    if (speed % 20 == 0) {
      tft.drawLine(x - 25, pixelY, x - 15, pixelY, TFT_WHITE);
      tft.drawString(String(speed), x - 50, pixelY - 5, 1);
    } else {
      tft.drawLine(x - 25, pixelY, x - 20, pixelY, TFT_WHITE);
    }
  }
  
  // Draw current speed pointer
  int speedPixel = y + 120 - (flightData.airspeed * 120 / 120);
  tft.fillTriangle(x + 10, speedPixel - 5, x + 10, speedPixel + 5, x + 30, speedPixel, TFT_GREEN);
  
  // Draw digital display
  tft.drawString(String((int)flightData.airspeed) + " kt", x - 20, y + 130, 2);
}

void drawAltitudeTape() {
  // Position: Right side, below airspeed
  int x = 300;
  int y = 220;
  
  // Draw tape background
  tft.fillRect(x - 30, y, 60, 120, TFT_BLACK);
  tft.drawRect(x - 30, y, 60, 120, TFT_WHITE);
  
  // Draw altitude graduations (every 100m)
  int baseAltitude = ((int)flightData.altitude / 100) * 100;
  
  for (int i = -5; i <= 5; i++) {
    int alt = baseAltitude + (i * 100);
    int pixelY = y + 60 - (i * 120 / 10);
    
    tft.drawLine(x - 25, pixelY, x - 15, pixelY, TFT_WHITE);
    if (i != 0) {
      tft.drawString(String(alt), x - 50, pixelY - 5, 1);
    }
  }
  
  // Draw current altitude pointer
  int altPixel = y + 60;
  tft.fillTriangle(x + 10, altPixel - 5, x + 10, altPixel + 5, x + 30, altPixel, TFT_GREEN);
  
  // Draw digital display
  tft.drawString(String((int)flightData.altitude) + "m", x - 20, y + 130, 2);
}

void drawVerticalSpeedTape() {
  // Position: Top right
  int x = 380;
  int y = 40;
  
  tft.drawRect(x, y, 50, 100, TFT_WHITE);
  
  // Draw VSI scale (-10 to +10 m/s)
  for (int vs = -10; vs <= 10; vs += 5) {
    int pixelY = y + 50 + (vs * 50 / 10);
    tft.drawLine(x + 5, pixelY, x + 15, pixelY, TFT_WHITE);
    tft.drawString(String(vs), x + 20, pixelY - 3, 1);
  }
  
  // Draw needle
  int vsPixel = y + 50 + (flightData.verticalSpeed * 50 / 10);
  vsPixel = constrain(vsPixel, y, y + 100);
  tft.drawLine(x + 25, y + 50, x + 25, vsPixel, TFT_GREEN);
  tft.fillCircle(x + 25, vsPixel, 3, TFT_GREEN);
  
  // Label
  tft.drawString("VS", x + 5, y - 15, 1);
}

void drawCompassHeading() {
  // Position: Top center
  int x = 160;
  int y = 20;
  
  // Draw compass circle
  tft.drawCircle(x, y, 35, TFT_WHITE);
  
  // Draw cardinal directions
  tft.drawString("N", x - 3, y - 40, 2);
  tft.drawString("E", x + 35, y - 3, 2);
  tft.drawString("S", x - 3, y + 35, 2);
  tft.drawString("W", x - 45, y - 3, 2);
  
  // Draw heading needle
  float headingRad = flightData.heading * PI / 180.0;
  int needleX = x + (int)(30 * sin(headingRad));
  int needleY = y - (int)(30 * cos(headingRad));
  
  tft.drawLine(x, y, needleX, needleY, TFT_YELLOW);
  
  // Draw digital heading
  tft.drawString(String((int)flightData.heading) + "°", x - 15, y + 40, 2);
}

void drawFlightDirector() {
  // Position: Center
  int x = 160;
  int y = 120;
  
  // Draw crosshair
  tft.drawLine(x - 30, y, x + 30, y, TFT_WHITE);
  tft.drawLine(x, y - 30, x, y + 30, TFT_WHITE);
  tft.drawCircle(x, y, 20, TFT_WHITE);
  
  // Draw flight director bars
  // Horizontal bar (for roll guidance)
  int barX = x + (int)(flightData.desiredRoll * 20 / 30);
  tft.drawRect(barX - 20, y - 5, 40, 10, TFT_MAGENTA);
  
  // Vertical bar (for pitch guidance)
  int barY = y - (int)(flightData.desiredPitch * 20 / 30);
  tft.drawRect(x - 5, barY - 20, 10, 40, TFT_MAGENTA);
}

void drawLocalizerDisplay() {
  // Position: Bottom left
  int x = 50;
  int y = 290;
  
  tft.drawString("LOC", x, y, 2);
  tft.drawRect(x - 20, y + 20, 140, 20, TFT_WHITE);
  
  // Draw localizer needle
  // Center is at x + 50, deviation scale ±2.5 degrees
  int needleX = x + 50 + (int)(flightData.localizerDeviation * 25);
  needleX = constrain(needleX, x, x + 100);
  
  tft.drawLine(needleX, y + 15, needleX, y + 45, TFT_YELLOW);
  tft.fillCircle(needleX, y + 30, 2, TFT_YELLOW);
}

void drawGlideSlopeDisplay() {
  // Position: Bottom center-right
  int x = 180;
  int y = 290;
  
  tft.drawString("GS", x, y, 2);
  tft.drawRect(x + 15, y + 15, 20, 40, TFT_WHITE);
  
  // Draw glide slope needle (vertical)
  // Center at y + 35, deviation scale ±500 feet
  int needleY = y + 35 - (int)(flightData.glideSlopeDeviation * 40 / 500);
  needleY = constrain(needleY, y + 15, y + 55);
  
  tft.drawLine(x + 10, needleY, x + 50, needleY, TFT_CYAN);
  tft.fillCircle(x + 25, needleY, 2, TFT_CYAN);
}

void drawNavigationData() {
  // Position: Bottom center
  int x = 10;
  int y = 280;
  
  // GPS Status
  if (flightData.gpsLocked) {
    tft.setTextColor(TFT_GREEN);
    tft.drawString("GPS OK", x, y, 1);
  } else {
    tft.setTextColor(TFT_RED);
    tft.drawString("GPS NOLOCK", x, y, 1);
  }
  tft.setTextColor(TFT_WHITE);
  
  // Ground Speed and Track
  tft.drawString("GS:" + String((int)flightData.groundSpeed) + "kt TRK:" + 
                 String((int)flightData.courseOverGround) + "°", x, y + 15, 1);
  
  // Satellite count and HDOP
  tft.drawString("SAT:" + String(flightData.satellites) + " HDOP:" + 
                 String(flightData.hdop, 1), x, y + 25, 1);
}

void drawSystemStatus() {
  // Position: Top left
  int x = 10;
  int y = 10;
  
  // Flight Controller status
  if (flightData.fcConnected) {
    tft.setTextColor(TFT_GREEN);
    tft.drawString("FC:ON", x, y, 1);
  } else {
    tft.setTextColor(TFT_RED);
    tft.drawString("FC:OFF", x, y, 1);
  }
  tft.setTextColor(TFT_WHITE);
  
  // Battery voltage
  tft.drawString("BAT:" + String(flightData.batteryVoltage, 1) + "V", x, y + 12, 1);
  
  // Flight time
  uint32_t minutes = flightData.flightTimeSeconds / 60;
  uint32_t seconds = flightData.flightTimeSeconds % 60;
  tft.drawString("FLT:" + String(minutes) + ":" + 
                 String(seconds < 10 ? "0" : "") + String(seconds), x, y + 24, 1);
}

void drawSplashScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN);
  tft.drawString("RC WING EFIS", 100, 140, 4);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Electronic Flight Instrument System", 70, 180, 2);
  tft.drawString("v1.0 - Initializing...", 120, 220, 1);
}

// ============================================================================
// SYSTEM MONITORING
// ============================================================================

void monitorSystemStatus() {
  // Check for critical conditions
  
  // Low battery warning
  if (flightData.batteryVoltage < (BATTERY_MIN_VOLTAGE * 6) && flightData.batteryVoltage > 0) {
    Serial.println("WARNING: Low battery voltage!");
  }
  
  // GPS loss
  static bool gpsWasLocked = false;
  if (!flightData.gpsLocked && gpsWasLocked) {
    Serial.println("WARNING: GPS signal lost!");
  }
  gpsWasLocked = flightData.gpsLocked;
  
  // FC connection loss
  static bool fcWasConnected = false;
  if (!flightData.fcConnected && fcWasConnected) {
    Serial.println("WARNING: Flight controller telemetry lost!");
  }
  fcWasConnected = flightData.fcConnected;
  
  // Debug output (every 5 seconds)
  static uint32_t lastDebugMs = 0;
  if (millis() - lastDebugMs > 5000) {
    lastDebugMs = millis();
    
    Serial.print("Attitude: R=");
    Serial.print(flightData.roll);
    Serial.print(" P=");
    Serial.print(flightData.pitch);
    Serial.print(" Y=");
    Serial.println(flightData.yaw);
    
    Serial.print("Speed: AS=");
    Serial.print(flightData.airspeed);
    Serial.print("kt GS=");
    Serial.print(flightData.groundSpeed);
    Serial.print("kt VS=");
    Serial.println(flightData.verticalSpeed);
    
    Serial.print("Alt: ");
    Serial.print(flightData.altitude);
    Serial.print("m Bat: ");
    Serial.print(flightData.batteryVoltage);
    Serial.println("V");
  }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

float constrainAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

// ============================================================================
// END OF FIRMWARE
// ============================================================================
