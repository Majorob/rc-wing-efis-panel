/* ============================================================================
 * AUTOPILOT INTERFACE - MAVLink Telemetry Parser
 * 
 * Decodes MAVLink messages from the flight controller (SpeedyBee F405 Wing)
 * Extracts attitude, navigation, and autopilot command data.
 * 
 * ============================================================================
 */

#ifndef AUTOPILOT_INTERFACE_H
#define AUTOPILOT_INTERFACE_H

#include <Arduino.h>

// ============================================================================
// AUTOPILOT DATA STRUCTURE
// ============================================================================

struct AutopilotData {
  // Attitude (Euler angles)
  float roll;                      // Roll angle (radians)
  float pitch;                     // Pitch angle (radians)
  float yaw;                       // Yaw/heading (radians)
  float rollRate;                  // Roll rate (rad/s)
  float pitchRate;                // Pitch rate (rad/s)
  float yawRate;                  // Yaw rate (rad/s)
  
  // Speed
  float airspeed;                  // Indicated airspeed (m/s)
  float groundspeed;               // Ground speed (m/s)
  float climbRate;                 // Vertical speed (m/s)
  
  // Altitude
  float altitude;                  // Barometric altitude (meters)
  float relativeAltitude;          // Altitude relative to home (meters)
  
  // Navigation
  float latitude;                  // GPS latitude (degrees)
  float longitude;                 // GPS longitude (degrees)
  float heading;                   // Magnetic heading (0-360 degrees)
  
  // Autopilot Status
  uint8_t flightMode;              // Current flight mode (0-255)
  bool armed;                      // Aircraft armed status
  bool gpsLocked;                  // GPS lock status
  uint8_t satelliteCount;          // Number of visible satellites
  
  // System Health
  float batteryVoltage;            // Battery voltage (volts)
  float batteryCurrent;            // Battery current (amps)
  uint8_t batteryRemaining;        // Battery remaining (0-100%)
  
  // Autopilot Desired States (for Flight Director)
  float desiredHeading;            // Desired heading from autopilot
  float desiredAirspeed;           // Desired airspeed
  float desiredClimbRate;          // Desired climb rate
  float desiredAltitude;           // Desired altitude
  
  // INAV/ArduPilot specific
  uint16_t flightTimer;            // Flight timer (seconds)
  bool navigationActive;           // Is navigation active?
  uint8_t navigationMode;          // Current navigation mode
  float navigationBearing;         // Bearing to waypoint
  float navigationDistance;        // Distance to waypoint
};

// ============================================================================
// AUTOPILOT INTERFACE CLASS
// ============================================================================

class AutopilotInterface {
  
private:
  AutopilotData data;
  uint32_t lastHeartbeatMs;
  bool fcConnected;
  
  // MAVLink state machine
  static const uint8_t MAVLINK_STX = 0xFE;    // MAVLink v1.0 start byte
  static const uint8_t MAVLINK_STX2 = 0xFD;   // MAVLink v2.0 start byte
  uint8_t mavlinkState;
  
public:
  AutopilotInterface();
  
  // Initialize
  void init();
  
  // Parse incoming MAVLink data
  void handleByte(uint8_t byte);
  
  // Get data
  AutopilotData getData();
  bool isConnected();
  uint32_t getTimeSinceLastHeartbeat();
  
  // Specific getters
  float getHeading();
  float getAltitude();
  float getAirspeed();
  float getClimbRate();
  uint8_t getFlightMode();
  bool isArmed();
  bool hasGPSLock();
  
  // MAVLink message handlers
  void handleHeartbeat(uint32_t msgData);
  void handleAttitude(float roll, float pitch, float yaw, float rollRate, float pitchRate, float yawRate);
  void handleVfrHud(float airspeed, float groundspeed, float heading, uint16_t throttle, float alt, float climbrate);
  void handleGpsRawInt(float lat, float lon, uint32_t alt, uint16_t hdop, uint16_t satCount);
  void handleSysStatus(uint16_t voltage, int16_t current, int8_t battRemaining);
  
  // Debug
  void printStatus();
};

// ============================================================================
// MAVLINK MESSAGE IDs (INAV compatible)
// ============================================================================

#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_VFR_HUD 74
#define MAVLINK_MSG_ID_GPS_RAW_INT 24
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT 62

#endif // AUTOPILOT_INTERFACE_H
