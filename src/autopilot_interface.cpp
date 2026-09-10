/* ============================================================================
 * AUTOPILOT INTERFACE IMPLEMENTATION
 * ============================================================================
 */

#include "autopilot_interface.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

AutopilotInterface::AutopilotInterface() {
  init();
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void AutoopilotInterface::init() {
  memset(&data, 0, sizeof(AutopilotData));
  lastHeartbeatMs = millis();
  fcConnected = false;
  mavlinkState = 0;
}

// ============================================================================
// MAVLINK BYTE HANDLER
// ============================================================================

void AutopilotInterface::handleByte(uint8_t byte) {
  // Simplified MAVLink v1.0 parser
  // In production, use full mavlink_parse_char() from mavlink library
  
  static uint8_t payloadCounter = 0;
  static uint8_t msgId = 0;
  
  switch (mavlinkState) {
    case 0:  // Waiting for STX
      if (byte == MAVLINK_STX) {
        mavlinkState = 1;
      }
      break;
      
    case 1:  // Payload length
      payloadCounter = byte;
      mavlinkState = 2;
      break;
      
    case 2:  // Sequence number (skip)
      mavlinkState = 3;
      break;
      
    case 3:  // System ID (skip)
      mavlinkState = 4;
      break;
      
    case 4:  // Component ID (skip)
      mavlinkState = 5;
      break;
      
    case 5:  // Message ID
      msgId = byte;
      mavlinkState = 6;
      break;
      
    case 6:  // Payload (simplified)
      // In production, accumulate payload and checksum
      // Then dispatch to appropriate message handler
      if (msgId == MAVLINK_MSG_ID_HEARTBEAT) {
        handleHeartbeat(0);  // Simplified
      }
      payloadCounter--;
      if (payloadCounter == 0) {
        mavlinkState = 7;  // Checksum
      }
      break;
      
    case 7:  // Checksum 1 (skip)
      mavlinkState = 8;
      break;
      
    case 8:  // Checksum 2
      mavlinkState = 0;  // Back to start
      break;
  }
}

// ============================================================================
// MESSAGE HANDLERS
// ============================================================================

void AutopilotInterface::handleHeartbeat(uint32_t msgData) {
  lastHeartbeatMs = millis();
  
  if (!fcConnected) {
    fcConnected = true;
    Serial.println("AutoPilot Interface: FC Connected!");
  }
}

void AutopilotInterface::handleAttitude(float roll, float pitch, float yaw, 
                                         float rollRate, float pitchRate, float yawRate) {
  data.roll = roll;
  data.pitch = pitch;
  data.yaw = yaw;
  data.rollRate = rollRate;
  data.pitchRate = pitchRate;
  data.yawRate = yawRate;
}

void AutopilotInterface::handleVfrHud(float airspeed, float groundspeed, float heading,
                                       uint16_t throttle, float alt, float climbrate) {
  data.airspeed = airspeed;
  data.groundspeed = groundspeed;
  data.heading = heading;
  data.altitude = alt;
  data.climbRate = climbrate;
}

void AutopilotInterface::handleGpsRawInt(float lat, float lon, uint32_t alt, 
                                          uint16_t hdop, uint16_t satCount) {
  data.latitude = lat;
  data.longitude = lon;
  data.altitude = alt / 1000.0;  // Convert from mm to meters
  data.satelliteCount = satCount;
  data.gpsLocked = (satCount >= 5);
}

void AutopilotInterface::handleSysStatus(uint16_t voltage, int16_t current, int8_t battRemaining) {
  data.batteryVoltage = voltage / 1000.0;  // Convert from mV to V
  data.batteryCurrent = current / 100.0;   // Convert from cA to A
  data.batteryRemaining = battRemaining;
}

// ============================================================================
// GETTERS
// ============================================================================

AutopilotData AutopilotInterface::getData() {
  return data;
}

bool AutopilotInterface::isConnected() {
  // Consider disconnected if no heartbeat for 2 seconds
  if (millis() - lastHeartbeatMs > 2000) {
    fcConnected = false;
  }
  return fcConnected;
}

uint32_t AutopilotInterface::getTimeSinceLastHeartbeat() {
  return millis() - lastHeartbeatMs;
}

float AutopilotInterface::getHeading() {
  return data.heading;
}

float AutopilotInterface::getAltitude() {
  return data.altitude;
}

float AutopilotInterface::getAirspeed() {
  return data.airspeed;
}

float AutopilotInterface::getClimbRate() {
  return data.climbRate;
}

uint8_t AutopilotInterface::getFlightMode() {
  return data.flightMode;
}

bool AutopilotInterface::isArmed() {
  return data.armed;
}

bool AutopilotInterface::hasGPSLock() {
  return data.gpsLocked;
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

void AutopilotInterface::printStatus() {
  if (!isConnected()) {
    Serial.println("AutoPilot: DISCONNECTED");
    return;
  }
  
  Serial.print("AP[H:");
  Serial.print(data.heading, 1);
  Serial.print("° A:");
  Serial.print(data.altitude, 1);
  Serial.print("m AS:");
  Serial.print(data.airspeed, 1);
  Serial.print("m/s CR:");
  Serial.print(data.climbRate, 1);
  Serial.print("m/s GPS:");
  Serial.print(data.gpsLocked ? "OK" : "NO");
  Serial.print(" SAT:");
  Serial.print(data.satelliteCount);
  Serial.print(" BAT:");
  Serial.print(data.batteryVoltage, 1);
  Serial.println("V]");
}
