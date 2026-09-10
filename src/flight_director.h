/* ============================================================================
 * FLIGHT DIRECTOR - STEERING GUIDANCE SYSTEM
 * 
 * This module calculates flight director steering bars based on:
 * - Current aircraft attitude (from IMU/FC)
 * - Desired flight path (from autopilot or navigation data)
 * - Deviation from desired course/altitude/heading
 * 
 * The flight director provides intuitive "fly-to" cues that guide the pilot
 * or autopilot back to the desired flight path.
 * 
 * ============================================================================
 */

#ifndef FLIGHT_DIRECTOR_H
#define FLIGHT_DIRECTOR_H

#include <Arduino.h>
#include <math.h>

// ============================================================================
// FLIGHT DIRECTOR MODES
// ============================================================================

enum FlightDirectorMode {
  FD_MODE_HEADING_HOLD = 0,      // Maintain heading bug
  FD_MODE_ALTITUDE_HOLD = 1,     // Maintain altitude
  FD_MODE_APPROACH = 2,           // ILS approach mode (LOC + GS)
  FD_MODE_CLIMB = 3,              // Climb to altitude
  FD_MODE_DESCENT = 4,            // Descend to altitude
  FD_MODE_GPS_TRACK = 5           // Follow GPS track
};

// ============================================================================
// FLIGHT DIRECTOR DATA STRUCTURE
// ============================================================================

struct FlightDirectorData {
  // Current state
  float currentRoll;               // Current aircraft roll (degrees)
  float currentPitch;              // Current aircraft pitch (degrees)
  float currentHeading;            // Current magnetic heading (0-360)
  float currentAltitude;           // Current altitude (meters)
  float currentVerticalSpeed;      // Current vertical speed (m/s)
  float currentAirspeed;           // Current airspeed (knots)
  
  // Desired state (autopilot targets)
  float desiredHeading;            // Target heading (0-360)
  float desiredAltitude;           // Target altitude (meters)
  float desiredVerticalSpeed;      // Target climb rate (m/s)
  float desiredAirspeed;           // Target airspeed (knots)
  
  // Flight Director output (steering commands)
  float rollCommand;               // Desired roll to correct course (-30 to +30 degrees)
  float pitchCommand;              // Desired pitch to correct altitude (-15 to +15 degrees)
  
  // Mode and gains
  FlightDirectorMode mode;
  float rollGain;                  // Proportional gain for roll correction
  float pitchGain;                 // Proportional gain for pitch correction
  float maxRollCommand;            // Maximum roll command (degrees)
  float maxPitchCommand;           // Maximum pitch command (degrees)
  
  // ILS specific
  float localizerDeviation;        // Lateral deviation from runway centerline (degrees)
  float glideSlopeDeviation;       // Vertical deviation from 3-degree glide slope (feet)
  bool localizerArmable;           // Can capture localizer
  bool glideSlopeArmable;          // Can capture glide slope
  bool localizerCaptured;          // On localizer
  bool glideSlopeCapture;          // On glide slope
};

// ============================================================================
// FLIGHT DIRECTOR CLASS
// ============================================================================

class FlightDirector {
  
private:
  FlightDirectorData data;
  uint32_t lastUpdateMs;
  
  // Configuration constants
  static const float HEADING_ERROR_SCALE;      // How degrees of error map to roll
  static const float ALTITUDE_ERROR_SCALE;     // How meters of error map to pitch
  static const float VS_ERROR_SCALE;           // How m/s error maps to pitch
  static const float BANK_ANGLE_LIMIT;         // Max bank angle from FD
  static const float PITCH_ANGLE_LIMIT;        // Max pitch angle from FD
  static const float LOC_CAPTURE_THRESHOLD;    // Degrees from centerline to capture
  static const float GS_CAPTURE_THRESHOLD;     // Feet from glide slope to capture
  
public:
  FlightDirector();
  
  // Initialize with default values
  void init();
  
  // Set mode
  void setMode(FlightDirectorMode newMode);
  FlightDirectorMode getMode();
  
  // Update current state
  void setCurrentAttitude(float roll, float pitch, float heading);
  void setCurrentAltitude(float altitude, float verticalSpeed);
  void setCurrentSpeed(float airspeed);
  
  // Set desired targets
  void setDesiredHeading(float heading);
  void setDesiredAltitude(float altitude, float verticalSpeed = 0.0);
  void setDesiredAirspeed(float airspeed);
  void setLocalizerDeviation(float deviation);
  void setGlideSlopeDeviation(float deviation);
  
  // Calculate flight director steering commands
  void update();
  
  // Get steering commands
  float getRollCommand();
  float getPitchCommand();
  
  // ILS capture logic
  void updateILSCapture();
  bool isLocalizerCaptured();
  bool isGlideSlopeCaptured();
  
  // Debug/telemetry
  void printStatus();
  FlightDirectorData getData();
};

// ============================================================================
// HELPER FUNCTIONS FOR ANGLE CALCULATIONS
// ============================================================================

// Normalize angle to -180 to +180
float normalizeHeadingError(float error) {
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  return error;
}

// Constrain value to limits
float constrainValue(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

// Dead zone filter (no output below threshold)
float applyDeadZone(float value, float deadZone) {
  if (fabs(value) < deadZone) return 0.0;
  return value;
}

// Low-pass filter for smooth transitions
float lowPassFilter(float newValue, float oldValue, float alpha) {
  return (newValue * alpha) + (oldValue * (1.0 - alpha));
}

#endif // FLIGHT_DIRECTOR_H
