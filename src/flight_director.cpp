/* ============================================================================
 * FLIGHT DIRECTOR IMPLEMENTATION
 * ============================================================================
 */

#include "flight_director.h"

// ============================================================================
// STATIC CONFIGURATION CONSTANTS
// ============================================================================

const float FlightDirector::HEADING_ERROR_SCALE = 0.5;     // 1 deg error = 0.5 deg roll command
const float FlightDirector::ALTITUDE_ERROR_SCALE = 0.3;    // 1 meter error = 0.3 deg pitch
const float FlightDirector::VS_ERROR_SCALE = 0.5;          // 1 m/s error = 0.5 deg pitch
const float FlightDirector::BANK_ANGLE_LIMIT = 30.0;       // Max 30 degrees bank from FD
const float FlightDirector::PITCH_ANGLE_LIMIT = 15.0;      // Max 15 degrees pitch from FD
const float FlightDirector::LOC_CAPTURE_THRESHOLD = 0.2;   // Capture within 0.2 degrees
const float FlightDirector::GS_CAPTURE_THRESHOLD = 50.0;   // Capture within 50 feet

// ============================================================================
// CONSTRUCTOR
// ============================================================================

FlightDirector::FlightDirector() {
  init();
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void FlightDirector::init() {
  // Clear all data
  memset(&data, 0, sizeof(FlightDirectorData));
  
  // Set default configuration
  data.mode = FD_MODE_HEADING_HOLD;
  data.rollGain = HEADING_ERROR_SCALE;
  data.pitchGain = ALTITUDE_ERROR_SCALE;
  data.maxRollCommand = BANK_ANGLE_LIMIT;
  data.maxPitchCommand = PITCH_ANGLE_LIMIT;
  
  // ILS settings
  data.localizerArmable = false;
  data.glideSlopeArmable = false;
  data.localizerCaptured = false;
  data.glideSlopeCapture = false;
  
  lastUpdateMs = millis();
}

// ============================================================================
// MODE CONTROL
// ============================================================================

void FlightDirector::setMode(FlightDirectorMode newMode) {
  data.mode = newMode;
  Serial.print("Flight Director Mode: ");
  Serial.println(newMode);
}

FlightDirectorMode FlightDirector::getMode() {
  return data.mode;
}

// ============================================================================
// STATE SETTERS
// ============================================================================

void FlightDirector::setCurrentAttitude(float roll, float pitch, float heading) {
  data.currentRoll = roll;
  data.currentPitch = pitch;
  data.currentHeading = heading;
}

void FlightDirector::setCurrentAltitude(float altitude, float verticalSpeed) {
  data.currentAltitude = altitude;
  data.currentVerticalSpeed = verticalSpeed;
}

void FlightDirector::setCurrentSpeed(float airspeed) {
  data.currentAirspeed = airspeed;
}

void FlightDirector::setDesiredHeading(float heading) {
  // Normalize to 0-360
  while (heading >= 360) heading -= 360;
  while (heading < 0) heading += 360;
  data.desiredHeading = heading;
}

void FlightDirector::setDesiredAltitude(float altitude, float verticalSpeed) {
  data.desiredAltitude = altitude;
  data.desiredVerticalSpeed = verticalSpeed;
}

void FlightDirector::setDesiredAirspeed(float airspeed) {
  data.desiredAirspeed = airspeed;
}

void FlightDirector::setLocalizerDeviation(float deviation) {
  data.localizerDeviation = deviation;
}

void FlightDirector::setGlideSlopeDeviation(float deviation) {
  data.glideSlopeDeviation = deviation;
}

// ============================================================================
// MAIN FLIGHT DIRECTOR UPDATE
// ============================================================================

void FlightDirector::update() {
  uint32_t now = millis();
  uint32_t deltaMs = now - lastUpdateMs;
  lastUpdateMs = now;
  
  // Clear previous commands
  data.rollCommand = 0.0;
  data.pitchCommand = 0.0;
  
  // Calculate commands based on mode
  switch (data.mode) {
    case FD_MODE_HEADING_HOLD:
      calculateHeadingHold();
      break;
      
    case FD_MODE_ALTITUDE_HOLD:
      calculateAltitudeHold();
      break;
      
    case FD_MODE_APPROACH:
      calculateApproach();
      updateILSCapture();
      break;
      
    case FD_MODE_CLIMB:
      calculateClimb();
      break;
      
    case FD_MODE_DESCENT:
      calculateDescent();
      break;
      
    case FD_MODE_GPS_TRACK:
      calculateGpsTrack();
      break;
  }
  
  // Constrain commands to safe limits
  data.rollCommand = constrainValue(data.rollCommand, -data.maxRollCommand, data.maxRollCommand);
  data.pitchCommand = constrainValue(data.pitchCommand, -data.maxPitchCommand, data.maxPitchCommand);
}

// ============================================================================
// FLIGHT DIRECTOR CALCULATION METHODS
// ============================================================================

// Heading Hold Mode - Maintain target heading
void FlightDirector::calculateHeadingHold() {
  float headingError = normalizeHeadingError(data.desiredHeading - data.currentHeading);
  
  // Roll command proportional to heading error
  // Positive error (left) -> positive roll command (right)
  data.rollCommand = headingError * data.rollGain;
  
  // Add damping based on current roll rate (simplified)
  if (fabs(data.currentRoll) > 5.0) {
    // If already rolled, reduce command
    data.rollCommand *= 0.8;
  }
  
  // Maintain current pitch/altitude in heading hold
  data.pitchCommand = 0.0;  // Let autopilot handle pitch
}

// Altitude Hold Mode - Maintain target altitude
void FlightDirector::calculateAltitudeHold() {
  // Altitude error in meters
  float altitudeError = data.desiredAltitude - data.currentAltitude;
  
  // Pitch command proportional to altitude error
  // Positive error (too low) -> positive pitch (climb)
  data.pitchCommand = constrainValue(altitudeError * data.pitchGain, 
                                     -data.maxPitchCommand, 
                                     data.maxPitchCommand);
  
  // Add vertical speed error feedback
  if (fabs(altitudeError) < 50.0) {  // Within 50m of target
    // Use vertical speed to fine-tune
    float vsError = data.desiredVerticalSpeed - data.currentVerticalSpeed;
    data.pitchCommand += vsError * VS_ERROR_SCALE * 0.5;
  }
  
  // Heading hold in altitude mode
  calculateHeadingHold();
}

// Approach Mode - ILS with localizer and glide slope
void FlightDirector::calculateApproach() {
  // Localizer - lateral guidance (roll command)
  if (data.localizerCaptured) {
    // On localizer - maintain with small corrections
    data.rollCommand = data.localizerDeviation * 5.0;  // Scale deviation to roll
    data.rollCommand = constrainValue(data.rollCommand, -10.0, 10.0);  // Limit to ±10 degrees
  } else {
    // Intercept localizer
    // If outside ±2.5 degrees, make larger correction
    if (fabs(data.localizerDeviation) > 1.0) {
      data.rollCommand = constrainValue(data.localizerDeviation * 10.0, -20.0, 20.0);
    }
  }
  
  // Glide Slope - vertical guidance (pitch command)
  if (data.glideSlopeCapture) {
    // On glide slope - maintain with small corrections
    // Deviation in feet, convert to pitch
    data.pitchCommand = (data.glideSlopeDeviation / 100.0) * 0.5;
    data.pitchCommand = constrainValue(data.pitchCommand, -5.0, 5.0);
  } else {
    // Intercept glide slope
    if (fabs(data.glideSlopeDeviation) > 150.0) {
      // Make larger correction
      data.pitchCommand = constrainValue(data.glideSlopeDeviation / 200.0, -10.0, 10.0);
    }
  }
}

// Climb Mode - Climb to target altitude
void FlightDirector::calculateClimb() {
  // Max climb pitch
  float altitudeError = data.desiredAltitude - data.currentAltitude;
  
  if (altitudeError > 100.0) {
    // Still far from target, climb at max rate
    data.pitchCommand = 8.0;
  } else if (altitudeError > 0) {
    // Near target, pitch based on error
    data.pitchCommand = (altitudeError / 100.0) * 8.0;
  } else {
    // At or above target, level off
    data.pitchCommand = 0.0;
  }
  
  // Maintain heading during climb
  calculateHeadingHold();
}

// Descent Mode - Descend to target altitude
void FlightDirector::calculateDescent() {
  float altitudeError = data.desiredAltitude - data.currentAltitude;
  
  if (altitudeError < -100.0) {
    // Still far from target, descend at max rate
    data.pitchCommand = -8.0;
  } else if (altitudeError < 0) {
    // Near target, pitch based on error
    data.pitchCommand = (altitudeError / 100.0) * 8.0;
  } else {
    // At or below target, level off
    data.pitchCommand = 0.0;
  }
  
  // Maintain heading during descent
  calculateHeadingHold();
}

// GPS Track Mode - Follow GPS navigation track
void FlightDirector::calculateGpsTrack() {
  // Similar to heading hold but using GPS track
  // In this case, desiredHeading should be set from GPS track
  calculateHeadingHold();
  
  // Maintain current altitude
  data.pitchCommand = 0.0;
}

// ============================================================================
// ILS CAPTURE LOGIC
// ============================================================================

void FlightDirector::updateILSCapture() {
  // Localizer capture
  if (!data.localizerCaptured && fabs(data.localizerDeviation) < LOC_CAPTURE_THRESHOLD) {
    data.localizerCaptured = true;
    Serial.println("Localizer CAPTURED");
  }
  
  if (data.localizerCaptured && fabs(data.localizerDeviation) > (LOC_CAPTURE_THRESHOLD * 2)) {
    data.localizerCaptured = false;
    Serial.println("Localizer LOST");
  }
  
  // Glide Slope capture
  if (!data.glideSlopeCapture && fabs(data.glideSlopeDeviation) < GS_CAPTURE_THRESHOLD) {
    data.glideSlopeCapture = true;
    Serial.println("Glide Slope CAPTURED");
  }
  
  if (data.glideSlopeCapture && fabs(data.glideSlopeDeviation) > (GS_CAPTURE_THRESHOLD * 2)) {
    data.glideSlopeCapture = false;
    Serial.println("Glide Slope LOST");
  }
}

// ============================================================================
// GETTERS
// ============================================================================

float FlightDirector::getRollCommand() {
  return data.rollCommand;
}

float FlightDirector::getPitchCommand() {
  return data.pitchCommand;
}

bool FlightDirector::isLocalizerCaptured() {
  return data.localizerCaptured;
}

bool FlightDirector::isGlideSlopeCaptured() {
  return data.glideSlopeCapture;
}

FlightDirectorData FlightDirector::getData() {
  return data;
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

void FlightDirector::printStatus() {
  Serial.print("FD[Mode:");
  Serial.print(data.mode);
  Serial.print("] Roll:");
  Serial.print(data.rollCommand, 1);
  Serial.print("° Pitch:");
  Serial.print(data.pitchCommand, 1);
  Serial.print("° LOC:");
  Serial.print(data.localizerCaptured ? "Y" : "N");
  Serial.print(" GS:");
  Serial.println(data.glideSlopeCapture ? "Y" : "N");
}
