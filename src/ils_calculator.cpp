/* ============================================================================
 * ILS CALCULATOR IMPLEMENTATION
 * ============================================================================
 */

#include "ils_calculator.h"

// ============================================================================
// STATIC CONSTANTS
// ============================================================================

const float ILSCalculator::EARTH_RADIUS_NM = 3440.065;  // Nautical miles
const float ILSCalculator::EARTH_RADIUS_M = 6371000.0;  // Meters
const float ILSCalculator::NM_TO_METERS = 1852.0;
const float ILSCalculator::METERS_TO_FEET = 3.28084;
const float ILSCalculator::DEG_TO_RAD = PI / 180.0;
const float ILSCalculator::RAD_TO_DEG = 180.0 / PI;

// ============================================================================
// CONSTRUCTOR
// ============================================================================

ILSCalculator::ILSCalculator() {
  // Initialize with default runway (example: KDFW runway 18R)
  runway.latitude = 32.8975;       // Dallas/Fort Worth
  runway.longitude = -97.0380;
  runway.elevation = 600;          // feet
  runway.heading = 180.0;          // Due south
  runway.length = 13401;           // meters (13,401 feet)
  runway.glideSlopeAngle = 3.0;    // Standard 3-degree ILS
  runway.localizerWidth = 2.5;     // Standard localizer width
  runway.gateDistanceNM = 5.0;     // 5 NM initial approach fix
  runway.gateAltitude = 2000;      // 2000 feet at gate
  
  memset(&deviations, 0, sizeof(ILSDeviations));
}

// ============================================================================
// RUNWAY SETUP
// ============================================================================

void ILSCalculator::setRunway(const Runway& newRunway) {
  runway = newRunway;
  Serial.println("ILS Runway updated");
}

void ILSCalculator::setRunwayByName(const char* name) {
  // Preset runways - add more as needed
  if (strcmp(name, "KDFW18R") == 0) {
    runway.latitude = 32.8975;
    runway.longitude = -97.0380;
    runway.heading = 180.0;
    runway.elevation = 600;
  } else if (strcmp(name, "KJFK04L") == 0) {
    runway.latitude = 40.6413;
    runway.longitude = -73.7781;
    runway.heading = 40.0;
    runway.elevation = 13;
  } else if (strcmp(name, "KLAX25L") == 0) {
    runway.latitude = 33.9425;
    runway.longitude = -118.4081;
    runway.heading = 250.0;
    runway.elevation = 285;
  }
  Serial.print("ILS Runway set to: ");
  Serial.println(name);
}

// ============================================================================
// MAIN CALCULATION
// ============================================================================

void ILSCalculator::calculate(float currentLat, float currentLon, float currentAltitude) {
  // Calculate distance and bearing to runway
  deviations.distanceToThresholdMeters = calculateDistance(currentLat, currentLon,
                                                           runway.latitude, runway.longitude);
  deviations.distanceToThreshold = deviations.distanceToThresholdMeters / NM_TO_METERS;
  
  deviations.bearingToThreshold = calculateBearing(currentLat, currentLon,
                                                    runway.latitude, runway.longitude);
  
  // Check if approaching runway
  static float lastDistance = 999999;
  deviations.approachingRunway = (deviations.distanceToThresholdMeters < lastDistance);
  lastDistance = deviations.distanceToThresholdMeters;
  
  // ========== LOCALIZER CALCULATION (Lateral Deviation) ==========
  
  // Calculate cross-track error using three points:
  // 1. Initial approach fix (IAF) at 5 NM out
  // 2. Runway threshold
  // 3. Current position
  float iafDistance = runway.gateDistanceNM * NM_TO_METERS * 2;  // Further out
  float iafLat = runway.latitude + ((iafDistance / EARTH_RADIUS_M) * RAD_TO_DEG * cos(runway.heading * DEG_TO_RAD));
  float iafLon = runway.longitude + ((iafDistance / EARTH_RADIUS_M) * RAD_TO_DEG * sin(runway.heading * DEG_TO_RAD) / cos(iafLat * DEG_TO_RAD));
  
  // Cross-track error from runway centerline
  float crossTrackMeters = calculateCrossTrackError(iafLat, iafLon,
                                                    runway.latitude, runway.longitude,
                                                    currentLat, currentLon);
  
  deviations.crossTrackError = crossTrackMeters;
  
  // Convert to angular deviation
  // Full scale is typically 2.5 degrees for a 5 NM approach
  float fullScaleDistance = runway.gateDistanceNM * NM_TO_METERS;
  float angularDeviation = atan(crossTrackMeters / max(fullScaleDistance, 1000.0)) * RAD_TO_DEG;
  
  deviations.localizerDeviation = constrainValue(angularDeviation, -1.25, 1.25);
  deviations.localizerNeedlePosition = deviations.localizerDeviation / 1.25;
  
  // ========== GLIDE SLOPE CALCULATION (Vertical Deviation) ==========
  
  // Expected altitude at current distance using 3-degree glide slope
  float altitudeGain = deviations.distanceToThresholdMeters * 
                      tan(runway.glideSlopeAngle * DEG_TO_RAD);
  
  float expectedAltitude = runway.elevation + altitudeGain;
  
  // Altitude error in feet
  float altitudeErrorMeters = currentAltitude - expectedAltitude;
  float altitudeErrorFeet = altitudeErrorMeters * METERS_TO_FEET;
  
  deviations.altitudeError = altitudeErrorMeters;
  deviations.glideSlopeDeviation = constrainValue(altitudeErrorFeet, -500, 500);
  deviations.glideSlopeNeedlePosition = deviations.glideSlopeDeviation / 500.0;
  
  // ========== CAPTURE ZONE CHECKS ==========
  
  // Localizer capture: within 0.2 degrees and 2 NM
  deviations.canCaptureLocalizer = (fabs(deviations.localizerDeviation) < 0.2) &&
                                    (deviations.distanceToThreshold < 2.0);
  
  // Glide slope capture: within 150 feet and on localizer
  deviations.canCaptureGlideslope = (fabs(deviations.glideSlopeDeviation) < 150) &&
                                     deviations.canCaptureLocalizer;
}

// ============================================================================
// APPROACH GATE CHECKS
// ============================================================================

bool ILSCalculator::isInInitialApproach() {
  // Initial approach fix: 5-10 NM from threshold
  return (deviations.distanceToThreshold >= 5.0) && 
         (deviations.distanceToThreshold <= 10.0);
}

bool ILSCalculator::isInIntermediateApproach() {
  // Intermediate approach: 2-5 NM
  return (deviations.distanceToThreshold >= 2.0) && 
         (deviations.distanceToThreshold < 5.0);
}

bool ILSCalculator::isInFinalApproach() {
  // Final approach: 0-2 NM
  return (deviations.distanceToThreshold < 2.0);
}

// ============================================================================
// GETTERS
// ============================================================================

ILSDeviations ILSCalculator::getDeviations() {
  return deviations;
}

float ILSCalculator::getLocalizerDeviation() {
  return deviations.localizerDeviation;
}

float ILSCalculator::getGlideSlopeDeviation() {
  return deviations.glideSlopeDeviation;
}

float ILSCalculator::getDistanceToThreshold() {
  return deviations.distanceToThreshold;
}

float ILSCalculator::getAltitudeError() {
  return deviations.altitudeError;
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

void ILSCalculator::printRunwayInfo() {
  Serial.println("\n=== RUNWAY INFORMATION ===");
  Serial.print("Position: ");
  Serial.print(runway.latitude, 4);
  Serial.print(", ");
  Serial.println(runway.longitude, 4);
  Serial.print("Heading: ");
  Serial.print(runway.heading);
  Serial.println("°");
  Serial.print("Glide Slope: ");
  Serial.print(runway.glideSlopeAngle);
  Serial.println("°");
}

void ILSCalculator::printDeviations() {
  Serial.println("\n=== ILS DEVIATIONS ===");
  Serial.print("Distance: ");
  Serial.print(deviations.distanceToThreshold, 2);
  Serial.println(" NM");
  Serial.print("Localizer: ");
  Serial.print(deviations.localizerDeviation, 3);
  Serial.println("°");
  Serial.print("Glide Slope: ");
  Serial.print(deviations.glideSlopeDeviation, 1);
  Serial.println(" ft");
  Serial.print("Altitude Error: ");
  Serial.print(deviations.altitudeError, 1);
  Serial.println(" m");
}

// ============================================================================
// HELPER FUNCTIONS - GEOGRAPHIC CALCULATIONS
// ============================================================================

// Haversine formula for great circle distance
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  float dLat = (lat2 - lat1) * ILSCalculator::DEG_TO_RAD;
  float dLon = (lon2 - lon1) * ILSCalculator::DEG_TO_RAD;
  
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(lat1 * ILSCalculator::DEG_TO_RAD) * cos(lat2 * ILSCalculator::DEG_TO_RAD) *
            sin(dLon / 2) * sin(dLon / 2);
  
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = ILSCalculator::EARTH_RADIUS_M * c;
  
  return distance;  // Returns meters
}

// Calculate bearing between two points
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = (lon2 - lon1) * ILSCalculator::DEG_TO_RAD;
  float y = sin(dLon) * cos(lat2 * ILSCalculator::DEG_TO_RAD);
  float x = cos(lat1 * ILSCalculator::DEG_TO_RAD) * sin(lat2 * ILSCalculator::DEG_TO_RAD) -
            sin(lat1 * ILSCalculator::DEG_TO_RAD) * cos(lat2 * ILSCalculator::DEG_TO_RAD) * cos(dLon);
  
  float bearing = atan2(y, x) * ILSCalculator::RAD_TO_DEG;
  bearing = fmod(bearing + 360, 360);  // Normalize to 0-360
  
  return bearing;
}

// Calculate cross-track error
float calculateCrossTrackError(float lat1, float lon1, 
                               float lat2, float lon2,
                               float lat3, float lon3) {
  // Distance from current position to the course line
  float d12 = calculateDistance(lat1, lon1, lat2, lon2);
  float d13 = calculateDistance(lat1, lon1, lat3, lon3);
  float bearing12 = calculateBearing(lat1, lon1, lat2, lon2) * ILSCalculator::DEG_TO_RAD;
  float bearing13 = calculateBearing(lat1, lon1, lat3, lon3) * ILSCalculator::DEG_TO_RAD;
  
  float crossTrack = asin(sin(d13 / ILSCalculator::EARTH_RADIUS_M) * sin(bearing13 - bearing12)) * 
                     ILSCalculator::EARTH_RADIUS_M;
  
  return crossTrack;  // Negative = left, Positive = right
}

float constrainValue(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}
