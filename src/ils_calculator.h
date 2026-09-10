/* ============================================================================
 * ILS CALCULATOR - INSTRUMENT LANDING SYSTEM
 * 
 * Calculates localizer and glide slope deviations for precision approaches.
 * Provides runway geometry, approach path calculations, and needle deflections.
 * 
 * ============================================================================
 */

#ifndef ILS_CALCULATOR_H
#define ILS_CALCULATOR_H

#include <Arduino.h>
#include <math.h>

// ============================================================================
// RUNWAY DEFINITION
// ============================================================================

struct Runway {
  float latitude;                  // Runway threshold latitude
  float longitude;                 // Runway threshold longitude
  float elevation;                 // Runway elevation (meters)
  float heading;                   // Runway magnetic heading (0-360 degrees)
  float length;                    // Runway length (meters)
  float glideSlopeAngle;           // ILS glide slope (typically 3.0 degrees)
  float localizerWidth;            // Full width of localizer beam (2.5 degrees)
  
  // Approach gate position (initial approach fix)
  float gateDistanceNM;            // Distance from threshold (nautical miles)
  float gateAltitude;              // Expected altitude at gate
};

// ============================================================================
// ILS DEVIATION DATA
// ============================================================================

struct ILSDeviations {
  // Localizer (lateral)
  float localizerDeviation;        // Degrees from centerline (-1.25 to +1.25)
  float localizerNeedlePosition;   // Normalized: -1.0 (left) to +1.0 (right)
  float crossTrackError;           // Meters left(-) or right(+)
  
  // Glide Slope (vertical)
  float glideSlopeDeviation;       // Feet from 3-degree path (-500 to +500)
  float glideSlopeNeedlePosition;  // Normalized: -1.0 (below) to +1.0 (above)
  float altitudeError;             // Meters from desired altitude
  
  // Distance and approach state
  float distanceToThreshold;       // Nautical miles to runway
  float distanceToThresholdMeters; // Distance in meters
  float bearingToThreshold;        // Magnetic bearing to runway (degrees)
  float trackFromThreshold;        // Track from runway center
  bool approachingRunway;          // Getting closer?
  bool canCaptureLocalizer;        // Within localizer capture zone
  bool canCaptureGlideslope;       // Within glide slope capture zone
};

// ============================================================================
// ILS CALCULATOR CLASS
// ============================================================================

class ILSCalculator {
  
private:
  Runway runway;
  ILSDeviations deviations;
  
  // Earth constants
  static const float EARTH_RADIUS_NM;      // Earth radius in nautical miles
  static const float EARTH_RADIUS_M;       // Earth radius in meters
  static const float NM_TO_METERS;         // Conversion factor
  
  // Conversion constants
  static const float METERS_TO_FEET;
  static const float DEG_TO_RAD;
  static const float RAD_TO_DEG;
  
public:
  ILSCalculator();
  
  // Initialize with runway data
  void setRunway(const Runway& newRunway);
  void setRunwayByName(const char* name);  // Preset runways
  
  // Calculate deviations
  void calculate(float currentLat, float currentLon, float currentAltitude);
  
  // Get results
  ILSDeviations getDeviations();
  float getLocalizerDeviation();
  float getGlideSlopeDeviation();
  float getDistanceToThreshold();
  float getAltitudeError();
  
  // Check approach gates
  bool isInInitialApproach();
  bool isInIntermediateApproach();
  bool isInFinalApproach();
  
  // Debug
  void printRunwayInfo();
  void printDeviations();
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Calculate distance between two GPS points (haversine formula)
float calculateDistance(float lat1, float lon1, float lat2, float lon2);

// Calculate bearing from point1 to point2
float calculateBearing(float lat1, float lon1, float lat2, float lon2);

// Calculate cross-track error
float calculateCrossTrackError(float lat1, float lon1, 
                               float lat2, float lon2,
                               float lat3, float lon3);

// Normalize angle
float normalizeAngle(float angle);

#endif // ILS_CALCULATOR_H
