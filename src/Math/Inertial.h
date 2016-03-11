#pragma once

#include <vector>
#include <Base.h>
#include <ColumnVector3.h>
#include <Location.h>

class Inertial {
public:
  Inertial();
  ~Inertial();

  double SLgravity(void) const;
  double omega(void) const;
  const ColumnVector3& GetOmegaPlanet() const;

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Calculates G acceleration vector based on radius from the Earth center
  double GetGAccel(double r) const;

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //
  // Calculate the WGS84 gravitation value in ECEF frame. Pass in the ECEF position
  // via the position parameter. The J2Gravity value returned is in ECEF frame,
  // and therefore may need to be expressed (transformed) in another frame,
  // depending on how it is used. See Stevens and Lewis eqn. 1.4-16.
  ColumnVector3 GetGravityJ2(const Location& location) const;

  double GetRefRadius(void) const;
  double GetSemiMajor(void) const;
  double GetSemiMinor(void) const;

private:
  ColumnVector3 vOmegaPlanet;
  double gAccel;
  double gAccelReference;
  double RadiusReference;
  double RotationRate;
  double GM;
  double C2_0; // WGS84 value for the C2,0 coefficient
  double J2;   // WGS84 value for J2
  double a;    // WGS84 semimajor axis length in feet 
  double b;    // WGS84 semiminor axis length in feet
};