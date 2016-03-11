#include <Inertial.h>
#include <iostream>

using namespace std;

Inertial::Inertial() {
  RotationRate    = 0.00007292115;
  GM              = 14.0764417572E15;   // WGS84 value
  C2_0            = -4.84165371736E-04; // WGS84 value for the C2,0 coefficient
  J2              = 1.08262982E-03;     // WGS84 value for J2
  a               = 20925646.32546;     // WGS84 semimajor axis length in feet
  b               = 20855486.5951;      // WGS84 semiminor axis length in feet
  RadiusReference = a;
  vOmegaPlanet = ColumnVector3( 0.0, 0.0, RotationRate );
  gAccelReference = GM /(RadiusReference*RadiusReference);
  gAccel          = GM /(RadiusReference*RadiusReference);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Inertial::~Inertial(){}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Inertial::SLgravity(void) const {
  return gAccelReference;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Inertial::omega(void) const {
  return RotationRate;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const ColumnVector3& Inertial::GetOmegaPlanet() const {
  return vOmegaPlanet;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Inertial::GetRefRadius(void) const {
  return RadiusReference;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Inertial::GetSemiMajor(void) const {
  return a;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Inertial::GetSemiMinor(void) const {
  return b;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Inertial::GetGAccel(double r) const {
  return GM/(r*r);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3 Inertial::GetGravityJ2(const Location& location) const {
  ColumnVector3 J2Gravity;

  // Gravitation accel
  double r = location.GetRadius();
  double sinLat = sin(location.GetLatitude());

  double adivr = a/r;
  double preCommon = 1.5*J2*adivr*adivr;
  double xy = 1.0 - 5.0*(sinLat*sinLat);
  double z = 3.0 - 5.0*(sinLat*sinLat);
  double GMOverr2 = GM/(r*r);

  J2Gravity(1) = -GMOverr2 * ((1.0 + (preCommon * xy)) * location(eX)/r);
  J2Gravity(2) = -GMOverr2 * ((1.0 + (preCommon * xy)) * location(eY)/r);
  J2Gravity(3) = -GMOverr2 * ((1.0 + (preCommon *  z)) * location(eZ)/r);

  return J2Gravity;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%