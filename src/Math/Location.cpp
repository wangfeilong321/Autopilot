#include <Location.h>
#include <cmath>

Location::Location(void) : mECLoc(1.0, 0.0, 0.0), mCacheValid(false) {
	SetEllipse();
  epa = 0.0;

  mLon = mLat = mRadius = 0.0;
  mGeodLat = GeodeticAltitude = 0.0;

  mTl2ec.InitMatrix();
  mTec2l.InitMatrix();
  mTi2ec.InitMatrix();
  mTec2i.InitMatrix();
  mTi2l.InitMatrix();
  mTl2i.InitMatrix();
}

Location::Location(double lon, double lat, double radius) : mCacheValid(false) {
	SetEllipse();
  epa = 0.0;

  mLon = mLat = mRadius = 0.0;
  mGeodLat = GeodeticAltitude = 0.0;

  mTl2ec.InitMatrix();
  mTec2l.InitMatrix();
  mTi2ec.InitMatrix();
  mTec2i.InitMatrix();
  mTi2l.InitMatrix();
  mTl2i.InitMatrix();

  double sinLat = sin(lat);
  double cosLat = cos(lat);
  double sinLon = sin(lon);
  double cosLon = cos(lon);
  mECLoc = ColumnVector3( radius*cosLat*cosLon,
                          radius*cosLat*sinLon,
                          radius*sinLat );
}

Location::Location(const ColumnVector3& lv) : mECLoc(lv), mCacheValid(false) {
	SetEllipse();
  epa = 0.0;

  mLon = mLat = mRadius = 0.0;
  mGeodLat = GeodeticAltitude = 0.0;

  mTl2ec.InitMatrix();
  mTec2l.InitMatrix();
  mTi2ec.InitMatrix();
  mTec2i.InitMatrix();
  mTi2l.InitMatrix();
  mTl2i.InitMatrix();
}

Location::Location(const Location& l) : mECLoc(l.mECLoc), mCacheValid(l.mCacheValid) {
  a = l.a;
  e2 = l.e2;
  c = l.c;
  ec = l.ec;
  ec2 = l.ec2;
  epa = l.epa;

  /*ag
   * if the cache is not valid, all of the following values are unset.
   * They will be calculated once ComputeDerivedUnconditional is called.
   * If unset, they may possibly contain NaN and could thus trigger floating
   * point exceptions.
   */
  if (!mCacheValid) return;

  mLon = l.mLon;
  mLat = l.mLat;
  mRadius = l.mRadius;

  mTl2ec = l.mTl2ec;
  mTec2l = l.mTec2l;
  mTi2ec = l.mTi2ec;
  mTec2i = l.mTec2i;
  mTi2l = l.mTi2l;
  mTl2i = l.mTl2i;

  mGeodLat = l.mGeodLat;
  GeodeticAltitude = l.GeodeticAltitude;
}

const Location& Location::operator=(const Location& l) {
  mECLoc = l.mECLoc;
  mCacheValid = l.mCacheValid;

  a = l.a;
  e2 = l.e2;
  c = l.c;
  ec = l.ec;
  ec2 = l.ec2;
  epa = l.epa;

  //ag See comment in constructor above
  if (!mCacheValid) return *this;

  mLon = l.mLon;
  mLat = l.mLat;
  mRadius = l.mRadius;

  mTl2ec = l.mTl2ec;
  mTec2l = l.mTec2l;
  mTi2ec = l.mTi2ec;
  mTec2i = l.mTec2i;
  mTi2l = l.mTi2l;
  mTl2i = l.mTl2i;

  mGeodLat = l.mGeodLat;
  GeodeticAltitude = l.GeodeticAltitude;

  return *this;
}

void Location::SetLongitude(double longitude) {
  double rtmp = mECLoc.Magnitude(eX, eY);
  // Check if we have zero radius.
  // If so set it to 1, so that we can set a position
  if (0.0 == mECLoc.Magnitude())
    rtmp = 1.0;

  // Fast return if we are on the north or south pole ...
  if (rtmp == 0.0)
    return;

  mCacheValid = false;

  mECLoc(eX) = rtmp*cos(longitude);
  mECLoc(eY) = rtmp*sin(longitude);
}

void Location::SetLatitude(double latitude) {
  mCacheValid = false;

  double r = mECLoc.Magnitude();
  if (r == 0.0) {
    mECLoc(eX) = 1.0;
    r = 1.0;
  }

  double rtmp = mECLoc.Magnitude(eX, eY);
  if (rtmp != 0.0) {
    double fac = r/rtmp*cos(latitude);
    mECLoc(eX) *= fac;
    mECLoc(eY) *= fac;
  } else {
    mECLoc(eX) = r*cos(latitude);
    mECLoc(eY) = 0.0;
  }
  mECLoc(eZ) = r*sin(latitude);
}

void Location::SetRadius(double radius) {
  mCacheValid = false;

  double rold = mECLoc.Magnitude();
  if (rold == 0.0)
    mECLoc(eX) = radius;
  else
    mECLoc *= radius/rold;
}

void Location::SetPosition(double lon, double lat, double radius) {
  mCacheValid = false;

  double sinLat = sin(lat);
  double cosLat = cos(lat);
  double sinLon = sin(lon);
  double cosLon = cos(lon);

  mECLoc = ColumnVector3( radius*cosLat*cosLon,
                          radius*cosLat*sinLon,
                          radius*sinLat );
}

void Location::SetPositionGeodetic(double lon, double lat, double height) {
  mCacheValid = false;

  double slat = sin(lat);
  double clat = cos(lat);
  double RN = a / sqrt(1.0 - e2*slat*slat);

  mECLoc(eX) = (RN + height)*clat*cos(lon);
  mECLoc(eY) = (RN + height)*clat*sin(lon);
  mECLoc(eZ) = ((1 - e2)*RN + height)*slat;
}

void Location::SetEllipse() {
  mCacheValid = false;

  ec = b/a;
  ec2 = ec * ec;
  e2 = 1.0 - ec2;
  c = a * e2;
}

void Location::SetEarthPositionAngle(double EPA) { epa = EPA; mCacheValid = false; }

void Location::IncrementEarthPositionAngle(double delta) { epa += delta; mCacheValid = false; }

double Location::GetLongitude() const { ComputeDerived(); return mLon; }

double Location::GetLongitudeDeg() const { ComputeDerived(); return radtodeg*mLon; }

double Location::GetSinLongitude() const { ComputeDerived(); return -mTec2l(2, 1); }

double Location::GetCosLongitude() const { ComputeDerived(); return mTec2l(2, 2); }

double Location::GetLatitude() const { ComputeDerived(); return mLat; }

double Location::GetGeodLatitudeRad(void) const { ComputeDerived(); return mGeodLat; }

double Location::GetLatitudeDeg() const { ComputeDerived(); return radtodeg*mLat; }

double Location::GetGeodLatitudeDeg(void) const { ComputeDerived(); return radtodeg*mGeodLat; }

double Location::GetGeodAltitude(void) const { ComputeDerived(); return GeodeticAltitude; }

double Location::GetSinLatitude() const { ComputeDerived(); return -mTec2l(3, 3); }

double Location::GetCosLatitude() const { ComputeDerived(); return mTec2l(1, 3); }

double Location::GetTanLatitude() const {
	ComputeDerived();
	double cLat = mTec2l(1, 3);
	if (cLat == 0.0)
		return 0.0;
	else
		return -mTec2l(3, 3) / cLat;
}

double Location::GetEPA() const { return epa; }

double Location::GetRadius() const { ComputeDerived(); return mRadius; }

const Matrix33& Location::GetTl2ec(void) const { ComputeDerived(); return mTl2ec; }

const Matrix33& Location::GetTec2l(void) const { ComputeDerived(); return mTec2l; }

const Matrix33& Location::GetTi2ec(void) const { ComputeDerived(); return mTi2ec; }

const Matrix33& Location::GetTec2i(void) const { ComputeDerived(); return mTec2i; }

const Matrix33& Location::GetTi2l(void) const { ComputeDerived(); return mTi2l; }

const Matrix33& Location::GetTl2i(void) const { ComputeDerived(); return mTl2i; }

Location Location::LocalToLocation(const ColumnVector3& lvec) const {
	ComputeDerived(); return mTl2ec*lvec + mECLoc;
}

ColumnVector3 Location::LocationToLocal(const ColumnVector3& ecvec) const {
	ComputeDerived(); return mTec2l*(ecvec - mECLoc);
}

double Location::operator()(unsigned int idx) const { return mECLoc.Entry(idx); }

double& Location::operator()(unsigned int idx) { mCacheValid = false; return mECLoc.Entry(idx); }

double Location::Entry(unsigned int idx) const { return mECLoc.Entry(idx); }

double& Location::Entry(unsigned int idx) {
	mCacheValid = false; return mECLoc.Entry(idx);
}

const Location& Location::operator=(const ColumnVector3& v) {
	mECLoc(eX) = v(eX);
	mECLoc(eY) = v(eY);
	mECLoc(eZ) = v(eZ);
	mCacheValid = false;
	//ComputeDerived();
	return *this;
}

bool Location::operator==(const Location& l) const {
	return mECLoc == l.mECLoc;
}

bool Location::operator!=(const Location& l) const { return !operator==(l); }

const Location& Location::operator+=(const Location &l) {
	mCacheValid = false;
	mECLoc += l.mECLoc;
	return *this;
}

const Location& Location::operator-=(const Location &l) {
	mCacheValid = false;
	mECLoc -= l.mECLoc;
	return *this;
}

const Location& Location::operator*=(double scalar) {
	mCacheValid = false;
	mECLoc *= scalar;
	return *this;
}

const Location& Location::operator/=(double scalar) {
	return operator*=(1.0 / scalar);
}

Location Location::operator+(const Location& l) const {
	return Location(mECLoc + l.mECLoc);
}

Location Location::operator-(const Location& l) const {
	return Location(mECLoc - l.mECLoc);
}

Location Location::operator*(double scalar) const {
	return Location(scalar*mECLoc);
}

Location::operator const ColumnVector3&() const {
	return mECLoc;
}

void Location::ComputeDerivedUnconditional(void) const {
  // The radius is just the Euclidean norm of the vector.
  mRadius = mECLoc.Magnitude();

  // The distance of the location to the Z-axis, which is the axis
  // through the poles.
  double r02 = mECLoc(eX)*mECLoc(eX) + mECLoc(eY)*mECLoc(eY);
  double rxy = sqrt(r02);

  // Compute the sin/cos values of the longitude
  double sinLon, cosLon;
  if (rxy == 0.0) {
    sinLon = 0.0;
    cosLon = 1.0;
  } else {
    sinLon = mECLoc(eY)/rxy;
    cosLon = mECLoc(eX)/rxy;
  }

  // Compute the sin/cos values of the latitude
  double sinLat, cosLat;
  if (mRadius == 0.0)  {
    sinLat = 0.0;
    cosLat = 1.0;
  } else {
    sinLat = mECLoc(eZ)/mRadius;
    cosLat = rxy/mRadius;
  }

  // Compute the longitude and latitude itself
  if ( mECLoc( eX ) == 0.0 && mECLoc( eY ) == 0.0 )
    mLon = 0.0;
  else
    mLon = atan2( mECLoc( eY ), mECLoc( eX ) );

  if ( rxy == 0.0 && mECLoc( eZ ) == 0.0 )
    mLat = 0.0;
  else
    mLat = atan2( mECLoc(eZ), rxy );

  // Compute the transform matrices from and to the earth centered frame.
  // See Stevens and Lewis, "Aircraft Control and Simulation", Second Edition,
  // Eqn. 1.4-13, page 40. In Stevens and Lewis notation, this is C_n/e - the
  // orientation of the navigation (local) frame relative to the ECEF frame,
  // and a transformation from ECEF to nav (local) frame.

  mTec2l = Matrix33( -cosLon*sinLat, -sinLon*sinLat,  cosLat,
                     -sinLon   ,     cosLon    ,    0.0 ,
                     -cosLon*cosLat, -sinLon*cosLat, -sinLat  );

  // In Stevens and Lewis notation, this is C_e/n - the
  // orientation of the ECEF frame relative to the nav (local) frame,
  // and a transformation from nav (local) to ECEF frame.

  mTl2ec = mTec2l.Transposed();

  // Calculate the inertial to ECEF and transpose matrices
  double cos_epa = cos(epa);
  double sin_epa = sin(epa);
  mTi2ec = Matrix33( cos_epa, sin_epa, 0.0,
                    -sin_epa, cos_epa, 0.0,
                         0.0,     0.0, 1.0 );
  mTec2i = mTi2ec.Transposed();

  // Now calculate the local (or nav, or ned) frame to inertial transform matrix,
  // and the inverse.
  mTl2i = mTec2i * mTl2ec;
  mTi2l = mTl2i.Transposed();

  // Calculate the geodetic latitude based on "Transformation from Cartesian
  // to geodetic coordinates accelerated by Halley's method", Fukushima T. (2006)
  // Journal of Geodesy, Vol. 79, pp. 689-693
  // Unlike I. Sofair's method which uses a closed form solution, Fukushima's
  // method is an iterative method whose convergence is so fast that only one
  // iteration suffices. In addition, Fukushima's method has a much better
  // numerical stability over Sofair's method at the North and South poles and
  // it also gives the correct result for a spherical Earth.

  double s0 = fabs(mECLoc(eZ));
  double zc = ec * s0;
  double c0 = ec * rxy;
  double c02 = c0 * c0;
  double s02 = s0 * s0;
  double a02 = c02 + s02;
  double a0 = sqrt(a02);
  double a03 = a02 * a0;
  double s1 = zc*a03 + c*s02*s0;
  double c1 = rxy*a03 - c*c02*c0;
  double cs0c0 = c*c0*s0;
  double b0 = 1.5*cs0c0*((rxy*s0-zc*c0)*a0-cs0c0);
  s1 = s1*a03-b0*s0;
  double cc = ec*(c1*a03-b0*c0);
  mGeodLat = sign(mECLoc(eZ))*atan(s1 / cc);
  double s12 = s1 * s1;
  double cc2 = cc * cc;
  GeodeticAltitude = (rxy*cc + s0*s1 - a*sqrt(ec2*s12 + cc2)) / sqrt(s12 + cc2);

  // Mark the cached values as valid
  mCacheValid = true;
}

void Location::ComputeDerived(void) const {
	if (!mCacheValid)
		ComputeDerivedUnconditional();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//  The calculations, below, implement the Haversine formulas to calculate
//  heading and distance to a set of lat/long coordinates from the current
//  position.
//
//  The basic equations are (lat1, long1 are source positions; lat2
//  long2 are target positions):
//
//  R = earth’s radius
//  Δlat = lat2 − lat1
//  Δlong = long2 − long1
//
//  For the waypoint distance calculation:
//
//  a = sin²(Δlat/2) + cos(lat1)∙cos(lat2)∙sin²(Δlong/2)
//  c = 2∙atan2(√a, √(1−a))
//  d = R∙c

double Location::GetDistanceTo(double target_longitude, double target_latitude) const {
  double delta_lat_rad = target_latitude  - GetLatitude();
  double delta_lon_rad = target_longitude - GetLongitude();

  double distance_a = pow(sin(0.5*delta_lat_rad), 2.0)
    + (GetCosLatitude() * cos(target_latitude)
       * (pow(sin(0.5*delta_lon_rad), 2.0)));

  return 2.0 * GetRadius() * atan2(sqrt(distance_a), sqrt(1.0 - distance_a));
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//  The calculations, below, implement the Haversine formulas to calculate
//  heading and distance to a set of lat/long coordinates from the current
//  position.
//
//  The basic equations are (lat1, long1 are source positions; lat2
//  long2 are target positions):
//
//  R = earth’s radius
//  Δlat = lat2 − lat1
//  Δlong = long2 − long1
//
//  For the heading angle calculation:
//
//  θ = atan2(sin(Δlong)∙cos(lat2), cos(lat1)∙sin(lat2) − sin(lat1)∙cos(lat2)∙cos(Δlong))

double Location::GetHeadingTo(double target_longitude, double target_latitude) const {
  double delta_lon_rad = target_longitude - GetLongitude();

  double Y = sin(delta_lon_rad) * cos(target_latitude);
  double X = GetCosLatitude() * sin(target_latitude)
    - GetSinLatitude() * cos(target_latitude) * cos(delta_lon_rad);

  double heading_to_waypoint_rad = atan2(Y, X);
  if (heading_to_waypoint_rad < 0) heading_to_waypoint_rad += 2.0*M_PI;

  return heading_to_waypoint_rad;
}