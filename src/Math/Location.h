#pragma once

#include <Base.h>
#include <ColumnVector3.h>
#include <Matrix33.h>

/** Location holds an arbitrary location in the Earth centered Earth fixed
    reference frame (ECEF). The coordinate frame ECEF has its center in the middle
    of the earth. The X-axis points from the center of the Earth towards a
    location with zero latitude and longitude on the Earth surface. The Y-axis
    points from the center of the Earth towards a location with zero latitude
    and 90 deg East longitude on the Earth surface. The Z-axis points from the
    Earth center to the geographic north pole.

    This class provides access functions to set and get the location as either
    the simple X, Y and Z values in ft or longitude/latitude and the radial
    distance of the location from the Earth center.

    It is common to associate a parent frame with a location. This frame is
    usually called the local horizontal frame or simply the local frame. It is
    also called the NED frame (North, East, Down), as well as the Navigation
    frame. This frame has its X/Y plane parallel to the surface of the Earth
    (with the assumption of a spherical Earth). The X-axis points towards north,
    the Y-axis points east and the Z-axis points to the center of the Earth.

    Since the local frame is determined by the location (and NOT by the
    orientation of the  vehicle IN any frame), this class also provides the
    rotation matrices required to transform from the Earth centered (ECEF) frame
    to the local horizontal frame and back. This class also "owns" the
    transformations that go from the inertial frame (Earth-centered Inertial, or
    ECI) to and from the ECEF frame, as well as to and from the local frame.
    Again, this is because the ECI, ECEF, and local frames do not involve the
    actual orientation of the vehicle - only the location on the Earth surface,
    and the angular difference between the ECI and ECEF frames. There are
    conversion functions for conversion of position vectors given in the one
    frame to positions in the other frame.

    To keep the transformation matrices between the ECI and ECEF frames up to
    date, the Earth angular position must be updated by calling
    SetEarthPositionAngle() or IncrementEarthPositionAngle(). This must be done
    prior to any conversion from and to the ECI frame.

    The Earth centered reference frame is NOT an inertial frame since it rotates
    with the Earth.

    The cartesian coordinates (X,Y,Z) in the Earth centered frame are the master values. All other
    values are computed from these master values and are cached as long as the
    location is changed by access through a non-const member function. Values
    are cached to improve performance. It is best practice to work with a
    natural set of master values. Other parameters that are derived from these
    master values are calculated only when needed, and IF they are needed and
    calculated, then they are cached (stored and remembered) so they do not need
    to be re-calculated until the master values they are derived from are
    themselves changed (and become stale).

    Accuracy and round off

    Given,

    - that we model a vehicle near the Earth
    - that the Earth surface radius is about 2*10^7, ft
    - that we use double values for the representation of the location

    we have an accuracy of about

    1e-16*2e7ft/1 = 2e-9 ft

    left. This should be sufficient for our needs. Note that this is the same
    relative accuracy we would have when we compute directly with
    lon/lat/radius. For the radius value this is clear. For the lon/lat pair
    this is easy to see. Take for example KSFO located at about 37.61 deg north
    122.35 deg west, which corresponds to 0.65642 rad north and 2.13541 rad
    west. Both values are of magnitude of about 1. But 1 ft corresponds to about
    1/(2e7*2*pi) = 7.9577e-09 rad. So the left accuracy with this representation
    is also about 1*1e-16/7.9577e-09 = 1.2566e-08 which is of the same magnitude
    as the representation chosen here.

    The advantage of this representation is that it is a linear space without
    singularities. The singularities are the north and south pole and most
    notably the non-steady jump at -pi to pi. It is harder to track this jump
    correctly especially when we need to work with error norms and derivatives
    of the equations of motion within the time-stepping code. Also, the rate of
    change is of the same magnitude for all components in this representation
    which is an advantage for numerical stability in implicit time-stepping.

    Note: The latitude is a GEOCENTRIC value. FlightGear converts latitude to a
    geodetic value and uses that. In order to get best matching relative to a
    map, geocentric latitude must be converted to geodetic.

    @see Stevens and Lewis, "Aircraft Control and Simulation", Second edition
    @see W. C. Durham "Aircraft Dynamics & Control", section 2.2

    @author Mathias Froehlich
    @version $Id: Location.h,v 1.35 2015/09/20 20:53:13 bcoconni Exp $
  */

class Location {
public:
  /** Default constructor. */
  Location(void);

  /** Constructor to set the longitude, latitude and the distance
      from the center of the earth.
      @param lon longitude
      @param lat GEOCENTRIC latitude
      @param radius distance from center of earth to vehicle in feet*/
  Location(double lon, double lat, double radius);

  /** Constructor to initialize the location with the cartesian coordinates
      (X,Y,Z) contained in the input ColumnVector3. Distances are in feet,
      the position is expressed in the ECEF frame.
      @param lv vector that contain the cartesian coordinates*/
  Location(const ColumnVector3& lv);

  /** Copy constructor. */
  Location(const Location& l);

  /** Set the longitude.
      @param longitude Longitude in rad to set.
      Sets the longitude of the location represented with this class
      instance to the value of the given argument. The value is meant
      to be in rad. The latitude and the radius value are preserved
      with this call with the exception of radius being equal to
      zero. If the radius is previously set to zero it is changed to be
      equal to 1.0 past this call. Longitude is positive east and negative west. */
  void SetLongitude(double longitude);

  /** Set the latitude.
      @param latitude Latitude in rad to set.
      Sets the latitude of the location represented with this class
      instance to the value of the given argument. The value is meant
      to be in rad. The longitude and the radius value are preserved
      with this call with the exception of radius being equal to
      zero. If the radius is previously set to zero it is changed to be
      equal to 1.0 past this call.
      Latitude is positive north and negative south.
      The arguments should be within the bounds of -pi/2 <= lat <= pi/2.
      The behavior of this function with arguments outside this range is
      left as an exercise to the gentle reader ... */
  void SetLatitude(double latitude);

  /** Set the distance from the center of the earth.
      @param radius Radius in ft to set.
      Sets the radius of the location represented with this class
      instance to the value of the given argument. The value is meant
      to be in ft. The latitude and longitude values are preserved
      with this call with the exception of radius being equal to
      zero. If the radius is previously set to zero, latitude and
      longitude is set equal to zero past this call.
      The argument should be positive.
      The behavior of this function called with a negative argument is
      left as an exercise to the gentle reader ... */
  void SetRadius(double radius);

  /** Sets the longitude, latitude and the distance from the center of the earth.
      @param lon longitude in radians
      @param lat GEOCENTRIC latitude in radians
      @param radius distance from center of earth to vehicle in feet*/
  void SetPosition(double lon, double lat, double radius);

  /** Sets the longitude, latitude and the distance above the reference ellipsoid.
      @param lon longitude in radians
      @param lat GEODETIC latitude in radians
      @param height distance above the reference ellipsoid to vehicle in feet*/
  void SetPositionGeodetic(double lon, double lat, double height);

  /** Sets the semimajor and semiminor axis lengths for this planet.
      The eccentricity and flattening are calculated from the semimajor
      and semiminor axis lengths */
  void SetEllipse(double semimajor, double semiminor);

  /** Sets the Earth position angle.
      This is the relative orientation of the ECEF frame with respect to the
      Inertial frame.
      @param EPA Earth fixed frame (ECEF) rotation offset about the axis with
                 respect to the Inertial (ECI) frame in radians. */
	void SetEarthPositionAngle(double EPA);
	
	/** Set the altitude above sea level.
		@param altitudeASL altitude above Sea Level in feet.
		@see SetGroundCallback */
	void SetAltitudeASL(double altitudeASL);
	
  /** Increments the Earth position angle.
      This is the relative orientation of the ECEF frame with respect to the
      Inertial frame.
      @param delta delta to the Earth fixed frame (ECEF) rotation offset about the axis with
                 respect to the Inertial (ECI) frame in radians. */
	void IncrementEarthPositionAngle(double delta);

  /** Get the longitude.
      @return the longitude in rad of the location represented with this
      class instance. The returned values are in the range between
      -pi <= lon <= pi. Longitude is positive east and negative west. */
	double GetLongitude() const;

  /** Get the longitude.
      @return the longitude in deg of the location represented with this
      class instance. The returned values are in the range between
      -180 <= lon <= 180.  Longitude is positive east and negative west. */
	double GetLongitudeDeg() const;

  /** Get the sine of Longitude. */
	double GetSinLongitude() const;

  /** Get the cosine of Longitude. */
	double GetCosLongitude() const;

  /** Get the latitude.
      @return the latitude in rad of the location represented with this
      class instance. The returned values are in the range between
      -pi/2 <= lon <= pi/2. Latitude is positive north and negative south. */
	double GetLatitude() const;

  /** Get the geodetic latitude.
      @return the geodetic latitude in rad of the location represented with this
      class instance. The returned values are in the range between
      -pi/2 <= lon <= pi/2. Latitude is positive north and negative south. */
	double GetGeodLatitudeRad(void) const;

  /** Get the latitude.
      @return the latitude in deg of the location represented with this
      class instance. The returned value is in the range between
      -90 <= lon <= 90. Latitude is positive north and negative south. */
	double GetLatitudeDeg() const;

  /** Get the geodetic latitude in degrees.
      @return the geodetic latitude in degrees of the location represented by
      this class instance. The returned value is in the range between
      -90 <= lon <= 90. Latitude is positive north and negative south. */
	double GetGeodLatitudeDeg(void) const;

  /** Gets the geodetic altitude in feet. */
	double GetGeodAltitude(void) const;

  /** Get the sine of Latitude. */
	double GetSinLatitude() const;

  /** Get the cosine of Latitude. */
	double GetCosLatitude() const;

  /** Get the cosine of Latitude. */
	double GetTanLatitude() const;

  /** Return the Earth Position Angle.
      This is the relative orientation of the ECEF frame with respect to the
      Inertial frame.
      @return the Earth fixed frame (ECEF) rotation offset about the axis with
              respect to the Inertial (ECI) frame in radians. */
	double GetEPA() const;

  /** Get the distance from the center of the earth.
      @return the distance of the location represented with this class
      instance to the center of the earth in ft. The radius value is
      always positive. */
  //double GetRadius() const { return mECLoc.Magnitude(); } // may not work with FlightGear
	double GetRadius() const;

	/** Get the local sea level radius
		@return the sea level radius at the location in feet.
		@see SetGroundCallback */
	double GetSeaLevelRadius(void) const;

  /** Transform matrix from local horizontal to earth centered frame.
      @return a const reference to the rotation matrix of the transform from
      the local horizontal frame to the earth centered frame. */
	const Matrix33& GetTl2ec(void) const;

  /** Transform matrix from the earth centered to local horizontal frame.
      @return a const reference to the rotation matrix of the transform from
      the earth centered frame to the local horizontal frame. */
	const Matrix33& GetTec2l(void) const;

  /** Transform matrix from inertial to earth centered frame.
      @return a const reference to the rotation matrix of the transform from
      the inertial frame to the earth centered frame (ECI to ECEF).
      @see SetEarthPositionAngle
      @see IncrementEarthPositionAngle */
	const Matrix33& GetTi2ec(void) const;

  /** Transform matrix from the earth centered to inertial frame.
      @return a const reference to the rotation matrix of the transform from
      the earth centered frame to the inertial frame (ECEF to ECI).
      @see SetEarthPositionAngle
      @see IncrementEarthPositionAngle */
	const Matrix33& GetTec2i(void) const;

  /** Transform matrix from the inertial to local horizontal frame.
      @return a const reference to the rotation matrix of the transform from
      the inertial frame to the local horizontal frame.
      @see SetEarthPositionAngle
      @see IncrementEarthPositionAngle */
	const Matrix33& GetTi2l(void) const;

  /** Transform matrix from local horizontal to inertial frame.
      @return a const reference to the rotation matrix of the transform from
      the local horizontal frame to the inertial frame.
      @see SetEarthPositionAngle
      @see IncrementEarthPositionAngle */
	const Matrix33& GetTl2i(void) const;

  /** Conversion from Local frame coordinates to a location in the
      earth centered and fixed frame.
      This function calculates the Location of an object which position
      relative to the vehicle is given as in input.
      @param lvec Vector in the local horizontal coordinate frame
      @return The location in the earth centered and fixed frame */
	Location LocalToLocation(const ColumnVector3& lvec) const;

  /** Conversion from a location in the earth centered and fixed frame
      to local horizontal frame coordinates.
      This function calculates the relative position between the vehicle and
      the input vector and returns the result expressed in the local frame.
      @param ecvec Vector in the earth centered and fixed frame
      @return The vector in the local horizontal coordinate frame */
	ColumnVector3 LocationToLocal(const ColumnVector3& ecvec) const;

  // For time-stepping, locations have vector properties...

  /** Read access the entries of the vector.
      @param idx the component index.
      Return the value of the matrix entry at the given index.
      Indices are counted starting with 1.
      Note that the index given in the argument is unchecked. */
	double operator()(unsigned int idx) const;

  /** Write access the entries of the vector.
      @param idx the component index.
      @return a reference to the vector entry at the given index.
      Indices are counted starting with 1.
      Note that the index given in the argument is unchecked. */
	double& operator()(unsigned int idx);

  /** Read access the entries of the vector.
      @param idx the component index.
      @return the value of the matrix entry at the given index.
      Indices are counted starting with 1.
      This function is just a shortcut for the <tt>double
      operator()(unsigned int idx) const</tt> function. It is
      used internally to access the elements in a more convenient way.
      Note that the index given in the argument is unchecked. */
	double Entry(unsigned int idx) const;

  /** Write access the entries of the vector.
      @param idx the component index.
      @return a reference to the vector entry at the given index.
      Indices are counted starting with 1.
      This function is just a shortcut for the double&
      operator()(unsigned int idx) function. It is
      used internally to access the elements in a more convenient way.
      Note that the index given in the argument is unchecked. */
	double& Entry(unsigned int idx);

  /** Sets this location via the supplied vector.
      The location can be set by an Earth-centered, Earth-fixed (ECEF) frame
      position vector. The cache is marked as invalid, so any future requests
      for selected important data will cause the parameters to be calculated.
      @param v the ECEF column vector in feet.
      @return a reference to the Location object. */
	const Location& operator=(const ColumnVector3& v);

  /** Sets this location via the supplied location object.
      @param v A location object reference.
      @return a reference to the Location object. */
  const Location& operator=(const Location& l);

  /** This operator returns true if the ECEF location vectors for the two
      location objects are equal. */
	bool operator==(const Location& l) const;

  /** This operator returns true if the ECEF location vectors for the two
      location objects are not equal. */
	bool operator!=(const Location& l) const;

  /** This operator adds the ECEF position vectors.
      The cartesian coordinates of the supplied vector (right side) are added to
      the ECEF position vector on the left side of the equality, and a reference
      to this object is returned. */
	const Location& operator+=(const Location &l);

  /** This operator substracts the ECEF position vectors.
      The cartesian coordinates of the supplied vector (right side) are
      substracted from the ECEF position vector on the left side of the
      equality, and a reference to this object is returned. */
	const Location& operator-=(const Location &l);

  /** This operator scales the ECEF position vector.
      The cartesian coordinates of the ECEF position vector on the left side of
      the equality are scaled by the supplied value (right side), and a
      reference to this object is returned. */
	const Location& operator*=(double scalar);

  /** This operator scales the ECEF position vector.
      The cartesian coordinates of the ECEF position vector on the left side of
      the equality are scaled by the inverse of the supplied value (right side),
      and a reference to this object is returned. */
	const Location& operator/=(double scalar);

  /** This operator adds two ECEF position vectors.
      A new object is returned that defines a position which is the sum of the
      cartesian coordinates of the two positions provided. */
	Location operator+(const Location& l) const;

  /** This operator substracts two ECEF position vectors.
      A new object is returned that defines a position which is the difference
      of the cartesian coordinates of the two positions provided. */
	Location operator-(const Location& l) const;

  /** This operator scales an ECEF position vector.
      A new object is returned that defines a position made of the cartesian
      coordinates of the provided ECEF position scaled by the supplied scalar
      value. */
	Location operator*(double scalar) const;

  /** Cast to a simple 3d vector */
	operator const ColumnVector3&() const;

	/** Get the geodetic distance between the current location and a given
			location. This corresponds to the shortest distance between the two
			locations. Earth curvature is taken into account.
			@param target_longitude the target longitude
			@param target_latitude the target latitude
			@return The geodetic distance between the two locations */
	double GetDistanceTo(double target_longitude, double target_latitude) const;

	/** Get the heading that should be followed from the current location to
			a given location along the shortest path. Earth curvature is
			taken into account.
			@param target_longitude the target longitude
			@param target_latitude the target latitude
			@return The heading that should be followed to reach the targeted
			location along the shortest path */
	double GetHeadingTo(double target_longitude, double target_latitude) const;

private:
  /** Computation of derived values.
      This function re-computes the derived values like lat/lon and
      transformation matrices. It does this unconditionally. */
  void ComputeDerivedUnconditional(void) const;

  /** Computation of derived values.
      This function checks if the derived values like lat/lon and
      transformation matrices are already computed. If so, it
      returns. If they need to be computed this is done here. */
	void ComputeDerived(void) const;

  /** The coordinates in the earth centered frame. This is the master copy.
      The coordinate frame has its center in the middle of the earth.
      Its x-axis points from the center of the earth towards a
      location with zero latitude and longitude on the earths
      surface. The y-axis points from the center of the earth towards a
      location with zero latitude and 90deg longitude on the earths
      surface. The z-axis points from the earths center to the
      geographic north pole.
      @see W. C. Durham "Aircraft Dynamics & Control", section 2.2 */
  ColumnVector3 mECLoc;

  /** The cached lon/lat/radius values. */
  mutable double mLon;
  mutable double mLat;
  mutable double mRadius;
  mutable double mGeodLat;
  mutable double GeodeticAltitude;

  /** The cached rotation matrices from and to the associated frames. */
  mutable Matrix33 mTl2ec;
  mutable Matrix33 mTec2l;
  mutable Matrix33 mTi2ec;
  mutable Matrix33 mTec2i;
  mutable Matrix33 mTi2l;
  mutable Matrix33 mTl2i;

  double epa;

  /* Terms for geodetic latitude calculation. Values are from WGS84 model */
  double a;  // WGS84 semimajor axis length in feet
  double e2; // Earth eccentricity squared
  double c;
  double ec;
  double ec2;

  /** A data validity flag.
      This class implements caching of the derived values like the
      orthogonal rotation matrices or the lon/lat/radius values. For caching we
      carry a flag which signals if the values are valid or not.
      The C++ keyword "mutable" tells the compiler that the data member is
      allowed to change during a const member function. */
  mutable bool mCacheValid;

	const double sign(const double num) const { return num >= 0.0 ? 1.0 : -1.0; }
};

/** Scalar multiplication.
    @param scalar scalar value to multiply with.
    @param l Vector to multiply.
    Multiply the Vector with a scalar value. */
inline Location operator*(double scalar, const Location& l) {
  return l.operator*(scalar);
}