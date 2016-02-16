#pragma once

#include <string>
#include <Base.h>
#include <ColumnVector3.h>

/**  Models the Quaternion representation of rotations.
    Quaternion is a representation of an arbitrary rotation through a
    quaternion. It has vector properties. This class also contains access
    functions to the euler angle representation of rotations and access to
    transformation matrices for 3D vectors. Transformations and euler angles are
    therefore computed once they are requested for the first time. Then they are
    cached for later usage as long as the class is not accessed trough
    a nonconst member function.

    Note: The order of rotations used in this class corresponds to a 3-2-1 sequence,
    or Y-P-R, or Z-Y-X, if you prefer.

    @see Cooke, Zyda, Pratt, and McGhee, "NPSNET: Flight Simulation Dynamic Modeling
    Using Quaternions", Presence, Vol. 1, No. 4, pp. 404-420  Naval Postgraduate
    School, January 1994
    @see D. M. Henderson, "Euler Angles, Quaternions, and Transformation Matrices",
    JSC 12960, July 1977
    @see Richard E. McFarland, "A Standard Kinematic Model for Flight Simulation at
    NASA-Ames", NASA CR-2497, January 1975
    @see Barnes W. McCormick, "Aerodynamics, Aeronautics, and Flight Mechanics",
    Wiley & Sons, 1979 ISBN 0-471-03032-5
    @see Bernard Etkin, "Dynamics of Flight, Stability and Control", Wiley & Sons,
    1982 ISBN 0-471-08936-2
    @author Mathias Froehlich, extended ColumnVector4 originally by Tony Peden and Jon Berndt
*/

class Matrix33;

class Quaternion {
public:
  /** Default initializer.
      Default initializer, initializes the class with the identity rotation.  */
	Quaternion();

  /** Copy constructor.
      Copy constructor, initializes the quaternion.
      @param q  a constant reference to another Quaternion instance  */
  Quaternion(const Quaternion& q);

  /** Initializer by euler angles.
      Initialize the quaternion with the euler angles.
      @param phi The euler X axis (roll) angle in radians
      @param tht The euler Y axis (attitude) angle in radians
      @param psi The euler Z axis (heading) angle in radians  */
  Quaternion(double phi, double tht, double psi);

  /** Initializer by euler angle vector.
      Initialize the quaternion with the euler angle vector.
      @param vOrient The euler axis angle vector in radians (phi, tht, psi) */
  Quaternion(ColumnVector3 vOrient);

  /** Initializer by one euler angle.
      Initialize the quaternion with the single euler angle where its index
      is given in the first argument.
      @param idx Index of the euler angle to initialize
      @param angle The euler angle in radians  */
	Quaternion(int idx, double angle);

  /** Initializer by a rotation axis and an angle.
      Initialize the quaternion to represent the rotation around a given
      angle and an arbitrary axis.
      @param angle The angle in radians
      @param axis  The rotation axis
   */
	Quaternion(double angle, const ColumnVector3& axis);

  /** Initializer by matrix.
      Initialize the quaternion with the matrix representing a transform from one frame
      to another using the standard aerospace sequence, Yaw-Pitch-Roll (3-2-1).
      @param m the rotation matrix */
  Quaternion(const Matrix33& m);

  /// Destructor.
	~Quaternion();

  /** Quaternion derivative for given angular rates.
      Computes the quaternion derivative which results from the given
      angular velocities
      @param PQR a constant reference to a rotation rate vector
      @return the quaternion derivative
      @see Stevens and Lewis, "Aircraft Control and Simulation", Second Edition,
           Equation 1.3-36. */
  Quaternion GetQDot(const ColumnVector3& PQR) const;

  /** Transformation matrix.
      @return a reference to the transformation/rotation matrix
      corresponding to this quaternion rotation.  */
	const Matrix33& GetT(void) const;

  /** Backward transformation matrix.
      @return a reference to the inverse transformation/rotation matrix
      corresponding to this quaternion rotation.  */
	const Matrix33& GetTInv(void) const;

  /** Retrieves the Euler angles.
      @return a reference to the triad of Euler angles corresponding
      to this quaternion rotation.
      units radians  */
	const ColumnVector3& GetEuler(void) const;

  /** Retrieves the Euler angles.
      @param i the Euler angle index.
      units radians.
      @return a reference to the i-th euler angles corresponding
      to this quaternion rotation.
   */
	double GetEuler(int i) const;

  /** Retrieves the Euler angles.
      @param i the Euler angle index.
      @return a reference to the i-th euler angles corresponding
      to this quaternion rotation.
      units degrees */
	double GetEulerDeg(int i) const;

  /** Retrieves the Euler angle vector.
      @return an Euler angle column vector corresponding
      to this quaternion rotation.
      units degrees */
	ColumnVector3 const GetEulerDeg(void) const;

  /** Retrieves sine of the given euler angle.
      @return the sine of the Euler angle theta (pitch attitude) corresponding
      to this quaternion rotation.  */
	double GetSinEuler(int i) const;

  /** Retrieves cosine of the given euler angle.
      @return the sine of the Euler angle theta (pitch attitude) corresponding
      to this quaternion rotation.  */
	double GetCosEuler(int i) const;

  /** Read access the entries of the vector.
	    @param idx the component index.
			Return the value of the matrix entry at the given index.
      Indices are counted starting with 1.
			Note that the index given in the argument is unchecked.
   */
	double operator()(unsigned int idx) const;

  /** Write access the entries of the vector.
      @param idx the component index.
      Return a reference to the vector entry at the given index.
      Indices are counted starting with 1.
      Note that the index given in the argument is unchecked.
   */
	double& operator()(unsigned int idx);

  /** Read access the entries of the vector.
      @param idx the component index.
      Return the value of the matrix entry at the given index.
      Indices are counted starting with 1.
      This function is just a shortcut for the <tt>double
      operator()(unsigned int idx) const</tt> function. It is
      used internally to access the elements in a more convenient way.
      Note that the index given in the argument is unchecked.
  */
	double Entry(unsigned int idx) const;

  /** Write access the entries of the vector.
      @param idx the component index.
      Return a reference to the vector entry at the given index.
      Indices are counted starting with 1.
      This function is just a shortcut for the <tt>double&
      operator()(unsigned int idx)</tt> function. It is
      used internally to access the elements in a more convenient way.
      Note that the index given in the argument is unchecked.
  */
	double& Entry(unsigned int idx);

  /** Assignment operator "=".
      Assign the value of q to the current object. Cached values are
      conserved.
      @param q reference to an Quaternion instance
      @return reference to a quaternion object  */
	const Quaternion& operator=(const Quaternion& q);

  /// Conversion from Quat to Matrix
	operator Matrix33() const;

  /** Comparison operator "==".
      @param q a quaternion reference
      @return true if both quaternions represent the same rotation.  */
	bool operator==(const Quaternion& q) const;

  /** Comparison operator "!=".
      @param q a quaternion reference
      @return true if both quaternions do not represent the same rotation.  */
	bool operator!=(const Quaternion& q) const;

	/** Arithmetic operator "+=".
	@param q a quaternion reference.
	@return a quaternion reference representing Q, where Q = Q + q. */
	const Quaternion& operator+=(const Quaternion& q);

  /** Arithmetic operator "-=".
      @param q a quaternion reference.
      @return a quaternion reference representing Q, where Q = Q - q. */
	const Quaternion& operator-=(const Quaternion& q);

  /** Arithmetic operator "*=".
      @param scalar a multiplicative value.
      @return a quaternion reference representing Q, where Q = Q * scalar. */
	const Quaternion& operator*=(double scalar);

  /** Arithmetic operator "/=".
      @param scalar a divisor value.
      @return a quaternion reference representing Q, where Q = Q / scalar. */
	const Quaternion& operator/=(double scalar);

  /** Arithmetic operator "+".
      @param q a quaternion to be summed.
      @return a quaternion representing Q, where Q = Q + q. */
	Quaternion operator+(const Quaternion& q) const;

  /** Arithmetic operator "-".
      @param q a quaternion to be subtracted.
      @return a quaternion representing Q, where Q = Q - q. */
	Quaternion operator-(const Quaternion& q) const;

  /** Arithmetic operator "*".
      Multiplication of two quaternions is like performing successive rotations.
      @param q a quaternion to be multiplied.
      @return a quaternion representing Q, where Q = Q * q. */
	Quaternion operator*(const Quaternion& q) const;

  /** Arithmetic operator "*=".
      Multiplication of two quaternions is like performing successive rotations.
      @param q a quaternion to be multiplied.
      @return a quaternion reference representing Q, where Q = Q * q. */
	const Quaternion& operator*=(const Quaternion& q);

  /** Inverse of the quaternion.
      Compute and return the inverse of the quaternion so that the orientation
      represented with *this multiplied with the returned value is equal to
      the identity orientation.
  */
	Quaternion Inverse(void) const;

  /** Conjugate of the quaternion.
      Compute and return the conjugate of the quaternion. This one is equal
      to the inverse iff the quaternion is normalized.
  */
	Quaternion Conjugate(void) const;

  friend Quaternion operator*(double, const Quaternion&);

  /** Length of the vector.
      Compute and return the euclidean norm of this vector.
  */
	double Magnitude(void) const;

  /** Square of the length of the vector.
      Compute and return the square of the euclidean norm of this vector.
  */
	double SqrMagnitude(void) const;

  /** Normalize.
      Normalize the vector to have the Magnitude() == 1.0. If the vector
      is equal to zero it is left untouched.
   */
  void Normalize(void);

  /** Zero quaternion vector. Does not represent any orientation.
      Useful for initialization of increments */
	static Quaternion zero(void);

  std::string Dump(const std::string& delimiter) const;

  friend Quaternion QExp(const ColumnVector3& omega);

private:
  /** Copying by assigning the vector valued components.  */
	Quaternion(double q1, double q2, double q3, double q4);

  /** Computation of derived values.
      This function recomputes the derived values like euler angles and
      transformation matrices. It does this unconditionally.  */
  void ComputeDerivedUnconditional(void) const;

  /** Computation of derived values.
      This function checks if the derived values like euler angles and
      transformation matrices are already computed. If so, it
      returns. If they need to be computed the real worker routine
      Quaternion::ComputeDerivedUnconditional(void) const
      is called. */
	void ComputeDerived(void) const;

	void InitializeFromEulerAngles(double phi, double tht, double psi);

  /** The quaternion values itself. This is the master copy. */
  double data[4];

  /** A data validity flag.
      This class implements caching of the derived values like the
      orthogonal rotation matrices or the Euler angles. For caching we
      carry a flag which signals if the values are valid or not.
      The C++ keyword "mutable" tells the compiler that the data member is
      allowed to change during a const member function.  */
  mutable bool mCacheValid;

  /** This stores the transformation matrices.  */
  mutable Matrix33 mT;
  mutable Matrix33 mTInv;

  /** The cached euler angles.  */
  mutable ColumnVector3 mEulerAngles;

  /** The cached sines and cosines of the euler angles.  */
  mutable ColumnVector3 mEulerSines;
  mutable ColumnVector3 mEulerCosines;

	enum { ePhi = 1, eTht, ePsi };
	enum { eP = 1, eQ, eR };
};

/** Scalar multiplication.
    @param scalar scalar value to multiply with.
    @param q Vector to multiply.
    Multiply the Vector with a scalar value.
*/
inline Quaternion operator*(double scalar, const Quaternion& q) {
  return Quaternion(scalar*q.data[0], scalar*q.data[1], scalar*q.data[2], scalar*q.data[3]);
}

/** Quaternion exponential
    @param omega rotation velocity
    Calculate the unit quaternion which is the result of the exponentiation of
    the vector 'omega'.
*/
inline Quaternion QExp(const ColumnVector3& omega) {
  Quaternion qexp;
  double angle = omega.Magnitude();
  double sina_a = angle > 0.0 ? sin(angle)/angle : 1.0;

  qexp.data[0] = cos(angle);
  qexp.data[1] = omega(1) * sina_a;
  qexp.data[2] = omega(2) * sina_a;
  qexp.data[3] = omega(3) * sina_a;

  return qexp;
}

/** Write quaternion to a stream.
    @param os Stream to write to.
    @param q Quaternion to write.
    Write the quaternion to a stream.*/
std::ostream& operator<<(std::ostream& os, const Quaternion& q);