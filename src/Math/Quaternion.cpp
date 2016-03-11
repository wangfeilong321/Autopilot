#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

using std::cerr;
using std::cout;
using std::endl;

#include <Matrix33.h>
#include <Quaternion.h>

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion::Quaternion() : mCacheValid(false) {
  data[0] = 1.0;
  data[1] = data[2] = data[3] = 0.0;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion::Quaternion(const Quaternion& q) : mCacheValid(q.mCacheValid) {
  data[0] = q(1);
  data[1] = q(2);
  data[2] = q(3);
  data[3] = q(4);
  if (mCacheValid) {
    mT = q.mT;
    mTInv = q.mTInv;
    mEulerAngles = q.mEulerAngles;
    mEulerSines = q.mEulerSines;
    mEulerCosines = q.mEulerCosines;
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion::Quaternion(double phi, double tht, double psi): mCacheValid(false) {
  InitializeFromEulerAngles(phi, tht, psi);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion::Quaternion(ColumnVector3 vOrient): mCacheValid(false) {
  double phi = vOrient(ePhi);
  double tht = vOrient(eTht);
  double psi = vOrient(ePsi);
  InitializeFromEulerAngles(phi, tht, psi);
}
 
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion::Quaternion(int idx, double angle) : mCacheValid(false) {
  double angle2 = 0.5*angle;

  double Sangle2 = sin(angle2);
  double Cangle2 = cos(angle2);

  if (idx == ePhi) {
    data[0] = Cangle2;
    data[1] = Sangle2;
    data[2] = 0.0;
    data[3] = 0.0;

  }
  else if (idx == eTht) {
    data[0] = Cangle2;
    data[1] = 0.0;
    data[2] = Sangle2;
    data[3] = 0.0;

  }
  else {
    data[0] = Cangle2;
    data[1] = 0.0;
    data[2] = 0.0;
    data[3] = Sangle2;

  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion::Quaternion(double angle, const ColumnVector3& axis) : mCacheValid(false) {
  double angle2 = 0.5 * angle;

  double length = axis.Magnitude();
  double Sangle2 = sin(angle2) / length;
  double Cangle2 = cos(angle2);

  data[0] = Cangle2;
  data[1] = Sangle2 * axis(1);
  data[2] = Sangle2 * axis(2);
  data[3] = Sangle2 * axis(3);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion::Quaternion(const Matrix33& m) : mCacheValid(false)
{
  data[0] = 0.50*sqrt(1.0 + m(1,1) + m(2,2) + m(3,3));
  double t = 0.25/data[0];
  data[1] = t*(m(2,3) - m(3,2));
  data[2] = t*(m(3,1) - m(1,3));
  data[3] = t*(m(1,2) - m(2,1));

  Normalize();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion::~Quaternion() {}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion Quaternion::GetQDot(const ColumnVector3& PQR) const {
  return Quaternion(
    -0.5*( data[1]*PQR(eP) + data[2]*PQR(eQ) + data[3]*PQR(eR)),
     0.5*( data[0]*PQR(eP) - data[3]*PQR(eQ) + data[2]*PQR(eR)),
     0.5*( data[3]*PQR(eP) + data[0]*PQR(eQ) - data[1]*PQR(eR)),
     0.5*(-data[2]*PQR(eP) + data[1]*PQR(eQ) + data[0]*PQR(eR))
  );
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const Matrix33& Quaternion::GetT(void) const { ComputeDerived(); return mT; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const Matrix33& Quaternion::GetTInv(void) const { ComputeDerived(); return mTInv; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const ColumnVector3& Quaternion::GetEuler(void) const {
  ComputeDerived();
  return mEulerAngles;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Quaternion::GetEuler(int i) const {
  ComputeDerived();
  return mEulerAngles(i);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Quaternion::GetEulerDeg(int i) const {
  ComputeDerived();
  return radtodeg*mEulerAngles(i);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3 const Quaternion::GetEulerDeg(void) const {
  ComputeDerived();
  return radtodeg*mEulerAngles;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Quaternion::GetSinEuler(int i) const {
  ComputeDerived();
  return mEulerSines(i);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Quaternion::GetCosEuler(int i) const {
  ComputeDerived();
  return mEulerCosines(i);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Quaternion::operator()(unsigned int idx) const { return data[idx - 1]; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double& Quaternion::operator()(unsigned int idx) { mCacheValid = false; return data[idx - 1]; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Quaternion::Entry(unsigned int idx) const { return data[idx - 1]; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double& Quaternion::Entry(unsigned int idx) {
  mCacheValid = false;
  return data[idx - 1];
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const Quaternion& Quaternion::operator=(const Quaternion& q) {
  // Copy the master values ...
  data[0] = q.data[0];
  data[1] = q.data[1];
  data[2] = q.data[2];
  data[3] = q.data[3];
  ComputeDerived();
  // .. and copy the derived values if they are valid
  mCacheValid = q.mCacheValid;
  if (mCacheValid) {
    mT = q.mT;
    mTInv = q.mTInv;
    mEulerAngles = q.mEulerAngles;
    mEulerSines = q.mEulerSines;
    mEulerCosines = q.mEulerCosines;
  }
  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion::operator Matrix33() const { return GetT(); }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool Quaternion::operator==(const Quaternion& q) const {
  return data[0] == q.data[0] && data[1] == q.data[1]
    && data[2] == q.data[2] && data[3] == q.data[3];
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool Quaternion::operator!=(const Quaternion& q) const { return !operator==(q); }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const Quaternion& Quaternion::operator+=(const Quaternion& q) {
  // Copy the master values ...
  data[0] += q.data[0];
  data[1] += q.data[1];
  data[2] += q.data[2];
  data[3] += q.data[3];
  mCacheValid = false;
  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const Quaternion& Quaternion::operator-=(const Quaternion& q) {
  // Copy the master values ...
  data[0] -= q.data[0];
  data[1] -= q.data[1];
  data[2] -= q.data[2];
  data[3] -= q.data[3];
  mCacheValid = false;
  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const Quaternion& Quaternion::operator*=(double scalar) {
  data[0] *= scalar;
  data[1] *= scalar;
  data[2] *= scalar;
  data[3] *= scalar;
  mCacheValid = false;
  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const Quaternion& Quaternion::operator/=(double scalar) {
  return operator*=(1.0 / scalar);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion Quaternion::operator+(const Quaternion& q) const {
  return Quaternion(data[0] + q.data[0], data[1] + q.data[1], data[2] + q.data[2], data[3] + q.data[3]);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion Quaternion::operator-(const Quaternion& q) const {
  return Quaternion(data[0] - q.data[0], data[1] - q.data[1], data[2] - q.data[2], data[3] - q.data[3]);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion Quaternion::operator*(const Quaternion& q) const {
  return Quaternion(data[0] * q.data[0] - data[1] * q.data[1] - data[2] * q.data[2] - data[3] * q.data[3],
    data[0] * q.data[1] + data[1] * q.data[0] + data[2] * q.data[3] - data[3] * q.data[2],
    data[0] * q.data[2] - data[1] * q.data[3] + data[2] * q.data[0] + data[3] * q.data[1],
    data[0] * q.data[3] + data[1] * q.data[2] - data[2] * q.data[1] + data[3] * q.data[0]);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const Quaternion& Quaternion::operator*=(const Quaternion& q) {
  double q0 = data[0] * q.data[0] - data[1] * q.data[1] - data[2] * q.data[2] - data[3] * q.data[3];
  double q1 = data[0] * q.data[1] + data[1] * q.data[0] + data[2] * q.data[3] - data[3] * q.data[2];
  double q2 = data[0] * q.data[2] - data[1] * q.data[3] + data[2] * q.data[0] + data[3] * q.data[1];
  double q3 = data[0] * q.data[3] + data[1] * q.data[2] - data[2] * q.data[1] + data[3] * q.data[0];
  data[0] = q0;
  data[1] = q1;
  data[2] = q2;
  data[3] = q3;
  mCacheValid = false;
  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion Quaternion::Inverse(void) const {
  double norm = SqrMagnitude();
  if (norm == 0.0)
    return *this;
  double rNorm = 1.0 / norm;
  return Quaternion(data[0] * rNorm, -data[1] * rNorm, -data[2] * rNorm, -data[3] * rNorm);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion Quaternion::Conjugate(void) const {
  return Quaternion(data[0], -data[1], -data[2], -data[3]);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Quaternion::Magnitude(void) const { return sqrt(SqrMagnitude()); }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Quaternion::SqrMagnitude(void) const {
  return data[0] * data[0] + data[1] * data[1]  + data[2] * data[2] + data[3] * data[3];
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Quaternion::Normalize() {
  // Note: this does not touch the cache since it does not change the orientation
  double norm = Magnitude();
  if (norm == 0.0 || fabs(norm - 1.000) < 1e-10) return;

  double rnorm = 1.0/norm;

  data[0] *= rnorm;
  data[1] *= rnorm;
  data[2] *= rnorm;
  data[3] *= rnorm;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion Quaternion::zero(void) { return Quaternion(0.0, 0.0, 0.0, 0.0); }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion::Quaternion(double q1, double q2, double q3, double q4) : mCacheValid(false) {
  data[0] = q1; data[1] = q2; data[2] = q3; data[3] = q4;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Quaternion::ComputeDerived(void) const {
  if (!mCacheValid)
    ComputeDerivedUnconditional();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Quaternion::ComputeDerivedUnconditional(void) const {
  mCacheValid = true;

  double q0 = data[0]; // use some aliases/shorthand for the quat elements.
  double q1 = data[1];
  double q2 = data[2];
  double q3 = data[3];

  // Now compute the transformation matrix.
  double q0q0 = q0*q0;
  double q1q1 = q1*q1;
  double q2q2 = q2*q2;
  double q3q3 = q3*q3;
  double q0q1 = q0*q1;
  double q0q2 = q0*q2;
  double q0q3 = q0*q3;
  double q1q2 = q1*q2;
  double q1q3 = q1*q3;
  double q2q3 = q2*q3;
  
  mT(1,1) = q0q0 + q1q1 - q2q2 - q3q3; // This is found from Eqn. 1.3-32 in
  mT(1,2) = 2.0*(q1q2 + q0q3);         // Stevens and Lewis
  mT(1,3) = 2.0*(q1q3 - q0q2);
  mT(2,1) = 2.0*(q1q2 - q0q3);
  mT(2,2) = q0q0 - q1q1 + q2q2 - q3q3;
  mT(2,3) = 2.0*(q2q3 + q0q1);
  mT(3,1) = 2.0*(q1q3 + q0q2);
  mT(3,2) = 2.0*(q2q3 - q0q1);
  mT(3,3) = q0q0 - q1q1 - q2q2 + q3q3;

  // Since this is an orthogonal matrix, the inverse is simply the transpose.

  mTInv = mT;
  mTInv.T();
  
  // Compute the Euler-angles

  mEulerAngles = mT.GetEuler();
  
  // FIXME: may be one can compute those values easier ???
  mEulerSines(ePhi) = sin(mEulerAngles(ePhi));
  // mEulerSines(eTht) = sin(mEulerAngles(eTht));
  mEulerSines(eTht) = -mT(1,3);
  mEulerSines(ePsi) = sin(mEulerAngles(ePsi));
  mEulerCosines(ePhi) = cos(mEulerAngles(ePhi));
  mEulerCosines(eTht) = cos(mEulerAngles(eTht));
  mEulerCosines(ePsi) = cos(mEulerAngles(ePsi));
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Quaternion::InitializeFromEulerAngles(double phi, double tht, double psi) {
  mEulerAngles(ePhi) = phi;
  mEulerAngles(eTht) = tht;
  mEulerAngles(ePsi) = psi;

  double thtd2 = 0.5*tht;
  double psid2 = 0.5*psi;
  double phid2 = 0.5*phi;

  double Sthtd2 = sin(thtd2);
  double Spsid2 = sin(psid2);
  double Sphid2 = sin(phid2);

  double Cthtd2 = cos(thtd2);
  double Cpsid2 = cos(psid2);
  double Cphid2 = cos(phid2);

  double Cphid2Cthtd2 = Cphid2*Cthtd2;
  double Cphid2Sthtd2 = Cphid2*Sthtd2;
  double Sphid2Sthtd2 = Sphid2*Sthtd2;
  double Sphid2Cthtd2 = Sphid2*Cthtd2;

  data[0] = Cphid2Cthtd2*Cpsid2 + Sphid2Sthtd2*Spsid2;
  data[1] = Sphid2Cthtd2*Cpsid2 - Cphid2Sthtd2*Spsid2;
  data[2] = Cphid2Sthtd2*Cpsid2 + Sphid2Cthtd2*Spsid2;
  data[3] = Cphid2Cthtd2*Spsid2 - Sphid2Sthtd2*Cpsid2;

  Normalize();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

std::string Quaternion::Dump(const std::string& delimiter) const {
  std::ostringstream buffer;
  buffer << std::setprecision(16) << data[0] << delimiter;
  buffer << std::setprecision(16) << data[1] << delimiter;
  buffer << std::setprecision(16) << data[2] << delimiter;
  buffer << std::setprecision(16) << data[3];
  return buffer.str();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
  os << q(1) << " , " << q(2) << " , " << q(3) << " , " << q(4);
  return os;
}