#include <ColumnVector3.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

using namespace std;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3::ColumnVector3(void) {
  data[0] = data[1] = data[2] = 0.0;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3::ColumnVector3(const double X, const double Y, const double Z) {
  data[0] = X;
  data[1] = Y;
  data[2] = Z;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3::ColumnVector3(const ColumnVector3& v) {
  data[0] = v.data[0];
  data[1] = v.data[1];
  data[2] = v.data[2];
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3::~ColumnVector3() {}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double ColumnVector3::operator()(const unsigned int idx) const { return data[idx - 1]; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double& ColumnVector3::operator()(const unsigned int idx) { return data[idx - 1]; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double ColumnVector3::Entry(const unsigned int idx) const { return data[idx - 1]; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double& ColumnVector3::Entry(const unsigned int idx) { return data[idx - 1]; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string ColumnVector3::Dump(const string& delimiter) const {
  ostringstream buffer;
  buffer << std::setprecision(16) << data[0] << delimiter;
  buffer << std::setprecision(16) << data[1] << delimiter;
  buffer << std::setprecision(16) << data[2];
  return buffer.str();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3& ColumnVector3::operator=(const ColumnVector3& b) {
  data[0] = b.data[0];
  data[1] = b.data[1];
  data[2] = b.data[2];
  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool ColumnVector3::operator==(const ColumnVector3& b) const {
  return data[0] == b.data[0] && data[1] == b.data[1] && data[2] == b.data[2];
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool ColumnVector3::operator!=(const ColumnVector3& b) const { return !operator==(b); }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3 ColumnVector3::operator*(const double scalar) const {
  return ColumnVector3(scalar*data[0], scalar*data[1], scalar*data[2]);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3 ColumnVector3::operator*(const ColumnVector3& V) const {
  return ColumnVector3(data[1] * V.data[2] - data[2] * V.data[1], data[2] * V.data[0] - data[0] * V.data[2], data[0] * V.data[1] - data[1] * V.data[0]);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3& ColumnVector3::operator*=(const double scalar) {
  data[0] *= scalar;
  data[1] *= scalar;
  data[2] *= scalar;
  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3 ColumnVector3::operator+(const ColumnVector3& B) const {
  return ColumnVector3(data[0] + B.data[0], data[1] + B.data[1], data[2] + B.data[2]);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3& ColumnVector3::operator+=(const ColumnVector3 &B) {
  data[0] += B.data[0];
  data[1] += B.data[1];
  data[2] += B.data[2];
  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3 ColumnVector3::operator-(const ColumnVector3& B) const {
  return ColumnVector3(data[0] - B.data[0], data[1] - B.data[1], data[2] - B.data[2]);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3& ColumnVector3::operator-=(const ColumnVector3 &B) {
  data[0] -= B.data[0];
  data[1] -= B.data[1];
  data[2] -= B.data[2];
  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3 ColumnVector3::operator/(const double scalar) const {
  if (scalar != 0.0)
    return operator*( 1.0/scalar );

  cerr << "Attempt to divide by zero in method \
    ColumnVector3::operator/(const double scalar), \
    object " << data[0] << " , " << data[1] << " , " << data[2] << endl;
  return ColumnVector3();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3& ColumnVector3::operator/=(const double scalar) {
  if (scalar != 0.0)
    operator*=( 1.0/scalar );
  else
    cerr << "Attempt to divide by zero in method \
      ColumnVector3::operator/=(const double scalar), \
      object " << data[0] << " , " << data[1] << " , " << data[2] << endl;

  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ColumnVector3::InitMatrix(void) { data[0] = data[1] = data[2] = 0.0; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ColumnVector3::InitMatrix(const double a) { data[0] = data[1] = data[2] = a; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void ColumnVector3::InitMatrix(const double a, const double b, const double c) {
  data[0] = a; data[1] = b; data[2] = c;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double ColumnVector3::Magnitude(void) const {
  return sqrt( data[0]*data[0] +  data[1]*data[1] +  data[2]*data[2] );
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3& ColumnVector3::Normalize() {
  double Mag = Magnitude();
  if (Mag != 0.0)
    operator*=( 1.0/Mag );
  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double ColumnVector3::Magnitude(const int idx1, const int idx2) const {
  return sqrt( data[idx1-1]*data[idx1-1] +  data[idx2-1]*data[idx2-1] );
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ostream& operator<<(ostream& os, const ColumnVector3& col) {
  os << col(1) << " , " << col(2) << " , " << col(3);
  return os;
}