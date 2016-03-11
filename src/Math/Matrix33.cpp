#include <Base.h>
#include <Matrix33.h>
#include <Quaternion.h>
#include <sstream>
#include <iomanip>

#include <iostream>

using namespace std;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33::Matrix33(void) {
  data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = data[7] = data[8] = 0.0;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33::Matrix33(const Matrix33& M) {
  data[0] = M.data[0];
  data[1] = M.data[1];
  data[2] = M.data[2];
  data[3] = M.data[3];
  data[4] = M.data[4];
  data[5] = M.data[5];
  data[6] = M.data[6];
  data[7] = M.data[7];
  data[8] = M.data[8];
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33::Matrix33(const double m11, const double m12, const double m13, const double m21, const double m22, const double m23, const double m31, const double m32, const double m33) {
  data[0] = m11;
  data[1] = m21;
  data[2] = m31;
  data[3] = m12;
  data[4] = m22;
  data[5] = m32;
  data[6] = m13;
  data[7] = m23;
  data[8] = m33;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33::~Matrix33() {}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string Matrix33::Dump(const string& delimiter) const {
  ostringstream buffer;
  buffer << setw(12) << setprecision(10) << data[0] << delimiter;
  buffer << setw(12) << setprecision(10) << data[3] << delimiter;
  buffer << setw(12) << setprecision(10) << data[6] << delimiter;
  buffer << setw(12) << setprecision(10) << data[1] << delimiter;
  buffer << setw(12) << setprecision(10) << data[4] << delimiter;
  buffer << setw(12) << setprecision(10) << data[7] << delimiter;
  buffer << setw(12) << setprecision(10) << data[2] << delimiter;
  buffer << setw(12) << setprecision(10) << data[5] << delimiter;
  buffer << setw(12) << setprecision(10) << data[8];
  return buffer.str();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string Matrix33::Dump(const string& delimiter, const string& prefix) const {
  ostringstream buffer;

  buffer << prefix << right << fixed << setw(9) << setprecision(6) << data[0] << delimiter;
  buffer << right << fixed << setw(9) << setprecision(6) << data[3] << delimiter;
  buffer << right << fixed << setw(9) << setprecision(6) << data[6] << endl;

  buffer << prefix << right << fixed << setw(9) << setprecision(6) << data[1] << delimiter;
  buffer << right << fixed << setw(9) << setprecision(6) << data[4] << delimiter;
  buffer << right << fixed << setw(9) << setprecision(6) << data[7] << endl;

  buffer << prefix << right << fixed << setw(9) << setprecision(6) << data[2] << delimiter;
  buffer << right << fixed << setw(9) << setprecision(6) << data[5] << delimiter;
  buffer << right << fixed << setw(9) << setprecision(6) << data[8];

  buffer << setw(0) << left;

  return buffer.str();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Matrix33::operator()(unsigned int row, unsigned int col) const {
  return data[(col - 1)*eRows + row - 1];
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double& Matrix33::operator()(unsigned int row, unsigned int col) {
  return data[(col - 1)*eRows + row - 1];
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Matrix33::Entry(unsigned int row, unsigned int col) const {
  return data[(col - 1)*eRows + row - 1];
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double& Matrix33::Entry(unsigned int row, unsigned int col) {
  return data[(col - 1)*eRows + row - 1];
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

unsigned int Matrix33::Rows() const { return eRows; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

unsigned int Matrix33::Cols() const { return eColumns; }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33 Matrix33::Transposed() const {
  return Matrix33(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Matrix33::T(void) {
  double tmp;

  tmp = data[3];
  data[3] = data[1];
  data[1] = tmp;

  tmp = data[6];
  data[6] = data[2];
  data[2] = tmp;

  tmp = data[7];
  data[7] = data[5];
  data[5] = tmp;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Matrix33::InitMatrix(void) {
  data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = data[7] = data[8] = 0.0;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Matrix33::InitMatrix(const double m11, const double m12, const double m13, const double m21, const double m22, const double m23, const double m31, const double m32, const double m33) {
  data[0] = m11;
  data[1] = m21;
  data[2] = m31;
  data[3] = m12;
  data[4] = m22;
  data[5] = m32;
  data[6] = m13;
  data[7] = m23;
  data[8] = m33;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Quaternion Matrix33::GetQuaternion(void) const {
  Quaternion Q;

  double tempQ[4];
  int idx;

  tempQ[0] = 1.0 + data[0] + data[4] + data[8];
  tempQ[1] = 1.0 + data[0] - data[4] - data[8];
  tempQ[2] = 1.0 - data[0] + data[4] - data[8];
  tempQ[3] = 1.0 - data[0] - data[4] + data[8];

  // Find largest of the above
  idx = 0;
  for (int i = 1; i<4; i++) if (tempQ[i] > tempQ[idx]) idx = i;

  switch (idx) {
  case 0:
    Q(1) = 0.50*sqrt(tempQ[0]);
    Q(2) = 0.25*(data[7] - data[5]) / Q(1);
    Q(3) = 0.25*(data[2] - data[6]) / Q(1);
    Q(4) = 0.25*(data[3] - data[1]) / Q(1);
    break;
  case 1:
    Q(2) = 0.50*sqrt(tempQ[1]);
    Q(1) = 0.25*(data[7] - data[5]) / Q(2);
    Q(3) = 0.25*(data[3] + data[1]) / Q(2);
    Q(4) = 0.25*(data[2] + data[6]) / Q(2);
    break;
  case 2:
    Q(3) = 0.50*sqrt(tempQ[2]);
    Q(1) = 0.25*(data[2] - data[6]) / Q(3);
    Q(2) = 0.25*(data[3] + data[1]) / Q(3);
    Q(4) = 0.25*(data[7] + data[5]) / Q(3);
    break;
  case 3:
    Q(4) = 0.50*sqrt(tempQ[3]);
    Q(1) = 0.25*(data[3] - data[1]) / Q(4);
    Q(2) = 0.25*(data[6] + data[2]) / Q(4);
    Q(3) = 0.25*(data[7] + data[5]) / Q(4);
    break;
  default:
    //error
    break;
  }
  return (Q);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3 Matrix33::GetEuler(void) const {
  ColumnVector3 mEulerAngles;

  if (data[8] == 0.0)
    mEulerAngles(1) = 0.5*M_PI;
  else
    mEulerAngles(1) = atan2(data[7], data[8]);
  
  if (data[6] < -1.0)
    mEulerAngles(2) = 0.5*M_PI;
  else if (1.0 < data[6])
    mEulerAngles(2) = -0.5*M_PI;
  else
    mEulerAngles(2) = asin(-data[6]);
  
  if (data[0] == 0.0)
    mEulerAngles(3) = 0.5*M_PI;
  else {
    double psi = atan2(data[3], data[0]);
    if (psi < 0.0)
      psi += 2*M_PI;
    mEulerAngles(3) = psi;
  }

  return mEulerAngles;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double Matrix33::Determinant(void) const {
  return data[0] * data[4] * data[8] + data[3] * data[7] * data[2]
    + data[6] * data[1] * data[5] - data[6] * data[4] * data[2]
    - data[3] * data[1] * data[8] - data[7] * data[5] * data[0];
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool Matrix33::Invertible() const { return 0.0 != Determinant(); }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33 Matrix33::Inverse(void) const {
  // Compute the inverse of a general matrix using Cramers rule.

  if (Determinant() != 0.0) {
    double rdet = 1.0 / Determinant();

    double i11 = rdet*(data[4] * data[8] - data[7] * data[5]);
    double i21 = rdet*(data[7] * data[2] - data[1] * data[8]);
    double i31 = rdet*(data[1] * data[5] - data[4] * data[2]);
    double i12 = rdet*(data[6] * data[5] - data[3] * data[8]);
    double i22 = rdet*(data[0] * data[8] - data[6] * data[2]);
    double i32 = rdet*(data[3] * data[2] - data[0] * data[5]);
    double i13 = rdet*(data[3] * data[7] - data[6] * data[4]);
    double i23 = rdet*(data[6] * data[1] - data[0] * data[7]);
    double i33 = rdet*(data[0] * data[4] - data[3] * data[1]);

    return Matrix33(i11, i12, i13,
      i21, i22, i23,
      i31, i32, i33);
  }
  else {
    return Matrix33(0, 0, 0,
      0, 0, 0,
      0, 0, 0);
  }
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33& Matrix33::operator=(const Matrix33& A) {
  data[0] = A.data[0];
  data[1] = A.data[1];
  data[2] = A.data[2];
  data[3] = A.data[3];
  data[4] = A.data[4];
  data[5] = A.data[5];
  data[6] = A.data[6];
  data[7] = A.data[7];
  data[8] = A.data[8];
  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ColumnVector3 Matrix33::operator*(const ColumnVector3& v) const {
  double v1 = v(1);
  double v2 = v(2);
  double v3 = v(3);

  double tmp1 = v1*data[0];  //[(col-1)*eRows+row-1]
  double tmp2 = v1*data[1];
  double tmp3 = v1*data[2];

  tmp1 += v2*data[3];
  tmp2 += v2*data[4];
  tmp3 += v2*data[5];

  tmp1 += v3*data[6];
  tmp2 += v3*data[7];
  tmp3 += v3*data[8];

  return ColumnVector3(tmp1, tmp2, tmp3);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33 Matrix33::operator-(const Matrix33& M) const {
  return Matrix33( data[0] - M.data[0],
                     data[3] - M.data[3],
                     data[6] - M.data[6],
                     data[1] - M.data[1],
                     data[4] - M.data[4],
                     data[7] - M.data[7],
                     data[2] - M.data[2],
                     data[5] - M.data[5],
                     data[8] - M.data[8] );
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33 Matrix33::operator+(const Matrix33& M) const
{
  return Matrix33(data[0] + M.data[0],
    data[3] + M.data[3],
    data[6] + M.data[6],
    data[1] + M.data[1],
    data[4] + M.data[4],
    data[7] + M.data[7],
    data[2] + M.data[2],
    data[5] + M.data[5],
    data[8] + M.data[8]);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33 Matrix33::operator*(const Matrix33& M) const {
  Matrix33 Product;

  Product.data[0] = data[0] * M.data[0] + data[3] * M.data[1] + data[6] * M.data[2];
  Product.data[3] = data[0] * M.data[3] + data[3] * M.data[4] + data[6] * M.data[5];
  Product.data[6] = data[0] * M.data[6] + data[3] * M.data[7] + data[6] * M.data[8];
  Product.data[1] = data[1] * M.data[0] + data[4] * M.data[1] + data[7] * M.data[2];
  Product.data[4] = data[1] * M.data[3] + data[4] * M.data[4] + data[7] * M.data[5];
  Product.data[7] = data[1] * M.data[6] + data[4] * M.data[7] + data[7] * M.data[8];
  Product.data[2] = data[2] * M.data[0] + data[5] * M.data[1] + data[8] * M.data[2];
  Product.data[5] = data[2] * M.data[3] + data[5] * M.data[4] + data[8] * M.data[5];
  Product.data[8] = data[2] * M.data[6] + data[5] * M.data[7] + data[8] * M.data[8];

  return Product;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33 Matrix33::operator*(const double scalar) const {
  return Matrix33(scalar * data[0],
    scalar * data[3],
    scalar * data[6],
    scalar * data[1],
    scalar * data[4],
    scalar * data[7],
    scalar * data[2],
    scalar * data[5],
    scalar * data[8]);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33 Matrix33::operator/(const double scalar) const {
  Matrix33 Quot;
  if (scalar != 0) {
    double tmp = 1.0 / scalar;
    Quot.data[0] = data[0] * tmp;
    Quot.data[3] = data[3] * tmp;
    Quot.data[6] = data[6] * tmp;
    Quot.data[1] = data[1] * tmp;
    Quot.data[4] = data[4] * tmp;
    Quot.data[7] = data[7] * tmp;
    Quot.data[2] = data[2] * tmp;
    Quot.data[5] = data[5] * tmp;
    Quot.data[8] = data[8] * tmp;
  }
  else {
    MatrixException mE;
    mE.Message = "Attempt to divide by zero in method Matrix33::operator/(const double scalar)";
    throw mE;
  }
  return Quot;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33& Matrix33::operator+=(const Matrix33 &M) {
  data[0] += M.data[0];
  data[3] += M.data[3];
  data[6] += M.data[6];
  data[1] += M.data[1];
  data[4] += M.data[4];
  data[7] += M.data[7];
  data[2] += M.data[2];
  data[5] += M.data[5];
  data[8] += M.data[8];

  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33& Matrix33::operator*=(const double scalar) {
  data[0] *= scalar;
  data[3] *= scalar;
  data[6] *= scalar;
  data[1] *= scalar;
  data[4] *= scalar;
  data[7] *= scalar;
  data[2] *= scalar;
  data[5] *= scalar;
  data[8] *= scalar;

  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33& Matrix33::operator*=(const Matrix33& M) {
  // FIXME: Make compiler friendlier
  double a,b,c;

  a = data[0]; b=data[3]; c=data[6];
  data[0] = a*M.data[0] + b*M.data[1] + c*M.data[2];
  data[3] = a*M.data[3] + b*M.data[4] + c*M.data[5];
  data[6] = a*M.data[6] + b*M.data[7] + c*M.data[8];

  a = data[1]; b=data[4]; c=data[7];
  data[1] = a*M.data[0] + b*M.data[1] + c*M.data[2];
  data[4] = a*M.data[3] + b*M.data[4] + c*M.data[5];
  data[7] = a*M.data[6] + b*M.data[7] + c*M.data[8];

  a = data[2]; b=data[5]; c=data[8];
  data[2] = a*M.data[0] + b*M.data[1] + c*M.data[2];
  data[5] = a*M.data[3] + b*M.data[4] + c*M.data[5];
  data[8] = a*M.data[6] + b*M.data[7] + c*M.data[8];

  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33& Matrix33::operator-=(const Matrix33 &M) {
  data[0] -= M.data[0];
  data[1] -= M.data[1];
  data[2] -= M.data[2];
  data[3] -= M.data[3];
  data[4] -= M.data[4];
  data[5] -= M.data[5];
  data[6] -= M.data[6];
  data[7] -= M.data[7];
  data[8] -= M.data[8];

  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Matrix33& Matrix33::operator/=(const double scalar) {
  if ( scalar != 0 ) {
    double tmp = 1.0/scalar;
    data[0] *= tmp;
    data[3] *= tmp;
    data[6] *= tmp;
    data[1] *= tmp;
    data[4] *= tmp;
    data[7] *= tmp;
    data[2] *= tmp;
    data[5] *= tmp;
    data[8] *= tmp;
  } else {
    MatrixException mE;
    mE.Message = "Attempt to divide by zero in method Matrix33::operator/=(const double scalar)";
    throw mE;
  }
  return *this;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ostream& operator<<(ostream& os, const Matrix33& M) {
  for (unsigned int i = 1; i <= M.Rows(); i++) {
    for (unsigned int j = 1; j <= M.Cols(); j++) {
      if (i == M.Rows() && j == M.Cols())
        os << M(i, j);
      else
        os << M(i, j) << ", ";
    }
  }
  return os;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

istream& operator>>(istream& is, Matrix33& M) {
  for (unsigned int i = 1; i <= M.Rows(); i++) {
    for (unsigned int j = 1; j <= M.Cols(); j++) {
      is >> M(i, j);
    }
  }
  return is;
}