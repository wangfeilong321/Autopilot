#pragma once

#include <iosfwd>
#include <string>

class ColumnVector3 {
public:
  /** Default initializer.
      Create a zero vector */
  ColumnVector3(void);

  /** Initialization by given values.
      @param X value of the x-conponent.
      @param Y value of the y-conponent.
      @param Z value of the z-conponent.
      Create a vector from the doubles given in the arguments.   */
  ColumnVector3(const double X, const double Y, const double Z);

  /** Copy constructor.
      @param v Vector which is used for initialization.
      Create copy of the vector given in the argument.   */
  ColumnVector3(const ColumnVector3& v);

  /// Destructor.
  ~ColumnVector3();

  /** Read access the entries of the vector.
      @param idx the component index.
      Return the value of the matrix entry at the given index.
      Indices are counted starting with 1.
      Note that the index given in the argument is unchecked.   */
  double operator()(const unsigned int idx) const;

  /** Write access the entries of the vector.
      @param idx the component index.
      Return a reference to the vector entry at the given index.
      Indices are counted starting with 1.
      Note that the index given in the argument is unchecked.   */
  double& operator()(const unsigned int idx);

  /** Read access the entries of the vector.
      @param idx the component index.
      Return the value of the matrix entry at the given index.
      Indices are counted starting with 1.
      This function is just a shortcut for the <tt>double
      operator()(unsigned int idx) const</tt> function. It is
      used internally to access the elements in a more convenient way.
      Note that the index given in the argument is unchecked.   */
  double Entry(const unsigned int idx) const;

  /** Write access the entries of the vector.
      @param idx the component index.
      Return a reference to the vector entry at the given index.
      Indices are counted starting with 1.
      This function is just a shortcut for the <tt>double&
      operator()(unsigned int idx)</tt> function. It is
      used internally to access the elements in a more convenient way.
      Note that the index given in the argument is unchecked.   */
  double& Entry(const unsigned int idx);

  /** Prints the contents of the vector
      @param delimeter the item separator (tab or comma)
      @return a string with the delimeter-separated contents of the vector  */
  std::string Dump(const std::string& delimeter) const;

  /** Assignment operator.
      @param b source vector.
      Copy the content of the vector given in the argument into *this.   */
  ColumnVector3& operator=(const ColumnVector3& b);

  /** Comparison operator.
      @param b other vector.
      Returns true if both vectors are exactly the same.   */
  bool operator==(const ColumnVector3& b) const;

  /** Comparison operator.
      @param b other vector.
      Returns false if both vectors are exactly the same.   */
  bool operator!=(const ColumnVector3& b) const;

  /** Multiplication by a scalar.
      @param scalar scalar value to multiply the vector with.
      @return The resulting vector from the multiplication with that scalar.
      Multiply the vector with the scalar given in the argument.   */
  ColumnVector3 operator*(const double scalar) const;
  
  /** Cross product multiplication.
  @param V vector to multiply with.
  @return The resulting vector from the cross product multiplication.
  Compute and return the cross product of the current vector with
  the given argument.   */
  ColumnVector3 operator*(const ColumnVector3& V) const;

  /// Scale by a scalar.
  ColumnVector3& operator*=(const double scalar);

  /// Addition operator.
  ColumnVector3 operator+(const ColumnVector3& B) const;

  /// Add an other vector.
  ColumnVector3& operator+=(const ColumnVector3 &B);

  /// Subtraction operator.
  ColumnVector3 operator-(const ColumnVector3& B) const;

  /// Subtract an other vector.
  ColumnVector3& operator-=(const ColumnVector3 &B);

   /** Multiply by 1/scalar.
      @param scalar scalar value to devide the vector through.
      @return The resulting vector from the division through that scalar.
      Multiply the vector with the 1/scalar given in the argument.   */
  ColumnVector3 operator/(const double scalar) const;
    
  /// Scale by a 1/scalar.
  ColumnVector3& operator/=(const double scalar);

  void InitMatrix(void);
  void InitMatrix(const double a);
  void InitMatrix(const double a, const double b, const double c);

  /** Length of the vector.
      Compute and return the euclidean norm of this vector.   */
  double Magnitude(void) const;

  /** Length of the vector in a coordinate axis plane.
      Compute and return the euclidean norm of this vector projected into
      the coordinate axis plane idx1-idx2.   */
  double Magnitude(const int idx1, const int idx2) const;

  /** Normalize.
      Normalize the vector to have the Magnitude() == 1.0. If the vector
      is equal to zero it is left untouched.   */
  ColumnVector3& Normalize(void);

private:
  double data[3];
};

/** Dot product of two vectors
    Compute and return the euclidean dot (or scalar) product of two vectors
    v1 and v2 */
inline double DotProduct(const ColumnVector3& v1, const ColumnVector3& v2) {
  return v1(1)*v2(1) + v1(2)*v2(2) + v1(3)*v2(3);
}

/** Scalar multiplication.
    @param scalar scalar value to multiply with.
    @param A Vector to multiply.
    Multiply the Vector with a scalar value. Note: At this time, this
    operator MUST be inlined, or a multiple definition link error will occur.*/
inline ColumnVector3 operator*(double scalar, const ColumnVector3& A) {
  // use already defined operation.
  return A*scalar;
}

/** Write vector to a stream.
    @param os Stream to write to.
    @param col vector to write.
    Write the vector to a stream.*/
std::ostream& operator<<(std::ostream& os, const ColumnVector3& col);