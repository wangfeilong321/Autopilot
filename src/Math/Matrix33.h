#pragma once

#include <string>
#include <iosfwd>

#include <ColumnVector3.h>

class Quaternion;

class MatrixException {
public:
  std::string Message;
};

class Matrix33 {
public:
  enum {
    eRows = 3,
    eColumns = 3
  };
	
	/** Default constructor */
	Matrix33();

  /** Copy constructor.
	    @param M Matrix which is used for initialization.
			Create copy of the matrix given in the argument.
  */
	Matrix33(const Matrix33& M);

  /** Initialization by given values.
	    @param m11 value of the 1,1 Matrix element.
      @param m12 value of the 1,2 Matrix element.
      @param m13 value of the 1,3 Matrix element.
      @param m21 value of the 2,1 Matrix element.
      @param m22 value of the 2,2 Matrix element.
      @param m23 value of the 2,3 Matrix element.
      @param m31 value of the 3,1 Matrix element.
      @param m32 value of the 3,2 Matrix element.
      @param m33 value of the 3,3 Matrix element.
			Create a matrix from the doubles given in the arguments.
  */
	Matrix33(const double m11, const double m12, const double m13, const double m21, const double m22, const double m23, const double m31, const double m32, const double m33);

  /** Destructor */
	~Matrix33();

  /** Prints the contents of the matrix.
      @param delimeter the item separator (tab or comma)
      @return a string with the delimeter-separated contents of the matrix  */
  std::string Dump(const std::string& delimeter) const;

  /** Prints the contents of the matrix.
      @param delimeter the item separator (tab or comma, etc.)
      @param prefix an additional prefix that is used to indent the 3X3 matrix printout
      @return a string with the delimeter-separated contents of the matrix  */
  std::string Dump(const std::string& delimiter, const std::string& prefix) const;

  /** Read access the entries of the matrix.
      @param row Row index.
      @param col Column index.
			@return the value of the matrix entry at the given row and
      column indices. Indices are counted starting with 1.
  */
	double operator()(unsigned int row, unsigned int col) const;

  /** Write access the entries of the matrix.
      Note that the indices given in the arguments are unchecked.
			@param row Row index.
      @param col Column index.
			@return a reference to the matrix entry at the given row and
      column indices. Indices are counted starting with 1.
  */
	double& operator()(unsigned int row, unsigned int col);

  /** Read access the entries of the matrix.
      This function is just a shortcut for the <tt>double&
      operator()(unsigned int row, unsigned int col)</tt> function. It is
      used internally to access the elements in a more convenient way.
			Note that the indices given in the arguments are unchecked.
			@param row Row index.
      @param col Column index.
			@return the value of the matrix entry at the given row and
      column indices. Indices are counted starting with 1.
  */
	double Entry(unsigned int row, unsigned int col) const;

  /** Write access the entries of the matrix.
      This function is just a shortcut for the <tt>double&
      operator()(unsigned int row, unsigned int col)</tt> function. It is
      used internally to access the elements in a more convenient way.
			Note that the indices given in the arguments are unchecked.
			@param row Row index.
      @param col Column index.
			@return a reference to the matrix entry at the given row and
      column indices. Indices are counted starting with 1.
  */
	double& Entry(unsigned int row, unsigned int col);

  /** Number of rows in the matrix.
      @return the number of rows in the matrix.
  */
	unsigned int Rows() const;

  /** Number of columns in the matrix.
      @return the number of columns in the matrix.
  */
	unsigned int Cols() const;

  /** Transposed matrix.
      This function only returns the transpose of this matrix. This matrix itself
      remains unchanged.
      @return the transposed matrix.
  */
	Matrix33 Transposed() const;

  /** Transposes this matrix.
      This function only transposes this matrix. Nothing is returned.
  */
  void T();

  /** Initialize the matrix.
    This function initializes a matrix to all 0.0.
  */
  void InitMatrix();

	/** Initialize the matrix.
    This function initializes a matrix to user specified values.
  */
	void InitMatrix(const double m11, const double m12, const double m13, const double m21, const double m22, const double m23, const double m31, const double m32, const double m33);

	/** Returns the quaternion associated with this direction cosine (rotation) matrix.
	*/
	Quaternion GetQuaternion(void) const;

  /** Returns the quaternion associated with this direction cosine (rotation) matrix.
  */
		//FGQuaternion GetQuaternion(void) const;
		/** Returns the Euler angle column vector associated with this matrix.
  */
  ColumnVector3 GetEuler() const;

  /** Determinant of the matrix.
      @return the determinant of the matrix.
  */
  double Determinant() const;

  /** Return if the matrix is invertible.
      Checks and returns if the matrix is nonsingular and thus
      invertible. This is done by simply computing the determinant and
      check if it is zero. Note that this test does not cover any
      instabilities caused by nearly singular matirces using finite
      arithmetics. It only checks exact singularity.
  */
	bool Invertible() const;

  /** Return the inverse of the matrix.
      Computes and returns if the inverse of the matrix. It is computed
      by Cramers Rule. Also there are no checks performed if the matrix
      is invertible. If you are not sure that it really is check this
      with the @ref Invertible() call before.
   */
  Matrix33 Inverse() const;

  /** Assignment operator.
	    @param A source matrix.
			Copy the content of the matrix given in the argument into *this.
   */
	Matrix33& operator=(const Matrix33& A);

  /** Matrix vector multiplication.
	    @param v vector to multiply with.
      @return matric vector product.
			Compute and return the product of the current matrix with the
      vector given in the argument.
   */
  ColumnVector3 operator*(const ColumnVector3& v) const;

  /** Matrix subtraction.
	    @param B matrix to add to.
      @return difference of the matrices.
			Compute and return the sum of the current matrix and the matrix
      B given in the argument.
  */
  Matrix33 operator-(const Matrix33& B) const;

  /** Matrix addition.
	    @param B matrix to add to.
      @return sum of the matrices.
			Compute and return the sum of the current matrix and the matrix
      B given in the argument.
  */
  Matrix33 operator+(const Matrix33& B) const;

  /** Matrix product.
	    @param B matrix to add to.
      @return product of the matrices.
			Compute and return the product of the current matrix and the matrix
      B given in the argument.
  */
  Matrix33 operator*(const Matrix33& B) const;

  /** Multiply the matrix with a scalar.
	    @param scalar scalar factor to multiply with.
      @return scaled matrix.
			Compute and return the product of the current matrix with the
      scalar value scalar given in the argument.
  */
  Matrix33 operator*(const double scalar) const;

  /** Multiply the matrix with 1.0/scalar.
	    @param scalar scalar factor to divide through.
      @return scaled matrix.
			Compute and return the product of the current matrix with the
      scalar value 1.0/scalar, where scalar is given in the argument.
  */
  Matrix33 operator/(const double scalar) const;

	/** In place matrix addition.
			@param B matrix to add.
			@return reference to the current matrix.
			Compute the sum of the current matrix and the matrix B
			given in the argument.
	*/
	Matrix33& operator+=(const Matrix33 &B);
	
	/** In place matrix multiplication.
			@param B matrix to multiply with.
			@return reference to the current matrix.
			Compute the product of the current matrix and the matrix B
			given in the argument.
	*/
	Matrix33& operator*=(const Matrix33 &B);

	/** In place matrix scale.
			@param scalar scalar value to multiply with.
			@return reference to the current matrix.
			Compute the product of the current matrix and the scalar value scalar
			given in the argument.
	*/
	Matrix33& operator*=(const double scalar);
	
  /** In place matrix subtraction.
	    @param B matrix to subtract.
      @return reference to the current matrix.
			Compute the diffence from the current matrix and the matrix B
      given in the argument.
  */
  Matrix33& operator-=(const Matrix33 &B);
    
  /** In place matrix scale.
      @param scalar scalar value to divide through.
      @return reference to the current matrix.
      Compute the product of the current matrix and the scalar value
      1.0/scalar, where scalar is given in the argument.
  */
  Matrix33& operator/=(const double scalar);

private:
  double data[eRows*eColumns];
};

/** Scalar multiplication.
    @param scalar scalar value to multiply with.
    @param A Matrix to multiply.
		Multiply the Matrix with a scalar value.
*/
inline Matrix33 operator*(double scalar, const Matrix33& A) {
  // use already defined operation.
  return A*scalar;
}

/** Write matrix to a stream.
    @param os Stream to write to.
    @param M Matrix to write.
		Write the matrix to a stream.
*/
std::ostream& operator<<(std::ostream& os, const Matrix33& M);

/** Read matrix from a stream.
    @param os Stream to read from.
    @param M Matrix to initialize with the values from the stream.
    Read matrix from a stream.
*/
std::istream& operator>>(std::istream& is, Matrix33& M);