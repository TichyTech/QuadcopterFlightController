#ifndef ALGEBRA
#define ALGEBRA

#include "definitions.h"

/**
*Map angle from [-360, 360] to [-180, 180].
*/
inline float constrain_angle(float angle){
  return fmod(angle + 540, 360) - 180;  // signed angle difference
}

/**
* Dot product of two Vectors.
*/
template <typename Derived0, typename Derived1, int rows, typename dtype>
inline float dot(const MatrixBase<Derived0, rows, 1, dtype>& a, const MatrixBase<Derived1, rows, 1, dtype>& b){
  float sum = 0;
  for (int i = 0; i < rows; i++) sum += a(i)*b(i);
  return sum;
}

/**
* The norm of a Vector
*/
template <typename Derived, int rows, typename dtype>
inline float norm(const MatrixBase<Derived, rows, 1, dtype>& v) {
  return sqrt(dot(v,v));
}

/**
 * find the maximum entry in a matrix
 */
template <int Rows, int Cols>
inline float max_norm(const Matrix<Rows,Cols>& A){
  float max_float = 0;

  for (int i = 0; i < Rows; i++){
    for (int j = 0; j < Cols; j++){
      if (A(i,j) > max_float) max_float = A(i,j);
    }
  }

  return max_float;
}

/**
 * find the minimum entry in a matrix
 */
template <int Rows, int Cols>
inline float min_norm(const Matrix<Cols,Rows>& A){
  float min_float = 0;

  for (int i = 0; i < Cols; i++){
    for (int j = 0; j < Rows; j++){
      if (A(i,j) < min_float) min_float = A(i,j);
    }
  }

  return min_float;
}

/**
* The determinant of a 3x3 matrix.
*/
inline float det(Matrix3 A){
  float sum = A(0,0)*(A(1,1)*A(2,2) - A(1,2)*A(2,1));
  sum += A(0,1)*(A(1,2)*A(2,0) - A(1,0)*A(2,2));
  sum += A(0,2)*(A(1,0)*A(2,1) - A(1,1)*A(2,0));
  return sum;
}

/**
* Divide a Vector by its norm.
*/
template <typename Derived, int rows, typename dtype>
inline MatrixBase<Derived, rows, 1, dtype> normalize(const MatrixBase<Derived, rows, 1, dtype>& v){
  float mult = 1/norm(v);
  return v*mult;
}

/**
 * Return a skew symmetric matrix created from a Vector3 variable $[v]_x$.
 */
inline Matrix3 skew(Vector3 v){  // skew symmetric matrix 
  Matrix3 S = {0, -v(2), v(1),
               v(2), 0, -v(0),
               -v(1), v(0), 0};
  return S;
}

/**
 * Return the trace of a 3x3 Matrix
*/
inline float trace(Matrix3 A){
  return A(0,0) + A(1,1) + A(2,2);
}

/**
 * Return the outer product $vv^T$ of a vector v.
 */
inline Matrix3 outer(Vector3 v){  // skew symmetric matrix 
  Matrix3 vvT = v*(~v);
  return vvT;
}


/**
 * This function implements the Rodriguez formula for a conversion from the axis angle representation to a rotation matrix representation.
 */
inline Matrix3 rodriguez(Vector3 axis, float angle){
  Matrix3 K = skew(axis);
  Matrix3 R = I_3 + K*sin(angle) + K*K*(1 - cos(angle));  // Rodriguez rotation formula
  return R;
}

/**
 * Normalize the columns of a 3x3 matrix.
 */
inline Matrix3 normalize_columns(Matrix3 A){
  Matrix<3, 1> a1,a2,a3;
  a1 = A.Column(0);
  a2 = A.Column(1);
  a3 = A.Column(2);
  a1 = normalize(a1);
  a2 = normalize(a2);
  a3 = normalize(a3);
  return a1 || a2 || a3;
}

/**
* This implements some clever way to normalize a 3x3 matrix. 
*/
inline Matrix3 normalize_matrix(Matrix3 A){
  Vector3 a1,a2,a3;
  float e = dot(A.Column(0), A.Column(1));
  a1 = A.Column(0) - A.Column(1)*e/2.0f;
  a2 = A.Column(1) - A.Column(0)*e/2.0f;
  a3 = skew(A.Column(0)) * A.Column(1);
  return normalize_columns(a1 || a2 || a3);
}

#endif
