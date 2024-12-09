#ifndef ALGEBRA
#define ALGEBRA

#include "definitions.h"

const Matrix3 I = {1,0,0,0,1,0,0,0,1};

/**
*Map angle from [-360, 360] to [-180, 180].
*/
inline float constrain_angle(float angle){
  return fmod(angle + 540, 360) - 180;  // signed angle difference
}

/**
* Dot product of two Vector3 variables.
*/
inline float dot(Vector3 a, Vector3 b){
  return a(0)*b(0) + a(1)*b(1) + a(2)*b(2);
}

/**
* The norm of a Vector3 variable
*/
inline float norm(Vector3 v) {
  return sqrt(dot(v,v));
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
* Divide a Vector3 variable by its norm.
*/
inline Vector3 normalize(Vector3 v){
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
 * This function implements the Rodriguez formula for a conversion from the axis angle representation to a rotation matrix representation.
 */
inline Matrix3 rodriguez(Vector3 axis, float angle){
  Matrix3 K = skew(axis);
  Matrix3 R = I + K*sin(angle) + K*K*(1 - cos(angle));  // Rodriguez rotation formula
  return R;
}

/**
 * Normalize the columns of a 3x3 matrix.
 */
inline Matrix3 normalize_columns(Matrix3 A){
  Vector3 a1,a2,a3;
  a1 = normalize(A.Column(0));
  a2 = normalize(A.Column(1));
  a3 = normalize(A.Column(2));
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
