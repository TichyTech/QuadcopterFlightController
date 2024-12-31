#include "kalman.h"

#define THRESHOLD (3.0f)
#define GYRO_TIMEOUT (300)  // milliseconds 

KalmanFilter::KalmanFilter(){
  q = {1,0,0,0};
  b = {0,0,0};

  // init variance matrices
  P.Fill(0);
  R.Fill(0);
  Q.Fill(0);
  // Set initial state variance
  P.Submatrix<4,4>(0,0) = I_4 * 0.01f;
  P.Submatrix<3,3>(4,4) = I_3 * 0.004f;
  var_w = 0.00006f;
  R = I_6*0.2f;

  R_acc = I_3*0.2f;
  R_mag = I_3*0.2f;

  a_w = {0,0,1};
  // mag_inc = 1.15191730632f;  // magnetic inclination [rad]
  mag_inc = 1.368266347647853f;
  // m_w = {cos(mag_inc), 0, -sin(mag_inc)};
  m_w = {1,0,0};
}

/** 
 * Init state quaternion using a DCM 
 */
void KalmanFilter::init_quat(Matrix3 DCM){
  q = {0.5f*sqrt(trace(DCM) + 1),
       0.5f*sgn(DCM(2,1) - DCM(1,2))*sqrt(DCM(0,0) - DCM(1,1) - DCM(2,2) + 1),
       0.5f*sgn(DCM(0,2) - DCM(2,0))*sqrt(DCM(1,1) - DCM(2,2) - DCM(0,0) + 1),
       0.5f*sgn(DCM(1,0) - DCM(0,1))*sqrt(DCM(2,2) - DCM(0,0) - DCM(1,1) + 1)};
  Serial.println("init quat: ");
  printVec4(q, 4);
  Serial1.println();
}


/** 
 * Gyro bias variance shaping according to current gyro measurement and time
 */
bool KalmanFilter::get_bias_var(Vector3 w, float dt){
  static float timer = 0;

  if ((abs(w(0)) > THRESHOLD*TO_RAD) || (abs(w(1)) > THRESHOLD*TO_RAD) || (abs(w(2)) > THRESHOLD*TO_RAD)){
    timer = 0;  // reset timer
  }
  else{
    timer += dt;
  }

  Serial.print("Timer: ");
  Serial.println(timer*1000.0f);
  if (timer > GYRO_TIMEOUT*0.001f){  // gyro less than threshold for atleast timeout
    // return (0.04f * dt);  // return higher variance
    Serial.println("high variance engaged");
    return 1;
  }
  // else return 0.00000001*dt;  // return small variance
  else return 0;
}

Vector4 KalmanFilter::predict(Vector3 gyro_vec, float dt){
  Vector3 w = TO_RAD*gyro_vec - b;

  // dq(t+1)/dq
  Matrix<4,4> quat_mult = 
  {0,   -w(0), -w(1), -w(2),
  w(0),  0,    -w(2),  w(1),
  w(1),  w(2),  0   , -w(0),
  w(2), -w(1),  w(0),  0};
  Matrix<4,4> F11 = I_4 - (0.5f*dt)*quat_mult;
  // dq(t+1)/db (*2/dt)
  Matrix<4,3> F12 = 
  {-q(1), -q(2), -q(3), 
    q(0),  q(3), -q(2),
   -q(3),  q(0),  q(1),
    q(2), -q(1),  q(0)};
  Zeros<3,4> Zeros3_4;  // db(t+1)/dq
  Matrix<7,7> F = (F11 || (0.5f*dt)*F12) && (Zeros3_4 || I_3);  // construct the F matrix

  Q.Submatrix<4,4>(0,0) = (0.25f*dt*dt*var_w)*F12*~F12;  // update quat variance using dq/dw
  bool update_time = get_bias_var(w, dt);
  float bias_var = 0.04*dt;
  if (!update_time) {
    bias_var = 0.00000001*dt;
    P.Submatrix<3,3>(4,4) = bias_var*I_3;  // reset variance matrix for bias to a low value
  }
  Q.Submatrix<3,3>(4,4) = I_3 * bias_var;  // init small process variance for bias

  q = F11*q;  // predict quaternion
  P = F*P*(~F) + Q;  // predict covariance

  q = normalize(q);  // renormalize quaternion
  return q;
}

Vector4 KalmanFilter::fuse_mag(Vector3 a, Vector3 m){
  m = normalize(m - a*(~m*a));  // remove magnetic field component parallel to acceleration
  // expected measurement h(x) = [R(q)a_w]
  Matrix3 Rq = quat2R(q);
  Matrix<3,1> hx = Rq*m_w;

  // actual measurement z = [m] 
  Matrix<3,1> z = m; 

  // prepare zero matrix for dh(x)p/db=0, since we do not measure the bias
  Zeros<3,3> Zeros3_3;
  float q0 = q(0);  // scalar part
  Vector3 v = q.Submatrix<3,1>(1,0);  // vector part
  Matrix3 v_x = skew(v);  // skew symmetric from v
  // H = dh(x)/dx using derivative of d(R(q)x)/dq
  Matrix<3,7> H = 2.0f*((q0*m_w + v_x*m_w) || (I_3*dot(v,m_w) + v*~m_w -q0*skew(m_w) -m_w*~v)) || Zeros3_3;

  // Compute the covariance matrix of the measurements and prepare Cholesky decomp
  Matrix<3,3> S = H*P*~H + R_mag;
  Matrix<3,3> S_cp = S;
  CholeskyDecomposition LLT = CholeskyDecompose(S_cp);  // Decompose the matrix LL^T

  // Kalman gain multiplied by S
  Matrix<7, 3> KS = P*~H;  // K = PH^TS^-1, KS = PH^T
  Matrix<7> x = q && b;  // full state vector
  // S\H computation
  Matrix<3,7> SinvH;
  for (int i=0; i<7; i++){  // solve S^-1 * H(:,i) for each column of H
    SinvH.Submatrix<3,1>(0,i) = CholeskySolve(LLT,H.Submatrix<3,1>(0,i));  
  }

  // Kalman Update of the state and state covariance
  Matrix<3,1> error = z - hx;
  Matrix<7,1> innovation = KS * CholeskySolve(LLT, error);
  x = x + innovation;  // x = x + P H^T S^-1 (z - h(x))
  P = (I_7 - KS*SinvH)*P;

  Serial.print("1000innovation mag: ");
  printVec7(1000.0f*innovation, 2);
  Serial.println();

  // Save result back to our state vectors
  q = x.Submatrix<4,1>(0,0);
  b = x.Submatrix<3,1>(4,0);

  Serial.print("bias: ");
  printVec3(b*TO_DEG, 2);
  Serial.println();

  Serial.println("1000bias-state covar: ");
  (1000.0f*P.Submatrix<3,4>(4,3)).printTo(Serial);
  Serial.println();

  Serial.println("1000bias var: ");
  (1000.0f*P.Submatrix<3,3>(4,4)).printTo(Serial);
  Serial.println();

  q = normalize(q);  // renormalize quaternion
  return q;
}

Vector4 KalmanFilter::fuse_acc(Vector3 a){
  // expected measurement h(x) = [R(q)a_w]
  Matrix3 Rq = quat2R(q);
  Matrix<3,1> hx = Rq*a_w;

  // actual measurement z = [a] 
  Matrix<3,1> z = a; 

  // prepare zero matrix for dh(x)/db=0, since we do not measure the bias
  Zeros<3,3> Zeros3_3;
  float q0 = q(0);  // scalar part
  Vector3 v = q.Submatrix<3,1>(1,0);  // vector part
  Matrix3 v_x = skew(v);  // skew symmetric from v
  // H = dh(x)/dx using derivative of d(R(q)x)/dq
  Matrix<3,7> H = 2.0f*((q0*a_w + v_x*a_w) || (I_3*dot(v,a_w) + v*~a_w -q0*skew(a_w) -a_w*~v)) || Zeros3_3;
  // Compute the covariance matrix of the measurements and prepare Cholesky decomp
  Matrix<3,3> S = H*P*~H + R_acc;
  Matrix<3,3> S_cp = S;
  CholeskyDecomposition LLT = CholeskyDecompose(S_cp);  // Decompose the matrix LL^T

  // Kalman gain multiplied by S
  Matrix<7, 3> KS = P*~H;  // K = PH^TS^-1, KS = PH^T
  Matrix<7> x = q && b;  // full state vector
  // S\H computation
  Matrix<3,7> SinvH;
  for (int i=0; i<7; i++){  // solve S^-1 * H(:,i) for each column of H
    SinvH.Submatrix<3,1>(0,i) = CholeskySolve(LLT,H.Submatrix<3,1>(0,i));  
  }

  // Kalman Update of the state and state covariance
  Matrix<3,1> error = z - hx;
  Matrix<7,1> innovation = KS * CholeskySolve(LLT, error);
  x = x + innovation;  // x = x + P H^T S^-1 (z - h(x))
  P = (I_7 - KS*SinvH)*P;

  Serial.print("1000innovation acc: ");
  printVec7(1000.0f*innovation, 2);
  Serial.println();

  // Save result back to our state vectors
  q = x.Submatrix<4,1>(0,0);
  b = x.Submatrix<3,1>(4,0);

  Serial.print("bias: ");
  printVec3(b*TO_DEG, 2);
  Serial.println();

  Serial.println("1000bias-state covar: ");
  (1000.0f*P.Submatrix<3,4>(4,3)).printTo(Serial);
  Serial.println();

  Serial.println("1000bias var: ");
  (1000.0f*P.Submatrix<3,3>(4,4)).printTo(Serial);
  Serial.println();

  q = normalize(q);  // renormalize quaternion
  return q;
}

Vector4 KalmanFilter::fuse_acc_mag(Vector3 a, Vector3 m){
  m = normalize(m - a*(~m*a));  // remove magnetic field component parallel to acceleration

  // expected measurement h(x) = [R(q)a_w \\ R(q)m_w]
  Matrix3 Rq = quat2R(q);
  Matrix<6,1> hx = (Rq*a_w) && (Rq*m_w);
  // h.Submatrix<3, 1>(0, 0) = R.Submatrix<3,1>(0, 2);  // R*[0,0,1]^T = Ra
  // h.Submatrix<3, 1>(3, 0) = cos(mag_inc)*R.Submatrix<3,1>(0, 0) + sin(mag_inc)*R.Submatrix<3,1>(0, 2);  // R*[cos(psi),0,sin(psi)]^T = Rm

  // actual measurement z = [a \\ m] 
  Matrix<6,1> z; 
  z.Submatrix<3, 1>(0, 0) = a;
  z.Submatrix<3, 1>(3, 0) = m;

  // prepare zero matrix for dh(x)/db=0, since we do not measure the bias
  Zeros<6,3> Zeros6_3;
  float q0 = q(0);  // scalar part
  Vector3 v = q.Submatrix<3,1>(1,0);  // vector part
  Matrix3 v_x = skew(v);  // skew symmetric from v
  // H = dh(x)/dx using derivative of d(R(q)x)/dq
  Matrix<6,7> H = 
  (
  2.0f*((q0*a_w + v_x*a_w) || (I_3*dot(v,a_w) + v*~a_w -q0*skew(a_w) -a_w*~v)) 
  && 
  2.0f*((q0*m_w + v_x*m_w) || (I_3*dot(v,m_w) + v*~m_w -q0*skew(m_w) -m_w*~v))
  ) || Zeros6_3;

  // Compute the covariance matrix of the measurements and prepare Cholesky decomp
  Matrix<6,6> S = H*P*~H + R;
  Matrix<6,6> S_cp = S;
  CholeskyDecomposition LLT = CholeskyDecompose(S_cp);  // Decompose the matrix LL^T

  // Kalman gain multiplied by S
  Matrix<7, 6> KS = P*~H;  // K = PH^TS^-1, KS = PH^T
  Matrix<7> x = q && b;  // full state vector
  // S\H computation
  Matrix<6,7> SinvH;
  for (int i=0; i<7; i++){  // solve S^-1 * H(:,i) for each column of H
    SinvH.Submatrix<6,1>(0,i) = CholeskySolve(LLT,H.Submatrix<6,1>(0,i));  
  }

  // Kalman Update of the state and state covariance
  Matrix<6,1> error = z - hx;
  Matrix<7,1> innovation = KS * CholeskySolve(LLT, error);
  x = x + innovation;  // x = x + P H^T S^-1 (z - h(x))
  P = (I_7 - KS*SinvH)*P;

  // Save result back to our state vectors
  q = x.Submatrix<4,1>(0,0);
  b = x.Submatrix<3,1>(4,0);

  Serial.print("bias: ");
  printVec3(b*TO_DEG, 2);
  Serial.println();

  (1000.0f*P.Submatrix<3,3>(4,4)).printTo(Serial);
  Serial.println();

  q = normalize(q);  // renormalize quaternion
  return q;
}

Matrix3 KalmanFilter::quat2R(Vector4 q){
  float q0 = q(0);  // scalar part
  Vector3 v = q.Submatrix<3,1>(1,0);  // vector part
  Matrix3 v_x = skew(v);  // skew symmetric from v
  Matrix3 R = (q0*q0)*I_3 + v*~v + (2*q0)*v_x + v_x*v_x; 
  return R;
}