#include "algebra.h"
#include "debug/debugging.h"

class KalmanFilter{
  private:
    Matrix<7,7> P;
    Matrix<7,7> Q;
    Matrix<6,6> R;

    Vector3 a_w;  // up vector in world frame
    Vector3 m_w;  // magnetic vector in world frame

    float var_w;
    float mag_inc;

  public:
    Vector4 q;  // attitude quaternion of the world frame NWU
    Vector3 b;  // gyro bias estimate

    KalmanFilter();
    void init_quat(Matrix3 DCM);
    Vector4 predict(Vector3 gyro_vec, float dt);
    Vector4 correct(Vector3 a, Vector3 m);
    Matrix3 quat2R(Vector4 q);
};
