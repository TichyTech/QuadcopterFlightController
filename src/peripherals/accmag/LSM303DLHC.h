#include "config.h"
#include "peripherals/writeReg.h"
#include "algebra.h"

class AccMag{
    private:
        Vector3 last_acc_vec;
        Vector3 filtered_acc_vec;
        Vector3 last_mag_vec;
        Vector3 filtered_mag_vec;

        Vector3 acc_bias;
        Vector3 mag_bias;
        Matrix3 mag_scale;

        unsigned long last_acc_timestamp;
        unsigned long last_mag_timestamp;

    public:
        bool acc_timeout;
        bool mag_timeout;

        AccMag();

        void calibrate_acc();
        void setup_acc();
        void setup_mag();
        Vector3 read_acc();
        Vector3 read_mag();
        Vector3 get_filtered_acc();
        Vector3 get_filtered_mag();
};