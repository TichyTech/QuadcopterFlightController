#include "config.h"
#include "peripherals/writeReg.h"
#include "definitions.h"

class Gyro{

    private:
        Vector3 gyro_bias;
        Vector3 last_gyro_vec;
        Vector3 filtered_gyro_vec;
        unsigned long last_gyro_timestamp; 

        // dynamic bias compensation variables
        bool bias_compensation_on;
        Vector3 dynamic_bias;
        uint32_t bias_timer;

    public:
        bool gyro_timeout;

        Gyro();

        void setup_gyro();
        Vector3 read_gyro();
        Vector3 get_filtered_gyro();
        void calibrate_gyro();

};