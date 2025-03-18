#include "config.h"
#include "peripherals/writeReg.h"
#include "definitions.h"
#include "debug/debugging.h"

class Gyro{

    private:
        Vector3 gyro_bias;
        Vector3 last_gyro_vec;
        Vector3 filtered_gyro_vec;
        unsigned long last_gyro_timestamp; 

        Vector3 stream_buff[4];

        // dynamic bias compensation variables
        bool bias_compensation_on;
        Vector3 dynamic_bias;
        uint32_t bias_timer;

    public:
        bool gyro_timeout;

        Gyro();
        void setup_gyro();
        void setup_gyro_bypass();
        void setup_gyro_stream();

        Vector3 read_gyro_stream();
        Vector3 read_gyro_single();
        Vector3 buff_to_Vec(uint8_t *buff);
        Vector3 get_filtered_gyro();
        void calibrate_gyro();

};