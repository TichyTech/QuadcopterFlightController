#include <Arduino.h>

class Altimeter
{

  private:
    int32_t t_fine;
    float last_alt_val;
    float filtered_alt_val;
    unsigned long last_alt_timestamp;

    int16_t read_alt16(uint8_t reg);
    inline int32_t read_alt24(uint8_t reg);

  public:
    bool alt_timed_out;
    Altimeter();
    void setup_alt();
    float read_alt();
    float get_filtered_alt();
    int32_t read_temp();
    float read_pressure();
    void get_alt_calibration();

};