#include "definitions.h"
#include "config.h"

#include "accmag/LSM303DLHC.h"
#include "gyro/L3G4200D.h"
#include "alt/BMP280.h"

class Battery{
  private:
    float filtered_val;
  public:
    Battery();
    float get_voltage();
    float get_filtered_voltage();
};

class Sensors{
	private:
		Gyro gyroscope;
		AccMag accmag;
		Altimeter altimeter;
    Battery battery;
	public: 
		bool imu_timed_out;
		bool alt_timed_out;

		Sensors();
		void setup();
		Measurements get_measurements();
		Measurements get_measurements_filtered();
		float update_integration_period();
};
