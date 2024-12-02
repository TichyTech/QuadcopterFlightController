final float ANGLE_FS = 180.0;
final float FORCE_FS = 200.0;
final float MAX_15BIT = 32767.0;
final float MAX_16BIT = 65535.0;

void loadValsTelemetry(String[] data){
  battery = float(data[1]);
  altitude = float(data[2]);
}

void loadValsState(String[] data) {
  ms = int(data[1]);
  RPY[0] = float(data[2]);
  RPY[1] = float(data[3]);
  RPY[2] = float(data[4]);
  motor_percentages[0] = float(data[5]);
  motor_percentages[1] = float(data[6]);
  motor_percentages[2] = float(data[7]);
  motor_percentages[3] = float(data[8]);
  forces[0] = float(data[9]);
  forces[1] = float(data[10]);
  forces[2] = float(data[11]);
}

void loadValsStateGibberish(ByteBuffer buffer_wrapped){
  short [] signed_vals = new short [6];
  int [] unsigned_vals = new int [4];
  
  ms = buffer_wrapped.getInt() & 0xFFFFFFFFL;  // first entry is a uint32_t ms
  
  for (int i = 0; i < 6; i++){
    signed_vals[i] = buffer_wrapped.getShort();
  }
  for (int i = 0; i < 4; i++){
    unsigned_vals[i] = buffer_wrapped.getShort() & 0xFFFF;
  }
  
  RPY[0] = float(signed_vals[0]) * (ANGLE_FS / MAX_15BIT);
  RPY[1] = float(signed_vals[1]) * (ANGLE_FS / MAX_15BIT);
  RPY[2] = float(signed_vals[2]) * (ANGLE_FS / MAX_15BIT);
  forces[0] = float(signed_vals[3]) * (FORCE_FS / MAX_15BIT);
  forces[1] = float(signed_vals[4]) * (FORCE_FS / MAX_15BIT);
  forces[2] = float(signed_vals[5]) * (FORCE_FS / MAX_15BIT);
  
  motor_percentages[0] = float(unsigned_vals[0]) * (1 / MAX_16BIT);
  motor_percentages[1] = float(unsigned_vals[1]) * (1 / MAX_16BIT);
  motor_percentages[2] = float(unsigned_vals[2]) * (1 / MAX_16BIT);
  motor_percentages[3] = float(unsigned_vals[3]) * (1 / MAX_16BIT);

}

void loadValsTelemetryGibberish(ByteBuffer buffer_wrapped){
  battery = buffer_wrapped.getFloat();
  altitude = buffer_wrapped.getFloat();
}
