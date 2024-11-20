void loadValsTelemetry(String[] data){
  battery = float(data[1]);
  altitude = float(data[2]);
}

void loadValsState(String[] data) {
  RPY[0] = float(data[1]);
  RPY[1] = float(data[2]);
  RPY[2] = float(data[3]);
  motor_percentages[0] = float(data[4]);
  motor_percentages[1] = float(data[5]);
  motor_percentages[2] = float(data[6]);
  motor_percentages[3] = float(data[7]);
}
