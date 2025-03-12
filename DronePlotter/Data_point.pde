class DataPoints{
  float [][] RPYs;
  float [][] mps;
  float [][] PIDs;
  float [][] refs;
  long [] mss;
  int start = 0;
  int items = 0;
  int len;
  
  DataPoints(int length){
    this.len = length;
    this.RPYs = new float [length][3];
    this.PIDs = new float [length][3];
    this.mps = new float [length][4];
    this.refs = new float[length][2];
    this.mss = new long [length];
  }
  
  void add_datapoint(float [] RPY, float [] mp, float[] PID, long ms, float[] ref){
    if (items < len){ // we can handle more data
      arrayCopy(RPY, this.RPYs[items]);
      arrayCopy(PID, this.PIDs[items]);
      arrayCopy(ref, this.refs[items]);
      arrayCopy(mp, this.mps[items]);
      this.mss[items] = ms;
      items++;
    }
    else {  // arrays are full
      arrayCopy(RPY, this.RPYs[start]);
      arrayCopy(PID, this.PIDs[start]);
      arrayCopy(ref, this.refs[start]);
      arrayCopy(mp, this.mps[start]);
      this.mss[start] = ms;
      start = (start + 1) % this.len;  // move start index instead
    }
  }
  
  void save_to_file(){
    PrintWriter writer = createWriter("DataLog.txt");
    for (int i = 0; i < this.items; i++){
      int idx = (this.start + i) % this.len;
      float [] c_mp = this.mps[idx];
      float [] c_RPY = this.RPYs[idx];
      float [] c_PID = this.PIDs[idx];
      float [] c_ref = this.refs[idx];
      long c_ms = this.mss[idx];
      String ms_string = String.format("%d ", c_ms);
      String RPY_string = String.format("%.3f %.3f %.3f ", c_RPY[0], c_RPY[1], c_RPY[2]);
      String mp_string = String.format("%.3f %.3f %.3f %.3f ", c_mp[0],c_mp[1],c_mp[2], c_mp[3]);
      String PID_string = String.format("%.3f %.3f %.3f ", c_PID[0], c_PID[1], c_PID[2]);
      String ref_string = String.format("%.3f %.3f ", c_ref[0], c_ref[1]);
      writer.println(ms_string + RPY_string + mp_string + PID_string + ref_string);
    }
    writer.close();  // Always close the writer
    println("Saved log to DataLog.txt");
  }
  
}

class SensorDataPoints{
  float [][] RPYs;
  float [][] accs;
  float [][] mags;
  float [][] gyros;
  long [] mss;
  
  int start = 0;
  int items = 0;
  int len;
  
  SensorDataPoints(int length){
    this.len = length;
    this.RPYs = new float [length][3];
    this.accs = new float [length][3];
    this.mags = new float [length][3];
    this.gyros = new float [length][3];
    this.mss = new long [length];
  }
  
  void add_datapoint(float[] RPY, float[] acc, float[] mag, float[] gyro, long ms){
    if (items < len){ // we can handle more data
      arrayCopy(RPY, this.RPYs[items]);
      arrayCopy(acc, this.accs[items]);
      arrayCopy(mag, this.mags[items]);
      arrayCopy(gyro, this.gyros[items]);
      this.mss[items] = ms;
      items++;
    }
    else {  // arrays are full
      arrayCopy(RPY, this.RPYs[start]);
      arrayCopy(acc, this.accs[start]);
      arrayCopy(mag, this.mags[start]);
      arrayCopy(gyro, this.gyros[start]);
      this.mss[start] = ms;
      start = (start + 1) % this.len;  // move start index instead
    }
  }
  
  void save_to_file(){
    PrintWriter writer = createWriter("SensorDataLog.txt");
    for (int i = 0; i < this.items; i++){
      int idx = (this.start + i) % this.len;
      float [] c_acc = this.accs[idx];
      float [] c_mag = this.mags[idx];
      float [] c_gyro = this.gyros[idx];
      float [] c_RPY = this.RPYs[idx];
      long c_ms = this.mss[idx];
      
      String ms_string = String.format("%d ", c_ms);
      String RPY_string = String.format("%.3f %.3f %.3f ", c_RPY[0], c_RPY[1], c_RPY[2]);
      String acc_string = String.format("%.3f %.3f %.3f ", c_acc[0], c_acc[1], c_acc[2]);
      String mag_string = String.format("%.3f %.3f %.3f ", c_mag[0], c_mag[1], c_mag[2]);
      String gyro_string = String.format("%.3f %.3f %.3f ", c_gyro[0], c_gyro[1], c_gyro[2]);
      writer.println(ms_string + RPY_string + acc_string + mag_string + gyro_string);
    }
    writer.close();  // Always close the writer
    println("Saved log to SensorDataLog.txt");
  }
  
}
