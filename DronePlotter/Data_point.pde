class DataPoints{
  float [][] RPYs;
  float [][] mps;
  long [] mss;
  int start = 0;
  int items = 0;
  int len;
  
  DataPoints(int length){
    this.len = length;
    this.RPYs = new float [length][3];
    this.mps = new float [length][4];
    this.mss = new long [length];
  }
  
  void add_datapoint(float [] RPY, float [] mp, long ms){
    if (items < len){ // we can handle more data
      arrayCopy(RPY, this.RPYs[items]);
      arrayCopy(mp, this.mps[items]);
      this.mss[items] = ms;
      items++;
    }
    else {  // arrays are full
      arrayCopy(RPY, this.RPYs[start]);
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
      long c_ms = this.mss[idx];
      String ms_string = String.format("%d ", c_ms);
      String RPY_string = String.format("%.2f %.2f %.2f ", c_RPY[0], c_RPY[1], c_RPY[2]);
      String mp_string = String.format("%.2f %.2f %.2f %.2f ", c_mp[0],c_mp[1],c_mp[2], c_mp[3]);
      writer.println(ms_string + RPY_string + mp_string);
    }
    writer.close();  // Always close the writer
    println("Saved log to file");
  }
  
}
