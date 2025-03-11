void drawText(String text, int x, int y){
  fill(0);
  textSize(20);
  textAlign(LEFT, BASELINE);
  text(text, x, y);  // Centered in the bar
}

void drawState(float[] RPY, int x, int y){
  fill(1);
  stroke(0);
  strokeWeight(2);
  rect(x, y, 120, 120, 10);  // Rounded rectangle

  // Display the text
  fill(0);
  textSize(20);
  textAlign(CENTER, CENTER);
  String[] names = {"Roll:", "Pitch:", "Yaw:"};    

  colorMode(HSB, 1);
  for (int i = 0; i < 3; i ++){
    float c = (RPY[i] > 0) ? 0.3 : 1;
    float val = constrain(abs(RPY[i] / 3), 0 , 0.99);
    fill(1, 1, 0);
    textAlign(LEFT, CENTER);
    text(names[i], x + 5, y + 25 + i*30);
    fill(c, 1, val);
    textAlign(RIGHT, CENTER);
    text(String.format("%.2f", RPY[i]), x + 110, y + 25 + i*30);
  }
  colorMode(RGB, 1);
}

void drawSensors(float[] acc, float[] mag, float[] gyro, int x, int y){
  fill(1);
  stroke(0);
  strokeWeight(2);
  rect(x, y, 180, 130, 10);  // Rounded rectangle

  // Display the text
  fill(0);
  textSize(20);
  textAlign(CENTER, CENTER);
  String[] names = {"Acc:", "Mag:", "Gyro:"};    

  colorMode(HSB, 1);
  fill(1, 1, 0);
  textAlign(LEFT, CENTER);
  text("Acc:", x + 5, y + 25);
  for (int i = 0; i < 3; i ++){
    textAlign(RIGHT, CENTER);
    text(String.format("%.2f", acc[i]), x + 40, y + 45 + i*30);
  }

  textAlign(LEFT, CENTER);
  text("Mag:", x + 65, y + 25);
  for (int i = 0; i < 3; i ++){
    textAlign(RIGHT, CENTER);
    text(String.format("%.2f", mag[i]), x + 100, y + 45 + i*30);
  }
  
  textAlign(LEFT, CENTER);
  text("Gyro:", x + 125, y + 25);
  for (int i = 0; i < 3; i ++){
    textAlign(RIGHT, CENTER);
    text(String.format("%.2f", gyro[i]), x + 160, y + 45 + i*30);
  }
  
  colorMode(RGB, 1);
}

void drawMP(float[] mp, int x, int y){
  int ww = 240; 
  int hh = 40;
  fill(1);
  stroke(0);
  strokeWeight(2);
  rect(x, y, ww, hh, 10);  // Rounded rectangle

  // Display the text
  fill(0);
  textSize(20);
  textAlign(CENTER, CENTER);
  String text = String.format("Motors: %.2f %.2f %.2f %.2f", mp[0], mp[1], mp[2], mp[3]);
  
  text(text, x + ww/2, y + hh/2);  // Centered in the bar
}

void drawTelemetry(float altitude, float battery, int x, int y){
  int ww = 140; 
  int hh = 80;
  fill(1);
  stroke(0);
  strokeWeight(2);
  rect(x, y, ww, hh, 10);  // Rounded rectangle

  // Display the text
  fill(0);
  textSize(20);
  textAlign(CENTER, CENTER);
  String text = String.format("Battery: %.2f \nAltitude: %.2f", battery, altitude);
  
  text(text, x + ww/2, y + hh/2);  // Centered in the bar
}

void drawForces(float[] forces, int x, int y){
  int ww = 150; 
  int hh = 80;
  fill(1);
  stroke(0);
  strokeWeight(2);
  rect(x, y, ww, hh, 10);  // Rounded rectangle

  // Display the text
  fill(0);
  textSize(20);
  textAlign(CENTER, CENTER);
  
  text("Forces: ", x + ww/2 - 40, y + hh/2);
  
  colorMode(HSB, 1);
  for (int i = 0; i < 3; i ++){
    float c = (forces[i] > 0) ? 0.3 : 1;
    float val = constrain(abs(forces[i])/40, 0 , 0.99);
    stroke(c, 1, val);
    fill(c, 1, val);
    text(String.format("%.2f", forces[i]), x + ww/2 + 30, y + hh/2 + i*20 - 20);
  }
  colorMode(RGB, 1);
}

void drawMessageCount(long dt, int x, int y){
  String msg_counter = String.format("Received messages: %d", num_messages);
  float update = new_messages * 1000/dt;
  mean_msg = 0.05 * update + 0.95*mean_msg;
  String msg_freq = String.format("Per second: %.2f ", mean_msg);
  new_messages = 0;
  
  fill(1);
  stroke(0);
  strokeWeight(2);
  rect(x, y-25, 250, 50, 10);  // Rounded rectangle
  
  drawText(msg_counter, x+5, y);
  drawText(msg_freq, x+5, y + 20);
}
