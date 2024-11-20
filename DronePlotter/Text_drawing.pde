void drawText(String text, int x, int y){
  fill(255);
  stroke(0);
  strokeWeight(2);
  rect(x, y, 600, 50, 10);  // Rounded rectangle

  // Display the text
  fill(0);
  textSize(20);
  textAlign(CENTER, CENTER);
  text(text, x + 300, y + 25);  // Centered in the bar
}

void drawState(float[] RPY, int x, int y){
  fill(255);
  stroke(0);
  strokeWeight(2);
  rect(x, y, 120, 120, 10);  // Rounded rectangle

  // Display the text
  fill(0);
  textSize(20);
  textAlign(CENTER, CENTER);
  String text = String.format("Roll: %.2f \nPitch: %.2f\nYaw: %.2f", RPY[0], RPY[1], RPY[2]);
  
  text(text, x + 60, y + 60);  // Centered in the bar
}

void drawMP(float[] mp, int x, int y){
  int ww = 240; 
  int hh = 40;
  fill(255);
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
  fill(255);
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
