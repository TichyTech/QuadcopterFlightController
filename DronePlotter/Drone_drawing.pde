

void drawRPYm(float[] RPY, float[] mp){
  // Draw a drone representation from RPY angles together with motor percentages
  pushMatrix();
  rotateZ(PI*RPY[2]/180);
  rotateY(PI*RPY[1]/180);
  rotateX(PI*RPY[0]/180);
  
  float scl = 4;

  noFill();
  fill(1,0,0);
  stroke(0.8);
  box(scl*100, scl*40, scl*20);
  
  pushMatrix();
  translate(scl*50,0,0);

  stroke(1,1,0);
  box(scl*10);
  popMatrix();
  
  for (int idx = 0; idx < 4; idx++){
    float x = 1 - 2*(idx / 2);  
    float y = 1 - 2*(idx % 2);
    if (idx >= 2) {
      y = -y;
    }
    
    pushMatrix();
    translate(scl*x*50, scl*y*50, 0);
    rotateZ(PI/4 + PI*idx/2);
    stroke(0.2);
    box(scl*70, scl*10, scl*10);
    popMatrix();
    
    pushMatrix();
    rotateZ(PI/4 - PI*idx/2);
    translate(scl*120, 0, 10);
    colorMode(HSB, 1);
    stroke(1, mp[idx], 1);
    box(2,2,scl + scl*100*mp[idx]);
    colorMode(RGB, 1);
    popMatrix();
  }
  
  popMatrix();
}
