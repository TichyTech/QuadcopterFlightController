void drawAxes(float size){
  // Draw simple 3D (x-y-z) axes in at the origin
  strokeWeight(16);
  stroke(1,0,0);
  translate(size, 0, 0);
  line(0,0,0,-size,0,0);
  sphere(10);

  stroke(0,1,0);
  translate(-size, size, 0);
  line(0,0,0,0,-size,0);
  sphere(10);

  stroke(0,0,1);
  translate(0, -size, size);
  line(0,0,0,0,0,-size);
  sphere(10);
  translate(0,0,-size);

}
