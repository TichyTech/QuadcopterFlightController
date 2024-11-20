import processing.opengl.*;
import processing.serial.*;
 
Serial sp;
byte[] buff = new byte[256];
float[] RPY = new float[3];
float[] motor_percentages = new float[4];
float altitude = 0;
float battery = 0;

// Command line variables
String inputText = "";        // Store the current input text
String lastInputText = "";    // Store the last input text (saved text)
int cursorPos = 0;          // Position of the cursor in the input text

 
void setup() {
  size(1080, 720, P3D);
  surface.setTitle("Drone Plotter");
  surface.setLocation(100, 100);
  colorMode(RGB, 1); 
  camera(width/2.0, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);
  
  printArray(Serial.list());
  String portName = Serial.list()[3];
  sp = new Serial(this, portName, 115200);
  println("Connecting to " + portName);
}
   
void draw() {
  if (sp.available() > 0){
    int numBytes = sp.readBytesUntil('\n', buff); 
    String mystr = (new String(buff, 0, numBytes)).trim();
    String[] split_string = split(mystr, ' ');
    String msg_type = split_string[0];
    
    if(msg_type.equals("State:")){    
      loadValsState(split_string); 
    }
    
    if(msg_type.equals("Telemetry:")){
      loadValsTelemetry(split_string);
    }
    
    background(0.5,0.5,0.5); 
    directionalLight(1, 1, 1, 0, 0, -1);
    pushMatrix();
    translate(width/2, height/2);
    scale(1, -1, 1); 
    rotateX(-PI/2);
    rotateX(0.1);
    rotateZ(0.1);
    drawAxes(300);
    drawRPYm(RPY, motor_percentages);
    popMatrix();
    //drawText(mystr, 200, 50);
    drawState(RPY, 50, 20);
    drawMP(motor_percentages, 200, 20);
    drawTelemetry(altitude, battery, 460, 20);
  }
  fill(240);  // Light gray background for the input box
  rect(50, height-90, 500, 40, 10);  // Draw input box
  fill(0);  // Black color for text
  textAlign(LEFT, BASELINE);
  text(inputText, 60, height-60);  // Display text inside the box
  float cursorX = textWidth(inputText.substring(0, cursorPos)) + 60; // Position based on the cursor position
  line(cursorX, height-60, cursorX, height-80);  // Draw a cursor
}
