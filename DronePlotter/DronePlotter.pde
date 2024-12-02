import processing.opengl.*;
import processing.serial.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

long last_draw = 0;
long num_messages = 0;
long new_messages = 0;
float mean_msg = 0;

Serial sp;
byte[] buff = new byte[256];
float[] RPY = new float[3];
float[] motor_percentages = new float[4];
float[] forces = new float[3];
long ms;
float altitude = 0;
float battery = 0;

// Command line variables
String inputText = "";        // Store the current input text
String lastInputText = "";    // Store the last input text (saved text)
int cursorPos = 0;          // Position of the cursor in the input text

// Logging structure 
DataPoints database;
 
void setup() {
  size(1080, 720, P3D);
  surface.setTitle("Drone Plotter");
  surface.setLocation(100, 100);
  colorMode(RGB, 1); 
  camera(width/2.0, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);
  
  printArray(Serial.list());
  String portName = Serial.list()[3];
  sp = new Serial(this, portName, 500000);
  println("Connecting to " + portName);
  
  database = new DataPoints(2048);
  wait_for_setup();
}
   
int numBytes = 0;
final boolean human_readable = false;  // are we receiving packed binary data, or human readable string

void draw() {
  
  if (human_readable) handle_serial_readable();
  else handle_serial_gibberish();
  
  long current_millis = millis();
  long dt = current_millis - last_draw;
  if (dt > 15){
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
    drawForces(forces, 200, 60);
    drawTelemetry(altitude, battery, 460, 20);
    drawMessageCount(dt, 800, 40);
    drawCommandLine();
    
    last_draw = current_millis;
    }
}
