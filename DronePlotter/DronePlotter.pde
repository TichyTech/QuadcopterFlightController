import processing.opengl.*;
import processing.serial.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

long last_draw = 0;
long num_messages = 0;
long new_messages = 0;
float mean_msg = 0;

Serial sp;
byte[] buff = new byte[512];

// telemetry variables
float[] RPY = new float[3];
float[] motor_percentages = new float[4];
float[] forces = new float[3];
float[] ref_angles = new float[2];
long ms;

// battery telemetry
float altitude = 0;
float battery = 0;

// sensor telemetry
float[] acc = new float[3];
float[] mag = new float[3];
float[] gyro = new float[3];


// Command line variables
String inputText = "";        // Store the current input text
String lastInputText = "";    // Store the last input text (saved text)
int cursorPos = 0;          // Position of the cursor in the input text

// Logging structure 
DataPoints database;
SensorDataPoints sensor_database;

int serialPort = 2;
 
void setup() {
  size(1440, 1080, P3D);
  surface.setTitle("Drone Plotter");
  surface.setLocation(100, 100);
  colorMode(RGB, 1); 
  camera(width/2.0, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);
  
  printArray(Serial.list());
  if (serialPort >= Serial.list().length){
    println("serial index out of range, exiting");
    exit();
    return;
  }
  String portName = Serial.list()[serialPort];
  sp = new Serial(this, portName, 500000);
  println("Connecting to " + portName);
  
  database = new DataPoints(2048);
  sensor_database = new SensorDataPoints(1024);
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
    drawRef(ref_angles, 50, 20);
    drawVector3(RPY, 50, 100, "RPY:", 20.0);
    drawVector3(forces, 50, 240, "PID: ", 40.0);
    drawSensors(acc, mag, gyro, width - 280, 160);
    drawMotorPercentage(motor_percentages, 240, 20);
    drawBattAlt(altitude, battery, width - 160, 0);
    drawMessageCount(dt, width - 440, 0);
    drawCommandLine(20, height - height/3);
    
    drawXYZ(sensor_database.mss, sensor_database.gyros, width - width/3, height - height/3, width/3, height/3, 50, sensor_database.start, sensor_database.items, "Gyro");
    drawXYZ(sensor_database.mss, sensor_database.accs, 0, height - height/3, width/3, height/3, 2, sensor_database.start, sensor_database.items, "Acc");
    drawXYZ(sensor_database.mss, sensor_database.mags, width/2 - width/6, height - height/3, width/3, height/3, 0.5, sensor_database.start, sensor_database.items, "Mag");


    last_draw = current_millis;
    }
}
