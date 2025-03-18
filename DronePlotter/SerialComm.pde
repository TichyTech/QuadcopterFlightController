void wait_for_setup(){
  while (sp.available() == 0){
    println("Waiting for Setup");
    delay(300);
  }  // wait for serial
  while (sp.available() > 0){  // wait for finishing setup of remote controller
    int numBytes = sp.readBytesUntil('\n', buff); // try reading bytes from serial
    String mystr = (new String(buff, 0, numBytes)).trim();
    println(mystr);
    delay(200);
    if (mystr.equals("Setup done")) break;
  }
  println("Setup done!");
}

void handle_serial_gibberish(){
  byte [] buffer = new byte [32];
  if (sp.available() >= 32){
    numBytes = sp.readBytes(buffer); // try reading bytes from serial
  }
  while (numBytes > 0){
    ByteBuffer buffer_wrapped = ByteBuffer.wrap(buffer);
    buffer_wrapped.order(ByteOrder.LITTLE_ENDIAN); // Match endianness of C++ side
    int msg_type = buffer_wrapped.getInt();
    if(msg_type == 0){    
      loadValsStateGibberish(buffer_wrapped); 
      database.add_datapoint(RPY, motor_percentages, forces, ms, ref_angles);
      num_messages++;
      new_messages++;
    }
    else if(msg_type == 1){
      loadValsTelemetryGibberish(buffer_wrapped);
      num_messages++;
      new_messages++;
    }
    else if(msg_type == 2){
      loadValsSensorGibberish(buffer_wrapped);
      sensor_database.add_datapoint(RPY, acc, mag, gyro, ms);
      num_messages++;
      new_messages++;
    }
    if (sp.available() >= 32){
      numBytes = sp.readBytes(buffer); // try reading bytes from serial
    }
    else numBytes = 0;
  }
}

void handle_serial_readable(){
if (sp.available() > 0){
    numBytes = sp.readBytesUntil('\n', buff); // try reading bytes from serial
  }
  while (numBytes > 0){
    String mystr = (new String(buff, 0, numBytes)).trim();
    String[] split_string = split(mystr, ' ');
    String msg_type = split_string[0];    
    if(msg_type.equals("State:")){    
      loadValsState(split_string); 
      database.add_datapoint(RPY, motor_percentages, forces, ms, ref_angles);
      num_messages++;
      new_messages++;
    }
    else if(msg_type.equals("Telemetry:")){
      loadValsTelemetry(split_string);
      num_messages++;
      new_messages++;
    }

    if (sp.available() > 0){
      numBytes = sp.readBytesUntil('\n', buff); 
    }
    else numBytes = 0;
  }
}
