#include "comms.h"

Communication::Communication(){
    radio = RF24(9, 10);
}

void Communication::setup_nrf(){

    SPI.setClockDivider(SPI_CLOCK_DIV2);

    if (!radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {} 
    }
  
    radio.setPALevel(RF24_PA_MAX, 1);
    radio.setChannel(3);
    radio.setAddressWidth(3);
    radio.setDataRate(RF24_2MBPS);
    radio.enableDynamicAck();
    radio.setAutoAck(1);
    radio.setRetries(0, 4);

    radio.openWritingPipe(CMDADD); 
    radio.openReadingPipe(1, STATADD); 
    radio.startListening();   

    radio.printDetails();

}

bool Communication::send_ctrl(ctrl_msg_t ctrl_msg)
{
    msg_t msg;
    msg.type = 0;
    msg.data.ctrl_data = ctrl_msg;

    radio.stopListening(); 
    // radio.flush_tx();
    bool report = radio.write(&msg, sizeof(msg), 0); 
    // delayMicroseconds(180);
    radio.startListening();
    radio.flush_rx();
    return report;
}

bool Communication::send_config(msg_t msg)
{
    radio.stopListening(); 
    bool report = radio.write(&msg, sizeof(msg_t), 0);
    // delayMicroseconds(180);    
    radio.startListening(); 
    return report;
}

bool Communication::receive_msg(telemetry_msg_t& msg)
{
    uint8_t pipe;
    if(radio.available(&pipe)) radio.read(&msg, sizeof(telemetry_msg_t));    
    else return 0;

    return 1;
}
