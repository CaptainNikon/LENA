// LENA project
// Skeleton code

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


// Some definition 
#define uint8 uint8_t
#define int8 int8_t

RF24 radio(8, 7);// CE, CSN

struct CanSat_struct {   // Structure declaration
  // I2C adresses,
  uint8 Sensor_I2C_addresses[1];
  // SPI pins, index [0] is radio
  uint8 Sensor_SPI_pins[1];
  byte address[6]="CANST";

  
}; 
struct Measurement_struct {   
  // add your Measurement data here
  // For example:
  uint8 temprature = 0;
}; 

// Our structs
Measurement_struct Measurement;
CanSat_struct CanSat;

// Your initialization function for your sensor
void Init_sensor_Radio(){
  radio.begin();

  radio.openWritingPipe(CanSat.address);
  radio.setPALevel(RF24_PA_MIN);
  
  radio.stopListening();
}

// Take your Measurement and save it in Measurement struct
void Send_Radio(){

  radio.write(Measurement, sizeof(Measurement_struct));
}



void setup(){

  Serial.begin(9600);
  
  // Call your sensor initialization function?
  Init_sensor_Radio();
}

void loop()
{
  Send_Radio();
  
  delay(100);
}