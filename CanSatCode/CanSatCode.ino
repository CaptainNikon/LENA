// LENA project
// THis is the code for the CanSat

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT.h>			   // Required header
#include <OneWire.h>		   // Required header
#include <DallasTemperature.h> // Required header
#include <Wire.h>			   // Wire library - used for I2C communication
#include <Adafruit_LIS2MDL.h>


// Defins the data storage size in bytes
#define STORAGE_SIZE 50

// Defins the pins
#define PIN_Ultrasonic_trig 3 // Ultrasonic triger pin
#define PIN_Ultrasonic_echo 4 // Ultrasonic echo pin
#define SPI_Radio 10		  // Radio CSN pin
#define CE_PIN 9
#define DS18B20_PIN 2 // Dataline is plugged into digital pin 2
int ADXL345 = 0x1D;	  // Adress for DRFduino

RF24 radio(CE_PIN, SPI_Radio);		 // CE, CSN
OneWire oneWire(DS18B20_PIN);		 // setup the oneWire to communicate with sensor
DallasTemperature sensors(&oneWire); // send the oneWire reference to DallasTemperature
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL();

// Some good definition
#define uint8 uint8_t
#define int8 int8_t
#define uint16 uint16_t
#define int16 int16_t
const byte address[6] = "Canst";

// wtf dose this macro?!?!
// #define runEvery(t) for (static typeof(t) _lasttime; (typeof(t))((typeof(t))millis() - _lasttime) > (t); _lasttime += (t))
//  runEvery(20){
//		Code....
//	}

// CanSat structure
struct CanSat_struct
{
	// Ground station name?
	// byte address[7] = {'G', 'r', 'o', 'u', 'd', '\0'};
	byte comands[10];
};
struct Measurement_struct
{
	// add your Measurement data here
	// For example:

	int distance = 0; // Stored as 0.1 cm
	float temp = 0;
  float accelerometer_X = 0, accelerometer_Y = 0, accelerometer_Z = 0;
  float calX = 0, calY = 0, calZ = 0;

};
struct Data_struct
{
	uint16 first_entry = 0;	 // First entries with data
	uint16 end_entry = 0;	 // First empty entriy
	byte data[STORAGE_SIZE]; // Data storage
};

// Declear our structs
Measurement_struct Measurement;
CanSat_struct CanSat;
Data_struct Data;

// Decleration of function
void My_Radio();
void Measurement_DHT();
void Measurement_Ultrasonic();
void Measurement_accelerometer();
void Measurement_hall_effect();


//  Decleration of initialization functions
void Init_Radio();
void Init_CanSat();
void Init_Ultrasonic();
void Init_dht();
void Init_accelerometer();
void Init_hall_effect();

uint8 Meassurment_size()
{
	uint8 size = sizeof(Measurement_struct);
	return size;
}
uint8 Data_entry_size(uint16 entry)
{
	return sizeof(Measurement_struct);
}
void Data_first_delete()
{
	// Get the size of the entry
	uint16 size = Data_entry_size(Data.first_entry);

	// Sett the memory to zero, byte by byte
	for (int i = 0; i < size; i++)
	{
		Data.data[(i + Data.first_entry) % STORAGE_SIZE] = 0;
	}

	// move first_entry with the size, to the new first entry
	Data.first_entry = (Data.first_entry + size) % STORAGE_SIZE;
}

void Move_meassurment_to_data()
{
	// Move all the meassurments from the Meassurment struct to last entris in data
	uint16 size = sizeof(Measurement);

	// pointer to measurment values
	byte *Measurment_ptr = (byte *)&Measurement;

	// Coppy evrything in measurment struct to data struct byte by byte
	for (uint16 i = 0; i < size; i++)
	{
		Data.data[(i + Data.end_entry) % STORAGE_SIZE] = Measurment_ptr[i];
	}

	// Move end pointer to next entry
	Data.end_entry = (Data.end_entry + size) % STORAGE_SIZE;
}
void setup()
{

	Serial.begin(9600);

	// Initialization function
	Init_CanSat(); // Dose nothing right now
	Init_Radio();
	Init_Ultrasonic();
	Init_DS18B20();
  Serial.print("Hello\n");
  Init_accelerometer(); //This function crashes
  //Init_hall_effect();
}

void loop()
{

	// Take the measurement
	// Measurement_DHT();
	Measurement_Ultrasonic();
	Measurement_DS18B20();
  //Measurement_accelerometer();
  //Measurement_hall_effect();

	Serial.print("distance\t");
	Serial.print(Measurement.distance);
	Serial.print("   temp\t");
	Serial.print(Measurement.temp);
	Serial.print("\n");

  Serial.print("Xa= ");
	Serial.print(Measurement.accelerometer_X);
	Serial.print("   Ya= ");
	Serial.print(Measurement.accelerometer_Y);
	Serial.print("   Za= ");
	Serial.println(Measurement.accelerometer_Z);


	// Move the Measurement to Data
	Move_meassurment_to_data();

	// Radio over the data from Data
	My_Radio();

	delay(500);
}

void Measurement_DS18B20()
{
	sensors.requestTemperatures(); // send request to device to get temperature
	Measurement.temp = sensors.getTempCByIndex(0);
}
void Measurement_Ultrasonic()
{
	// Clears the PIN_Ultrasonic_trig
	digitalWrite(PIN_Ultrasonic_trig, LOW);
	delayMicroseconds(2);
	// Sets the PIN_Ultrasonic_trig on HIGH state for 10 micro seconds
	digitalWrite(PIN_Ultrasonic_trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(PIN_Ultrasonic_trig, LOW);
	// Reads the PIN_Ultrasonic_echo, returns the sound wave travel time in microseconds
	long duration = pulseIn(PIN_Ultrasonic_echo, HIGH);
	// Calculating the distance
	long distance = duration * 0.034 * 10 / 2.;

	if (distance > 200.0)
	{
		// distance = 200;
	}
	Measurement.distance = distance;
}

void Init_Radio()
{
	radio.begin();

	// radio.setDataRate(RF24_250KBPS); // setting data rate to 250 kbit/s
	// radio.setCRCLength(RF24_CRC_16); // Set check sum length, check sum=CRC
	// radio.toggleAllPipes(true);		 // Toggle all pipes together, is this good idea?

	radio.openWritingPipe(address);
	// radio.setChannel(21); // set the channel to 21
	//  we have teh chanels 21-30 and 81-90

	// radio.setPALevel(RF24_PA_HIGH); // Change this to RF24_PA_HIGH when we want high power

	radio.stopListening();
}

void Init_Ultrasonic()
{
	pinMode(PIN_Ultrasonic_trig, OUTPUT); // Sets the PIN_Ultrasonic_trig as an Output
	pinMode(PIN_Ultrasonic_echo, INPUT);  // Sets the PIN_Ultrasonic_echo as an Input
}

void Init_DS18B20()
{
	sensors.begin(); // start the sensor (wiring)
}
void Init_CanSat()
{
}

void Init_accelerometer(){ //This function crashes
  /*Wire.beginTransmission(ADXL345); // Start communicating with the device 
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable 
  Wire.endTransmission();*/
}

void Init_hall_effect(){
    if (!mag.begin()) {
    Serial.println("LIS2MDL not found. Check wiring.");
    while (1);
  }
}

void Measurement_accelerometer()
{
	float X_out, Y_out, Z_out; // Outputs
	// === Read acceleromter data === //
	Wire.beginTransmission(ADXL345);
	Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(ADXL345, 6, true);		  // Read 6 registers total, each axis value is stored in 2 registers
	X_out = (Wire.read() | Wire.read() << 8); // X-axis value
	X_out = X_out / 256;					  // For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
	Y_out = (Wire.read() | Wire.read() << 8); // Y-axis value
	Y_out = Y_out / 256;
	Z_out = (Wire.read() | Wire.read() << 8); // Z-axis value
	Z_out = Z_out / 256;

	Measurement.accelerometer_X = X_out;
	Measurement.accelerometer_Y = Y_out;
	Measurement.accelerometer_Z = Z_out;

	Serial.print("Xa= ");
	Serial.print(X_out);
	Serial.print("   Ya= ");
	Serial.print(Y_out);
	Serial.print("   Za= ");
	Serial.println(Z_out);

}

void Measurement_hall_effect()
{
	sensors_event_t event;
	mag.getEvent(&event);
	float minX = -50.0, maxX = 50.0;
	float minY = -50.0, maxY = 50.0;
	float minZ = -50.0, maxZ = 50.0;

	// Raw readings in microteslas (µT)
	float rawX = event.magnetic.x;
	float rawY = event.magnetic.y;
	float rawZ = event.magnetic.z;

	// Simple hard iron offset calibration
	float offsetX = (maxX + minX) / 2;
	float offsetY = (maxY + minY) / 2;
	float offsetZ = (maxZ + minZ) / 2;

	Measurement.calX = rawX - offsetX;
	Measurement.calY = rawY - offsetY;
	Measurement.calZ = rawZ - offsetZ;

	// Output raw and calibrated data
	Serial.print("Raw: ");
	Serial.print(rawX, 2);
	Serial.print(", ");
	Serial.print(rawY, 2);
	Serial.print(", ");
	Serial.print(rawZ, 2);
	Serial.print(" µT  |  ");

	Serial.print("Calibrated: ");
	Serial.print(Measurement.calX, 2);
	Serial.print(", ");
	Serial.print(Measurement.calY, 2);
	Serial.print(", ");
	Serial.print(Measurement.calZ, 2);
	Serial.println(" µT");

	delay(2); // Sample rate: 500 Hz
}


void My_Radio()
{
	// Getting the size of datae
	uint16 size = Data_entry_size(Data.first_entry);

	// radio.write(&(Data.data[Data.first_entry]), size);
	radio.write(&(Measurement), sizeof(Measurement_struct));

	// We need to check if the message got recived
	// end if it did, delete the entry, but for now we assume it worked

	// Delete entry
	Data_first_delete();
}
