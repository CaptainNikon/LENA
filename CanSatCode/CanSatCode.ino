// LENA project
// THis is the code for the CanSat

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <OneWire.h>		   // Required header
#include <DallasTemperature.h> // Required header
#include <Wire.h>			   // Wire library - used for I2C communication
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <printf.h>

#include <Servo.h>

// Defins the data storage size in bytes
#define STORAGE_SIZE 150

//RF24 radio(7, 8); // CE, CSN


// Defins the pins
#define PIN_Ultrasonic_trig 3 // Ultrasonic triger pin
#define PIN_Ultrasonic_echo 4 // Ultrasonic echo pin
#define SPI_Radio 8		  // Radio CSN pin
#define CE_PIN 7
#define DS18B20_PIN 2 // Dataline is plugged into digital pin 2
int ADXL345 = 0x1D;	  // Adress for DRFduino


RF24 radio(CE_PIN, SPI_Radio);		 // CE, CSN
OneWire oneWire(DS18B20_PIN);		 // setup the oneWire to communicate with sensor
DallasTemperature sensors(&oneWire); // send the oneWire reference to DallasTemperature
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL();
Servo myservo;  // create servo object to control a servo

// Some good definition
#define uint8 uint8_t
#define int8 int8_t
#define uint16 uint16_t
#define int16 int16_t
const byte address_g[7] = "Ground";
const byte address_c[6] = "Canst";

// For LIS2MDL
float sumX = 0, sumY = 0, sumZ = 0;
unsigned int sampleCount = 0;
unsigned long lastPrintTime = 0;

// wtf dose this macro?!?!
// #define runEvery(t) for (static typeof(t) _lasttime; (typeof(t))((typeof(t))millis() - _lasttime) > (t); _lasttime += (t))
//  runEvery(20){
//		Code....
//	}

// CanSat structure
struct CanSat_struct
{
	// Ground station name?
	// byte address_g[7] = {'G', 'r', 'o', 'u', 'd', '\0'};
	byte comands[10];
	int Servo_pos = 0;    // variable to store the servo position

};
// __attribute__((packed)) 
struct __attribute__((packed)) Measurement_struct
{
	uint16 time=0;
	uint8 distance = 0;
	uint16 temp = 0;
	int16 accelerometer_X = 0, accelerometer_Y = 0, accelerometer_Z = 0;
	int16 calX = 0, calY = 0, calZ=0;
};
struct Data_struct
{
	uint16 first_entry = 0;	 // First entries with data
	uint16 end_entry = 0;	 // First empty entriy
	byte data[STORAGE_SIZE+20]; // Data storage
};

// Declear our structs
Measurement_struct Measurement;
CanSat_struct CanSat;
Data_struct Data;

// Decleration of function
void Radio_send();
void Radio_read();
void Measurement_DS18B20();
void Measurement_Ultrasonic();
void Measurement_accelerometer();
void Measurement_hall_effect();
void Servo_move();

//  Decleration of initialization functions
void Init_Radio();
void Init_CanSat();
void Init_Ultrasonic();
void Init_accelerometer();
void Init_hall_effect();
void Init_servo();

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
	Measurement.time = millis()/10;

	// Move all the meassurments from the Meassurment struct to last entris in data
	uint16 size = sizeof(Measurement_struct);

	// pointer to measurment values
	byte *Measurment_ptr = (byte *)&Measurement;

	// Coppy evrything in measurment struct to data struct byte by byte
	if(Data.end_entry+size>STORAGE_SIZE){
		Data.end_entry=0;
	}
	for (uint16 i = 0; i < size; i++)
	{
		Data.data[(i + Data.end_entry) % STORAGE_SIZE] = Measurment_ptr[i];
	}

	// Move end pointer to next entry
	Data.end_entry = (Data.end_entry + size) % STORAGE_SIZE;
}
void setup()
{
	Serial.begin(115200);
	Wire.begin();

	// Initialization function
	Init_CanSat(); // Dose nothing right now
	Init_Radio();
	Init_Ultrasonic();
	
	Init_accelerometer(); // This function crashes
	Init_hall_effect();
	Init_servo();
}

void print_values()
{
	Serial.print(Measurement.time);
	Serial.print("     =========== new ===========\n");
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

	Serial.print("Xmag= ");
	Serial.print(Measurement.calX);
	Serial.print("   Ymag= ");
	Serial.print(Measurement.calY);
	Serial.print("   Zmag= ");
	Serial.println(Measurement.calZ);
}


int loop_counter=0;             //holds the count for every loop pass
long loop_timer_now=0;          //holds the current millis
long previous_millis=0;         //holds the previous millis
float loop_timer=0;              //holds difference (loop_timer_now - previous_millis) = total execution time
int loop_test_times = 20000;  //Run loop 20000 times then calculate time

void loop()
{

    loop_counter++;
  if (loop_counter == 20)
  { 
    previous_millis = loop_timer_now; 
    loop_timer_now = millis(); 
    loop_counter = 0;
    loop_timer = 1/((loop_timer_now - previous_millis) / 20000.0); 
    Serial.print("Messurment pull rate: ");
    Serial.println(loop_timer);
  }

	// Take the measurement
	Measurement_Ultrasonic();
	Measurement_accelerometer();
	Measurement_hall_effect();
	Measurement_DS18B20();
	//print_values();

	// Move the Measurement to Data
	Move_meassurment_to_data();

	// Radio over the data from Data
	Radio_send();
	Radio_read();

	delay(100);
	
}


void Init_Radio()
{
	
	// radio.begin(1000000); // Test this
	radio.begin();

	radio.setDataRate(RF24_250KBPS); // setting data rate to 250 kbit/s, RF24_250KBPS
	// radio.setCRCLength(RF24_CRC_16); // Set check sum length, check sum=CRC
	//  radio.toggleAllPipes(true);		 // Toggle all pipes together, is this good idea?
	radio.setChannel(21); // set the channel to 21
	radio.openWritingPipe(address_g);
	radio.setAutoAck(true);
	radio.setRetries(1, 15);
	radio.setCRCLength(RF24_CRC_16);



	//  we have teh chanels 21-30 and 81-90

	radio.setPALevel(RF24_PA_LOW); // Change this to RF24_PA_HIGH when we want high power

	radio.openReadingPipe(1, address_c);
	radio.startListening();

	printf_begin();
	radio.printPrettyDetails();
 
}

void Init_Ultrasonic()
{
	pinMode(PIN_Ultrasonic_trig, OUTPUT); // Sets the PIN_Ultrasonic_trig as an Output
	pinMode(PIN_Ultrasonic_echo, INPUT);  // Sets the PIN_Ultrasonic_echo as an Input
}

void Init_DS18B20()
{
	sensors.begin(); // start the sensor (wiring)
	sensors.setResolution(8);
}
void Init_CanSat()
{
	//
}

void Init_accelerometer()
{									 // This function crashes
	Wire.beginTransmission(ADXL345); // Start communicating with the device
	Wire.write(0x2D);				 // Access/ talk to POWER_CTL Register - 0x2D
	// Enable measurement
	Wire.write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable
	Wire.endTransmission();
}

void Init_hall_effect()
{

	Serial.println("Scanning I2C bus...");
	for (byte addr = 1; addr < 127; addr++)
	{
		Wire.beginTransmission(addr);
		if (Wire.endTransmission() == 0)
		{
			Serial.print("Found device at 0x");
			Serial.println(addr, HEX);
		}
	}

	if (!mag.begin())
	{
		Serial.println("LIS2MDL not found. Check wiring.");
		while (1)
			;
	}
}

void Init_servo()
{
  myservo.attach(6);  
}

void Measurement_DS18B20()
{
	
	//sensors.requestTemperatures();					// send request to device to get temperature
	Measurement.temp = sensors.getTempCByIndex(0) * 10; // switch to bit manipulation aka, << n

	// what dose this function do?!?!?! this is faster then getTempCByIndex by 30%.....
	//Measurement.temp = sensors.getTemp(0);

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
	uint16 duration = pulseIn(PIN_Ultrasonic_echo, HIGH);
	// Calculating the distance
	Measurement.distance = duration *0.17;// 0.17 = 0.034 * 10 / 2;

}

void Measurement_accelerometer()
{
	// === Read acceleromter data === //
	Wire.beginTransmission(ADXL345);
	Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
										/*
									  float X_out, Y_out, Z_out; // Outputs
										X_out = (Wire.read() | Wire.read() << 8); // X-axis value
										X_out = X_out / 256;					  // For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
										Y_out = (Wire.read() | Wire.read() << 8); // Y-axis value
										Y_out = Y_out / 256;
										Z_out = (Wire.read() | Wire.read() << 8); // Z-axis value
										Z_out = Z_out / 256;
									  */
	int16 X_out, Y_out, Z_out;			// Outputs
	// Reading the values, and 10 bit to int8
	X_out = (Wire.read() | Wire.read() << 8) >> 2; // X-axis value
	Y_out = (Wire.read() | Wire.read() << 8) >> 2; // Y-axis value
	Z_out = (Wire.read() | Wire.read() << 8) >> 2; // Z-axis value

	Measurement.accelerometer_X = X_out;
	Measurement.accelerometer_Y = Y_out;
	Measurement.accelerometer_Z = Z_out;
}

//This might be the way to get raw data from the accelerometer

void Measurement_hall_effect()
{
	mag.read();  // Triggers I2C read, stores values in mag.raw

	int16_t rawX = mag.raw.x;
	int16_t rawY = mag.raw.y;
	int16_t rawZ = mag.raw.z;

	Measurement.calX = rawX;
	Measurement.calY = rawY;
	Measurement.calZ = rawZ;
}

void Servo_move(){

  myservo.write(CanSat.Servo_pos);              // tell servo to go to position in variable 'pos'
	if(CanSat.Servo_pos>100){
		CanSat.Servo_pos=0;
	}else {
		CanSat.Servo_pos=200;
	}
}

void Radio_send()
{
	
	// Getting the size of datae
	uint16 size = Data_entry_size(Data.first_entry);

	radio.stopListening();
	//radio.stopListening();
	// radio.write(&(Data.data[Data.first_entry]), size);
	bool sent=radio.write(&(Measurement), sizeof(Measurement_struct));
	radio.startListening();
	if(sent){
		Serial.println("ack");
	}else{
		Serial.println("fuck");
	}

	Data_first_delete();
}
void Radio_read()
{
    if ( radio.available() ) {
        radio.read( &CanSat.comands, sizeof(CanSat.comands) );
        
		Serial.print("Comand: ");
		Serial.write(CanSat.comands[0]);
		Serial.println();
		//Serial.print("\n");
		Serial.println(__LINE__);
		Servo_move();
    }
}























