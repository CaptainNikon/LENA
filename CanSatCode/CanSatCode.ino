// LENA project
// THis is the code for the CanSat

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT.h>			   // Required header
#include <OneWire.h>		   // Required header
#include <DallasTemperature.h> // Required header

// Defins the data storage size in bytes
#define STORAGE_SIZE 50

// Defins the pins
#define PIN_Ultrasonic_trig 3 // Ultrasonic triger pin
#define PIN_Ultrasonic_echo 4 // Ultrasonic echo pin
#define SPI_Radio 10		  // Radio CSN pin
#define CE_PIN 9
#define DS18B20_PIN 2 // Dataline is plugged into digital pin 2

RF24 radio(CE_PIN, SPI_Radio);		 // CE, CSN
OneWire oneWire(DS18B20_PIN);		 // setup the oneWire to communicate with sensor
DallasTemperature sensors(&oneWire); // send the oneWire reference to DallasTemperature

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

//  Decleration of initialization functions
void Init_Radio();
void Init_CanSat();
void Init_Ultrasonic();
void Init_dht();

uint8 measurement_size()
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

void Move_measurement_to_data()
{
	// Move all the measurements from the measurement struct to last entris in data
	uint16 size = sizeof(Measurement);

	// pointer to measurment values
	byte *Measurment_ptr = (byte *)&Measurement;

	// Coppy evrything in measurment struct to data struct byte by byte
	// % STORAGE_SIZE is to loop around the memory so we do not wrtite outsize of our array
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
}

void loop()
{

	// Take the measurement
	// Measurement_DHT();
	Measurement_Ultrasonic();
	Measurement_DS18B20();

	Serial.print("distance\t");
	Serial.print(Measurement.distance);
	Serial.print("   temp\t");
	Serial.print(Measurement.temp);
	Serial.print("\n");

	// Move the Measurement to Data
	Move_measurement_to_data();

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

void My_Radio()
{
	// Getting the size of datae
	uint16 size = Data_entry_size(Data.first_entry);

	radio.write(&(Data.data[Data.first_entry]), size);
	// radio.write(&(Measurement), sizeof(Measurement_struct));

	// We need to check if the message got recived
	// end if it did, delete the entry, but for now we assume it worked

	// Delete entry
	Data_first_delete();
}
