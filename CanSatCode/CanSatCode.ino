// LENA project
// THis is the code for the CanSat

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT.h> // Required header

// Defins the pins
#define DATA_BYTES 200 // Size of data storage in bytes
// Defins the pins
#define PIN_DHT 3	  // Data line to pin 3
#define SPI_Radio 7	  // Data line to pin 8
#define DHTType DHT11 // Specify the type of DHT

DHT dht = DHT(PIN_DHT, DHTType); // instance of DHT class
RF24 radio(8, SPI_Radio);		 // CE, CSN

// Some good definition
#define uint8 uint8_t
#define int8 int8_t
#define uint16 uint16_t
#define int16 int16_t

// CanSat structure
struct CanSat_struct
{
	// Ground station name?
	const byte address[7] = {'G', 'r', 'o', 'u', 'd', '\0'};
};
struct Measurement_struct
{
	// add your Measurement data here
	// For example:
	float temperature = 0;
	float humidity = 0;
};
struct Data_struct
{
	uint16 start = 0;	   // First entries with data
	uint16 end = 0;		   // First empty entriy
	byte data[DATA_BYTES]; // Data storage
};
// Declear our structs
Measurement_struct Measurement;
CanSat_struct CanSat;
Data_struct Data;

//  initialization functions
void Init_sensor_Radio();
void Init_CanSat() {}

// Decleration of function
void Radio();
void Measurement_DHT();

uint8 Data_entriy_size(uint16 *entry)
{
	return sizeof(Measurement_struct);
}

void setup()
{

	Serial.begin(9600);

	// Initialization function?
	Init_sensor_Radio();
	dht.begin(); // Start the sensor (wiring)
}

void loop()
{
	Measurement_DHT();
	Radio();
	delay(500);
}

void Init_sensor_Radio()
{
	radio.begin();

	radio.openWritingPipe(CanSat.address);

	radio.setPALevel(RF24_PA_MIN); // Change this to RF24_PA_HIGH when we want high power

	radio.stopListening();
}
void Radio()
{

	radio.write(&(Data.data[Data.start]), Data_entriy_size(&Data.start));
}

void Measurement_DHT()
{

	// CanSat.temperature  = dht.readTemperature();  // Reading temperature
	float humidity, temperature = 0;

	humidity = dht.readHumidity(); // Reading humidity
	temperature = dht.readTemperature();
	Measurement.humidity = humidity;
	Measurement.temperature = humidity;
	if (isnan(temperature) || isnan(humidity))
	{
		Serial.println("Failed to read values from the DHT sensor!");
		delay(1000);
		return; // Do not continue the rest of the loop and rewind!
	}

	// Print out values
	// Serial.print("Temperature: ");
	// Serial.print(temperature);
	Serial.print("°C   Humidity: ");
	Serial.print(humidity);
	Serial.println("%");
}
