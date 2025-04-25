// LENA project
// THis is the code for the CanSat

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT.h> // Required header

// Defins the data storage size in bytes
#define STORAGE_SIZE 200

// Defins the pins
#define PIN_DHT 3			   // DHT data line to pin 3
#define PIN_Ultrasonic_trig 9  // Ultrasonic triger pin
#define PIN_Ultrasonic_echo 10 // Ultrasonic echo pin
#define SPI_Radio 7			   // Radio data line to pin 8

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
	byte address[7] = {'G', 'r', 'o', 'u', 'd', '\0'};
	byte comands[10];
};
struct Measurement_struct
{
	// add your Measurement data here
	// For example:
	float temperature = 0;
	float humidity = 0;
	uint8 distance = 0; // Stored as 0.1 cm
};
struct Data_struct
{
	uint16 first_entry = 0;	 // First entries with data
	uint16 end = 0;			 // First empty entriy
	byte data[STORAGE_SIZE]; // Data storage
};

// Declear our structs
Measurement_struct Measurement;
CanSat_struct CanSat;
Data_struct Data;

//  Decleration of initialization functions
void Init_Radio();
void Init_CanSat();
void Init_Ultrasonic();
void Init_dht();

// Decleration of function
void Radio();
void Measurement_DHT();

uint8 Data_entry_size(uint16 entry)
{
	return sizeof(Measurement_struct);
}
void Data_first_delete()
{
	// Get the size of the entry
	uint16 size = Data_entry_size(Data.first_entry);

	// Sett the memory to zero
	memset(&Data.data[Data.first_entry], 0, size);

	// move first_entry with the size, to the new first entry
	Data.first_entry = (Data.first_entry + size) % STORAGE_SIZE;
}

void Move_meassurment_to_data()
{

	uint16 size = sizeof(Measurement);
	// Coppy evrything in meassurment struct to data struct
	memcpy(&Measurement, &Data.end, size);
	// Move end pointer to
	Data.end = (Data.end + size) % STORAGE_SIZE;
}
void setup()
{

	Serial.begin(9600);

	// Initialization function
	Init_CanSat();
	Init_Radio();
	Init_Ultrasonic();
	Init_dht();
}

void loop()
{
	// Take the measurement
	Measurement_DHT();
	Measurement_Ultrasonic();

	// Move the Measurement to Data
	Move_meassurment_to_data();

	// Radio over the data in Data
	Radio();
	delay(500);
}

void Init_Radio()
{
	radio.begin();

	radio.openWritingPipe(CanSat.address);

	radio.setPALevel(RF24_PA_MIN); // Change this to RF24_PA_HIGH when we want high power

	radio.stopListening();
}
void Init_Ultrasonic()
{
	pinMode(PIN_Ultrasonic_trig, OUTPUT); // Sets the PIN_Ultrasonic_trig as an Output
	pinMode(PIN_Ultrasonic_echo, INPUT);  // Sets the PIN_Ultrasonic_echo as an Input
}
void Init_dht()
{
	dht.begin(); // first_entry the sensor (wiring)
}
void Init_CanSat()
{
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
	long
		distance = duration * 0.034 * 10 / 2.;
	if (distance > 200.0)
	{
		distance = 0;
	}
	Measurement.distance = distance;
}

void Radio()
{
	// Getting the size of data
	uint16 size = Data_entry_size(Data.first_entry);

	radio.write(&(Data.first_entry), size);

	// We need to check if the message got recived
	// end if it did, delete the entry, but for now we assume it worked

	// Delete entry
	Data_first_delete();
}






