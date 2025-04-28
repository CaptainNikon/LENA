// C code for nRF24L01 Receiver
// Shahab Fatemi (Jan 2023)
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT.h> // Required header

#define PIN_DHT 2 // DHT data pin
RF24 radio(9, 8); // CE, CSN

#define DHTType DHT11 // Specify the type of DHT

// Some good definition
#define uint8 uint8_t
#define int8 int8_t
#define uint16 uint16_t
#define int16 int16_t

// Address through which Rx/Tx modules communicate.
byte address[6] = "Canst";

DHT dht = DHT(PIN_DHT, DHTType); // instance of DHT class

struct Measurement_struct
{
	// add your Measurement data here
	// For example:
	int distance = 0;
	float temp = 0;
};

struct Ground_struct
{
	// add your Measurement data here
	// For example:
	float humidity = 0;
	float temperature = 0;
};

void Init_dht();

Measurement_struct Measurement;
Ground_struct Ground;
void setup()
{
	Init_dht();
	while (!Serial)
		;
	Serial.begin(9600);
	radio.begin();

	// radio.setDataRate(RF24_250KBPS); // setting data rate to 250 kbit/s
	// radio.setCRCLength(RF24_CRC_16); // Set check sum length, check sum=CRC
	//  radio.toggleAllPipes(true);		 // Toggle all pipes together, is this good idea?

	radio.openReadingPipe(0, address);
	// radio.setChannel(21); // set the channel to 21
	//  we have the chanels 21-30 and 81-90
	radio.startListening();
	// radio.setPALevel(RF24_PA_MIN); // Change this to RF24_PA_HIGH when we want high power
}

void loop()
{
	// Read the data if available in buffer
	if (radio.available())
	{
		Measurement_struct Measurement;
		radio.read(&Measurement, sizeof(Measurement));

		Serial.print("Recieved data: \n");

		Serial.print(Measurement.distance);
		Serial.print("mm\t");
		Serial.print(Measurement.temp);
		Serial.print("C\t");
		Serial.print("\n");
	}
	else
	{
		// Measurement_DHT();
		// Serial.print("No radio\n");
		// delay(500);
	}
	Measurement_DHT();
	delay(500);
}

void Init_dht()
{
	dht.begin(); // first_entry the sensor (wiring)
}

void Measurement_DHT()
{

	// Ground.temperature  = dht.readTemperature();  // Reading temperature

	Ground.humidity = dht.readHumidity(); // Reading humidity

	Ground.temperature = dht.readTemperature();
	if (isnan(Ground.temperature) || isnan(Ground.humidity))
	{
		Serial.println("Failed to read values from the DHT sensor!");
		delay(1000);
		return; // Do not continue the rest of the loop and rewind!
	}

	// Print out values
	Serial.print("Temperature: ");
	Serial.print(Ground.temperature);
	Serial.print("°C   Humidity: ");
	Serial.print(Ground.humidity);
	Serial.println("%");
}