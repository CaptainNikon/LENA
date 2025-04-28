// C code for nRF24L01 Receiver
// Shahab Fatemi (Jan 2023)
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT.h> // Required header

#define PIN_DHT 3 // DHT data pin
RF24 radio(8, 7); // CE, CSN

#define DHTType DHT11 // Specify the type of DHT

// Some good definition
#define uint8 uint8_t
#define int8 int8_t
#define uint16 uint16_t
#define int16 int16_t

// Address through which Rx/Tx modules communicate.
byte address[7] = {'G', 'r', 'o', 'u', 'd', '\0'};

DHT dht = DHT(PIN_DHT, DHTType); // instance of DHT class

struct Measurement_struct
{
	// add your Measurement data here
	// For example:
	uint8 distance = 0; // Stored as 0.1 cm
	uint8 temp = 0;
};

void Init_dht();

Measurement_struct Measurement;
void setup()
{
	while (!Serial)
		;
	Serial.begin(9600);
	radio.begin();

	radio.setDataRate(RF24_250KBPS); // setting data rate to 250 kbit/s
	radio.setCRCLength(RF24_CRC_16); // Set check sum length, check sum=CRC
	// radio.toggleAllPipes(true);		 // Toggle all pipes together, is this good idea?

	radio.openWritingPipe(address);
	radio.setChannel(21); // set the channel to 21
	// we have the chanels 21-30 and 81-90

	radio.setPALevel(RF24_PA_MIN); // Change this to RF24_PA_HIGH when we want high power
}

void loop()
{
	// Read the data if available in buffer
	if (radio.available())
	{
		Measurement_struct Measurement;
		radio.read(&Measurement, sizeof(Measurement));

    Serial.print(Measurement.distance);
    Serial.print("mm\t");
    Serial.print(Measurement.temp);
    Serial.print("C\t");
    Serial.print("\n");
	}
	else
	{
		//Measurement_DHT();
    Serial.print("No radio\n");
		delay(500);
	}
}

void Init_dht()
{
	dht.begin(); // first_entry the sensor (wiring)
}

void Measurement_DHT()
{

	// CanSat.temperature  = dht.readTemperature();  // Reading temperature
	float humidity = 0, temperature = 0;

	humidity = dht.readHumidity(); // Reading humidity
	temperature = dht.readTemperature();

	if (isnan(temperature) || isnan(humidity))
	{
		Serial.println("Failed to read values from the DHT sensor!");
		delay(1000);
		return; // Do not continue the rest of the loop and rewind!
	}

	// Print out values
	Serial.print("Temperature: ");
	Serial.print(temperature);
	Serial.print("°C   Humidity: ");
	Serial.print(humidity);
	Serial.println("%");
}
