#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT.h>

// Include the libraries for the OLED display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define PIN_DHT 2 // DHT data pin
RF24 radio(9, 8); // CE, CSN

#define DHTType DHT11 // Specify the type of DHT

// OLED config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

		updateDisplayIfNeeded();
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

void Init_display() {
	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
		while (true); // hang on failure
	}
	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE);
	display.setCursor(0, 0);
	display.println("Waiting...");
	display.display();
}

void updateDisplayIfNeeded() {
	static unsigned long lastUpdate = 0;
	if (millis() - lastUpdate >= 1000) {
		lastUpdate = millis();
		packetsPerSecond = packetCount;
		packetCount = 0;

		display.clearDisplay();
		display.setCursor(0, 0);
		display.print("Packets/s: ");
		display.println(packetsPerSecond);
		display.setCursor(0, 16);
		display.print("Temp: ");
		display.print(Ground.temperature);
		display.print(" C");
		display.display();
	}
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