// C code for nRF24L01 Receiver
// Shahab Fatemi (Jan 2023)
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT.h> // Required header

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define PIN_DHT 2 // DHT data pin
RF24 radio(9, 8); // CE, CSN

#define DHTType DHT11 // Specify the type of DHT

// Some good definition
#define uint8 uint8_t
#define int8 int8_t
#define uint16 uint16_t
#define int16 int16_t


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Address through which Rx/Tx modules communicate.
byte address[6] = "Canst";

DHT dht = DHT(PIN_DHT, DHTType); // instance of DHT class

struct Measurement_struct
{
	// add your Measurement data here
	// For example:
	int distance = 0; // Stored as 0.1 cm
	float temp = 0;
	float accelerometer_X, accelerometer_Y, accelerometer_Z;
	float calX, calY, calZ;
};
Measurement_struct Measurement;

struct Ground_struct
{
	// add your Measurement data here
	// For example:
	float humidity = 0;
	float temperature = 0;
};
Ground_struct Ground;

void Init_dht()
{
	dht.begin(); // first_entry the sensor (wiring)
}

void setup()
{
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

  // Initialize the display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C is typical
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Stop if display init fails
  }

  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 10);            // Position (x, y)
  display.print("Hello OLED!");
  display.display();

  Init_dht();
}

void loop()
{
	// Read the data if available in buffer
	if (radio.available())
	{
		radio.read(&Measurement, sizeof(Measurement));

		Serial.print(Measurement.distance);
		Serial.print("mm\t");
		Serial.print(Measurement.temp);
		Serial.print("C\t");
		Serial.print("\n");
    delay(100);
	}
	else
	{
		Measurement_DHT();
		Serial.print("No radio\n");
		delay(50);
	}
}

void Measurement_DHT()
{
  Ground.humidity = dht.readHumidity();
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