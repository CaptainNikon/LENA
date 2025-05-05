// C code for LENA CanSat Receiver
// Used in the Ground station
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Radio Setup
RF24 radio(9, 8); // CE, CSN
byte address[6] = "Canst";

// Some good definition
#define uint8 uint8_t
#define int8 int8_t
#define uint16 uint16_t
#define int16 int16_t

// DHT variables
#define PIN_DHT 2 // DHT data pin
#define DHTType DHT11 // Specify the type of DHT
DHT dht = DHT(PIN_DHT, DHTType);

// Screen Setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET     -1
int dispupdate = 0;
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 1000; // ms
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// Setup the data structures for ground and CanSat
struct Measurement_struct
{
	uint16_t distance = 0; 	// [mm]
	uint16_t temp = 0; 		// *100 [°C]
	int16_t accelerometer_X, accelerometer_Y, accelerometer_Z; // [g] ?
	float magX, magY, magZ; // ? 
};

struct Ground_struct
{
	float humidity = 0;
	float temperature = 0;
};

Measurement_struct Measurement;
Ground_struct Ground;


// Initialization functions
void Init_dht()
{
	dht.begin();
}

void Init_display() {
	display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 10);            // Position (x, y)
  display.print("Starting...");
  display.display();
	delay(100);
}

void setup()
{
	while (!Serial)
		;
	Serial.begin(115200);
	radio.begin();
	//radio.setCRCLength(RF24_CRC_16); // Set check sum length, check sum=CRC
	//  radio.toggleAllPipes(true);		 // Toggle all pipes together, is this good idea?
	radio.setChannel(21);
	radio.setAutoAck(1);
	//radio.setPALevel(RF24_PA_LOW);
	radio.setDataRate(RF24_250KBPS);
	radio.openReadingPipe(0, address);
	//  we have the chanels 21-30 and 81-90
	radio.startListening();

  // Initialize the display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C is typical
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Stop if display init fails
  }

	Init_display();
  Init_dht();
}
int count = 0;
void loop()
{
	// Read the data if available in buffer
	if (radio.available())
	{
		//Serial.print(count++);
		//Serial.print("\n");
		radio.read(&Measurement, sizeof(Measurement));
		Serial.print(Measurement.distance);
		Serial.print("mm\t");
		Serial.print(Measurement.temp);
		Serial.print("C\t");
		Serial.print("\n");
		Serial.print(float(Measurement.accelerometer_X)* 0.015748);Serial.print("\t");
		Serial.print(float(Measurement.accelerometer_Y)* 0.015748);Serial.print("\t");
		Serial.print(float(Measurement.accelerometer_Z)* 0.015748);Serial.print("\t");
		Serial.print("\n");
		Serial.print(Measurement.magX);Serial.print("\t");
		Serial.print(Measurement.magY);Serial.print("\t");
		Serial.print(Measurement.magZ);Serial.print("\t");
		Serial.print("\n");
	}
	else
	{
		//Measurement_DHT();
		//if (millis() - lastDisplayTime >= displayInterval) 
		//{
    //lastDisplayTime = millis();
    //display_ground();
		//}
	}
}

void display_ground() {
	int precision = 3;
  char tempStr[precision+1];
  char humStr[precision+1];

  // Convert float to string: (value, width, precision, name)
  dtostrf(Ground.temperature, precision, 0, tempStr);
  dtostrf(Ground.humidity,    precision, 0, humStr);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.print("T:"); display.print(tempStr); display.print("C|Hum:"); display.print(humStr); display.print("%"); display.print("No:"); 
	display.println(dispupdate++);
  display.display();
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
}