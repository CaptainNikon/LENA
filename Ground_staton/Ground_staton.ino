// C code for LENA CanSat Receiver
// Used in the Ground station
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <printf.h>

// Radio Setup
RF24 radio(7, 8); // CE, CSN
byte adress_g[7] = "Ground";
byte adress_c[6] = "Canst";

// Some good definition
#define uint8 uint8_t
#define int8 int8_t
#define uint16 uint16_t
#define int16 int16_t

// DHT variables
#define PIN_DHT 2	  // DHT data pin
#define DHTType DHT11 // Specify the type of DHT
DHT dht = DHT(PIN_DHT, DHTType);

// Screen Setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
int dispupdate = 0;
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 1000; // ms
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


//Update variables
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 10000;


// Setup the data structures for ground and CanSat
struct __attribute__((packed)) Measurement_struct
{
  uint16_t time;
  uint8_t distance;
  uint16_t temp;
  int16_t accelerometer_X, accelerometer_Y, accelerometer_Z;
  int16_t magX, magY, magZ;
};


struct Ground_struct
{
	uint8_t humidity = 0;
	uint8_t temperature = 0;
};

Measurement_struct Measurement;
Ground_struct Ground;

// Initialization functions
void Init_dht()
{
	dht.begin();
}

void Init_display()
{
	display.clearDisplay();
	display.setTextSize(1);				 // Normal 1:1 pixel scale
	display.setTextColor(SSD1306_WHITE); // Draw white text
	display.setCursor(0, 10);			 // Position (x, y)
	display.print("Starting...");
	display.display();
	delay(100);
}


void debug_print() {
  Serial.print(Measurement.time); Serial.print("\t");  // Timestamp
  Serial.print(Measurement.distance); Serial.print("\t");
  Serial.print(Measurement.temp); Serial.print("\t");  // Still in tenths of °C
  Serial.print(Measurement.accelerometer_X); Serial.print("\t");  // Raw ints
  Serial.print(Measurement.accelerometer_Y); Serial.print("\t");
  Serial.print(Measurement.accelerometer_Z); Serial.print("\t");
  Serial.print(Measurement.magX); Serial.print("\t");
  Serial.print(Measurement.magY); Serial.print("\t");
  Serial.print(Measurement.magZ); Serial.print("\t");
  Serial.print(Ground.temperature); Serial.print("\t");
  Serial.println(Ground.humidity);
}



void setup()
{
	while (!Serial)
		;
	Serial.begin(115200);
	radio.begin();
	// radio.setCRCLength(RF24_CRC_16); // Set check sum length, check sum=CRC
	//   radio.toggleAllPipes(true);		 // Toggle all pipes together, is this good idea?
	radio.setChannel(21);
	radio.setAutoAck(true);
	// radio.setPALevel(RF24_PA_LOW);
	radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(adress_c);
	radio.openReadingPipe(0, adress_g);
	//  we have the chanels 21-30 and 81-90
	radio.setRetries(1, 15);
  radio.setCRCLength(RF24_CRC_16);
	radio.startListening();
  printf_begin();
  radio.printPrettyDetails();

  // Initialize the display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C is typical
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Stop if display init fails
  }


	Init_display();
	Init_dht();
}

void loop() {
  // Always prioritize reading from the radio
  if (radio.available()) {
    radio.read(&Measurement, sizeof(Measurement));
    // Send one clean tab-separated line over serial
      Serial.print(Measurement.time); Serial.print("\t");  // Timestamp
      Serial.print(Measurement.distance); Serial.print("\t");
      Serial.print(Measurement.temp); Serial.print("\t");  // Still in tenths of °C
      Serial.print(Measurement.accelerometer_X); Serial.print("\t");  // Raw ints
      Serial.print(Measurement.accelerometer_Y); Serial.print("\t");
      Serial.print(Measurement.accelerometer_Z); Serial.print("\t");
      Serial.print(Measurement.magX); Serial.print("\t");
      Serial.print(Measurement.magY); Serial.print("\t");
      Serial.print(Measurement.magZ); Serial.print("\t");
      Serial.print(Ground.temperature); Serial.print("\t");
      Serial.println(Ground.humidity);
  }
  
   // Check for incoming serial command
  if (Serial.available()) {
  String command = Serial.readStringUntil('\n');
  command.trim();  // Remove whitespace or newline

  if (command.length() == 1) {
    char c = command.charAt(0);
    radio.stopListening();
    bool success = radio.write(&c, 1);
    radio.startListening();

    if (success == 0) {
      Serial.println("Not Nice");
    }
    else {
      Serial.println("nice");
    }
  } else {
    Serial.println("Invalid command: must be 1 char");
  }}


  // Occasionally sample DHT and update display
  unsigned long now = millis();
  if (now - lastUpdateTime >= updateInterval) {
    lastUpdateTime = now;

    // Read DHT
    Measurement_DHT();

    // Update OLED display
    display_ground();

    //debug_print();
  }
}


void display_ground() {
	int precision = 5;
	char tempStr[precision + 1];
	char humStr[precision + 1];

	// Convert float to string: (value, width, precision, name)
	dtostrf(Ground.temperature, precision, 0, tempStr);
	dtostrf(Ground.humidity, precision, 0, humStr);

	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE);
	display.setCursor(0, 0);

	display.print("T:");
	display.print(tempStr);
	display.print("C|Hum:");
	display.print(humStr);
	display.print("%");
	display.print("No:");
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