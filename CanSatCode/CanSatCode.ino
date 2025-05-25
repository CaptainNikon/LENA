// LENA project
// THis is the code for the CanSat

// Careful when using this code, two libraries were chaned so the code can run
// mag.read is not available as a public function this has to be copied from private to public in #include <Adafruit_Sensor.h>
// 

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <OneWire.h>		   // Required header
#include <DallasTemperature.h> // Required header
#include <Wire.h>			   // Wire library - used for I2C communication
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <printf.h>

// #include <Servo.h>

// Comment out the line before, to remove all the serial print -> smaller code -> more code for storage
//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__);
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__);
#else
#define DEBUG_PRINT(...);
#define DEBUG_PRINTLN(...);
#endif

#define STORAGE_SIZE 500




// Defins the data storage size in bytes

// RF24 radio(7, 8); // CE, CSN

// Defins the pins
#define PIN_Ultrasonic_trig 3 // Ultrasonic triger pin
#define PIN_Ultrasonic_echo 4 // Ultrasonic echo pin
#define SPI_Radio 8			  // Radio CSN pin
#define CE_PIN 7
#define DS18B20_PIN 2 // Dataline is plugged into digital pin 2
int ADXL345 = 0x1D;	  // Adress for DRFduino

RF24 radio(CE_PIN, SPI_Radio, 1000000); // CE, CSN
OneWire oneWire(DS18B20_PIN);			// setup the oneWire to communicate with sensor
DallasTemperature sensors(&oneWire);	// send the oneWire reference to DallasTemperature
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL();
// Servo myservo;  // create servo object to control a servo

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

unsigned int burstMode = 100;  // Sends data 10 times per sec
unsigned int surveyMode = 500; // Sends data 2 times per sec

// CanSat structure
struct CanSat_struct {
	char command[4] = {0}; // 3-character command + null terminator
	int8_t Servo_pos = 0;

	struct {
		bool Sensor_Ultrasonic : 1;
		bool Sensor_Halleffect : 1;
		bool Sensor_Temperature : 1;
		bool Sensor_Acceleration : 1;
		int8_t timer_count : 4;
	};
	int currentDataSendingMode;
	long pullrate = 250;
};


// __attribute__((packed))
struct __attribute__((packed)) Measurement_struct
{
	uint16 time=0;
	uint8 distance = 0;
	int16 temp = 0;
	int16 accelerometer_X = 0, accelerometer_Y = 0, accelerometer_Z = 0;
	int16 calX = 0, calY = 0, calZ=0;
};
struct Data_struct
{
	uint16 first_entry = 0;		  // First entries with data
	uint16 end_entry = 0;		  // First empty entriy
	byte data[STORAGE_SIZE + 20]; // Data storage
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
void Init_DS18B20();

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
		Data.data[(i + Data.first_entry)] = 0;
	}

	// move first_entry with the size, to the new first entry
	Data.first_entry = (Data.first_entry + size);
	if (Data.first_entry > STORAGE_SIZE)
	{
		Data.first_entry = 0;
	}
}

void Move_meassurment_to_data()
{
	Measurement.time = millis() / 10;
	// Measurement.accelerometer_X=1111;
	// DEBUG_PRINTLN(Measurement.time);
	//  Move all the meassurments from the Meassurment struct to last entris in data

	uint16 size = sizeof(Measurement_struct);

	// pointer to measurment values
	byte *Measurment_ptr = (byte *)&Measurement;

	// Coppy evrything in measurment struct to data struct byte by byte
	if (Data.end_entry + size > STORAGE_SIZE)
	{
		Data.end_entry = 0;
	}
	for (uint16 i = 0; i < size; i++)
	{
		Data.data[(i + Data.end_entry)] = Measurment_ptr[i];
	}

	// Move end pointer to next entry
	Data.end_entry = (Data.end_entry + size);
	if (Data.end_entry > STORAGE_SIZE)
	{
		Data.end_entry = 0;
	}
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
	Init_DS18B20();
	// Init_servo();

}

void print_values()
{
	DEBUG_PRINT(Measurement.time);
	DEBUG_PRINT("     =========== new ===========\n");
	DEBUG_PRINT("distance\t");
	DEBUG_PRINT(Measurement.distance);
	DEBUG_PRINT("   temp\t");
	DEBUG_PRINT(Measurement.temp);
	DEBUG_PRINT("\n");

	DEBUG_PRINT("Xa= ");
	DEBUG_PRINT(Measurement.accelerometer_X);
	DEBUG_PRINT("   Ya= ");
	DEBUG_PRINT(Measurement.accelerometer_Y);
	DEBUG_PRINT("   Za= ");
	DEBUG_PRINTLN(Measurement.accelerometer_Z);

	DEBUG_PRINT("Xmag= ");
	DEBUG_PRINT(Measurement.calX);
	DEBUG_PRINT("   Ymag= ");
	DEBUG_PRINT(Measurement.calY);
	DEBUG_PRINT("   Zmag= ");
	DEBUG_PRINTLN(Measurement.calZ);
}

int loop_counter = 0;		 // holds the count for every loop pass
long loop_timer_now = 0;	 // holds the current millis
long previous_millis = 0;	 // holds the previous millis
float loop_timer = 0;		 // holds difference (loop_timer_now - previous_millis) = total execution time
int loop_test_times = 20000; // Run loop 20000 times then calculate time

unsigned long previousMillis = 0; // will store last time LED was updated
// constants won't change:

void loop()
{
	loop_counter++;
  
	unsigned long currentMillis = millis();
	if (currentMillis - previousMillis >= CanSat.pullrate/10)
	{
		// The loop will run 10 times before it gos in to this if statement, and take the last measurments
		if(CanSat.timer_count==0){
		#ifdef DEBUG
			loop_timer = (((float)loop_counter) * 1000.0) / (currentMillis - previousMillis);
			loop_counter = 0;
			previousMillis = currentMillis;
			DEBUG_PRINT("Messurment pull rate: ");
			DEBUG_PRINTLN(loop_timer);
		#endif
		
			CanSat.timer_count=0; // Reset counter
			// Take the measurement
			if (CanSat.Sensor_Temperature)
			{
				Measurement_DS18B20();
			}

			if (CanSat.Sensor_Ultrasonic)
			{
				Measurement_Ultrasonic();
			}
			Move_meassurment_to_data();
		}
		if (CanSat.Sensor_Acceleration)

		
		{
			Measurement_accelerometer();
		}
		if (CanSat.Sensor_Halleffect)
		{
			Measurement_hall_effect();
		}

		++CanSat.timer_count;
	}

	// print_values();

	// Move the Measurement to Data
	//Move_meassurment_to_data(); This should be needed here no ? Samuel
	// Radio over the data from Data
	Radio_send();
	Radio_read();
	Run_Commands();

	// Servo_move();
	delay(50);
}


void Init_Radio()
{

	// radio.begin(3000000); // Test this
	radio.begin();

	//radio.setDataRate(2); // Linux expression
	radio.setDataRate(RF24_250KBPS); // Windows expression in case we have to debug with Windows
	// radio.setCRCLength(RF24_CRC_16); // Set check sum length, check sum=CRC
	//  radio.toggleAllPipes(true);		 // Toggle all pipes together, is this good idea?
	radio.setChannel(21); // set the channel to 21
	radio.openWritingPipe(address_g);
	radio.setAutoAck(true);
	radio.setRetries(5, 15);
	radio.setCRCLength(RF24_CRC_16);
	radio.enableDynamicPayloads();

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

void Init_DS18B20() {
    sensors.begin(); // Discover devices

    DeviceAddress addr;
    if (sensors.getAddress(addr, 0)) {
        sensors.setResolution(addr, 10);
        DEBUG_PRINT("Requested resolution: 10\n");
        DEBUG_PRINT("Actual resolution now: ");
        DEBUG_PRINTLN(sensors.getResolution(addr));
    } else {
        DEBUG_PRINTLN("ERROR: No DS18B20 sensor found!");
    }
}

void Init_CanSat()
{
	CanSat.Sensor_Acceleration = false;
	CanSat.Sensor_Halleffect = false;
	CanSat.Sensor_Temperature = false;
	CanSat.Sensor_Ultrasonic = false;
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

	DEBUG_PRINTLN("Scanning I2C bus...");
	for (byte addr = 1; addr < 127; addr++)
	{
		Wire.beginTransmission(addr);
		if (Wire.endTransmission() == 0)
		{
			DEBUG_PRINT("Found device at 0x");
			DEBUG_PRINTLN(addr, HEX);
		}
	}

	if (!mag.begin())
	{
		DEBUG_PRINTLN("LIS2MDL not found. Check wiring.");
		while (1)
			;
	}
}

void Init_servo()
{
	// myservo.attach(6);
}

// Low level raw reader, adapted from Dallas Temperature Deivde
int16_t readDS18B20Raw(DallasTemperature &sensor, uint8_t index = 0) {
    DeviceAddress addr;
    if (!sensor.getAddress(addr, index)) return DEVICE_DISCONNECTED_RAW;
    uint8_t scratchPad[9];
    if (!sensor.readScratchPad(addr, scratchPad)) return DEVICE_DISCONNECTED_RAW;

    return (int16_t)((scratchPad[1] << 8) | scratchPad[0]); // LSB + MSB
}


// Needed to really measure raw data
void Measurement_DS18B20() {
	sensors.requestTemperatures(); // This is needed to update the sensor
	Measurement.temp = readDS18B20Raw(sensors);
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
	uint16 duration = pulseIn(PIN_Ultrasonic_echo, HIGH,12000); //Maximum delay of 12000 when timeout
	// Calculating the distance
	float distance_cm = duration * 0.0343 / 2.0;
	Measurement.distance = (uint8_t)(distance_cm + 0.5); // round to nearest cm
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
	/*
	// Old code
	int16 X_out, Y_out, Z_out;			// Outputs
	// Reading the values, and 10 bit to int8
	X_out = (Wire.read() | Wire.read() << 8) >> 2; // X-axis value
	Y_out = (Wire.read() | Wire.read() << 8) >> 2; // Y-axis value
	Z_out = (Wire.read() | Wire.read() << 8) >> 2; // Z-axis value
*/
	Measurement.accelerometer_X = (Measurement.accelerometer_X + (Wire.read() | Wire.read() << 8) >> 2) >> 1; // X-axis value; // = (c1+c2)/2
	Measurement.accelerometer_Y = (Measurement.accelerometer_X + (Wire.read() | Wire.read() << 8) >> 2) >> 1; // Y-axis value;
	Measurement.accelerometer_Z = (Measurement.accelerometer_X + (Wire.read() | Wire.read() << 8) >> 2) >> 1; // Z-axis value;
}


// This might be the way to get raw data from the accelerometer
void Measurement_hall_effect()
{
	mag.read(); 

	/*
	int16_t rawX = mag.raw.x;
	int16_t rawY = mag.raw.y;
	int16_t rawZ = mag.raw.z;
	*/
	Measurement.calX = (Measurement.calX + mag.raw.x) >> 1; // = (c1+c2)/2
	Measurement.calY = (Measurement.calY + mag.raw.y) >> 1;
	Measurement.calZ = (Measurement.calZ + mag.raw.z) >> 1;
}

void Servo_move()
{

	// myservo.write(CanSat.Servo_pos);              // tell servo to go to position in variable 'pos'
}

void Radio_send()
{

	if (Data.first_entry == Data.end_entry)
	{
		return; // No data to sent
	}
	//DEBUG_PRINT("Sending ");



	// Getting the size of datae
	uint16 size = Data_entry_size(Data.first_entry);

	radio.stopListening();
	// radio.stopListening();
	bool sent = radio.write(&(Data.data[Data.first_entry]), size);
	// bool sent = radio.write(&(Measurement), sizeof(Measurement_struct));
	radio.startListening();
	if (sent)
	{
		Data_first_delete();
	}
	else
	{
		DEBUG_PRINTLN("fuck");
	}
}

void Radio_read()
{

	if (radio.available()) {
			radio.read(CanSat.command, 3);
			CanSat.command[3] = '\0';  // Null-terminate the string

			DEBUG_PRINT("Command received: ");
			DEBUG_PRINTLN(CanSat.command[0]);

			Run_Commands();  // Handle the command logic
	}

}

/*
First element in the array Cansat.commands will contain either a command for the sensors (S) or a command for the CanSat (C)
Second element will contain information about which sensor we want to change information about (T = DS18B20, U = ultrasonic, A = accelerometer, H = 3D Hall, S = servo)
or at which rate the measurement data is to be sent.
Third element is the command that ground wants to change it output to. For sensors '1' is an active state and '2' is dormant.
*/

void Run_Commands() {
	if (strlen(CanSat.command) != 3) return;

	char type = toupper(CanSat.command[0]);
	char sensor = toupper(CanSat.command[1]);
	char action = toupper(CanSat.command[2]);

	if (type == 'S') {
		bool enable = (action == 'I'); // I = enable (on), O = disable (off)

		if (sensor == 'T') {
			CanSat.Sensor_Temperature = enable;
			if (!enable) Measurement.temp = 0;
		} else if (sensor == 'U') {
			CanSat.Sensor_Ultrasonic = enable;
			if (!enable) Measurement.distance = 0;
		} else if (sensor == 'A') {
			CanSat.Sensor_Acceleration = enable;
			if (!enable) {
				Measurement.accelerometer_X = 0;
				Measurement.accelerometer_Y = 0;
				Measurement.accelerometer_Z = 0;
			}
		} else if (sensor == 'H') {
			CanSat.Sensor_Halleffect = enable;
			if (!enable) {
				Measurement.calX = 0;
				Measurement.calY = 0;
				Measurement.calZ = 0;
			}
		}
	} else if (type == 'C') {
		if (sensor == 'B' && action == 'B') {
			CanSat.currentDataSendingMode = burstMode;
		} else if (sensor == 'S' && action == 'S') {
			CanSat.currentDataSendingMode = surveyMode;
		}
	}
	else
	{
		Serial.println("Not a correct command. Try again");
	}
	CanSat.command[0] = 0;
}


