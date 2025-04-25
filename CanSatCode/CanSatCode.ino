// LENA project
// Skeleton code

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT.h> // Required header

#define PIN_DHT 7     // Data line to pin 7
#define SPI_Radio 7   // Data line to pin 7
#define DHTType DHT11 // Specify the type of DHT

DHT dht = DHT(PIN_DHT, DHTType); // instance of DHT class
RF24 radio(8, SPI_Radio);        // CE, CSN

// Some definition
#define uint8 uint8_t
#define int8 int8_t

// CanSat structure
struct CanSat_struct
{
  // Ground station name
  const byte address[7] = {'G', 'r', 'o', 'u', 'd', '\0'};

  // I2C adresses
  uint8 Sensor_I2C_addresses[1];

  // SPI pins, index [0] is radio
  uint8 Sensor_SPI_pins[];

  uint8 Sensor_pin[1];
};
struct Measurement_struct
{
  // add your Measurement data here
  // For example:
  float temperature = 0;
  float humidity = 0;
};

// Our structs
Measurement_struct Measurement;
CanSat_struct CanSat;

// Your initialization function for your sensor
void Init_sensor_Radio()
{
  radio.begin();

  radio.openWritingPipe(CanSat.address);

  radio.setPALevel(RF24_PA_MIN);

  radio.stopListening();
}
void Init_CanSat()
{
}

// Take your Measurement and save it in Measurement struct
void Send_Radio()
{

  radio.write(&Measurement, sizeof(Measurement_struct));
}

void Measurement_DHT()
{

  // CanSat.temperature  = dht.readTemperature();  // Reading temperature
  float humidity, temperature = 0;
  humidity = dht.readHumidity(); // Reading humidity
  Measurement.humidity = humidity;
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

void setup()
{

  Serial.begin(9600);

  // Call your sensor initialization function?
  Init_sensor_Radio();
  dht.begin(); // Start the sensor (wiring)
}

void loop()
{
  Measurement_DHT();
  Send_Radio();
  delay(500);
}