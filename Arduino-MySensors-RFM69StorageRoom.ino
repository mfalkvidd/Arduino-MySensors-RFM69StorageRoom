// Enable debug prints
#define MY_DEBUG
#define MY_NODE_ID 1

// Enable and select radio type attached
#define MY_RFM69_NEW_DRIVER
//#define MY_RX_MESSAGE_BUFFER_FEATURE
#define MY_RFM69_ATC_MODE_DISABLED
#define MY_RFM69_CSMA_LIMIT_DBM -75
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY RFM69_433MHZ
#define MY_RF69_IRQ_PIN 2
#define MY_RF69_SPI_CS 10

//#define MY_DEBUG_VERBOSE_RFM69 // Temporary

#define MY_TRANSPORT_WAIT_READY_MS (30000) // Don't stay awake for more than 30s if communication is broken

#include <MySensors.h> // From Library Manager

#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

static const uint64_t UPDATE_INTERVAL = 300000;
#define WAIT_TIME 150
#define ALTITUDE 6

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_VCC_BEFORE 2
#define CHILD_ID_VCC_AFTER 3
#define CHILD_ID_PRESSURE 6

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgVccBefore(CHILD_ID_VCC_BEFORE, V_VOLTAGE);
MyMessage msgVccAfter(CHILD_ID_VCC_AFTER, V_VOLTAGE);
MyMessage msgPressure(CHILD_ID_PRESSURE, V_PRESSURE);
//TODO: Add light detection (irq-based, LDR)
//TODO: Add flooding detection

void presentation()
{
  // Send the sketch version information to the gateway
  sendSketchInfo(F("StorageRoom " __DATE__), F("1.2"));
  wait(WAIT_TIME);
  present(CHILD_ID_HUM, S_HUM);
  wait(WAIT_TIME);
  present(CHILD_ID_TEMP, S_TEMP);
  wait(WAIT_TIME);
  present(CHILD_ID_VCC_BEFORE, S_CUSTOM);
  wait(WAIT_TIME);
  present(CHILD_ID_VCC_AFTER, S_CUSTOM);
  wait(WAIT_TIME);
  present(CHILD_ID_PRESSURE, S_BARO);
}

void setup()
{
  if (! bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring! Continuing without sensor...");
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);
}

void loop()
{
  static float lastTemp, lastHum, lastPressure;
  send(msgVccBefore.set(readVcc() / 1000.0, 3));
  bme.takeForcedMeasurement();
  float temperature = bme.readTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed reading temperature");
  } else if (temperature != lastTemp) {
    // Only send temperature if it changed since the last measurement
    lastTemp = temperature;
    send(msgTemp.set(temperature, 1));
  }
#ifdef MY_DEBUG
  Serial.print("T: ");
  Serial.println(temperature);
#endif

  float humidity = bme.readHumidity();
  if (isnan(humidity)) {
    Serial.println("Failed reading humidity");
  } else if (humidity != lastHum) {
    // Only send humidity if it changed since the last measurement
    lastHum = humidity;
    send(msgHum.set(humidity, 1));
  }
#ifdef MY_DEBUG
  Serial.print("H: ");
  Serial.println(humidity);
#endif
  float pressure = bme.readPressure() / pow((1.0 - (ALTITUDE / 44330.0)), 5.255); // Adjust to sea level pressure using user altitude
  if (isnan(pressure)) {
    Serial.println("Failed reading pressure");
  } else if (pressure != lastPressure) {
    // Only send pressure if it changed since the last measurement
    lastPressure = pressure;
    send(msgPressure.set(pressure, 1));
  }
#ifdef MY_DEBUG
  Serial.print("P: ");
  Serial.println(pressure);
#endif

  send(msgVccAfter.set(readVcc() / 1000.0, 3));
  // Sleep for a while to save energy
  sleep(UPDATE_INTERVAL);
}

long readVcc() {
  // From http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
