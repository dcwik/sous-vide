/**
 * Cwik Sous-Vide
 * Author: Dennis Cwik
 * Date: June 4, 2016
 *
 * Logic for reading temperature using the OneWire v2.3.2 library,
 * (commit 57c18c6de80c13429275f70875c7c341f1719201), with an
 * externally powered (ie, not in parasitic mode) DS18B20 sensor
 * connected to pin 10, as well as a 4.7K Ohm resistor between 5V
 * and pin 10.
 *
 * This example code is in the public domain.
 */
 
 // TODOs
 // -ensure 12 bit resolution
 // -figure out "read time slots"
 // -ensure single device
 // -ensure device has an external power supply

#include <OneWire.h>

// DEBUG can be toggled to make serial communication less noisy
#define DEBUG true

// pin assignments
#define ONE_WIRE_PIN 10

// error codes
#define NO_ERROR                      0
#define ERROR_OWB_NO_DEVICES          1
#define ERROR_OWB_TOO_MANY_DEVICES    2
#define ERROR_OWB_ADDRESS_CRC         3
#define ERROR_OWB_NOT_28_FAMILY       4
#define ERROR_OWB_SCRATCHPAD_READ_CRC 5

// create the 1-wire bus
OneWire ds(ONE_WIRE_PIN);

void setup(void)
{
  Serial.begin(9600);
}

void loop(void)
{
  // TODO figure out why this is 12!
  byte data[12];
  
  // 8 byte address for devices on the 1-wire bus
  byte addr[8];
  
  // the 2-byte temperature reading from the sensor
  int sensorReading = 0;
  
  int result;
  
  result = findSingleDevice(addr);
  if (result != NO_ERROR)
  {
    showError(result);
    delay(5000);
    return;
  }
  
  // print out the device's unique address
  if (DEBUG)
  {
    Serial.print("addr=");
    printHexBytes(addr, 8);
    Serial.println();
  }
  
  result = querySensor(addr, data);
  if (result != NO_ERROR)
  {
    showError(result);
    delay(5000);
    return;
  }
  
  // print out data from the scratchpad
  if (DEBUG)
  {
    Serial.print("data=");
    printHexBytes(data, 9);
    Serial.println();
  }
  
  result = validatePrecision(addr, data);
  if (result != NO_ERROR)
  {
    showError(result);
    delay(5000);
    return;
  }
  
  // first 2 bytes read are the temperature data, with the first
  // byte being the least significant bits
  sensorReading = (data[1] << 8) | data[0];
  
  showTemperature(sensorReading);
  
  delay(2000);
}

/**
 * Looks for a single device on the 1-wire bus. If found, it's
 * address will be written into addr and NO_ERROR will be
 * returned, otherwise an error code will be returned.
 */
int findSingleDevice(byte *addr)
{
  // ensure we're looking for the first device on the 1-wire bus
  ds.reset_search();
  
  // Search for the first device on the 1-wire bus. If found, its
  // unique 8-byte address will be written into addr and true is
  // returned, otherwise false is returned.
  if (!ds.search(addr))
  {
    return ERROR_OWB_NO_DEVICES;
  }
  
  if (ds.search(addr))
  {
    return ERROR_OWB_TOO_MANY_DEVICES;
  }
  
  // verify valid address via CRC
  if (OneWire::crc8(addr, 7) != addr[7])
  {
    return ERROR_OWB_ADDRESS_CRC;
  }
  
  if (addr[0] != 0x28)
  {
    if (DEBUG)
    {
      Serial.print("ERROR: Only 0x28 device families are ");
      Serial.print("supported, found 0x");
      Serial.println(addr[0], HEX);
    }
    return ERROR_OWB_NOT_28_FAMILY;
  }
  
  return NO_ERROR;
}

int querySensor(byte *addr, byte *data)
{
  // all 1-wire bus command start with a reset
  ds.reset();
  ds.select(addr);
  
  // trigger a Convert T command (ie, get temperature and store it
  // on the DS18B20's Scratchpad memory)
  ds.write(0x44);
  
  ds.reset();
  ds.select(addr);
  
  ds.write(0xBE);

  for (int i = 0; i < 9; ++i)
  {
    data[i] = ds.read();
  }
  
  if (OneWire::crc8(data, 8) != data[8])
  {
    return ERROR_OWB_SCRATCHPAD_READ_CRC;
  }
  return NO_ERROR;
}

int validatePrecision(byte *addr, byte *data)
{
  // TODO implement
  return NO_ERROR;
}

void printHexBytes(byte *bytes, int len)
{
  for (int i = 0; i < len; ++i)
  {
    if (0 != i)
    {
      Serial.print(' ');
    }
    printHex(bytes[i]);
  }
}

/**
 * Prints a 2 digit hex value to the serial port.
 */
void printHex(byte b)
{
  // Serial.print(x, HEX), where 0 <= x <= F, only prints a single
  // character, so pad it with a '0' (ie, print "04" instead of
  // "4").
  if (b < 0x10)
  {
    Serial.print('0');
  }
    
  Serial.print(b, HEX);
}

void showTemperature(int sensorReading)
{
  // On the DS18B20, the 12th through the 16th bit are the sign
  // bits. We'll only check the 16th bit to see if it's negative,
  // ie, the binary number matches 1xxx xxxx xxxx xxxx.
  boolean isNegative = (sensorReading & 0x8000) != 0;
  
  if (isNegative)
  {
    // for 2's complement, flip all bits and add 1 to convert from
    // negative to postive
    sensorReading = (sensorReading ^ 0xffff) + 1;
  }
  
  // At this point, we've forced sensorReading to be a positive,
  // 16-bit number, meaning the 16-bit sensorReading integer looks
  // like this in binary:
  //                       0000 0xxx xxxx xxxx
  //                       ------^^^ ^^^^ ^^^^
  //                         ^   ||| |||| ||||
  //                         |   ||| |||| ||||
  //   sign bits (positive) -+   ||| |||| ||||
  //   2^6 Celsius --------------+|| |||| ||||
  //   2^5 Celsius ---------------+| |||| ||||
  //   2^4 Celsius ----------------+ |||| ||||
  //   2^3 Celsius ------------------+||| ||||
  //   2^2 Celsius -------------------+|| ||||
  //   2^1 Celsius --------------------+| ||||
  //   2^0 Celsius ---------------------+ ||||
  //   2^(-1) Celsius --------------------+|||
  //   2^(-2) Celsius ---------------------+||
  //   2^(-3) Celsius ----------------------+|
  //   2^(-4) Celsius -----------------------+
  //
  // Since 2^0 Celsius is 4 bits away from the right bit, it means
  // the temperature reading is bit shifted to the left (ie, "<<")
  // 4 bit, or in other words, this integer representing the
  // temperature is 16 times (2^4 = 16) than the actual temperature
  // reading.
  //
  // Example 1:
  //   sensorReading = 0000 0000 0001 0000 (binary)
  //                 = 0010 (hex)
  //                 = 16 (decimal)
  //                 = 1 Celsius
  //
  // Example 2:
  //   sensorReading = 0000 0010 0000 1000 (binary)
  //                 = 0208 (hex)
  //                 = 520 (decimal)
  //                 = 32.5 Celsius
  //   This should make sense because the 2^5 Celsius bit (ie, 32
  //   Celsius) is enabled, and the 2^(-1) Celsius bit (ie, 0.5
  //   Celsius) is enabled, hence 32.5 Celsius (and 520 / 16 =
  //   32.5).

  // To get the whole part of the temperature, divide by 16
  // (which is the same as bit shifting to the right by 4).
  int wholeCelsius = sensorReading >> 4;
  
  // The fractional part of the temperature is represented by the
  // last 4 bits, so grab those (by masking with
  // 0000 0000 0000 1111), and bit shift them to the right by
  // 4. To convert the fraction to an integer, we'll multiply by
  // 100.
  //
  // For example, if the last 4 bits of the sensorReading are 1100
  // it means that it's 0.75 Celsius (because it's 2^(-1) +
  // 2^(-2)). We just have to bit shift 1100 to the
  // right by 4, so that 1100 results in 0.1100, then multiply by
  // 100 to go from the fraction to a whole number (ie,
  // 0.75 * 100 = 75). However, since we're storing it as an
  // integer, if we bit shift 1100 by 4 first, it would actually
  // result in 0000 (not 0.1100) since integers only store whole
  // numbers (the 1's get pushed out)! To prevent this, we must
  // FIRST multiply by 100, THEN bit shift, ending up with 75 as
  // the "fractional part".
  int fractCelsius = ((sensorReading & 0x000F) * 100) >> 4;
  
  if (isNegative)
  {
    Serial.print('-');
  }
  Serial.print(wholeCelsius);
  Serial.print('.');
  Serial.print(fractCelsius);
  Serial.println(" C");
  
  // TODO show on 7-segment display
}

void showError(int errorCode)
{
  if (DEBUG)
  {
    Serial.print("Error ");
    Serial.println(errorCode);
  }
  
  // TODO show on 7-segment display
}
