/**
 * Cwik Sous-Vide
 * Author: Dennis Cwik
 * Date: June 4, 2016
 *
 * Logic for reading temperature using the OneWire v2.3.2 library,
 * (commit 57c18c6de80c13429275f70875c7c341f1719201), with a
 * DS18B20 sensor connected to pin 10.
 *
 * This example code is in the public domain.
 */

#include <OneWire.h>

// DEBUG can be toggled to make serial communication less noisy
#define DEBUG true

#define ONE_WIRE_PIN 10

// create the 1-wire bus
OneWire ds(ONE_WIRE_PIN);

void setup(void)
{
  Serial.begin(9600);
}

void loop(void)
{
  // re-usable iterator
  byte i;
  
  // TODO figure out the point of this!
  byte present = 0;
  
 // TODO figure out why this is 12!
  byte data[12];
  
  // 8 byte address for devices on the 1-wire bus
  byte addr[8];
  
  boolean isNegative = false;
  int whole = 0;
  int fract = 0;
  int sensorReading = 0;
  
  // Search for the next device on the 1-wire bus. If found, its
  // unique 8-byte address will be written into addr and true is
  // returned, otherwise false is returned.
  if (!ds.search(addr))
  {
    if (DEBUG)
    {
      Serial.println("No more addresses.");
    }
    
    // if there are multiple devices on the 1-wire bus, this will
    // reset so that the first device is returned on the next
    // search 
    ds.reset_search();
    delay(1000);
    return;
  }
  
  // print out the next device's unique address
  Serial.print("addr=");
  for (i = 0; i < 8; ++i)
  {
    printHex(addr[i]);
    Serial.print(' ');
  }
 
  // verify valid address via CRC
  if (OneWire::crc8(addr, 7) != addr[7])
  {
    Serial.println("CRC is not valid!");
    return;
  }
  
  if (addr[0] == 0x28)
  {
    Serial.println("is a DS18B20 family device.");
  }
  else if (addr[0] == 0x10)
  {
    Serial.print("is a DS18S20 family device, only ");
    Serial.println("DS18B20 family devices are supported.");
    return;
  }
  else
  {
    Serial.print(", device family is not recognized: 0x");
    Serial.println(addr[0], HEX);
    return;
  }
  
  // reset state, select the discovered device on the 1-wire bus
  ds.reset();
  ds.select(addr);
  
  // trigger a Convert T command (ie, get temperature and store it
  // on the DS18B20's Scratchpad memory)
  ds.write(0x44);
  
  present = ds.reset();
  ds.select(addr);
  
  ds.write(0xBE);
  
  Serial.print("data=");
  for (i = 0; i < 9; ++i)
  {
    data[i] = ds.read();
    printHex(data[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  if (OneWire::crc8(data, 8) != data[8])
  {
    Serial.println("CRC validation failed on data read!");
    return;
  }
  
  Serial.println();
  
  sensorReading = (data[1] << 8) | data[0];
  
  // On the DS18B20, the 12th through the 16th bit are the sign
  // bits. We'll only check the 16th bit to see if it's negative,
  // ie, the binary number matches 1xxx xxxx xxxx xxxx
  isNegative = (sensorReading & 0x8000) != 0;
  
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
  whole = sensorReading >> 4;
  
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
  fract = ((sensorReading & 0x000F) * 100) >> 4;
  
  if (isNegative)
  {
    Serial.print('-');
  }
  Serial.print(whole);
  Serial.print('.');
  Serial.print(fract);
  Serial.println(" C");
}

/**
 * Prints a 2 digit hex value to the serial port.
 */
void printHex(byte b)
{
  // Serial.print(x, HEX), where 0 <= x <= F only prints a single
  // character, so pad it with a '0' (ie, print "04" instead of
  // "4").
  if (b < 0x10)
  {
    Serial.print('0');
  }
    
  Serial.print(b, HEX);
}
