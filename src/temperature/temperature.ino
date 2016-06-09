/**
 * Cwik Sous-Vide
 * Author: Dennis Cwik
 * Date: June 4, 2016
 *
 * Logic for reading temperature using the OneWire v2.3.2 library,
 * (commit 57c18c6de80c13429275f70875c7c341f1719201), with an
 * externally powered (ie, not in parasite power mode) DS18B20
 * sensor connected to pin 10, as well as a 4.7K Ohm resistor
 * between 5V and pin 10.
 *
 * This example code is in the public domain.
 */
 
#include <OneWire.h>
#include <LedControl.h>

// DEBUG can be toggled to make serial communication less noisy
#define DEBUG true

// pin assignments
#define ONE_WIRE_BUS_PIN 10

// 1-wire bus commands
#define COMMAND_CONVERT_T         0x44
#define COMMAND_READ_SCRATCHPAD   0xBE
#define COMMAND_WRITE_SCRATCHPAD  0x4E
#define COMMAND_COPY_SCRATCHPAD   0x48
#define COMMAND_READ_POWER_SUPPLY 0xB4

// error codes
#define NO_ERROR                         0
#define ERROR_OWB_NO_DEVICES             1
#define ERROR_OWB_TOO_MANY_DEVICES       2
#define ERROR_OWB_ADDRESS_CRC            3
#define ERROR_OWB_NOT_28_FAMILY          4
#define ERROR_OWB_SCRATCHPAD_READ_CRC    5
#define ERROR_OWB_SCRATCHPAD_WRITE_CRC   6
#define ERROR_OWB_NOT_EXTERNALLY_POWERED 7

// create the 1-wire bus
OneWire ds(ONE_WIRE_BUS_PIN);

// create LED controller
LedControl lc = LedControl(7, 6, 5, 1);

void setup(void)
{
  Serial.begin(9600);

  // wake up the 7-segment display, set brightness, clear
  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);
}

void loop(void)
{
  // we only ever read the first 9 bytes from the Scratchpad
  byte data[9];
  
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
  
  // ensure the device is powered externally
  result = verifyExternallyPowered(addr);
  if (result != NO_ERROR)
  {
    showError(result);
    delay(5000);
    return;
  }
  
  // query the DS18B20, write into data
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
  
  // ensure the DS18B20 has the right bit resolution
  result = validateResolution(addr, data);
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

int verifyExternallyPowered(byte *addr)
{
  // issue a "read power supply" command
  ds.reset();
  ds.select(addr);
  ds.write(COMMAND_READ_POWER_SUPPLY);

  // parasite powered devices will pull the bus low, externally
  // powered devices will let the bus remain high
  if (ds.read_bit() == 0)
  {
    return ERROR_OWB_NOT_EXTERNALLY_POWERED;
  }
  
  return NO_ERROR;
}

int querySensor(byte *addr, byte *data)
{
  // trigger a Convert T command (ie, get temperature and store it
  // on the DS18B20's Scratchpad memory)
  ds.reset();
  ds.select(addr);
  ds.write(COMMAND_CONVERT_T);
  
  // During a Convert T with an externally powered DS18B20, the
  // device keeps the 1-wire bus low until it is done the convert.
  // This means that we don't have to delay an arbitrary amount
  // of time, instead we just keep polling until the bus goes
  // high. If we were using parasite power mode, we as the master
  // of the bus would have to keep the bus high to power the device
  // and must wait an arbitrary (but long enough) amount of time
  // for the convert to complete.
  while (ds.read_bit() == 0);
  
  
  // send read command and read the bytes
  ds.reset();
  ds.select(addr);
  ds.write(COMMAND_READ_SCRATCHPAD);
  ds.read_bytes(data, 9);
  
  if (OneWire::crc8(data, 8) != data[8])
  {
    return ERROR_OWB_SCRATCHPAD_READ_CRC;
  }
  return NO_ERROR;
}

int validateResolution(byte *addr, byte *data)
{
  // config register format:
  //
  //   0  R1 R2 1    1  1  1  1
  //   ^        ^    ^  ^  ^  ^
  //   |        |    |  |  |  |
  //   +--------+----+--+--+--+
  //                 |
  //   Do not change these, they should remain
  //   as they are. Only change R1 and R2 as 
  //   per the table below.
  //
  //   R1  R2  Value  Resolution (bits)
  //  --------------------------------
  //   0   0   0x0    9
  //   0   1   0x1    10
  //   1   0   0x2    11
  //   1   1   0x3    12
  
  // can only be set to 0x0, 0x1, 0x2, or 0x3
  byte desiredR1R2 = 0x3;
  
  // 5th byte is config register
  byte actualR1R2 = data[4] >> 5;

  if (actualR1R2 != desiredR1R2)
  {
    if (DEBUG)
    {
      Serial.print("Wanted ");
      Serial.print(desiredR1R2 + 9);
      Serial.print("-bit resolution, found ");
      Serial.print(actualR1R2 + 9);
      Serial.println("-bit");
    }
    
    // try writing the desired resolution to the Scratchpad
    
    byte writeAttempts = 0;
    while (true)
    {
      writeAttempts +=1;
      
      // send "write Scratchpad" command
      ds.reset();
      ds.select(addr);
      ds.write(COMMAND_WRITE_SCRATCHPAD);
      
      // The DS18B20 now expects 3 bytes to be written (the TH,
      // TL, and config register), which it puts in the
      // Scratchpad. TH and TL are the high and low alarm
      // triggers, meaning if the temperature was out of that
      // range and we issued a 0xEC alarm search, the device
      // would respond. Since we don't use the alarm triggers,
      // we could store anything we'd like there, and it would
      // be non-volatile! Oh well, let's use what was already
      // set there.
      ds.write(data[2]); // unchanged TH
      ds.write(data[3]); // unchanged TL
      ds.write((desiredR1R2 << 5) | 0x1F); // new config reg
      
      // read the Scratchpad to ensure write was successful via CRC
      ds.reset();
      ds.select(addr);
      ds.write(COMMAND_READ_SCRATCHPAD);
      ds.read_bytes(data, 9);
      
      if ((data[4] >> 5) == desiredR1R2
          && OneWire::crc8(data, 8) == data[8])
      {
        // yay, write to Scratchpad was successful!
        break;
      }
      else if (writeAttempts >= 2)
      {
        // error out if we've tried too many times
        return ERROR_OWB_SCRATCHPAD_WRITE_CRC;
      }
    }
    
    // finally, copy Scratchpad to EEPROM (ie, persistent storage)
    ds.reset();
    ds.select(addr);
    ds.write(COMMAND_COPY_SCRATCHPAD);
  }
  
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
  
  if (DEBUG)
  {
    if (isNegative)
    {
      Serial.print('-');
    }
    Serial.print(wholeCelsius);
    Serial.print('.');
    Serial.print(fractCelsius);
    Serial.println(" C");
  }
  
  if (wholeCelsius < 100)
  {
    int tens = wholeCelsius / 10;
    int ones = wholeCelsius % 10;
    int tenths = fractCelsius / 10;
    int hundreths = fractCelsius % 10;

    lc.setDigit(0, 7, tens, false);
    lc.setDigit(0, 6, ones, true);
    lc.setDigit(0, 5, tenths, false);
    lc.setDigit(0, 4, hundreths, false);
    lc.setRow(0, 3, 0x4E);
  }
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
