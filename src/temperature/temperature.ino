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
#define PIN_ONE_WIRE_BUS 10
#define PIN_LED_DIN       7
#define PIN_LED_CLK       6
#define PIN_LED_CS        5
#define PIN_BUTTON_LEFT   2
#define PIN_BUTTON_RIGHT  8
#define PIN_BUTTON_SELECT 4

#define LED_CHAR_C      0x4E
#define LED_CHAR_E      0x4F
#define LED_CHAR_F      0x47
#define LED_CHAR_I      0x04
#define LED_CHAR_N      0x15
#define LED_CHAR_R      0x05
#define LED_CHAR_T      0x0F
#define LED_CHAR_U      0x1C
#define LED_CHAR_DOT    0x80
#define LED_CHAR_MINUS  0x01
#define LED_CHAR_Q_MARK 0x65

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

#define STATE_SET_UNITS    0
#define STATE_SET_TEMP_INT 1
#define STATE_READ_TEMP    2
#define STATE_ERROR        3

#define DELAY_MS_DEBOUNCE     50
#define DELAY_MS_LONG_PRESS 2000
#define PERIOD_MS_FLASH     1000

#define MASK_BUTTON_PRESSED      0x01
#define MASK_BUTTON_LONG_PRESSED 0x10

#define MAX_DESIRED_SENSOR_READING 99 << 4
#define STARTING_DESIRED_SENSOR_READING 60 << 4 // 140F, "safe"
#define MIN_DESIRED_SENSOR_READING 25 << 4

// create the 1-wire bus
OneWire ds(PIN_ONE_WIRE_BUS);

// create LED controller
LedControl mLedControl = LedControl(
    PIN_LED_DIN,
    PIN_LED_CLK,
    PIN_LED_CS,
    1); // 1 device

// consider byte?
int mState;

boolean mDisplayInCelsius;

// the 2-byte desired temperature reading, in sensor scale,
// ie, in celsius, << 4 (or 16 times the desired celsius
// value)
int mDesiredSensorReading = STARTING_DESIRED_SENSOR_READING;

// the 2-byte temperature reading from the sensor
int mActualSensorReading = 0;

long mLastTempReadTime = 0;

// current button reading, bouncy
int mButtonState[3];

// the debounced state
int mLastButtonState[3];

// the last time the state changed/bounced
long mLastDebounceTime[3];

// time since last button state went HIGH
long mButtonDownSince[3];

void setup(void)
{
  Serial.begin(9600);
  
  // Set button pins to INPUT_PULLUP, meaning each pin is
  // connected to Vin (5V) via a 20K Ohm resistor. With the
  // button normally opened, each pin remains HIGH. When
  // the button is closed, current will draw to the ground
  // of the button, all voltage is dropped across the
  // resistor, and the pin will read LOW
  pinMode(PIN_BUTTON_LEFT, INPUT_PULLUP);
  pinMode(PIN_BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BUTTON_SELECT, INPUT_PULLUP);

  // wake up the 7-segment display, set brightness, clear
  mLedControl.shutdown(0, false);
  mLedControl.setIntensity(0, 8);
  mLedControl.clearDisplay(0);
  
  // start in temperature setting mode
  mState = STATE_SET_UNITS;
  mActualSensorReading = 0;
  
  mDisplayInCelsius = true;
}

void loop(void)
{
  mState = processInputs(mState);
  display(mState);
  
  delay(100);
}

/**
 * Synchronously read inputs, see if the state changed. There
 * Should be no delays in any of the process methods.
 */
int processInputs(int state)
{
  switch (state)
  {
    case STATE_SET_UNITS:
      return processSetUnits();
      break;
    case STATE_SET_TEMP_INT:
      return processSetTempInt();
      break;
    case STATE_READ_TEMP:
      return processReadTemp();
      break;
    case STATE_ERROR:
    default:
      break;
  }
}

/**
 * See if we have the left or right button to change between F
 * and C, or the select button to move us into setting the
 * integer value of the temperature.
 */
int processSetUnits()
{
  byte buttonState = readButtonState();
  
  if (checkButtonState(
          buttonState,
          PIN_BUTTON_SELECT,
          MASK_BUTTON_PRESSED))
  {
    // move to setting integer part of the temperature since
    // select was pressed
    return STATE_SET_TEMP_INT;
  }
  else if (
      checkButtonState(
          buttonState, PIN_BUTTON_LEFT, MASK_BUTTON_PRESSED)
      || checkButtonState(
          buttonState, PIN_BUTTON_RIGHT, MASK_BUTTON_PRESSED))
  {
    // toggle between F and C if left or right is pressed
    mDisplayInCelsius = !mDisplayInCelsius;
  }
  
  // stay in same state
  return STATE_SET_UNITS;
}

/**
 * See if we have the left or right button pressed to change the
 * integer value of the temperature, or the select button to move
 * into setting the fractional part of the temperature.
 */
int processSetTempInt()
{
  byte buttonState = readButtonState();
  
  if (checkButtonState(
          buttonState,
          PIN_BUTTON_LEFT,
          MASK_BUTTON_PRESSED))
  {
    // decrease temp
    mDesiredSensorReading =
        constrain(
          mDesiredSensorReading - (1 << 4),
          MIN_DESIRED_SENSOR_READING,
          MAX_DESIRED_SENSOR_READING);
  }
  else if (checkButtonState(
          buttonState,
          PIN_BUTTON_RIGHT,
          MASK_BUTTON_LONG_PRESSED))
  {
    // decrease temp fast
  }
  else if (checkButtonState(
          buttonState,
          PIN_BUTTON_RIGHT,
          MASK_BUTTON_PRESSED))
  {
    // increase temp
    mDesiredSensorReading =
        constrain(
          mDesiredSensorReading + (1 << 4),
          MIN_DESIRED_SENSOR_READING,
          MAX_DESIRED_SENSOR_READING);
  }
  else if (checkButtonState(
          buttonState,
          PIN_BUTTON_RIGHT,
          MASK_BUTTON_LONG_PRESSED))
  {
    // increase temp fast
  }
            
  return STATE_SET_TEMP_INT;
}

int processReadTemp()
{
  // we only ever read the first 9 bytes from the Scratchpad
  byte data[9];
  
  // 8 byte address for devices on the 1-wire bus
  byte addr[8];
  
  int result;
  
  result = findSingleDevice(addr);
  if (result != NO_ERROR)
  {
    showError(result);
    delay(5000);
    return STATE_ERROR;
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
    return STATE_ERROR;
  }
  
  long now = millis();
  
  // break out early if we've read recently
  if (now - mLastTempReadTime < 2000)
  {
    return STATE_READ_TEMP;
  }
  
  mLastTempReadTime = now;
  
  // query the DS18B20, write into data
  result = querySensor(addr, data);
  if (result != NO_ERROR)
  {
    showError(result);
    delay(5000);
    return STATE_ERROR;
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
    return STATE_ERROR;
  }
  
  // first 2 bytes read are the temperature data, with the first
  // byte being the least significant bits
  mActualSensorReading = (data[1] << 8) | data[0];
  
  return STATE_READ_TEMP;
}

void display(int state)
{
  switch (state)
  {
    case STATE_SET_UNITS:
      displayUnits();
      break;
    case STATE_SET_TEMP_INT:
      displaySetTempInt();
      break;
    case STATE_READ_TEMP:
      displayReadTemp();
      break;
    case STATE_ERROR:
    default:
      displayError();
      break;
  }
}

/**
 * Looks for a single device on the 1-wire bus. If found, its
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

byte readButtonState()
{
  byte state = 0;
  int reading;

  for (byte i = 0; i < 3; ++i)
  {
    reading = digitalRead(getPinNumber(i));
    
    int now = millis();
    
    if (reading != mLastButtonState[i])
    {
      mLastDebounceTime[i] = now;
    }
    
    if ((now - mLastDebounceTime[i]) > DELAY_MS_DEBOUNCE)
    {
      if (reading != mButtonState[i])
      {
        mButtonState[i] = reading;
        
        if (mButtonState[i] == LOW)
        {
          state |= (MASK_BUTTON_PRESSED << (i * 2));
          mButtonDownSince[i] = now;
        }
        else
        {
          // state is already zero
        }
      }
      else if (now - mButtonDownSince[i] > DELAY_MS_LONG_PRESS)
      {
        if (mButtonState[i] == LOW)
        {
          state |= (MASK_BUTTON_LONG_PRESSED << (i * 2));
        }
      }
    }
    
    mLastButtonState[i] = reading;
  }
  
  return state;
}

byte getPinNumber(byte i)
{
  switch (i)
  {
    case 0:
      return PIN_BUTTON_LEFT;
    case 1:
      return PIN_BUTTON_RIGHT;
    case 2:
      return PIN_BUTTON_SELECT;
  }
}

boolean checkButtonState(byte state, byte pin, byte mask)
{
  byte shift;
  
  switch (pin)
  {
    case PIN_BUTTON_LEFT:
      shift = 0;
      break;
    case PIN_BUTTON_RIGHT:
      shift = 1;
      break;
    case PIN_BUTTON_SELECT:
      shift = 2;
      break;
    default:
      return false;
  }
    
  return (state & (mask << (shift * 2))) != 0;
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
  
  int hundreds = wholeCelsius / 100;
  int tens = (wholeCelsius % 100) / 10;
  int ones = wholeCelsius % 10;
  int tenths = fractCelsius / 10;
  int hundreths = fractCelsius % 10;

  mLedControl.setRow(0, 7, 0); // blank

  if (isNegative && hundreds != 0)
  {
    mLedControl.setRow(0, 6, LED_CHAR_MINUS);
  }
  
  // print hundreds of celsius if its there
  if (hundreds != 0)
  {
    mLedControl.setDigit(0, 5, hundreds, false);
  }
  
  mLedControl.setDigit(0, 4, tens, false);
  mLedControl.setDigit(0, 3, ones, true);
  mLedControl.setDigit(0, 2, tenths, false);
  mLedControl.setDigit(0, 1, hundreths, false);
  mLedControl.setRow(0, 0, LED_CHAR_C); // "C"
}

void displayUnits()
{
  // TODO don't write if already set?
  mLedControl.setRow(0, 7, LED_CHAR_U);
  mLedControl.setRow(0, 6, LED_CHAR_N);
  mLedControl.setRow(0, 5, LED_CHAR_I);
  mLedControl.setRow(0, 4, LED_CHAR_T | LED_CHAR_DOT);
  mLedControl.setRow(0, 3, LED_CHAR_Q_MARK);
  mLedControl.setRow(0, 2, 0);
  mLedControl.setRow(0, 1, 0);
  
  blinkChar(0, mDisplayInCelsius ? LED_CHAR_C : LED_CHAR_F);
}

void displaySetTempInt()
{
  showTemperature(mDesiredSensorReading);
  return;
  
  blinkDigit(7, 0, false);
  blinkDigit(6, 0, false);
  blinkDigit(5, 0, false);
  blinkDigit(4, 0, true);
  mLedControl.setDigit(0, 3, 0, false);
  mLedControl.setDigit(0, 2, 0, false);
  mLedControl.setDigit(0, 1, 0, false);
  mLedControl.setRow(
      0, 0, mDisplayInCelsius ? LED_CHAR_C : LED_CHAR_F);
}

void displayReadTemp()
{
  showTemperature(mActualSensorReading);
}

void blinkDigit(int row, byte value, bool showPoint)
{
  int offsetInPeriod = millis() % PERIOD_MS_FLASH;
  if (offsetInPeriod == (offsetInPeriod % (PERIOD_MS_FLASH >> 1)))
  {
    mLedControl.setDigit(0, row, value, showPoint);
  }
  else
  {
    // clear out digit (for blink effect)
    mLedControl.setRow(0, row, showPoint ? LED_CHAR_DOT : 0);
  }
}
void blinkChar(int row, byte value)
{
  int offsetInPeriod = millis() % PERIOD_MS_FLASH;
  if (offsetInPeriod == (offsetInPeriod % (PERIOD_MS_FLASH >> 1)))
  {
    mLedControl.setRow(0, row, value);
  }
  else
  {
    // clear out digit (for blink effect)
    mLedControl.setRow(0, row, 0);
  }
}

void displayError()
{
  mLedControl.setRow(0, 7, LED_CHAR_E);
  mLedControl.setRow(0, 6, LED_CHAR_R);
  mLedControl.setRow(0, 5, LED_CHAR_R);
  mLedControl.setRow(0, 4, 0);
  mLedControl.setRow(0, 3, LED_CHAR_R);
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
