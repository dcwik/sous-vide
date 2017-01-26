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
 * Data can be read from a terminal using the following:
 * $ cu -l /dev/ttyACM0 -s 9600 > myCsv.csv
 *
 * This example code is in the public domain.
 */
 
#include <OneWire.h>
#include <LedControl.h>

// DEBUG can be toggled to make serial communication less noisy
#define DEBUG true

#define MAX_UNSIGNED_LONG 0xFFFFFFFF // 2 ^ 32 - 1

// pin assignments
#define PIN_ONE_WIRE_BUS 10
#define PIN_LED_DIN       7
#define PIN_LED_CLK       6
#define PIN_LED_CS        5
#define PIN_BUTTON_LEFT   2
#define PIN_BUTTON_RIGHT  8
#define PIN_BUTTON_SELECT 4
#define PIN_RELAY        13

#define LED_CHAR_C      0x4E
#define LED_CHAR_E      0x4F
#define LED_CHAR_F      0x47
#define LED_CHAR_I      0x04
#define LED_CHAR_N      0x15
#define LED_CHAR_O      0x1D
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
#define STILL_CONVERTING                 1
#define ERROR_OWB_NO_DEVICES             2
#define ERROR_OWB_TOO_MANY_DEVICES       3
#define ERROR_OWB_ADDRESS_CRC            4
#define ERROR_OWB_NOT_28_FAMILY          5
#define ERROR_OWB_SCRATCHPAD_READ_CRC    6
#define ERROR_OWB_SCRATCHPAD_WRITE_CRC   7
#define ERROR_OWB_NOT_EXTERNALLY_POWERED 8

#define READ_SENSOR_SUCCESS    0
#define READ_SENSOR_FAIL       1
#define READ_SENSOR_PROCESSING 2

#define STATE_SET_UNITS       0
#define STATE_SET_TEMP_INT    1
#define STATE_SET_TEMP_FRAC   2
#define STATE_READ_TEMP_START 3
#define STATE_READ_TEMP_WAIT  4
#define STATE_ERROR           5

#define DELAY_MS_DEBOUNCE     50
#define DELAY_MS_LONG_PRESS  480
#define PERIOD_MS_FLASH     1000
#define PERIOD_PID_CYCLE_MS 4000

#define MASK_BUTTON_PRESSED      0x01
#define MASK_BUTTON_LONG_PRESSED 0x02

#define C_TO_F(x) ((x * 9 / 5) + 32)

#define MAX_SET_TEMP_C   99
#define START_SET_TEMP_C 60
#define MIN_SET_TEMP_C   25
#define MAX_SET_TEMP_F   C_TO_F(MAX_SET_TEMP_C)
#define START_SET_TEMP_F C_TO_F(START_SET_TEMP_C)
#define MIN_SET_TEMP_F   C_TO_F(MIN_SET_TEMP_C)

// PID coefficients
#define OVERSHOOT (2.5 * 16 * 1.4) // 2.5C * "sensor unit conversion" * buffer
#define K_P ((float) 1 / OVERSHOOT) // P(t) = K_P * e(t), we want P(t) = 1 when e(t) = OVERSHOOT
#define K_I ((float) 1 / (3000 * 5.5)) // ~3000 integral at steady state
#define K_D 0.6

#define NUM_ERRORS 32

#define SENSOR_READING_HASNT_HAPPENED -32768 // -(1 << 15)

// create the 1-wire bus
OneWire ds(PIN_ONE_WIRE_BUS);

// create LED controller
LedControl mLedControl = LedControl(
    PIN_LED_DIN,
    PIN_LED_CLK,
    PIN_LED_CS,
    1); // 1 device

// consider byte?
int mState = STATE_SET_UNITS;

// error state, one of ERROR_*
int mErrorCode = 0;

boolean mDisplayInCelsius = true;

// the integer portion of the desired temperature, in the
// units according to mDisplayInCelsius
int mDesiredTempIntUserUnits = 0;

// the fraction portion of the desired temperature, in the
// units according to mDisplayInCelsius (eg, 9 --> 0.9)
byte mDesiredTempFracUserUnits = 0;

// the 2-byte desired temperature reading, in sensor scale,
// ie, in celsius, << 4 (or 16 times the desired celsius
// value), to be set in temperature setting state
int mSensorReadingDesired = 0;

// the 2-byte temperature reading from the sensor
int mSensorReadingActual = SENSOR_READING_HASNT_HAPPENED;

// current button reading, bouncy
int mButtonState[3];

// the debounced state
int mLastButtonState[3];

boolean mLastRelayState = false;

// the last time the state changed/bounced
long mLastDebounceTime[3];

// time since last button state went HIGH
long mButtonDownSince[3];

// we only ever read the first 9 bytes from the Scratchpad
byte mData[9];

// 8 byte address for devices on the 1-wire bus
byte mAddr[8];

// the last time a blink started
long mLastBlinkTime = 0;

// ring buffer of the latest integrals calculated at the start
// of each PID cycle
float mIntegral = 0;

// ring buffer of the latest error calculated at the start
// of each PID cycle
float mError[NUM_ERRORS];

// ring buffer of the latest times (captured with millis()) at
// which mError was updated
unsigned long mErrorTime[NUM_ERRORS];

// the index of the current error in mError
int mErrorIdx = 0;

// whether the PID cycle is the very first one, so that state
// can be initialized
boolean mFirstPidCycle = true;

// percentage of how long the relay should be on for during
// a single PID cycle, in the range [0, 1]
float mDuty;

void setup(void)
{
  Serial.begin(9600);
  
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);
  
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

  for (int i = 0; i < NUM_ERRORS; ++i)
  {
    mError[i] = 0;
    mErrorTime[i] = 0;
  }
}

void loop(void)
{
  // to avoid mixing display and logic, we'll always first
  // process input and state, then display
  mState = processInputs(mState);
  
  display(mState);
  
  // small delay to avoid a lot of unnecessary processing
  delay(5);
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
    case STATE_SET_TEMP_INT:
      return processSetTempInt();
    case STATE_SET_TEMP_FRAC:
      return processSetTempFrac();
    case STATE_READ_TEMP_START:
      return processReadTempStart();
    case STATE_READ_TEMP_WAIT:
      return processReadTempWait();
    case STATE_ERROR:
    default:
      return processError();
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
    // set the starting default temperature
    mDesiredTempIntUserUnits = mDisplayInCelsius
        ? START_SET_TEMP_C
        : START_SET_TEMP_F;

    mDesiredTempFracUserUnits = 0;
    
    Serial.print("units set as ");
    Serial.println(mDisplayInCelsius ? 'C' : 'F');
    
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
    onInput();
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
      PIN_BUTTON_SELECT,
      MASK_BUTTON_PRESSED))
  {

    Serial.print("Integer temp set as ");
    Serial.println(mDesiredTempIntUserUnits);
    
    // move on to setting the fractional part
    return STATE_SET_TEMP_FRAC;
  }
  
  if (checkButtonState(
      buttonState,
      PIN_BUTTON_LEFT,
      MASK_BUTTON_PRESSED | MASK_BUTTON_LONG_PRESSED))
  {
    // decrease temp
    changeSetTempInt(-1);
    onInput();
  }
  else if (checkButtonState(
      buttonState,
      PIN_BUTTON_RIGHT,
      MASK_BUTTON_PRESSED | MASK_BUTTON_LONG_PRESSED))
  {
    // increase temp
    changeSetTempInt(1);
    onInput();
  }
            
  return STATE_SET_TEMP_INT;
}

void changeSetTempInt(int amount)
{
  mDesiredTempIntUserUnits = constrain(
    mDesiredTempIntUserUnits + amount,
    mDisplayInCelsius ? MIN_SET_TEMP_C : MIN_SET_TEMP_F,
    mDisplayInCelsius ? MAX_SET_TEMP_C : MAX_SET_TEMP_F);
}

int processSetTempFrac()
{
  byte buttonState = readButtonState();
  
  if (checkButtonState(
      buttonState,
      PIN_BUTTON_SELECT,
      MASK_BUTTON_PRESSED))
  {
    Serial.print("Fraction temperature set as 0.");
    Serial.println(mDesiredTempFracUserUnits);
    
    // convert the set temperature into the "sensor reading scale",
    // ie, in Celsius and 16 larger.
    // move on to temperature controlling
    if (mDisplayInCelsius)
    {
      // make sure the fractional part rounds to the nearest 1/16
      // of a degree, rather than doing integer division (eg,
      // 85.5C * 16 = 1372.8, let's round that to 1373 in "sensor
      // reading scale")
      mSensorReadingDesired = (mDesiredTempIntUserUnits << 4)
          + round((mDesiredTempFracUserUnits << 4) / (float) 10);
          

    }
    else
    {
      //   tc = (tf - 32) * (5/9);
      // 16tc = 16((tf - 32) * (5/9))
      //      = (tf - 32) * (5 * 16) / 9
      //      = (tf - 32) * 80 / 9
      //      = (80tf - 2560) / 9
      // 
      // but tf = tf(int) + tf(frac)/10 (since we're keeping track
      //                                 of the fraction part as an
      //                                 integer)
      //
      // 16tc = (80(tf(int) + tf(frac)/10) - 2560) / 9
      //      = (80tf(int) + 8tf(frac) - 2560) / 9
      mSensorReadingDesired = round(
          ((80 * mDesiredTempIntUserUnits) +
          (8 * mDesiredTempFracUserUnits) -
          2560) / (float) 9);
    }
    
    if (DEBUG)
    {
      Serial.print("Desired: ");
      Serial.print(mDesiredTempIntUserUnits);
      Serial.print('.');
      Serial.print(mDesiredTempFracUserUnits);
      Serial.println(mDisplayInCelsius ? 'C' : 'F');
      Serial.print("16 * tc: ");
      Serial.println(mSensorReadingDesired);
      
      Serial.print("kp: ");
      Serial.println(K_P, 4);
      Serial.print("ki: ");
      Serial.println(K_I, 8);
      Serial.print("kd: ");
      Serial.println(K_D, 4);
    }
    
    // zero out the unit-less variables so we don't accidentally
    // use them
    mDesiredTempIntUserUnits = 0;
    mDesiredTempFracUserUnits = 0;

    Serial.println("Attempting to read temperature");
    return STATE_READ_TEMP_START;
  }
  
  if (checkButtonState(
      buttonState,
      PIN_BUTTON_LEFT,
      MASK_BUTTON_PRESSED | MASK_BUTTON_LONG_PRESSED))
  {
    // decrease temp (+9 is the same as -1, but avoiding any
    // unexpected negative issues with modulo)
    mDesiredTempFracUserUnits = (mDesiredTempFracUserUnits + 9) % 10;
    onInput();
  }
  else if (checkButtonState(
      buttonState,
      PIN_BUTTON_RIGHT,
      MASK_BUTTON_PRESSED | MASK_BUTTON_LONG_PRESSED))
  {
    // decrease temp (+9 is the same as -1, but avoiding any
    // unexpected negative issues with modulo)
    mDesiredTempFracUserUnits = (mDesiredTempFracUserUnits + 1) % 10;
    onInput();
  }
            
  return STATE_SET_TEMP_FRAC;
}

int processReadTempStart()
{
  updateRelayState();
  
  // find a single device, write its address to mAddr
  if (!findSingleDevice(mAddr))
  {
    Serial.println("Could not find a single device");
    return STATE_ERROR;
  }

  /*
  if (DEBUG)
  {
    // print out the device's unique address
    Serial.print("addr=");
    printHexBytes(mAddr, 8);
    Serial.println();
  }
  */
  
  // ensure the device is powered externally (otherwise we
  // cannot query the device to ask if it's done its convert 
  // T)
  if (!verifyExternallyPowered(mAddr))
  {
    Serial.println("Could not verify device externally powered");
    return STATE_ERROR;
  }

  // query the DS18B20
  startQuerySensor(mAddr);
  
  return STATE_READ_TEMP_WAIT;
}

int processReadTempWait()
{
  updateRelayState();
  
  int result = readSensorData(mAddr, mData);
  
  if (result == READ_SENSOR_PROCESSING)
  {
    return STATE_READ_TEMP_WAIT;
  }
  else if (result == READ_SENSOR_FAIL)
  {
    Serial.println("Error reading data from sensor");
    return STATE_ERROR;
  }
  
  /*
  // print out data from the scratchpad
  if (DEBUG)
  {
    Serial.print("data=");
    printHexBytes(mData, 9);
    Serial.println();
  }
  */
  
  // while it may seem like we want to do this before reading
  // the temperature, we have to do the read before we even
  // know the resolution
  if (!validateResolution(mAddr, mData))
  {
    Serial.println("Failed to validate resolution after convert T");
    return STATE_ERROR;
  }
  
  // first 2 bytes read are the temperature data, with the first
  // byte being the least significant bits
  mSensorReadingActual = (mData[1] << 8) | mData[0];
  
  // start a PID cycle
  return STATE_READ_TEMP_START;
}

void updateRelayState()
{
  // wait until we have an actual sensor reading before
  // modifying relay state
  if (mSensorReadingActual == SENSOR_READING_HASNT_HAPPENED)
  {
    return;
  }
  
  unsigned long now = millis();
  
  unsigned long cycleStart = mErrorTime[mErrorIdx];
  unsigned long cycleEnd = mErrorTime[mErrorIdx] + PERIOD_PID_CYCLE_MS;

  // handle wrap around of time
  unsigned long msSinceCycleStart = (cycleStart > cycleEnd)
        ? (MAX_UNSIGNED_LONG - cycleStart + now)
        : (now - cycleStart);

  // are we still in the middle of a cycle
  if (msSinceCycleStart < PERIOD_PID_CYCLE_MS)
  {
    // turn off if we've met our duty during this PID cycle
    
    float pcntThroughCycle =
        ((float) msSinceCycleStart) / PERIOD_PID_CYCLE_MS;
    
    if (pcntThroughCycle >= mDuty)
    {
      setRelayState(false);
    }
  }
  else
  {
    // time to start a new PID cycle!
    float newError = mSensorReadingDesired - mSensorReadingActual;
    
    if (mFirstPidCycle)
    {
      mFirstPidCycle = false;
      
      unsigned long lastTime = now - PERIOD_PID_CYCLE_MS;
      
      // pretend we've had a lot of PID cycles at this error,
      // so that our derivatives can be calculated with somewhat
      // reasonable numbers instead of 0
      for (int i = 0; i < NUM_ERRORS; ++i)
      {
        mError[i] = newError;
        mErrorTime[i] = lastTime;
      }
    }

    // increment the error index for mError and mErrorTime
    mErrorIdx = (mErrorIdx + 1) % NUM_ERRORS;

    // write the new error and error time
    mError[mErrorIdx] = newError;
    mErrorTime[mErrorIdx] = now;
    
    float proportionalTerm = K_P * mError[mErrorIdx];
    float integralTerm = 0.0;
    float derivativeTerm = 0.0;

    // only start controlling I and D terms if the
    // proportional is not pegging out at +/- 100% duty
    if (abs(proportionalTerm) < 1.0)
    {
      float derivative = getDerivative();

      // If derivative of the error is large and positive (ie, the
      // temperature is rapidly falling), pause the integral term.
      // There was an upset to the system (probably cold food being
      // added), and we pause the integral term to prevent over
      // reacting and triggering integral wind-up.
      if (derivative < 0.5)
      {
        // divide integral by 10,000 just to prevent dealing
        // with huge numbers, and then having to deal with
        // a very tiny K_I
        mIntegral += newError * (((float) msSinceCycleStart) / 10000);
      }

      integralTerm = K_I * mIntegral;
      derivativeTerm = K_D * derivative;
    }
    else
    {
      // prevent integral windup, zero out
      mIntegral = 0;
    }
    
    mDuty = proportionalTerm + integralTerm + derivativeTerm;
    mDuty = constrain(mDuty, 0, 1);

    Serial.print(now);
    Serial.print(',');
    Serial.print(mSensorReadingActual);
    Serial.print(',');
    Serial.print(mIntegral, 6);
    Serial.print(',');
    Serial.print(getDerivative(), 6);
    Serial.print(',');
    Serial.println(mDuty);
    
    // if there's a noticeable amount of duty, start the relay
    if (mDuty > 0.001)
    {
      setRelayState(true);
    }
  }
}

float getDerivative()
{
  int oldIdx = (mErrorIdx + 1) % NUM_ERRORS;
  
  unsigned long pastTime = mErrorTime[oldIdx];
  unsigned long now = mErrorTime[mErrorIdx];
  
  // time can wrap around after MAX_UNSIGNED_LONG, account for it
  unsigned long duration = (pastTime > now)
      ? MAX_UNSIGNED_LONG - pastTime + now
      : now - pastTime;
  
  // derivative = rise / run
  float dxdt = (mError[mErrorIdx] - mError[oldIdx]) / duration;

  // Scale it so that the derivative has a range of
  // roughly [-1, 1].
  // Through measurement under full duty for a while, it took
  // 1,116,759ms to change the temperature 10 degrees celsius,
  // or 160 (ie, 10 << 4) "sensor units" while under full duty.
  // So, max derivative is roughly 160 / 1,116,759. To scale
  // that to 1, divide by that number (or multiply by the
  // inverse).
  return dxdt * 1116759 / 160;
}

int processError()
{
  // turn off the relay because there's an error!
  setRelayState(false);
  return STATE_ERROR;
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
    case STATE_SET_TEMP_FRAC:
      displaySetTempFrac();
      break;
    case STATE_READ_TEMP_START:
      displayReadTempStart();
      break;
    case STATE_READ_TEMP_WAIT:
      displayReadTempWait();
      break;
    case STATE_ERROR:
      displayError();
      break;
    default:
      Serial.println("Unknown state, " + state);
      displayError();
      break;
  }
}

/**
 * Looks for a single device on the 1-wire bus. If found, its
 * address will be written into addr and true will be
 * returned, otherwise false will be returned, and the error
 * code willl be written.
 */
boolean findSingleDevice(byte *addr)
{
  // ensure we're looking for the first device on the 1-wire bus
  ds.reset_search();
  
  // Search for the first device on the 1-wire bus. If found, its
  // unique 8-byte address will be written into addr and true is
  // returned, otherwise false is returned.
  if (!ds.search(addr))
  {
    mErrorCode = ERROR_OWB_NO_DEVICES;
    return false;
  }
  
  if (ds.search(addr))
  {
    mErrorCode = ERROR_OWB_TOO_MANY_DEVICES;
    return false;
  }
  
  // verify valid address via CRC
  if (OneWire::crc8(addr, 7) != addr[7])
  {
    mErrorCode = ERROR_OWB_ADDRESS_CRC;
    return false;
  }
  
  if (addr[0] != 0x28)
  {
    if (DEBUG)
    {
      Serial.print("ERROR: Only 0x28 device families are ");
      Serial.print("supported, found 0x");
      Serial.println(addr[0], HEX);
    }
    mErrorCode = ERROR_OWB_NOT_28_FAMILY;
    return false;
  }
  
  return true;
}

/**
 * Returns true if the DS18B20 at the address specified is
 * externally powered (ie, not in parasitic power mode),
 * otherwise false.
 */
boolean verifyExternallyPowered(byte *addr)
{
  // issue a "read power supply" command
  ds.reset();
  ds.select(addr);
  ds.write(COMMAND_READ_POWER_SUPPLY);

  // parasite powered devices will pull the bus low, externally
  // powered devices will let the bus remain high
  if (ds.read_bit() == 0)
  {
    mErrorCode = ERROR_OWB_NOT_EXTERNALLY_POWERED;
    return false;
  }
  
  return true;
}

/**
 * Trigger a Convert T command (ie, get temperature and store
 * it on the DS18B20's Scratchpad memory).
 */
void startQuerySensor(byte *addr)
{
  ds.reset();
  ds.select(addr);
  ds.write(COMMAND_CONVERT_T);
}

/**
 * Attempts to read from the DS18B20's scratchpad.
 */
int readSensorData(byte *addr, byte *data)
{
  // During a Convert T with an externally powered DS18B20, the
  // device keeps the 1-wire bus low until it is done the convert.
  // This means that we don't have to delay an arbitrary amount
  // of time, instead we just keep polling until the bus goes
  // high. If we were using parasite power mode, we as the master
  // of the bus would have to keep the bus high to power the device
  // and must wait an arbitrary (but long enough) amount of time
  // for the convert to complete.
  
  // TODO reset and select addr?
  
  if (ds.read_bit() == 0)
  {
    // not done converting yet
    return READ_SENSOR_PROCESSING;
  }
  
  // done converting!
  
  // send read command and read the bytes
  ds.reset();
  ds.select(addr);
  ds.write(COMMAND_READ_SCRATCHPAD);
  ds.read_bytes(data, 9);
  
  // make sure it matches the CRC
  if (OneWire::crc8(data, 8) != data[8])
  {
    mErrorCode = ERROR_OWB_SCRATCHPAD_READ_CRC;
    return READ_SENSOR_FAIL;
  }
  return READ_SENSOR_SUCCESS;
}

/**
 * Validates that the temperature read is done so to a 12-bit
 * resolution (ie, down to 1/16th of a degree Celsius). If not,
 * resolution will be changed, but the sensor reading will not
 * change until the next Convert T.
 */
boolean validateResolution(byte *addr, byte *data)
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
        mErrorCode = ERROR_OWB_SCRATCHPAD_WRITE_CRC;
        return false;
      }
    }
    
    // finally, copy Scratchpad to EEPROM (ie, persistent storage)
    ds.reset();
    ds.select(addr);
    ds.write(COMMAND_COPY_SCRATCHPAD);
  }
  
  return true;
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
    byte pinNumber = i == 0
        ? PIN_BUTTON_LEFT
        : (i == 1 ? PIN_BUTTON_RIGHT : PIN_BUTTON_SELECT);

    reading = digitalRead(pinNumber);
    
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
 * Helper method to not call digitalWrite if the relay is
 * already set to the desired state.
 */
void setRelayState(boolean powered)
{
  if (mLastRelayState != powered)
  {
    mLastRelayState = powered;
    digitalWrite(PIN_RELAY, powered ? HIGH : LOW);
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
  int readingInScale = mDisplayInCelsius
      ? sensorReading
      : ((sensorReading * 9 / 5) + (32 << 4));

  // On the DS18B20, the 12th through the 16th bit are the sign
  // bits. We'll only check the 16th bit to see if it's negative,
  // ie, the binary number matches 1xxx xxxx xxxx xxxx.
  boolean isNegative = (readingInScale & 0x8000) != 0;
  
  if (isNegative)
  {
    // for 2's complement, flip all bits and add 1 to convert from
    // negative to postive
    readingInScale = (readingInScale ^ 0xffff) + 1;
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
  int wholeCelsius = readingInScale >> 4;
  
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
  int fractCelsius = ((readingInScale & 0x000F) * 100) >> 4;
  
  /*
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
  */
  
  int hundreds = wholeCelsius / 100;
  int tens = (wholeCelsius % 100) / 10;
  int ones = wholeCelsius % 10;
  int tenths = fractCelsius / 10;
  int hundreths = fractCelsius % 10;

  mLedControl.setRow(0, 7, 0); // blank

  mLedControl.setRow(0, 6,
      isNegative && hundreds != 0 ? LED_CHAR_MINUS : 0);
  
  // print hundreds of celsius if it's there
  if (hundreds != 0)
  {
    mLedControl.setDigit(0, 5, hundreds, false);
  }
  else
  {
    mLedControl.setRow(0, 5, isNegative ? LED_CHAR_MINUS : 0);
  }
  
  mLedControl.setDigit(0, 4, tens, false);
  mLedControl.setDigit(0, 3, ones, true);
  mLedControl.setDigit(0, 2, tenths, false);
  mLedControl.setDigit(0, 1, hundreths, false);
  mLedControl.setRow(0, 0,
      mDisplayInCelsius ? LED_CHAR_C : LED_CHAR_F);
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
  int hundreds = mDesiredTempIntUserUnits / 100;
  int tens = (mDesiredTempIntUserUnits % 100) / 10;
  int ones = mDesiredTempIntUserUnits % 10;
  
  // empty out the first 3 digits
  mLedControl.setRow(0, 7, 0);
  mLedControl.setRow(0, 6, 0);
  mLedControl.setRow(0, 5, 0);
  
  // blink the integer part of the temp, with a '.' at the end
  blinkDigit(4, hundreds, false);
  blinkDigit(3, tens, false);
  blinkDigit(2, ones, true);
  
  // don't blink the fraction, show C/F
  mLedControl.setDigit(0, 1, 0, false);
  mLedControl.setRow(0, 0,
      mDisplayInCelsius ? LED_CHAR_C : LED_CHAR_F);
}

void displaySetTempFrac()
{
  int hundreds = mDesiredTempIntUserUnits / 100;
  int tens = (mDesiredTempIntUserUnits % 100) / 10;
  int ones = mDesiredTempIntUserUnits % 10;
  
  // empty out the first 3 digits
  mLedControl.setRow(0, 7, 0);
  mLedControl.setRow(0, 6, 0);
  mLedControl.setRow(0, 5, 0);
  
  // show integer part (not blinking)
  if (hundreds != 0)
  {
    mLedControl.setDigit(0, 4, hundreds, false);
  }
  else
  {
    // blank
    mLedControl.setRow(0, 4, 0);
  }
  mLedControl.setDigit(0, 3, tens, false);
  mLedControl.setDigit(0, 2, ones, true);
  
  // show blinking fractoin
  blinkDigit(1, mDesiredTempFracUserUnits, false);
  
  // and finally the C/F
  mLedControl.setRow(0, 0,
      mDisplayInCelsius ? LED_CHAR_C : LED_CHAR_F);
}

void displayReadTempStart()
{
  showTemperature(mSensorReadingActual);
}

void displayReadTempWait()
{
  showTemperature(mSensorReadingActual);
}

void blinkDigit(int row, byte value, bool showPoint)
{
  if (shouldBlinkOn())
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
  if (shouldBlinkOn())
  {
    mLedControl.setRow(0, row, value);
  }
  else
  {
    // clear out digit (for blink effect)
    mLedControl.setRow(0, row, 0);
  }
}

boolean shouldBlinkOn()
{
  int offsetInPeriod = (millis() - mLastBlinkTime) % PERIOD_MS_FLASH;
  return offsetInPeriod == (offsetInPeriod % (PERIOD_MS_FLASH >> 1));
}

void onInput()
{
  mLastBlinkTime = millis();
}

void displayError()
{
  mLedControl.setRow(0, 7, LED_CHAR_E);
  mLedControl.setRow(0, 6, LED_CHAR_R);
  mLedControl.setRow(0, 5, LED_CHAR_R);
  mLedControl.setRow(0, 4, LED_CHAR_O);
  mLedControl.setRow(0, 3, LED_CHAR_R);
  mLedControl.setRow(0, 2, 0);
  mLedControl.setRow(0, 1, 0);
  mLedControl.setRow(0, 0, 0);
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
