// Robot Race 2023 - Tyler Hoover & Peter Faben


/*
 * Simple3piLineFollower - demo code for the Pololu 3pi Robot
 * 
 * This code will follow a black line on a white background, using a
 * very simple algorithm.  It demonstrates auto-calibration and use of
 * the 3pi IR sensors, motor control, bar graphs using custom
 * characters, and music playback, making it a good starting point for
 * developing your own more competitive line follower.
 *
 * http://www.pololu.com/docs/0J21
 * http://www.pololu.com
 * http://forum.pololu.com
 *
 */

// The 3pi include file must be at the beginning of any program that
// uses the Pololu AVR library and 3pi.  Pololu3pi.h includes all of the
// other Orangutan Arduino libraries that can be used to control the
// on-board hardware such as LCD, buzzer, and motor drivers.
#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLEDs.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>
#include <avr/pgmspace.h>

Pololu3pi robot;
unsigned int sensors[5]; // an array to hold sensor values
int last_proportional;

// I added this - Tyler
int last_position;
int sensor_count = 5;
int modded_sensor_count = 3;

int integral;
// Other array for our sensors
unsigned int n_sensors[5];



// Introductory messages.  The "PROGMEM" identifier causes the data to
// go into program space.
const char welcome_line1[] PROGMEM = " Pololu";
const char welcome_line2[] PROGMEM = "3\xf7 Robot";
const char demo_name_line1[] PROGMEM = "Line";
const char demo_name_line2[] PROGMEM = "follower";

// A couple of simple tunes, stored in program space.
const char welcome[] PROGMEM = ">g32>>c32";
const char go[] PROGMEM = "L16 cdegreg4";

// Data for generating the characters used in load_custom_characters
// and display_readings.  By reading levels[] starting at various
// offsets, we can generate all of the 7 extra characters needed for a
// bargraph.  This is also stored in program space.
const char levels[] PROGMEM = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

unsigned int get_custom_line() {
  bool onLine = false;
  uint32_t avg = 0; // this is for the weighted total
  uint16_t sum = 0; // this is for the denominator, which is <= 64000

  robot.readLine(n_sensors, IR_EMITTERS_ON);

  //avg = (n_sensors[0] * 0) + (n_sensors[1] * 1000) + (n_sensors[2] * 2000) + (n_sensors[3] * 3000) + (n_sensors[4] * 4000);
  //sum = (n_sensors[0] + n_sensors[1] + n_sensors[2] + n_sensors[3] + n_sensors[4]);


  if (n_sensors[0] > n_sensors[1] && n_sensors[2] > n_sensors[1]) {
    n_sensors[0] = 0;
  }

  if (n_sensors[4] > n_sensors[3] && n_sensors[2] > n_sensors[3]) {
    n_sensors[4] = 0;
  }

  for (uint8_t i = 0; i < sensor_count; i++)
  {
    uint16_t value = n_sensors[i];

    // keep track of whether we see the line at all
    if (value > 200) { onLine = true; }

    // only average in values that are above a noise threshold
    if (value > 50)
    {
      avg += (uint32_t)value * (i * 1000);
      sum += value;
    }
  }

  if (!onLine)
  {
    // If it last read to the left of center, return 0.
    if (last_position < (sensor_count - 1) * 1000 / 2)
    {
      return 0;
    }
    // If it last read to the right of center, return the max.
    else
    {
      return (sensor_count - 1) * 1000;
    }
  }
  last_position = avg/sum;
  return last_position;
}

//unsigned int get_custom_line() {
//  robot.readCalibrated(n_sensors, IR_EMITTERS_ON);
//  uint32_t avg = 0;
//  for(int i = 0; i < 5; ++i) {
//    avg += ((i * 1000)*n_sensors[i]);
//    sum += (i * 1000);
//  }
//  return ((n_sensors[0] * 0) + (n_sensors[1] * 1000) + (n_sensors[2] * 2000) + (n_sensors[3] * 3000) + (n_sensors[4] * 4000)) / (n_sensors[0] + n_sensors[1] + n_sensors[2] + n_sensors[3] + n_sensors[4]);
//}

// This function loads custom characters into the LCD.  Up to 8
// characters can be loaded; we use them for 7 levels of a bar graph.
void load_custom_characters()
{
  OrangutanLCD::loadCustomCharacter(levels + 0, 0); // no offset, e.g. one bar
  OrangutanLCD::loadCustomCharacter(levels + 1, 1); // two bars
  OrangutanLCD::loadCustomCharacter(levels + 2, 2); // etc...
  OrangutanLCD::loadCustomCharacter(levels + 3, 3);
  OrangutanLCD::loadCustomCharacter(levels + 4, 4);
  OrangutanLCD::loadCustomCharacter(levels + 5, 5);
  OrangutanLCD::loadCustomCharacter(levels + 6, 6);
  OrangutanLCD::clear(); // the LCD must be cleared for the characters to take effect
}

// This function displays the sensor readings using a bar graph.
void display_readings(const unsigned int *calibrated_values)
{
  unsigned char i;

  for (i=0;i<5;i++) {
    // Initialize the array of characters that we will use for the
    // graph.  Using the space, an extra copy of the one-bar
    // character, and character 255 (a full black box), we get 10
    // characters in the array.
    const char display_characters[10] = { ' ', 0, 0, 1, 2, 3, 4, 5, 6, (char)255 };

    // The variable c will have values from 0 to 9, since
    // calibrated values are in the range of 0 to 1000, and
    // 1000/101 is 9 with integer math.
    char c = display_characters[calibrated_values[i] / 101];

    // Display the bar graph character.
    OrangutanLCD::print(c);
  }
}

// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.  This function is automatically called
// by the Arduino framework at the start of program execution.
void setup()
{
  unsigned int counter; // used as a simple timer

  // This must be called at the beginning of 3pi code, to set up the
  // sensors.  We use a value of 2000 for the timeout, which
  // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
  robot.init(2000);

  load_custom_characters(); // load the custom characters

  // Play welcome music and display a message
  OrangutanLCD::printFromProgramSpace(welcome_line1);
  OrangutanLCD::gotoXY(0, 1);
  OrangutanLCD::printFromProgramSpace(welcome_line2);
  OrangutanBuzzer::playFromProgramSpace(welcome);
  delay(1000);

  OrangutanLCD::clear();
  OrangutanLCD::printFromProgramSpace(demo_name_line1);
  OrangutanLCD::gotoXY(0, 1);
  OrangutanLCD::printFromProgramSpace(demo_name_line2);
  delay(1000);

  // Display battery voltage and wait for button press
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    int bat = OrangutanAnalog::readBatteryMillivolts();

    OrangutanLCD::clear();
    OrangutanLCD::print(bat);
    OrangutanLCD::print("mV");
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::print("Press B");

    delay(100);
  }

  // Always wait for the button to be released so that 3pi doesn't
  // start moving until your hand is away from it.
  OrangutanPushbuttons::waitForRelease(BUTTON_B);
  delay(1000);

  // Auto-calibration: turn right and left while calibrating the
  // sensors.
  for (counter=0; counter<80; counter++)
  {
    if (counter < 20 || counter >= 60)
      OrangutanMotors::setSpeeds(40, -40);
    else
      OrangutanMotors::setSpeeds(-40, 40);

    // This function records a set of sensor readings and keeps
    // track of the minimum and maximum values encountered.  The
    // IR_EMITTERS_ON argument means that the IR LEDs will be
    // turned on during the reading, which is usually what you
    // want.
    robot.calibrateLineSensors(IR_EMITTERS_ON);

    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(20);
  }
  OrangutanMotors::setSpeeds(0, 0);

  // Display calibrated values as a bar graph.
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    // Read the sensor values and get the position measurement.
    //unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
    unsigned int position = get_custom_line();

    // Display the position measurement, which will go from 0
    // (when the leftmost sensor is over the line) to 4000 (when
    // the rightmost sensor is over the line) on the 3pi, along
    // with a bar graph of the sensor readings.  This allows you
    // to make sure the robot is ready to go.
    OrangutanLCD::clear();
    OrangutanLCD::print(position);
    OrangutanLCD::gotoXY(0, 1);
    display_readings(sensors);

    delay(100);
  }
  OrangutanPushbuttons::waitForRelease(BUTTON_B);

  OrangutanLCD::clear();

  OrangutanLCD::print("Go!");    

  // Play music and wait for it to finish before we start driving.
  OrangutanBuzzer::playFromProgramSpace(go);
  while(OrangutanBuzzer::isPlaying());
}

// The main function.  This function is repeatedly called by
// the Arduino framework.
void loop()
{
// Get the position of the line.  Note that we *must* provide
// the "sensors" argument to read_line() here, even though we
// are not interested in the individual sensor readings.
//unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
unsigned int position = get_custom_line();


// The "proportional" term should be 0 when we are on the line.
int proportional = ((int)position) - 2000;
 
// Compute the derivative (change) and integral (sum) of the
// position.
int derivative = proportional - last_proportional;
integral += proportional;
 
// Remember the last position.
last_proportional = proportional;
// Compute the difference between the two motor power settings,
// m1 - m2.  If this is a positive number the robot will turn
// to the right.  If it is a negative number, the robot will
// turn to the left, and the magnitude of the number determines
// the sharpness of the turn.
int power_difference = proportional/15 + integral/10000 + derivative*3/2;
 
// Compute the actual motor settings.  We never set either motor
// to a negative value.

OrangutanLCD::clear();
OrangutanLCD::print(position);

const int max_param = 80;

int max_mod = abs(proportional/20);
int max_ceiling = max_param/2;
if (max_mod > max_ceiling) max_mod = max_ceiling;
else if (max_mod < 0) max_mod = 0;

const int max = max_param - max_mod;
if(power_difference > max)
    power_difference = max;
if(power_difference < -max)
    power_difference = -max;

if (abs(power_difference) < 10) power_difference = 0;
 
if(power_difference < 0)
    // Turn left
    OrangutanMotors::setSpeeds(max+(1.75*power_difference), max);
else
  // Turn right
    OrangutanMotors::setSpeeds(max, max-(1.75*power_difference));
}
