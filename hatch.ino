#include <DHT.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Screen setup
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET 4        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Humidity + temperature sensor setup
#define DHTPIN 8      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);
float temperature = 0;
float humidity = 0;
unsigned long t_env_check = 0;
unsigned long t_0_env_check = 0;
int const env_check_frequency = 10000; // how often should the environment be checked

// Pins out
int const lights_pin = 4;            // Relay controlling the leds
int const hatch_open_relay_pin = 5;  // Relay controlling hatch in opening direction
int const hatch_close_relay_pin = 6; // Relay controlling hatch in closing direction

// Pins in
int const open_button_pin = 10;      // a button that opens/closes the hatch
int const overcurrent_pin = 11;      // a input from overcurrent protection board that indicates an error
int const emergency_button_pin = 12; // a button inside the cellar that will open the hatch

// Button states for state machine
enum button_states
{
  reset,     // one time, catch all condition
  start,     // checks value of button, if pressed moves on to `go`
  go,        // records the time of the press and moves on to wait
  wait,      // wait for the button to settle (debouncing)
  armed,     // makes sure trigger only happens once the button is released
  triggered, // one time pulse of button
};

// Constants
unsigned long const time_to_close = 90000; // 90s - how much time does the hatch take to close
int const bounce_delay = 10;               // tells how long should the button hold the HIGH value till its triggered (debounce time)

// Open button variables
int state_open_button = button_states::reset;
int state_prev_open_button = state_open_button;
int val_open_button = 0;
unsigned long t_open_button = 0;
unsigned long t_0_open_button = 0;

// Emergency button variables
int state_emergency_button = button_states::reset;
int state_prev_emergency_button = state_open_button;
int val_emergency_button = 0;
unsigned long t_emergency_button = 0;
unsigned long t_0_emergency_button = 0;

// Close signal timer variables
unsigned long t_closing = 0;   // holds new records of time after a closing trigger
unsigned long t_0_closing = 0; // holds initial time when the hatch started closing

// State variables
enum state
{
  closing,       // one time setter for closing the hatch
  closed,        // rest state for the hatch
  opening,       // one time setter for opening the hatch; its the default value
  moving,        // used after the closing and opening state
  closing_error, // one time trigger for overcurrent protection, transitions to opening
  opening_error, // one time trigger for overcurrent protection, transitions to closing
  rest,          // after closed waiting for a button press
};
int state_hatch = state::opening;
int prev_state_hatch = state_hatch;
int hatch_error = LOW; // if the overcurrent protection kicks in this should be turned to true
bool DEBUG = true;     // should the debugging be turned on

enum lights
{
  off = HIGH,
  on = LOW
};

enum actuator_direction
{
  open,
  close,
  stop
};

void setup()
{
  Serial.begin(115200);
  // Setting outputs
  pinMode(lights_pin, OUTPUT);
  pinMode(hatch_open_relay_pin, OUTPUT);
  pinMode(hatch_close_relay_pin, OUTPUT);

  // Setting inputs
  pinMode(open_button_pin, INPUT_PULLUP);
  pinMode(emergency_button_pin, INPUT_PULLUP);
  pinMode(overcurrent_pin, INPUT_PULLUP);

  dht.begin(); // starting humidity sensor

  // Screen related stuff
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setCursor(0, 20);
  display.setTextSize(3);
  display.setTextColor(WHITE);
};

void loop()
{
  update_signals();
  update_environment_readings();

  switch (state_hatch)
  {
  case state::closing:
    closing_hatch();
    break;
  case state::closed:
    closed_hatch();
    break;
  case state::opening:
    opening_hatch();
    break;
  case state::moving:
    moving_hatch();
    break;
  case state::closing_error:
    closing_error_hatch();
    break;
  case state::opening_error:
    opening_error_hatch();
    break;
  case state::rest:
    rest_hatch();
    break;
  }
};

/**
 * @brief State machine that debounces a button.
 * @param pin which pin is the button connected to and should be debounced
 * @param state_button a pointer to a value that stores a current state (`button_states`) of the button
 * @param val_button a pointer to a variable that stores a value (`HIGH/LOW`) of the button
 * @param t_0 a pointer to a time when button was pressed first
 * @param t a pointer to a variable that stores time for each loop of the program
 */
void sm_debounce_button(int pin, int *state_button, int *val_button, unsigned long *t_0, unsigned long *t)
{
  switch (*state_button)
  {
  case button_states::reset:
    *state_button = button_states::start;
    break;

  case button_states::start:
    *val_button = digitalRead(pin);
    if (*val_button == LOW)
    {
      *state_button = button_states::go;
    }
    break;

  case button_states::go:
    *t_0 = millis();
    *state_button = button_states::wait;
    break;

  case button_states::wait:
    *val_button = digitalRead(pin);
    *t = millis();
    if (*val_button == HIGH)
    {
      *state_button = button_states::reset;
    }
    if (*t - *t_0 > bounce_delay)
    {
      *state_button = button_states::armed;
    }
    break;

  case button_states::triggered:
    // debugging part
    char message[50];
    sprintf(message, "Button (pin: %d) was triggered", pin);
    debug(message);
    // actual code
    *state_button = button_states::reset;
    break;

  case button_states::armed:
    *val_button = digitalRead(pin);
    if (*val_button == HIGH)
    {
      *state_button = button_states::triggered;
    }
    break;
  }
};

void closing_hatch()
{
  prev_state_hatch = state_hatch;
  t_0_closing = millis();
  t_closing = t_0_closing;
  debug("Closing");
  control_actuator(actuator_direction::close);
  control_lights(lights::on);
  state_hatch = state::moving;
  debug("Moving");
  signal_state("Closing");
};

void opening_hatch()
{
  prev_state_hatch = state_hatch;
  debug("Opening");
  control_actuator(actuator_direction::open);
  control_lights(lights::on);
  state_hatch = state::moving;
  debug("Moving");
  signal_state("Opening");
};

void rest_hatch()
{
  if (state_open_button == button_states::triggered || state_emergency_button == button_states::triggered)
  {
    state_hatch = state::opening;
  };
}

void closed_hatch()
{
  prev_state_hatch = state_hatch;
  debug("Closed");
  control_actuator(actuator_direction::stop);
  control_lights(lights::off);
  state_hatch = state::rest;
  debug("Resting");
  signal_state("Closed");
};

void closing_error_hatch()
{
  debug("Closing error");
  state_hatch = state::opening;
  signal_state("Error closing");
};

void opening_error_hatch()
{
  prev_state_hatch = state_hatch;
  debug("Opening error");
  state_hatch = state::closing;
  signal_state("Error opening");
};

/*
This function is used for limitting unnecessary state changes.
Instead of setting closing/opening every arduino loop, i set the direction of the actuator
and light once and then check for the conditions in this function.
*/
void moving_hatch()
{
  // updating the time for each loop so the condition after works
  if (prev_state_hatch == state::closing)
  {
    t_closing = millis();
  }

  if (prev_state_hatch == state::closing && t_closing - t_0_closing >= time_to_close)
  {
    Serial.println("hatch closed signal");
    state_hatch = state::closed;
  }
  else if (hatch_error == HIGH)
  {
    if (state_hatch == state::closing)
    {
      state_hatch = state::closing_error;
    }
    else if (state_hatch == state::opening)
    {
      state_hatch = state::opening_error;
    }
  }
  else if (state_open_button == button_states::triggered || state_emergency_button == button_states::triggered)
  {
    if (prev_state_hatch == state::closing)
    {
      state_hatch = state::opening;
    }
    else if (prev_state_hatch == state::opening)
    {
      state_hatch = state::closing;
    }
  };
};

void update_signals()
{
  sm_debounce_button(open_button_pin, &state_open_button, &val_open_button, &t_0_open_button, &t_open_button);
  sm_debounce_button(emergency_button_pin, &state_emergency_button, &val_emergency_button, &t_0_emergency_button, &t_emergency_button);
  hatch_error = !digitalRead(overcurrent_pin);
};

void control_actuator(actuator_direction dir)
{
  switch (dir)
  {
  case actuator_direction::open:
    digitalWrite(hatch_open_relay_pin, HIGH);
    digitalWrite(hatch_close_relay_pin, LOW);
    break;
  case actuator_direction::close:
    digitalWrite(hatch_open_relay_pin, LOW);
    digitalWrite(hatch_close_relay_pin, HIGH);
    break;
  case actuator_direction::stop:
    digitalWrite(hatch_open_relay_pin, HIGH);
    digitalWrite(hatch_close_relay_pin, HIGH);
    break;
  default:
    break;
  }
}

// controlls the lights
void control_lights(lights io)
{
  digitalWrite(lights_pin, io);
}

void debug(String status)
{
  if (DEBUG)
  {
    Serial.print(status);
    Serial.print(" | Hatch state: ");
    Serial.print(state_hatch);
    Serial.print(" | Prev hatch state: ");
    Serial.println(prev_state_hatch);

    Serial.print(F("Humidity: "));
    Serial.print((int)humidity);
    Serial.print(F("%  Temperature: "));
    Serial.print((int)temperature);
    Serial.println(F("??C "));
  }
};

void update_environment_readings()
{
  if (t_env_check - t_0_env_check >= env_check_frequency)
  {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (!isnan(h))
    {
      humidity = h;
    }

    if (!isnan(t))
    {
      temperature = t;
    }

    display.clearDisplay();
    display.print((int)humidity);
    display.print("% ");
    display.print((int)temperature);
    display.println("C");
    display.display();

    t_0_env_check = millis();
    t_env_check = t_0_env_check;
  }
  else
  {
    t_env_check = millis();
  }
};

void signal_state(String state_name)
{
  // making sure the state is shown for the 10 ish seconds
  t_0_env_check = millis();
  t_env_check = t_0_env_check;
  display.clearDisplay();
  display.print(state_name);
  display.display();
}