// Pins out
int const lights_pin = 4;            // Relay controlling the leds
int const hatch_open_relay_pin = 5;  // Relay controlling hatch in opening direction
int const hatch_close_relay_pin = 6; // Relay controlling hatch in closing direction

// Pins in
int const open_button_pin = 10; // a button that opens/closes the hatch
int const overcurrent_pin = 11; // a input from overcurrent protection board that indicates an error

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

// Open button variables
int state_open_button = reset;
int state_prev_open_button = reset;
int val_open_button = 0;
unsigned long t_open_button = 0;
unsigned long t_0_open_button = 0;
unsigned long bounce_delay_open_button = 10;

// Close button variables
int state_close_button = reset;
int state_prev_close_button = reset;
int val_close_button = 0;
unsigned long t_close_button = 0;
unsigned long t_0_close_button = 0;
int const bounce_delay_close_button = 10;

// Close signal timer variables
unsigned long t_closing = 0;               // holds new records of time after a closing trigger
unsigned long t_0_closing = 0;             // holds initial time when the hatch started closing
unsigned long const time_to_close = 90000; // 90s - how much time does the hatch take to close

// State variables
enum state
{
  closing,       // one time setter for closing the hatch
  closed,        // rest state for the hatch
  opening,       // one time setter for opening the hatch; its the default value
  moving,        // used after the closing and opening state
  closing_error, // one time trigger for overcurrent protection, transitions to opening
  opening_error, // one time trigger for overcurrent protection, transitions to closing
  rest,          // After closed waiting for a button press
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
  pinMode(overcurrent_pin, INPUT_PULLUP);
};

void loop()
{
  update_signals();

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

void sm_open_button()
{
  switch (state_open_button)
  {
  case button_states::reset:
    state_open_button = button_states::start;
    break;

  case button_states::start:
    val_open_button = digitalRead(open_button_pin);
    if (val_open_button == LOW)
    {
      state_open_button = button_states::go;
    }
    break;

  case button_states::go:
    t_0_open_button = millis();
    state_open_button = button_states::wait;
    break;

  case button_states::wait:
    val_open_button = digitalRead(open_button_pin);
    t_open_button = millis();
    if (val_open_button == HIGH)
    {
      state_open_button = button_states::reset;
    }
    if (t_open_button - t_0_open_button > bounce_delay_open_button)
    {
      state_open_button = button_states::armed;
    }
    break;

  case button_states::triggered:
    debug("Triggered open button");
    state_open_button = button_states::reset;
    break;

  case button_states::armed:
    val_open_button = digitalRead(open_button_pin);
    if (val_open_button == HIGH)
    {
      state_open_button = button_states::triggered;
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
};

void opening_hatch()
{
  prev_state_hatch = state_hatch;
  debug("Opening");
  control_actuator(actuator_direction::open);
  control_lights(lights::on);
  state_hatch = state::moving;
  debug("Moving");
};

void rest_hatch()
{
  if (state_open_button == button_states::triggered)
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
};

void closing_error_hatch()
{
  debug("Closing error");
  state_hatch = state::opening;
};

void opening_error_hatch()
{
  prev_state_hatch = state_hatch;
  debug("Opening error");
  state_hatch = state::closing;
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

  if (prev_state_hatch == state::closing && t_closing - t_0_closing > time_to_close)
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
  else if (state_open_button == button_states::triggered)
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
  sm_open_button();
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
  }
};