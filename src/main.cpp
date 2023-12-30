#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

// Voltage configuration
#define V_pin A0
#define V_div 20.7333333333

// Resistance configuration
#define R_pin A1   // Analog pin measuring the voltage drop
#define R1_pin 2   // First resistor pin
#define R2_pin 3   // Second resistor pin
#define R3_pin 4   // Third resistor pin
#define R1_val 1206   // First resistor value (should be the smallest)
#define R2_val 15046   // Second resistor value (should be bettween of the other two resistors values)
#define R3_val 151e3   // Third resistor value (should be the largest)

// Current configuration
#define I_pin A2   // Analog pin measuring the voltage drop across the shunt resistor
#define I_R_ref 105e-3   // Shunt resistor value in Ohms

// Voltage references
#define V_low_ref 1.1f   // Low reference voltage (on atmega328 1.1V)
#define V_high_ref 4.95f   // High reference voltage (on usually atmega328 5V)

// Buttons
#define Button_1 5   // Button 1 pin (Mode: Voltage)
#define Button_2 6   // Button 2 pin (Mode: Resistance)
#define Button_3 7   // Button 3 pin (Mode: Current)
#define Button_4 8   // Button 4 pin (Mode: Diode test)

// Sound
#define Buzzer_pin 9   // Pin to which the buzzer is connected
#define SOUND_ON true   // Turn the sound on/off
#define BEEP_FREQUENCY 2000   // The frequency in Hz of the beep sound
#define BEEP_DURATION 150   // The duration of the beep sound in milliseconds

// Time settings
#define FRAME_TIME_MS 500   // Frame time for the LCD display in milliseconds (500 = 2 FPS)
#define MEASUREMENTS_DELAY_MS 100   // Delay between teaking measurements (100 = 10 measurements / second)
#define BUTTONS_DELAY_MS 20   // Delay between reading the buttons and updating the mode (20 = 50 updates / second)

#define MAX_STR_LEN 20   // Maximum length of a string eg. The mode title length


// Create the object of the LCD display
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);

// Ohm symbol for the resistance measurement
byte lcd_omega_symbol[8] = {
  B00000,
  B00000,
  B01110,
  B10001,
  B10001,
  B01010,
  B11011,
  B00000
};

// Symbol for filling the whole character space
byte lcd_fill_symbol[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};


// This type stores, formats, prints, and displays any given measurement
struct measurement_t {
  float value = 0;
  bool open_line = false;
  uint8_t decimal_places = 0;
  uint8_t symbol = 0;
  char prefix = ' ';
  char unit[MAX_STR_LEN + 1];
  char title[MAX_STR_LEN + 1];

  // Set the unit for the measurement
  void set_unit(const char *u) {
    for (uint8_t i = 0; i < MAX_STR_LEN; i++) {
      unit[i] = u[i];
      if (u[i] == '\0') break;
    }
  }

  // Set the title for the measurement
  void set_title(const char *u) {
    for (uint8_t i = 0; i < MAX_STR_LEN; i++) {
      title[i] = u[i];
      if (u[i] == '\0') break;
    }
  }

  // Display the current measurement to the LCD
  void display() {
    lcd.setCursor(0, 0);
    lcd.print("Mode: ");
    lcd.print(title);
    lcd.setCursor(0, 2);
    lcd.print("Value: ");
    if (open_line) {
      lcd.print("OPEN LINE   ");
    }
    else {
      lcd.print(value, decimal_places);
      lcd.print(" ");
      if (prefix) lcd.print(prefix);
      (symbol) ? lcd.write(symbol) : lcd.print(unit);
      lcd.print("      ");
    }
  }

  // Print the current measurement to the serial port
  void print() {
    if (open_line) {
      Serial.print("OPEN LINE");
    }
    else {
      Serial.print(value, decimal_places);
      Serial.print(" ");
      if (prefix) Serial.print(prefix);
      Serial.print(unit);
    }
    Serial.println();
  }

  // Format the currently stored value to make it easily readable
  void format() {
    if (value >= 1e6) {
      value /= 1e6f;
      decimal_places = 3;
      prefix = 'M';
    }
    else if (value >= 1e5) {
      value /= 1e3f;
      decimal_places = 1;
      prefix = 'k';
    }
    else if (value >= 1e4) {
      value /= 1e3f;
      decimal_places = 2;
      prefix = 'k';
    }
    else if (value >= 1e3) {
      value /= 1e3f;
      decimal_places = 3;
      prefix = 'k';
    }
    else if (value >= 1e2) {
      decimal_places = 1;
      prefix = 0;
    }
    else if (value >= 10) {
      decimal_places = 2;
      prefix = 0;
    }
    else if (value >= 1) {
      decimal_places = 3;
      prefix = 0;
    }
    else {
      value *= 1e3f;
      decimal_places = 0;
      prefix = 'm';
    }
  }
} voltage, resistance, current, diode_test;

// This type stores, and updates, the mode in which the multimeter is operating
struct mode_t {
  uint8_t id = 0;
  bool b1 = false;
  bool b2 = false;
  bool b3 = false;
  bool b4 = false;
  
  // Update the mode according to the buttons states
  bool update() {
    bool b1_read = digitalRead(Button_1) ? false : true;
    bool b2_read = digitalRead(Button_2) ? false : true;
    bool b3_read = digitalRead(Button_3) ? false : true;
    bool b4_read = digitalRead(Button_4) ? false : true;

    bool updated = false;

    if (b1_read != b1) {
      b1 = b1_read;
      if (b1_read) {
        id = 0;
        updated = true;
      }
    }
    else if (b2_read != b2) {
      b2 = b2_read;
      if (b2_read) {
        id = 1;
        updated = true;
      }
    }
    else if (b3_read != b3) {
      b3 = b3_read;
      if (b3_read) {
        id = 2;
        updated = true;
      }
    }
    else if (b4_read != b4) {
      b4 = b4_read;
      if (b4_read) {
        id = 3;
        updated = true;
      }
    }

    if (updated) {
      lcd.clear();
      // noTone(Buzzer_pin);
      if (SOUND_ON) tone(Buzzer_pin, BEEP_FREQUENCY, BEEP_DURATION);
    }

    return updated;
  }

} mode;

// Measure and calculate the voltage value
void update_voltage(struct measurement_t *m) {
  uint16_t v_raw = analogRead(V_pin);
  if (v_raw == 1023) {
    analogReference(DEFAULT);
    v_raw = analogRead(V_pin);
    analogReference(INTERNAL);
    m->value = (float)v_raw * V_high_ref * V_div / 1023.f;
  }
  else {
    m->value = (float)v_raw * V_low_ref * V_div / 1023.f;
  }
}

// Measure and calculate the resistance value
void update_resistance(struct measurement_t *m) {
  float V_ref = V_low_ref;
  
  pinMode(R1_pin, OUTPUT);
  pinMode(R2_pin, INPUT);
  pinMode(R3_pin, INPUT);
  digitalWrite(R1_pin, HIGH);
  float R_ref = R1_val;

  uint16_t v_raw = analogRead(R_pin);
  
  if (v_raw == 1023) {
    pinMode(R1_pin, INPUT);
    pinMode(R2_pin, OUTPUT);
    pinMode(R3_pin, INPUT);
    digitalWrite(R2_pin, HIGH);
    R_ref = R2_val;

    v_raw = analogRead(R_pin);
  
    if (v_raw == 1023) {
      
      pinMode(R1_pin, INPUT);
      pinMode(R2_pin, INPUT);
      pinMode(R3_pin, OUTPUT);
      digitalWrite(R3_pin, HIGH);
      R_ref = R3_val;

      v_raw = analogRead(R_pin);

      if (v_raw == 1023) {
        analogReference(DEFAULT);
        V_ref = V_high_ref;
        v_raw = analogRead(R_pin);
        analogReference(INTERNAL);
      }
    }
  }
  float v = (float)v_raw * V_ref / 1023.f;
  m->value = R_ref * v / (5.f - v);
  m->open_line = (v_raw == 1023) ? true : false;
}

// Measure and calculate the current value
void update_current(struct measurement_t *m) {
  uint16_t v_raw = analogRead(I_pin);
  m->value = (float)v_raw * V_low_ref / 1023.f / I_R_ref;
}

// Measure and calculate the voltage drop
void update_diode_test(struct measurement_t *m) {
  pinMode(R1_pin, OUTPUT);
  pinMode(R2_pin, INPUT);
  pinMode(R3_pin, INPUT);
  digitalWrite(R1_pin, HIGH);
  
  analogReference(DEFAULT);
  uint16_t v_raw = analogRead(R_pin);
  analogReference(INTERNAL);
  float v = (float)v_raw * V_high_ref / 1023.f;
  
  if (SOUND_ON) {
    (v <= 0.1) ? tone(Buzzer_pin, BEEP_FREQUENCY) : noTone(Buzzer_pin);
  }

  m->value = v;
  m->open_line = (v_raw == 1023) ? true : false;
}

// Display results with a specific frame time [EXECUTION_TIME: 53ms]
void display_hndl() {
  static uint64_t last_update = millis();
  if (millis() - last_update > FRAME_TIME_MS) {
    last_update = millis();
    switch (mode.id) {
      case 0:
        voltage.display();
        break;
      
      case 1:
        resistance.display();
        break;
      
      case 2:
        current.display();
        break;
      
      case 3:
        diode_test.display();
        break;
    }
  }
}

// Take the measurements with a specific interval [EXECUTION_TIME: 1.3ms]
void measurements_hndl() {
  static uint64_t last_update = millis();
  if (millis() - last_update > MEASUREMENTS_DELAY_MS) {
    last_update = millis();
    switch (mode.id) {
      case 0:
        update_voltage(&voltage);
        voltage.format();
        voltage.print();
        break;
      
      case 1:
        update_resistance(&resistance);
        resistance.format();
        resistance.print();
        break;
      
      case 2:
        update_current(&current);
        current.format();
        current.print();
        break;
      
      case 3:
        update_diode_test(&diode_test);
        diode_test.format();
        diode_test.print();
        break;
    }
  }
}

// Upgrade the selected mode with a specific interval[EXECUTION_TIME: 1.5ms]
void buttons_hndl() {
  static uint64_t last_update = millis();
  if (millis() - last_update > BUTTONS_DELAY_MS) {
    last_update = millis();
    mode.update();
  }
}

// Loading animation
void loading_animation() {
  tone(Buzzer_pin, BEEP_FREQUENCY, BEEP_DURATION);
  lcd.createChar(2, lcd_fill_symbol);
  for (int8_t col = 0; col < 10; col++) {
    lcd.setCursor(col, 0);
    lcd.write(2);
    lcd.setCursor(col, 1);
    lcd.write(2);
    lcd.setCursor(col, 2);
    lcd.write(2);
    lcd.setCursor(col, 3);
    lcd.write(2);
    lcd.setCursor(19-col, 0);
    lcd.write(2);
    lcd.setCursor(19-col, 1);
    lcd.write(2);
    lcd.setCursor(19-col, 2);
    lcd.write(2);
    lcd.setCursor(19-col, 3);
    lcd.write(2);
    delay(30);
  }
  lcd.clear();
  for (int8_t col = 9; col >= 0; col--) {
    lcd.setCursor(col, 0);
    lcd.write(2);
    lcd.setCursor(col, 1);
    lcd.write(2);
    lcd.setCursor(col, 2);
    lcd.write(2);
    lcd.setCursor(col, 3);
    lcd.write(2);
    lcd.setCursor(19-col, 0);
    lcd.write(2);
    lcd.setCursor(19-col, 1);
    lcd.write(2);
    lcd.setCursor(19-col, 2);
    lcd.write(2);
    lcd.setCursor(19-col, 3);
    lcd.write(2);
    delay(30);
  }
  lcd.clear();
}


// Set up serial communication, IO pins, measurement objects, and the lcd display
void setup()
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(V_pin, INPUT);
  pinMode(R_pin, INPUT);
  pinMode(I_pin, INPUT);
  pinMode(Button_1, INPUT_PULLUP);
  pinMode(Button_2, INPUT_PULLUP);
  pinMode(Button_3, INPUT_PULLUP);
  pinMode(Button_4, INPUT_PULLUP);

  analogReference(INTERNAL);

  voltage.set_unit("V");
  voltage.set_title("VOLTAGE   ");

  resistance.set_unit("Ohm");
  resistance.set_title("RESISTANCE");
  resistance.symbol = 1;

  current.set_unit("A");
  current.set_title("CURRENT   ");

  diode_test.set_unit("V");
  diode_test.set_title("Diode Test");

  lcd.init();
  lcd.backlight();
  lcd.createChar(1, lcd_omega_symbol);
  loading_animation();
}

// Call handle function for display, measurements, and buttons
void loop()
{
  display_hndl();
  measurements_hndl();
  buttons_hndl();
}
