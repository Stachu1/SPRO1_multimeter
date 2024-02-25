# SPRO1_multimeter
Software for a multimeter made as a first semester project in my engineering studies


## Download
Clone the repo
```bash
git clone https://github.com/Stachu1/SPRO1_multimeter.git
```


## Configuration
```C
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
```


## Schematics
<img width="900" alt="image" src="https://github.com/Stachu1/SPRO1_multimeter/assets/77758413/52c87763-49a5-4e10-9209-e11495bb89a6">


## PCB
<img width="900" alt="image" src="https://github.com/Stachu1/SPRO1_multimeter/assets/77758413/2b567e13-5d82-4654-9d8e-e2bb8ccac86f">

## Casing example
<img width="450" alt="image" src="https://github.com/Stachu1/SPRO1_multimeter/assets/77758413/2afa8b8b-ef3e-4fee-be7c-0f29740d36dc">
