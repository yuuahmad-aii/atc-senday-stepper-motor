#include <Arduino.h>
#include "AccelStepper.h"

// emergency stop
bool emergency = false;
byte keadaanEmergencyTerakhir = LOW;
unsigned long lastTimeButtonStateChanged = 0;

// pin input untuk proximity (ATC)
// #define INPUT0 PB9  // proxy 1
// #define INPUT1 PB8  // proxy 2
// #define INPUT2 PA8  // proxy 3
// #define INPUT3 PB10 // proxy 4
// #define INPUT4 PB5  // proxy 5
// #define INPUT5 PB4  // proxy 6
// #define INPUT6 PB3  // emergency stop
const int inputPins[6] = {PB9, PB8, PA8, PB10, PB5, PB4}; // Replace with your input pin numbers

// pinout untuk relay hidrolik
#define OUTPUT0 PB14         // PIN UNTUK LOCK
#define OUTPUT1 PA15         // PIN UNTUK UNLOC
#define stepPin PC13         // PIN CW (step )
#define dirPin PB3           // PIN CCW (dir)
#define enablePin PC14       // PIN CCW (dir)
#define motorInterfaceType 1 // artinya menggunakan stepper driver

// motor interface
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
bool motorRun = false;            // nilainya false di awal program
bool selesaiMotorRunning = true;  // nilainya true di awal program
bool motorRunPertama = true;      // nilainya true di awal program
bool initialMotorRunning = false; // nilainya false di awal program
// bool toolsSudahPas = false;

// Parsing
static char line[40];
String dt[8];
int i;
bool parsing = false;
uint8_t action = 0;
static size_t buffer_pos = 0;

// data kirim ke pc
int data[] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup()
{
    // pinMode(INPUT0, INPUT_PULLUP);
    // pinMode(INPUT1, INPUT_PULLUP);
    // pinMode(INPUT2, INPUT_PULLUP);
    // pinMode(INPUT3, INPUT_PULLUP);
    // pinMode(INPUT4, INPUT_PULLUP);
    // pinMode(INPUT5, INPUT_PULLUP);
    for (int i = 0; i < 6; i++)
        pinMode(inputPins[i], INPUT_PULLUP);

    pinMode(PA3, INPUT_PULLUP); // EMERGENCY

    pinMode(OUTPUT0, OUTPUT);
    pinMode(OUTPUT1, OUTPUT);

    Serial.begin(9600); // Start serial communication
    stepper.setMaxSpeed(2000);
    stepper.setEnablePin(enablePin);
    // stepper.step(800.0);

    digitalWriteFast(digitalPinToPinName(enablePin), HIGH);
    stepper.stop();
    motorRun = false;
    // stepper.setCurrentPosition(-100); // kompensasi posisi awal tidak pas (tidak dgunakan karena tidak berdampak)
    digitalWriteFast(digitalPinToPinName(OUTPUT0), HIGH);
    digitalWriteFast(digitalPinToPinName(OUTPUT1), LOW);
    stepper.setCurrentPosition(0);
}

uint8_t Parsing_data()
{
    uint8_t char_counter = 0;
    char letter = 0;
    float value = 0.0;
    uint8_t int_value = 0;
    while (line[char_counter] != 0) // Loop until no more g-code words in line.
    {
        letter = line[char_counter];
        char_counter++;
        int_value = trunc(value);
        switch (letter)
        {
        case 'L': // LOCK ATC
            // stepper.disableOutputs();
            digitalWriteFast(digitalPinToPinName(enablePin), HIGH);
            stepper.stop();
            motorRun = false;
            digitalWriteFast(digitalPinToPinName(OUTPUT0), HIGH);
            digitalWriteFast(digitalPinToPinName(OUTPUT1), LOW);
            break;
        case 'U': // UNLOCK ATC
            // stepper.disableOutputs();
            digitalWriteFast(digitalPinToPinName(enablePin), HIGH);
            stepper.stop();
            motorRun = false;
            digitalWriteFast(digitalPinToPinName(OUTPUT0), LOW);
            digitalWriteFast(digitalPinToPinName(OUTPUT1), HIGH);
            break;
        case 'A': // BERPUTAR CW
            // stepper.enableOutputs();
            digitalWriteFast(digitalPinToPinName(enablePin), LOW);
            // stepper.setSpeed(1000);
            motorRun = true;
            digitalWriteFast(digitalPinToPinName(OUTPUT0), LOW);
            digitalWriteFast(digitalPinToPinName(OUTPUT1), HIGH);
            break;
        case 'B': // BERPUTAR CCW
            // stepper.enableOutputs();
            digitalWriteFast(digitalPinToPinName(enablePin), LOW);
            // stepper.setSpeed(1000);
            motorRun = true;
            digitalWriteFast(digitalPinToPinName(OUTPUT0), LOW);
            digitalWriteFast(digitalPinToPinName(OUTPUT1), HIGH);
            break;
        case 'C': // BERHENTI BERPUTAR
            // stepper.disableOutputs();
            digitalWriteFast(digitalPinToPinName(enablePin), LOW);
            stepper.stop();
            motorRun = false;
            // toolsSudahPas = true;
            digitalWriteFast(digitalPinToPinName(OUTPUT0), LOW);
            digitalWriteFast(digitalPinToPinName(OUTPUT1), HIGH);
            break;
        }
    } // complete parsing
    return 1;
}

void parsing_function()
{
    if (Serial.available() > 0)
    {
        char c = Serial.read();
        c = toUpperCase(c);
        if (c == '\n')
        {                            // end of message
            line[buffer_pos] = '\0'; // terminate the string
            parsing = true;
            if (parsing)
            {
                Parsing_data();
                action = 0;
                parsing = false;
                buffer_pos = 0; // reset for next message
            }
        }
        else if (buffer_pos < sizeof line - 1)
        {
            line[buffer_pos++] = c; // buffer the character
        }
    }
}

void loop_orient()
{
    // baca status pin input, kemudian kirim data ke PC
    Serial.print("P");
    for (int i = 0; i < 6; i++)
    {
        data[i] = !digitalReadFast(digitalPinToPinName(inputPins[i]));
        Serial.print(data[i]);
    }
    Serial.println("");
}

void loop()
{
    if (millis() - lastTimeButtonStateChanged > 500)
    {
        byte keadaanEmergency = digitalRead(PA3);
        if (keadaanEmergency != keadaanEmergencyTerakhir)
        {
            lastTimeButtonStateChanged = millis();
            keadaanEmergencyTerakhir = keadaanEmergency;
            // artinya tertrigger (karena emergency default low)
            keadaanEmergency == HIGH ? emergency = true : emergency = false;
        }
    }

    if (emergency)
    {
        digitalWriteFast(digitalPinToPinName(OUTPUT0), HIGH);
        digitalWriteFast(digitalPinToPinName(OUTPUT1), LOW);
    }
    else
    {
        parsing_function();
        loop_orient();
    }

    if (motorRun && motorRunPertama)
        initialMotorRunning = true;
    else if (motorRun && (stepper.currentPosition() == 800 || stepper.currentPosition() == 1000))
    {
        stepper.setCurrentPosition(0);
        selesaiMotorRunning = false;
    }

    if (initialMotorRunning)
    {
        stepper.setSpeed(1000);
        stepper.moveTo(1000);
        stepper.runSpeedToPosition();
        selesaiMotorRunning = true;
        motorRunPertama = false;
        initialMotorRunning = false;
    }
    else if (!selesaiMotorRunning)
    {
        stepper.setSpeed(1000);
        stepper.moveTo(800);
        stepper.runSpeedToPosition();
        selesaiMotorRunning = true;
    }
}