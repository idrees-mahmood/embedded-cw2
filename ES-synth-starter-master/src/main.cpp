#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_rcc_ex.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_can.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

// Constants
const uint32_t interval = 100; // Display update interval
const uint32_t f0 = 440;       // Reference frequency (A4 = 440 Hz)
const uint32_t fs = 22000;     // Sample rate

const char *noteNames[12] = {"C", "C#", "D", "D#", "E", "F",
                             "F#", "G", "G#", "A", "A#", "B"};

// Base step sizes for octave 0
const uint32_t stepSizes[12] = {
    (uint32_t)((pow(2.0, (0.0 / 12.0)) * f0 / fs) * pow(2.0, 32)),  // C
    (uint32_t)((pow(2.0, (1.0 / 12.0)) * f0 / fs) * pow(2.0, 32)),  // C#
    (uint32_t)((pow(2.0, (2.0 / 12.0)) * f0 / fs) * pow(2.0, 32)),  // D
    (uint32_t)((pow(2.0, (3.0 / 12.0)) * f0 / fs) * pow(2.0, 32)),  // D#
    (uint32_t)((pow(2.0, (4.0 / 12.0)) * f0 / fs) * pow(2.0, 32)),  // E
    (uint32_t)((pow(2.0, (5.0 / 12.0)) * f0 / fs) * pow(2.0, 32)),  // F
    (uint32_t)((pow(2.0, (6.0 / 12.0)) * f0 / fs) * pow(2.0, 32)),  // F#
    (uint32_t)((pow(2.0, (7.0 / 12.0)) * f0 / fs) * pow(2.0, 32)),  // G
    (uint32_t)((pow(2.0, (8.0 / 12.0)) * f0 / fs) * pow(2.0, 32)),  // G#
    (uint32_t)((pow(2.0, (9.0 / 12.0)) * f0 / fs) * pow(2.0, 32)),  // A
    (uint32_t)((pow(2.0, (10.0 / 12.0)) * f0 / fs) * pow(2.0, 32)), // A#
    (uint32_t)((pow(2.0, (11.0 / 12.0)) * f0 / fs) * pow(2.0, 32))  // B
};


// Waveform Generation

enum Waveform
{
    SINE = 0,
    SAWTOOTH = 1,
    TRIANGLE = 2,
    SQUARE = 3,
    WAVEFORM_COUNT = 4
};

// default wave
volatile uint8_t currentWaveform = SAWTOOTH;

volatile int8_t octaveShift = 0;

const int8_t sineLUT[512] = {
    0, 1, 3, 4, 6, 7, 9, 10, 12, 13, 15, 17, 18, 20, 21, 23,
    24, 26, 27, 29, 30, 32, 33, 35, 36, 38, 39, 41, 42, 44, 45, 47,
    48, 50, 51, 52, 54, 55, 57, 58, 59, 61, 62, 63, 65, 66, 67, 69,
    70, 71, 73, 74, 75, 76, 78, 79, 80, 81, 82, 84, 85, 86, 87, 88,
    89, 90, 91, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 102, 103, 104,
    105, 106, 107, 108, 108, 109, 110, 111, 112, 112, 113, 114, 114, 115, 116, 116,
    117, 117, 118, 119, 119, 120, 120, 121, 121, 121, 122, 122, 123, 123, 123, 124,
    124, 124, 125, 125, 125, 125, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126,
    127, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 125, 125, 125, 125, 124,
    124, 124, 123, 123, 123, 122, 122, 121, 121, 121, 120, 120, 119, 119, 118, 117,
    117, 116, 116, 115, 114, 114, 113, 112, 112, 111, 110, 109, 108, 108, 107, 106,
    105, 104, 103, 102, 102, 101, 100, 99, 98, 97, 96, 95, 94, 93, 91, 90,
    89, 88, 87, 86, 85, 84, 82, 81, 80, 79, 78, 76, 75, 74, 73, 71,
    70, 69, 67, 66, 65, 63, 62, 61, 59, 58, 57, 55, 54, 52, 51, 50,
    48, 47, 45, 44, 42, 41, 39, 38, 36, 35, 33, 32, 30, 29, 27, 26,
    24, 23, 21, 20, 18, 17, 15, 13, 12, 10, 9, 7, 6, 4, 3, 1,
    0, -1, -3, -4, -6, -7, -9, -10, -12, -13, -15, -17, -18, -20, -21, -23,
    -24, -26, -27, -29, -30, -32, -33, -35, -36, -38, -39, -41, -42, -44, -45, -47,
    -48, -50, -51, -52, -54, -55, -57, -58, -59, -61, -62, -63, -65, -66, -67, -69,
    -70, -71, -73, -74, -75, -76, -78, -79, -80, -81, -82, -84, -85, -86, -87, -88,
    -89, -90, -91, -93, -94, -95, -96, -97, -98, -99, -100, -101, -102, -102, -103, -104,
    -105, -106, -107, -108, -108, -109, -110, -111, -112, -112, -113, -114, -114, -115, -116, -116,
    -117, -117, -118, -119, -119, -120, -120, -121, -121, -121, -122, -122, -123, -123, -123, -124,
    -124, -124, -125, -125, -125, -125, -126, -126, -126, -126, -126, -126, -126, -126, -126, -126,
    -127, -126, -126, -126, -126, -126, -126, -126, -126, -126, -126, -125, -125, -125, -125, -124,
    -124, -124, -123, -123, -123, -122, -122, -121, -121, -121, -120, -120, -119, -119, -118, -117,
    -117, -116, -116, -115, -114, -114, -113, -112, -112, -111, -110, -109, -108, -108, -107, -106,
    -105, -104, -103, -102, -102, -101, -100, -99, -98, -97, -96, -95, -94, -93, -91, -90,
    -89, -88, -87, -86, -85, -84, -82, -81, -80, -79, -78, -76, -75, -74, -73, -71,
    -70, -69, -67, -66, -65, -63, -62, -61, -59, -58, -57, -55, -54, -52, -51, -50,
    -48, -47, -45, -44, -42, -41, -39, -38, -36, -35, -33, -32, -30, -29, -27, -26,
    -24, -23, -21, -20, -18, -17, -15, -13, -12, -10, -9, -7, -6, -4, -3, -1,
};

const int8_t sawtoothLUT[512] = {
    -127, -126, -126, -125, -125, -124, -124, -123, -123, -122, -122, -121, -121, -120, -120, -119,
    -119, -118, -118, -117, -117, -116, -116, -115, -115, -114, -114, -113, -113, -112, -112, -111,
    -111, -110, -110, -109, -109, -108, -108, -107, -107, -106, -106, -105, -105, -104, -104, -103,
    -103, -102, -102, -101, -101, -100, -100, -99, -99, -98, -98, -97, -97, -96, -96, -95,
    -95, -94, -94, -93, -93, -92, -92, -91, -91, -90, -90, -89, -89, -88, -88, -87,
    -87, -86, -86, -85, -85, -84, -84, -83, -83, -82, -82, -81, -81, -80, -80, -79,
    -79, -78, -78, -77, -77, -76, -76, -75, -75, -74, -74, -73, -73, -72, -72, -71,
    -71, -70, -70, -69, -69, -68, -68, -67, -67, -66, -66, -65, -65, -64, -64, -63,
    -63, -62, -62, -61, -61, -60, -60, -59, -59, -58, -58, -57, -57, -56, -56, -55,
    -55, -54, -54, -53, -53, -52, -52, -51, -51, -50, -50, -49, -49, -48, -48, -47,
    -47, -46, -46, -45, -45, -44, -44, -43, -43, -42, -42, -42, -41, -41, -40, -40,
    -39, -39, -38, -38, -37, -37, -36, -36, -35, -35, -34, -34, -33, -33, -32, -32,
    -31, -31, -30, -30, -29, -29, -28, -28, -27, -27, -26, -26, -25, -25, -24, -24,
    -23, -23, -22, -22, -21, -21, -20, -20, -19, -19, -18, -18, -17, -17, -16, -16,
    -15, -15, -14, -14, -13, -13, -12, -12, -11, -11, -10, -10, -9, -9, -8, -8,
    -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, -1, 0, 0,
    0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7,
    8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15,
    16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23,
    24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31,
    32, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37, 38, 38, 39, 39,
    40, 40, 41, 41, 42, 42, 42, 43, 43, 44, 44, 45, 45, 46, 46, 47,
    47, 48, 48, 49, 49, 50, 50, 51, 51, 52, 52, 53, 53, 54, 54, 55,
    55, 56, 56, 57, 57, 58, 58, 59, 59, 60, 60, 61, 61, 62, 62, 63,
    63, 64, 64, 65, 65, 66, 66, 67, 67, 68, 68, 69, 69, 70, 70, 71,
    71, 72, 72, 73, 73, 74, 74, 75, 75, 76, 76, 77, 77, 78, 78, 79,
    79, 80, 80, 81, 81, 82, 82, 83, 83, 84, 84, 85, 85, 86, 86, 87,
    87, 88, 88, 89, 89, 90, 90, 91, 91, 92, 92, 93, 93, 94, 94, 95,
    95, 96, 96, 97, 97, 98, 98, 99, 99, 100, 100, 101, 101, 102, 102, 103,
    103, 104, 104, 105, 105, 106, 106, 107, 107, 108, 108, 109, 109, 110, 110, 111,
    111, 112, 112, 113, 113, 114, 114, 115, 115, 116, 116, 117, 117, 118, 118, 119,
    119, 120, 120, 121, 121, 122, 122, 123, 123, 124, 124, 125, 125, 126, 126, 127,
};

const int8_t triangleLUT[512] = {
    -127, -126, -125, -124, -123, -122, -121, -120, -119, -118, -117, -116, -115, -114, -113, -112,
    -111, -110, -109, -108, -107, -106, -105, -104, -103, -102, -101, -100, -99, -98, -97, -96,
    -95, -94, -93, -92, -91, -90, -89, -88, -87, -86, -85, -84, -83, -82, -81, -80,
    -79, -78, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64,
    -63, -62, -61, -60, -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48,
    -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32,
    -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16,
    -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
    32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
    48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
    64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
    80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95,
    96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
    112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127,
    127, 126, 125, 124, 123, 122, 121, 120, 119, 118, 117, 116, 115, 114, 113, 112,
    111, 110, 109, 108, 107, 106, 105, 104, 103, 102, 101, 100, 99, 98, 97, 96,
    95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 80,
    79, 78, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64,
    63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48,
    47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32,
    31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16,
    15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
    0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -15,
    -16, -17, -18, -19, -20, -21, -22, -23, -24, -25, -26, -27, -28, -29, -30, -31,
    -32, -33, -34, -35, -36, -37, -38, -39, -40, -41, -42, -43, -44, -45, -46, -47,
    -48, -49, -50, -51, -52, -53, -54, -55, -56, -57, -58, -59, -60, -61, -62, -63,
    -64, -65, -66, -67, -68, -69, -70, -71, -72, -73, -74, -75, -76, -77, -78, -79,
    -80, -81, -82, -83, -84, -85, -86, -87, -88, -89, -90, -91, -92, -93, -94, -95,
    -96, -97, -98, -99, -100, -101, -102, -103, -104, -105, -106, -107, -108, -109, -110, -111,
    -112, -113, -114, -115, -116, -117, -118, -119, -120, -121, -122, -123, -124, -125, -126, -127,
};

const int8_t squareLUT[256] = {
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127,
    -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127,
    -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127,
    -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127,
    -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127,
    -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127,
    -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127,
    -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127,
};


// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Global variable to store the notes
volatile uint16_t localActiveNotes = 0;
volatile uint16_t remoteActiveNotes = 0;

// Global volatile variable for volume control
// This will be accessed by both the ISR and the knob update task
volatile int8_t volumeControl = 0;
volatile int32_t pitchBend = 0;

// Timer object
HardwareTimer sampleTimer(TIM1);

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

void drawWaveform(uint8_t waveform)
{
    int xStart = 80, yStart = 10; // Position on OLED
    int width = 16, height = 8;   // Waveform drawing size
    int yMid = yStart + height / 2;

    for (int x = 0; x < width; x++)
    {
        float angle = (x * 2.0f * PI) / width; // Map to 0 - 360°
        int y = yMid;                          // Default position

        switch (waveform)
        {
        case 0: // Sine Wave
            y = yMid - (sin(angle) * (height / 2));
            break;

        case 1: // Sawtooth Wave
            y = yStart + (height * x) / width;
            break;

        case 2: // Triangle Wave
            y = yStart + (height * abs((2 * x / (float)width) - 1));
            break;

        case 3: // Square Wave
            y = (x < width / 2) ? yStart : yStart + height;
            break;
        }

        u8g2.drawPixel(xStart + x, y);
    }
}

/*------------------------------------------------------------------------------------------*/

// Knob class definition for managing rotary encoders
class Knob
{
private:
    uint8_t rowIndex;  // Row where the knob signals are connected
    uint8_t colA;      // Column for A signal
    uint8_t colB;      // Column for B signal
    uint8_t prevState; // Previous state of the quadrature inputs
    int8_t value;      // Current rotation value
    int8_t minValue;   // Minimum allowed value
    int8_t maxValue;   // Maximum allowed value

    // Lookup table for rotation direction based on state transitions
    static const int8_t rotationTable[4][4];

public:
    // Constructor
    Knob(uint8_t row, uint8_t colA, uint8_t colB, int8_t initialValue = 0,
         int8_t min = -127, int8_t max = 127)
        : rowIndex(row), colA(colA), colB(colB), prevState(0),
          value(initialValue), minValue(min), maxValue(max) {}

    // Update rotation value using latest inputs
    bool update(std::bitset<4> columnReadings)
    {
        uint8_t currentState = (columnReadings[colB] << 1) | columnReadings[colA];
        int8_t rotationChange = rotationTable[prevState][currentState];

        prevState = currentState;

        if (rotationChange != 0)
        {
            // Apply change with bounds checking
            int8_t newValue = value + rotationChange;
            if (newValue >= minValue && newValue <= maxValue)
            {
                value = newValue;
                // Serial.println("Knob " + rowIndex);
                // Serial.println("Value: " + value);

                return true; // Value was changed
            }
        }
        return false; // No change
    }

    // Get current row index
    uint8_t getRow() const
    {
        return rowIndex;
    }

    // Set upper and lower limits
    void setLimits(int8_t min, int8_t max)
    {
        minValue = min;
        maxValue = max;

        // Ensure current value is within new limits
        if (value < minValue)
            value = minValue;
        if (value > maxValue)
            value = maxValue;
    }

    // Read current rotation value
    int8_t getValue() const
    {
        return value;
    }

    // Set current rotation value (with bounds checking)
    void setValue(int8_t newValue)
    {
        if (newValue >= minValue && newValue <= maxValue)
        {
            value = newValue;
        }
    }
};

// Initialize the static rotation lookup table
const int8_t Knob::rotationTable[4][4] = {
    {0, 1, -1, 0},
    {-1, 0, 0, 1},
    {1, 0, 0, -1},
    {0, -1, 1, 0}};

// Threading
struct
{
    SemaphoreHandle_t mutex;
    QueueHandle_t txQueue;
    QueueHandle_t rxQueue;
    SemaphoreHandle_t CAN_TX_Semaphore;
    Knob knobs[4] = {
        Knob(3, 0, 1, 0, 0, 7),                  // Knob 3: Row 3, cols 0,1, volume control (0-7)
        Knob(3, 2, 3, 0, 0, WAVEFORM_COUNT - 1), // Knob 2: Row 3, cols 2,3,
        Knob(4, 0, 1, 0, -4, 3),                 // Knob 1: Row 4, cols 0,1, unused
        Knob(4, 2, 3, 0, 0, 15)                  // Knob 0: Row 4, cols 2,3, unused
    };
    uint8_t RX_Message[8]; //
} sysState;

// Task handles
TaskHandle_t scanKeysHandle = NULL;
TaskHandle_t displayUpdateHandle = NULL;
TaskHandle_t decodeTaskHandle = NULL;

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
    digitalWrite(REN_PIN, LOW);
    digitalWrite(RA0_PIN, bitIdx & 0x01);
    digitalWrite(RA1_PIN, bitIdx & 0x02);
    digitalWrite(RA2_PIN, bitIdx & 0x04);
    digitalWrite(OUT_PIN, value);
    digitalWrite(REN_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(REN_PIN, LOW);
}

std::bitset<4> readCols()
{
    std::bitset<4> result;
    result[0] = digitalRead(C0_PIN);
    result[1] = digitalRead(C1_PIN);
    result[2] = digitalRead(C2_PIN);
    result[3] = digitalRead(C3_PIN);
    return result;
}

void setRow(uint8_t rowIdx)
{
    // Disable row select
    digitalWrite(REN_PIN, LOW);

    // Set row select address
    digitalWrite(RA0_PIN, rowIdx & 0x01);
    digitalWrite(RA1_PIN, rowIdx & 0x02);
    digitalWrite(RA2_PIN, rowIdx & 0x04);

    // Enable row select
    digitalWrite(REN_PIN, HIGH);
}

/*------------------------------ISRs-------------------------------------------*/

void sampleISR()
{
    static uint32_t phaseAcc[12] = {0}; // Phase accumulator, static (stores value between calls)
    uint16_t localActive = __atomic_load_n(&localActiveNotes, __ATOMIC_RELAXED);
    uint16_t remoteActive = __atomic_load_n(&remoteActiveNotes, __ATOMIC_RELAXED);
    uint16_t allActive = localActive | remoteActive;

    int32_t pitch = __atomic_load_n(&pitchBend, __ATOMIC_RELAXED);
    uint8_t waveform = __atomic_load_n(&currentWaveform, __ATOMIC_RELAXED);
    int8_t octave = __atomic_load_n(&octaveShift, __ATOMIC_RELAXED);

    int32_t Vout = 0;
    int numActive = __builtin_popcount(allActive);

    // Serial.println(pitch);
    for (int i = 0; i < 12; i++)
    {
        if (allActive & (1 << i))
        {
            // Apply octave shift by multiplying step size by 2^octave
            // This effectively doubles/halves the frequency for each octave up/down
            uint32_t octaveAdjustedStepSize;
            if (octave >= 0) {
                octaveAdjustedStepSize = stepSizes[i] << octave; // Shift left for positive octaves
            } else {
                octaveAdjustedStepSize = stepSizes[i] >> (-octave); // Shift right for negative octaves
            }
            
            phaseAcc[i] += octaveAdjustedStepSize + pitch;

            // Use the phase accumulator to index into the appropriate waveform LUT
            uint8_t index = (phaseAcc[i] >> 24) & 0xFF; // Use top 8 bits as index

            switch (waveform)
            {
            case SINE:
                Vout += sineLUT[index];
                break;
            case SAWTOOTH:
                Vout += sawtoothLUT[index];
                break;
            case TRIANGLE:
                Vout += triangleLUT[index];
                break;
            case SQUARE:
                Vout += squareLUT[index];
                break;
            default:
                Vout += sawtoothLUT[index]; // Default to sawtooth
            }
        }
    }

    if (numActive > 0)
    {
        Vout /= numActive;
    }
    else
    {
        Vout = 0;
    }

    // Use the global volumeControl variable - atomic and ISR-safe
    int8_t volume = __atomic_load_n(&volumeControl, __ATOMIC_RELAXED);

    // Apply volume control by bit-shifting (with bounds checking)
    if (volume > 0 && volume < 8)
    {
        Vout = Vout >> (8 - volume);
    }
    else
    {
        Vout = 0;
    }

    // Clamp output to 8-bit range
    Vout = (Vout > 127) ? 127 : (Vout < -128) ? -128
                                              : Vout;

    analogReadResolution(12);
    analogWrite(OUTR_PIN, Vout + 128);
    analogWrite(OUTL_PIN, Vout + 128); // Add output to left channel too
}

void CAN_RX_ISR()
{
    uint32_t ID;
    uint8_t RX_Message_ISR[8];
    while (CAN_CheckRXLevel())
    {
        CAN_RX(ID, RX_Message_ISR);
        // l.println("CAN RX ISR triggered");
        xQueueSendFromISR(sysState.rxQueue, RX_Message_ISR, NULL);
    }
}

void CAN_TX_ISR()
{
    xSemaphoreGiveFromISR(sysState.CAN_TX_Semaphore, NULL);
}

/*---------------------------------------TASKS--------------------------------------*/
// Display update task
void displayUpdateTask(void *pvParameters)
{
    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t localRxMessage[8];

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        int8_t volume = __atomic_load_n(&volumeControl, __ATOMIC_RELAXED);
        uint8_t waveform = __atomic_load_n(&currentWaveform, __ATOMIC_RELAXED);
        int8_t octave = __atomic_load_n(&octaveShift, __ATOMIC_RELAXED);

        if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE)
        {
            memcpy(localRxMessage, sysState.RX_Message, 8);
            xSemaphoreGive(sysState.mutex);
        }

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);

        // Display pressed key
        u8g2.drawStr(2, 10, "Key: ");
        u8g2.setCursor(30, 10);
        u8g2.print(noteNames[localRxMessage[2]]);

        // Display Volume
        u8g2.drawStr(2, 20, "Volume: ");
        u8g2.setCursor(50, 20);
        u8g2.print(volume);

        u8g2.drawStr(2, 30, "Oct: ");
        u8g2.setCursor(30, 30);
        u8g2.print(octave+4);

        // Draw Waveform (0-360 degrees)
        drawWaveform(waveform);

        // Display RX Message
        u8g2.setCursor(80, 30);
        if (localRxMessage[0] != 0)
        {
            u8g2.print((char)localRxMessage[0]);
            u8g2.print(localRxMessage[1]);
            u8g2.print(localRxMessage[2]);
        }
        else
        {
            u8g2.print("---");
        }

        u8g2.sendBuffer();
        digitalToggle(LED_BUILTIN);
    }
}

// Key scanning task
void scanKeysTask(void *pvParameters)
{
    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t txMsg[8] = {0};
    static uint16_t prevLocalActiveNotes = 0;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint16_t newLocalActiveNotes = 0;
        int lastKeyPressed = -1; // default: no key pressed

        // First scan knob rows (rows 3 and 4)
        for (int k = 0; k < 4; k++)
        {
            // Take mutex before accessing the knob
            if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE)
            {
                uint8_t row = sysState.knobs[k].getRow();
                xSemaphoreGive(sysState.mutex);

                setRow(row);
                delayMicroseconds(3);
                std::bitset<4> colState = readCols();

                // Update knob with latest readings
                if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE)
                {
                    bool changed = sysState.knobs[k].update(colState);

                    // If this is the volume knob (knob 0) and it changed, update volume immediately
                    if (k == 0 && changed)
                    {
                        int8_t newVolume = sysState.knobs[0].getValue();
                        xSemaphoreGive(sysState.mutex);

                        // Update the atomic volume control variable
                        __atomic_store_n(&volumeControl, newVolume, __ATOMIC_RELAXED);
                    }
                    else if (k == 1 && changed)
                    {
                        int8_t waveSelection = sysState.knobs[1].getValue();
                        xSemaphoreGive(sysState.mutex);

                        uint8_t newWaveform = sysState.knobs[1].getValue();

                        __atomic_store_n(&currentWaveform, newWaveform, __ATOMIC_RELAXED);
                    }
                    else if (k == 2 && changed)
                    {
                        // This is the octave shift knob
                        int8_t newOctave = sysState.knobs[2].getValue();
                        xSemaphoreGive(sysState.mutex);

                        // Update the atomic octave shift variable
                        __atomic_store_n(&octaveShift, newOctave, __ATOMIC_RELAXED);
                    }
                    else
                    {
                        xSemaphoreGive(sysState.mutex);
                    }
                }
            }
        }

        // Now scan piano key rows (rows 0-2)
        for (uint8_t row = 0; row < 3; row++)
        {
            setRow(row);
            delayMicroseconds(3);
            std::bitset<4> result = readCols();

            for (uint8_t col = 0; col < 4; col++)
            {
                int keyIndex = row * 4 + col;
                if (keyIndex < 12 && !result[col])
                { // Key pressed
                    newLocalActiveNotes |= (1 << keyIndex);
                }
            }
        }

        int32_t freqOffset = 0;
        // Scan joystick to apply pitch bend
        int joyX = analogRead(JOYX_PIN);
        // joyX reads min 18 max 1024
        // deadzone is around 477 so set deadzone to 400-550
        int joyY = analogRead(JOYY_PIN);

        // Serial.println(joyX);
        //  Apply pitch bend to the notes
        if (joyX > 550 || joyX < 400)
        {
            freqOffset = map(joyX, 0, 1024, 3000000, -3000000);
        }
        else
        {
            freqOffset = 0;
        }
        // Serial.println("Freq offset: ");
        // Serial.println(freqOffset);

        __atomic_store_n(&pitchBend, freqOffset, __ATOMIC_RELAXED);

        // Detect note changes and send CAN messages
        uint16_t pressed = newLocalActiveNotes & ~prevLocalActiveNotes;
        uint16_t released = prevLocalActiveNotes & ~newLocalActiveNotes;

        for (int i = 0; i < 12; i++)
        {
            if (pressed & (1 << i))
            {
                txMsg[0] = 'P';
                txMsg[1] = 4;
                txMsg[2] = i;
                xQueueSend(sysState.txQueue, txMsg, 0);
            }
            if (released & (1 << i))
            {
                txMsg[0] = 'R';
                txMsg[1] = 4;
                txMsg[2] = i;
                xQueueSend(sysState.txQueue, txMsg, 0);
            }
        }

        prevLocalActiveNotes = newLocalActiveNotes;
        __atomic_store_n(&localActiveNotes, newLocalActiveNotes, __ATOMIC_RELAXED);
    }
}

void decodeTask(void *pvParameters)
{
    uint8_t rxMsg[8];

    while (1)
    {
        if (xQueueReceive(sysState.rxQueue, rxMsg, portMAX_DELAY) == pdTRUE)
        {
            if (rxMsg[0] == 'P')
            {
                uint8_t note = rxMsg[2];
                __atomic_fetch_or(&remoteActiveNotes, (1 << note), __ATOMIC_RELAXED);
            }
            else if (rxMsg[0] == 'R')
            {
                uint8_t note = rxMsg[2];
                __atomic_fetch_and(&remoteActiveNotes, ~(1 << note), __ATOMIC_RELAXED);
            }

            // Update RX_Message for display
            if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE)
            {
                memcpy(sysState.RX_Message, rxMsg, 8);
                xSemaphoreGive(sysState.mutex);
            }
        }
    }
}

void canTxTask(void *pvParameters)
{
    uint8_t txMsg[8];

    while (1)
    {
        // Wait for a message from the TX queue
        xQueueReceive(sysState.txQueue, txMsg, portMAX_DELAY);
        xSemaphoreTake(sysState.CAN_TX_Semaphore, portMAX_DELAY);
        CAN_TX(0x123, txMsg);
    }
}
/*----------------------------THE REST--------------------------------------------*/

void setup()
{
    // Set pin directions
    pinMode(RA0_PIN, OUTPUT);
    pinMode(RA1_PIN, OUTPUT);
    pinMode(RA2_PIN, OUTPUT);
    pinMode(REN_PIN, OUTPUT);
    pinMode(OUT_PIN, OUTPUT);
    pinMode(OUTL_PIN, OUTPUT);
    pinMode(OUTR_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(C0_PIN, INPUT);
    pinMode(C1_PIN, INPUT);
    pinMode(C2_PIN, INPUT);
    pinMode(C3_PIN, INPUT);
    pinMode(JOYX_PIN, INPUT);
    pinMode(JOYY_PIN, INPUT);

    // Initialise display
    setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
    delayMicroseconds(2);
    setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
    u8g2.begin();
    setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

    // Initialise UART
    Serial.begin(9600);
    Serial.println("Synthesizer Started");

    // Initialize the keyboard
    setOutMuxBit(HKOE_BIT, HIGH); // Enable keyboard output

    // Init CAN bus
    CAN_Init(); // true for loopback, false for normal
    CAN_RegisterRX_ISR(CAN_RX_ISR);
    CAN_RegisterTX_ISR(CAN_TX_ISR);
    setCANFilter(0x123, 0x7ff);
    CAN_Start();

    // Create queues
    sysState.txQueue = xQueueCreate(100, 8); // Queue for outgoing messages
    sysState.rxQueue = xQueueCreate(100, 8); // Queue for incoming messages

    // Create semaphore
    sysState.CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);

    if (sysState.txQueue == NULL || sysState.rxQueue == NULL || sysState.CAN_TX_Semaphore == NULL)
    {
        Serial.println("ERROR: CAN Queue or semaphore creation failed");
        while (1)
            ;
    }

    // Create mutex
    sysState.mutex = xSemaphoreCreateMutex();
    if (sysState.mutex == NULL)
    {
        Serial.println("ERROR: Mutex creation failed");
        while (1)
            ;
    }

    // Initialize volume control with default value
    __atomic_store_n(&volumeControl, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&currentWaveform, SAWTOOTH, __ATOMIC_RELAXED);
    __atomic_store_n(&octaveShift, 0, __ATOMIC_RELAXED);

    // Timer and interrupt set up - do this AFTER initializing volume control
    sampleTimer.setOverflow(22000, HERTZ_FORMAT);
    sampleTimer.attachInterrupt(sampleISR);
    sampleTimer.resume();

    // Create tasks BEFORE starting the scheduler
    xTaskCreate(
        scanKeysTask,     // Function to run
        "scanKeys",       // Task name
        128,              // Stack size in words
        NULL,             // Parameters
        2,                // Priority
        &scanKeysHandle); // Task handle

    xTaskCreate(
        displayUpdateTask,     // Function to run
        "displayUpdate",       // Task name
        256,                   // Stack size in words
        NULL,                  // Parameters
        1,                     // Priority
        &displayUpdateHandle); // Task handle

    xTaskCreate(
        decodeTask,         // Function to run
        "decode",           // Task name
        128,                // Stack size in words
        NULL,               // Parameters
        1,                  // Priority
        &decodeTaskHandle); // Task handle

    xTaskCreate(
        canTxTask, // Function to run
        "canTx",   // Task name
        128,       // Stack size in words
        NULL,      // Parameters
        1,         // Priority
        NULL);     // Task handle

    // Start the scheduler
    Serial.println("Starting FreeRTOS scheduler");
    vTaskStartScheduler();

    // Code should never reach here if scheduler starts properly
    Serial.println("ERROR: FreeRTOS scheduler failed to start");

    while (1)
        ; // Infinite loop if scheduler fails
}

void loop()
{
    // Not used with FreeRTOS
}