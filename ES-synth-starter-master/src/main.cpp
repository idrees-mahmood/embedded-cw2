#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

// STM32 HAL includes
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_rcc_ex.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_can.h"

// Project-specific includes
#include "waveform.h"

// Debug flags (comment out to disable)
#define TEST_SCAN_KEYS
#define TEST_DISPLAY
#define TEST_DECODE
#define TEST_CAN_TX

//----------------------
// Hardware configuration
//----------------------

// Pin definitions
struct Pins
{
    // Row select and enable
    static const int RA0 = D3;
    static const int RA1 = D6;
    static const int RA2 = D12;
    static const int REN = A5;

    // Matrix input and output
    static const int C0 = A2;
    static const int C1 = D9;
    static const int C2 = A6;
    static const int C3 = D1;
    static const int OUT = D11;

    // Audio output
    static const int OUTL = A4;
    static const int OUTR = A3;

    // Joystick input
    static const int JOYY = A0;
    static const int JOYX = A1;
};

// Multiplexer control bits
struct MuxBits
{
    static const int DEN = 3;  // Display enable
    static const int DRST = 4; // Display reset
    static const int HKOW = 5; // Handshake output west
    static const int HKOE = 6; // Handshake output east
};

// Timer and Display objects
HardwareTimer sampleTimer(TIM1);
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

//------------------
// Synthesizer state
//------------------

// Note management
volatile uint16_t localActiveNotes = 0;
volatile uint16_t remoteActiveNotes = 0;

// Sound control
volatile int8_t volumeControl = 0;
volatile int32_t pitchBend = 0;
volatile int8_t localOctaveShift = 0;
volatile int8_t remoteOctaveShift = 0;

//---------------------
// Performance monitoring
//---------------------
#ifdef TEST_SCAN_KEYS
volatile uint32_t scanKeysInitTime = 0;
volatile uint32_t scanKeysExecTimeTotal = 0;
volatile uint32_t scanKeysExecTimeMax = 0;
volatile uint32_t scanKeysExecCount = 0;
#endif

#ifdef TEST_DISPLAY
volatile uint32_t displayInitTime = 0;
volatile uint32_t displayExecTimeTotal = 0;
volatile uint32_t displayExecTimeMax = 0;
volatile uint32_t displayExecCount = 0;
#endif

#ifdef TEST_DECODE
volatile uint32_t decodeInitTime = 0;
volatile uint32_t decodeExecTimeTotal = 0;
volatile uint32_t decodeExecTimeMax = 0;
volatile uint32_t decodeExecCount = 0;
#endif

#ifdef TEST_CAN_TX
volatile uint32_t canTxInitTime = 0;
volatile uint32_t canTxExecTimeTotal = 0;
volatile uint32_t canTxExecTimeMax = 0;
volatile uint32_t canTxExecCount = 0;
#endif

volatile uint32_t schedulerStartTime = 0;

// Task handles
TaskHandle_t scanKeysHandle = NULL;
TaskHandle_t displayUpdateHandle = NULL;
TaskHandle_t decodeTaskHandle = NULL;

// Constants
const uint32_t interval = 100; // Display update interval

// CAN message format
struct CANMsg
{
    static const uint8_t MSG_SIZE = 8;
    static const uint32_t ID = 0x123;

    // Message byte indices
    static const uint8_t TYPE = 0;   // Message type (P/R)
    static const uint8_t OCTAVE = 1; // Octave number (0-7)
    static const uint8_t NOTE = 2;   // Note number (0-11)

    // Message types
    static const char PRESS = 'P';
    static const char RELEASE = 'R';
};

/*------------------------------------------------------------------------------------------*/
// Knob configuration
struct KnobConfig
{
    static const uint8_t VOLUME = 0;   // Volume control (0-7)
    static const uint8_t WAVEFORM = 1; // Waveform selection (0-3)
    static const uint8_t OCTAVE = 2;   // Octave control (0-7)
    static const uint8_t UNUSED = 3;   // Unused knob
};

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

    // Accumulator for rotation movement
    int8_t accumulator;
    // Threshold to trigger an actual value change
    static const int8_t ROTATION_THRESHOLD = 2;

    // Lookup table for rotation direction based on state transitions
    static const int8_t rotationTable[4][4];

public:
    // Constructor
    Knob(uint8_t row, uint8_t colA, uint8_t colB, int8_t initialValue = 0,
         int8_t min = -127, int8_t max = 127)
        : rowIndex(row), colA(colA), colB(colB), prevState(0),
          value(initialValue), minValue(min), maxValue(max), accumulator(0) {}

    // Update rotation value using latest inputs
    bool update(std::bitset<4> columnReadings)
    {
        uint8_t currentState = (columnReadings[colB] << 1) | columnReadings[colA];
        int8_t rotationChange = rotationTable[prevState][currentState];

        prevState = currentState;

        if (rotationChange != 0)
        {
            // Add rotation to accumulator
            accumulator += rotationChange;

            // Check if accumulator exceeds threshold (in either direction)
            if (accumulator >= ROTATION_THRESHOLD)
            {
                // Apply positive change with bounds checking
                int8_t newValue = value + 1;
                if (newValue <= maxValue)
                {
                    value = newValue;
                    // Reset accumulator, but keep remainder for smoother feel
                    accumulator -= ROTATION_THRESHOLD;
                    return true; // Value was changed
                }
                else
                {
                    // Clamp at max
                    accumulator = ROTATION_THRESHOLD - 1;
                }
            }
            else if (accumulator <= -ROTATION_THRESHOLD)
            {
                // Apply negative change with bounds checking
                int8_t newValue = value - 1;
                if (newValue >= minValue)
                {
                    value = newValue;
                    // Reset accumulator, but keep remainder for smoother feel
                    accumulator += ROTATION_THRESHOLD;
                    return true; // Value was changed
                }
                else
                {
                    // Clamp at min
                    accumulator = -(ROTATION_THRESHOLD - 1);
                }
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
            // Reset accumulator when setting value directly
            accumulator = 0;
        }
    }
};

// Initialize the static rotation lookup table
const int8_t Knob::rotationTable[4][4] = {
    {0, 1, -1, 0},
    {-1, 0, 0, 1},
    {1, 0, 0, -1},
    {0, -1, 1, 0}};

// Threading state and resources
struct SystemState
{
    SemaphoreHandle_t mutex;
    QueueHandle_t txQueue;
    QueueHandle_t rxQueue;
    SemaphoreHandle_t CAN_TX_Semaphore;
    Knob knobs[4] = {
        Knob(3, 0, 1, 0, 0, 7),                                                 // Volume knob
        Knob(3, 2, 3, 0, 0, static_cast<int8_t>(Waveform::WAVEFORM_COUNT) - 1), // Waveform knob
        Knob(4, 0, 1, 0, -3, 3),                                                 // Octave knob
        Knob(4, 2, 3, 0, 0, 15)                                                 // Unused knob
    };
    uint8_t RX_Message[8];
} sysState;

// Function to draw waveform visualization on display
void drawWaveform(uint8_t waveform)
{
    int xStart = 80, yStart = 10; // Position on OLED
    int width = 16, height = 8;   // Waveform drawing size
    int yMid = yStart + height / 2;

    for (int x = 0; x < width; x++)
    {
        float angle = (x * 2.0f * PI) / width; // Map to 0 - 360°
        int y = yMid;                          // Default position

        switch (static_cast<Waveform>(waveform))
        {
        case Waveform::SINE:
            y = yMid - (sin(angle) * (height / 2));
            break;
        case Waveform::SAWTOOTH:
            y = yStart + (height * x) / width;
            break;
        case Waveform::TRIANGLE:
            y = yStart + (height * abs((2 * x / (float)width) - 1));
            break;
        case Waveform::SQUARE:
            y = (x < width / 2) ? yStart : yStart + height;
            break;
        default:
            y = yMid; // Default to center position
            break;
        }

        u8g2.drawPixel(xStart + x, y);
    }
}

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
    digitalWrite(Pins::REN, LOW);
    digitalWrite(Pins::RA0, bitIdx & 0x01);
    digitalWrite(Pins::RA1, bitIdx & 0x02);
    digitalWrite(Pins::RA2, bitIdx & 0x04);
    digitalWrite(Pins::OUT, value);
    digitalWrite(Pins::REN, HIGH);
    delayMicroseconds(2);
    digitalWrite(Pins::REN, LOW);
}

// Read columns from the matrix
std::bitset<4> readCols()
{
    std::bitset<4> result;
    result[0] = digitalRead(Pins::C0);
    result[1] = digitalRead(Pins::C1);
    result[2] = digitalRead(Pins::C2);
    result[3] = digitalRead(Pins::C3);
    return result;
}

// Set the active row for scanning
void setRow(uint8_t rowIdx)
{
    // Disable row select
    digitalWrite(Pins::REN, LOW);

    // Set row select address
    digitalWrite(Pins::RA0, rowIdx & 0x01);
    digitalWrite(Pins::RA1, rowIdx & 0x02);
    digitalWrite(Pins::RA2, rowIdx & 0x04);

    // Enable row select
    digitalWrite(Pins::REN, HIGH);
}

/*------------------------------ISRs-------------------------------------------*/

// Audio sample generation ISR
void sampleISR()
{
    static uint32_t phaseAcc[12] = {0}; // Phase accumulator
    uint16_t localActive = __atomic_load_n(&localActiveNotes, __ATOMIC_RELAXED);
    uint16_t remoteActive = __atomic_load_n(&remoteActiveNotes, __ATOMIC_RELAXED);
    uint16_t allActive = localActive | remoteActive;

    int32_t pitch = __atomic_load_n(&pitchBend, __ATOMIC_RELAXED);
    int8_t localOctave = __atomic_load_n(&localOctaveShift, __ATOMIC_RELAXED);
    int8_t remoteOctave = __atomic_load_n(&remoteOctaveShift, __ATOMIC_RELAXED);
    Waveform waveform = static_cast<Waveform>(__atomic_load_n(&currentWaveform, __ATOMIC_RELAXED));

    int32_t Vout = 0;
    int numActive = __builtin_popcount(allActive);

    for (int i = 0; i < 12; i++)
    {
        // Process local notes with local octave
        if (localActive & (1 << i))
        {
            // Apply local octave shift
            uint32_t octaveAdjustedStepSize;
            if (localOctave >= 0)
            {
                octaveAdjustedStepSize = stepSizes[i] << localOctave;
            }
            else
            {
                octaveAdjustedStepSize = stepSizes[i] >> (-localOctave);
            }

            phaseAcc[i] += octaveAdjustedStepSize + pitch;
            uint8_t index = (phaseAcc[i] >> 24) & 0xFF;
            Vout += getWaveformSample(waveform, index);
        }
        // Process remote notes with remote octave
        else if (remoteActive & (1 << i))
        {
            // Apply remote octave shift
            uint32_t octaveAdjustedStepSize;
            if (remoteOctave >= 0)
            {
                octaveAdjustedStepSize = stepSizes[i] << remoteOctave;
            }
            else
            {
                octaveAdjustedStepSize = stepSizes[i] >> (-remoteOctave);
            }

            phaseAcc[i] += octaveAdjustedStepSize + pitch;
            uint8_t index = (phaseAcc[i] >> 24) & 0xFF;
            Vout += getWaveformSample(waveform, index);
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
    analogWrite(Pins::OUTR, Vout + 128);
    analogWrite(Pins::OUTL, Vout + 128); // Add output to left channel too
}

// CAN RX interrupt handler
void CAN_RX_ISR()
{
    uint32_t ID;
    uint8_t RX_Message_ISR[8];
    while (CAN_CheckRXLevel())
    {
        CAN_RX(ID, RX_Message_ISR);
        xQueueSendFromISR(sysState.rxQueue, RX_Message_ISR, NULL);
    }
}

// CAN TX interrupt handler
void CAN_TX_ISR()
{
    xSemaphoreGiveFromISR(sysState.CAN_TX_Semaphore, NULL);
}

/*---------------------------------------TASKS--------------------------------------*/
// Display update task
void displayUpdateTask(void *pvParameters)
{
#ifdef TEST_DISPLAY
    displayInitTime = micros() - schedulerStartTime;
#endif
    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t localRxMessage[8];

    while (1)
    {
#ifdef TEST_DISPLAY
        uint32_t startTime = micros();
#endif

        int8_t volume = __atomic_load_n(&volumeControl, __ATOMIC_RELAXED);
        int8_t octave = __atomic_load_n(&localOctaveShift, __ATOMIC_RELAXED);
        int8_t remoteOct = __atomic_load_n(&remoteOctaveShift, __ATOMIC_RELAXED);
        Waveform waveform = static_cast<Waveform>(__atomic_load_n(&currentWaveform, __ATOMIC_RELAXED));

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
        u8g2.print(localRxMessage[1]);

        // Display Volume
        u8g2.drawStr(2, 20, "Volume: ");
        u8g2.setCursor(50, 20);
        u8g2.print(volume);

        u8g2.drawStr(2, 30, "Oct: ");
        u8g2.setCursor(30, 30);
        u8g2.print(octave);

        // Draw Waveform (0-360 degrees)
        drawWaveform(waveformToUint8(waveform));

        // Display waveform name
        u8g2.setCursor(75, 30);
        u8g2.print(waveformToText(waveform));

        u8g2.sendBuffer();
        digitalToggle(LED_BUILTIN);
#ifdef TEST_DISPLAY
        uint32_t execTime = micros() - startTime;
        displayExecTimeTotal += execTime;
        if (execTime > displayExecTimeMax)
            displayExecTimeMax = execTime;
        displayExecCount++;

        if (displayExecCount >= 100)
        {
            float avg = (float)displayExecTimeTotal / displayExecCount / 1000.0;
            Serial.print("Display: Init=");
            Serial.print(displayInitTime / 1000.0, 2);
            Serial.print("ms, AvgExec=");
            Serial.print(avg, 3);
            Serial.print("ms, MaxExec=");
            Serial.print(displayExecTimeMax / 1000.0, 2);
            Serial.println("ms");
            // Reset counters
            displayExecTimeTotal = 0;
            displayExecTimeMax = 0;
            displayExecCount = 0;
        }
#endif
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Key scanning task
void scanKeysTask(void *pvParameters)
{
#ifdef TEST_SCAN_KEYS
    scanKeysInitTime = micros() - schedulerStartTime;
#endif

    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t txMsg[8] = {0};
    static uint16_t prevLocalActiveNotes = 0;

    while (1)
    {
#ifdef TEST_SCAN_KEYS
        uint32_t startTime = micros();
#endif

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
                    if (k == KnobConfig::VOLUME && changed)
                    {
                        int8_t newVolume = sysState.knobs[KnobConfig::VOLUME].getValue();
                        xSemaphoreGive(sysState.mutex);
                        __atomic_store_n(&volumeControl, newVolume, __ATOMIC_RELAXED);
                    }
                    else if (k == KnobConfig::WAVEFORM && changed)
                    {
                        int8_t waveSelection = sysState.knobs[KnobConfig::WAVEFORM].getValue();
                        xSemaphoreGive(sysState.mutex);
                        __atomic_store_n(&currentWaveform, waveSelection, __ATOMIC_RELAXED);
                    }
                    else if (k == KnobConfig::OCTAVE && changed)
                    {
                        int8_t newOctave = sysState.knobs[KnobConfig::OCTAVE].getValue();
                        xSemaphoreGive(sysState.mutex);
                        __atomic_store_n(&localOctaveShift, newOctave, __ATOMIC_RELAXED);
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

        // Scan joystick to apply pitch bend
        int joyX = analogRead(Pins::JOYX);
        int joyY = analogRead(Pins::JOYY);

        // Define constants for pitch bend
        const int DEADZONE_LOW = 400;
        const int DEADZONE_HIGH = 500;
        const int32_t MAX_PITCH_BEND = 1000000; // Reduced from 3000000 for more musical range
        const float SMOOTHING_FACTOR = 0.1f;    // For smooth transitions

        static int32_t currentPitchBend = 0; // Store current pitch bend for smoothing

        // Calculate pitch bend with deadzone
        if (joyX > DEADZONE_HIGH)
        {
            // Map right side (550-1024) to positive pitch bend
            int32_t targetBend = map(joyX, DEADZONE_HIGH, 1024, 0, MAX_PITCH_BEND + 1000000);
            currentPitchBend = currentPitchBend + (targetBend - currentPitchBend) * SMOOTHING_FACTOR;
        }
        else if (joyX < DEADZONE_LOW)
        {
            // Map left side (0-400) to negative pitch bend
            int32_t targetBend = map(joyX, DEADZONE_LOW, 0, 0, -MAX_PITCH_BEND);
            currentPitchBend = currentPitchBend + (targetBend - currentPitchBend) * SMOOTHING_FACTOR;
        }
        else
        {
            // In deadzone, gradually return to center
            currentPitchBend = currentPitchBend * (1.0f - SMOOTHING_FACTOR);
        }

        __atomic_store_n(&pitchBend, currentPitchBend, __ATOMIC_RELAXED);

        // Detect note changes and send CAN messages
        uint16_t pressed = newLocalActiveNotes & ~prevLocalActiveNotes;
        uint16_t released = prevLocalActiveNotes & ~newLocalActiveNotes;

        for (int i = 0; i < 12; i++)
        {
            if (pressed & (1 << i))
            {
                txMsg[CANMsg::TYPE] = CANMsg::PRESS;
                txMsg[CANMsg::OCTAVE] = __atomic_load_n(&localOctaveShift, __ATOMIC_RELAXED) + 4;
                txMsg[CANMsg::NOTE] = i;
                xQueueSend(sysState.txQueue, txMsg, 0);
            }
            if (released & (1 << i))
            {
                txMsg[CANMsg::TYPE] = CANMsg::RELEASE;
                txMsg[CANMsg::OCTAVE] = __atomic_load_n(&localOctaveShift, __ATOMIC_RELAXED) + 4;
                txMsg[CANMsg::NOTE] = i;
                xQueueSend(sysState.txQueue, txMsg, 0);
            }
        }

        prevLocalActiveNotes = newLocalActiveNotes;
        __atomic_store_n(&localActiveNotes, newLocalActiveNotes, __ATOMIC_RELAXED);
#ifdef TEST_SCAN_KEYS
        uint32_t execTime = micros() - startTime;
        scanKeysExecTimeTotal += execTime;
        if (execTime > scanKeysExecTimeMax)
            scanKeysExecTimeMax = execTime;
        scanKeysExecCount++;

        if (scanKeysExecCount >= 100)
        {
            float avg = (float)scanKeysExecTimeTotal / scanKeysExecCount / 1000.0;
            Serial.print("ScanKeys: Init=");
            Serial.print(scanKeysInitTime / 1000.0, 2);
            Serial.print("ms, AvgExec=");
            Serial.print(avg, 3);
            Serial.print("ms, MaxExec=");
            Serial.print(scanKeysExecTimeMax / 1000.0, 2);
            Serial.println("ms");
            // Reset counters
            scanKeysExecTimeTotal = 0;
            scanKeysExecTimeMax = 0;
            scanKeysExecCount = 0;
        }
#endif
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void decodeTask(void *pvParameters)
{
#ifdef TEST_DECODE
    decodeInitTime = micros() - schedulerStartTime;
#endif
    uint8_t rxMsg[CANMsg::MSG_SIZE];

    while (1)
    {
#ifdef TEST_DECODE
        uint32_t startTime = micros();
#endif

        if (xQueueReceive(sysState.rxQueue, rxMsg, portMAX_DELAY) == pdTRUE)
        {
            if (rxMsg[CANMsg::TYPE] == CANMsg::PRESS)
            {
                uint8_t note = rxMsg[CANMsg::NOTE];
                int8_t receivedOctave = rxMsg[CANMsg::OCTAVE] - 4; // Convert from 0-7 back to -4 to +4
                __atomic_fetch_or(&remoteActiveNotes, (1 << note), __ATOMIC_RELAXED);
                __atomic_store_n(&remoteOctaveShift, receivedOctave, __ATOMIC_RELAXED);
            }
            else if (rxMsg[CANMsg::TYPE] == CANMsg::RELEASE)
            {
                uint8_t note = rxMsg[CANMsg::NOTE];
                __atomic_fetch_and(&remoteActiveNotes, ~(1 << note), __ATOMIC_RELAXED);
            }

            if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE)
            {
                memcpy(sysState.RX_Message, rxMsg, CANMsg::MSG_SIZE);
                xSemaphoreGive(sysState.mutex);
            }
        }
#ifdef TEST_DECODE
        uint32_t execTime = micros() - startTime;
        decodeExecTimeTotal += execTime;
        if (execTime > decodeExecTimeMax)
            decodeExecTimeMax = execTime;
        decodeExecCount++;
#endif
    }
}

void canTxTask(void *pvParameters)
{
    uint8_t txMsg[CANMsg::MSG_SIZE];

    while (1)
    {
        xQueueReceive(sysState.txQueue, txMsg, portMAX_DELAY);
        CAN_TX(CANMsg::ID, txMsg);
    }
}

/*----------------------------SETUP AND LOOP--------------------------------------------*/

void setup()
{
    // Set pin directions
    pinMode(Pins::RA0, OUTPUT);
    pinMode(Pins::RA1, OUTPUT);
    pinMode(Pins::RA2, OUTPUT);
    pinMode(Pins::REN, OUTPUT);
    pinMode(Pins::OUT, OUTPUT);
    pinMode(Pins::OUTL, OUTPUT);
    pinMode(Pins::OUTR, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(Pins::C0, INPUT);
    pinMode(Pins::C1, INPUT);
    pinMode(Pins::C2, INPUT);
    pinMode(Pins::C3, INPUT);
    pinMode(Pins::JOYX, INPUT);
    pinMode(Pins::JOYY, INPUT);

    // Initialise display
    setOutMuxBit(MuxBits::DRST, LOW); // Assert display logic reset
    delayMicroseconds(2);
    setOutMuxBit(MuxBits::DRST, HIGH); // Release display logic reset
    u8g2.begin();
    setOutMuxBit(MuxBits::DEN, HIGH); // Enable display power supply

    // Initialise UART
    Serial.begin(9600);
    Serial.println("Synthesizer Started");

    // Initialize the keyboard
    setOutMuxBit(MuxBits::HKOE, HIGH); // Enable keyboard output

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
    __atomic_store_n(&volumeControl, 5, __ATOMIC_RELAXED);

    __atomic_store_n(&octaveShift, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&currentWaveform, waveformToUint8(Waveform::SAWTOOTH), __ATOMIC_RELAXED);

    __atomic_store_n(&localOctaveShift, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&remoteOctaveShift, 0, __ATOMIC_RELAXED);
    // Initialize waveform LUTs
    // initWaveformLUTs();  // Removed since we're using hardcoded values

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
    schedulerStartTime = micros(); // Record start time
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