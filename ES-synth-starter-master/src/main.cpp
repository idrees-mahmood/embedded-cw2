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
const char *noteNames[12] = {"C", "C#", "D", "D#", "E", "F",
                             "F#", "G", "G#", "A", "A#", "B"};
const uint32_t stepSizes[12] = {
    // Step sizes of 12 notes
    51076056, // C4
    54113197, // C#4
    57330935, // D4
    60740009, // D#4
    64351798, // E4
    68178356, // F4
    72232452, // F#4
    76527617, // G4
    81078186, // G#4
    85899345, // A4 (440 Hz)
    91007186, // A#4
    96418755  // B4
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

// Global variable to store current step size
volatile uint32_t currentStepSize = 0;
volatile uint16_t localActiveNotes = 0;
volatile uint16_t remoteActiveNotes = 0;

// Global volatile variable for volume control
// This will be accessed by both the ISR and the knob update task
volatile int8_t volumeControl = 0;

// Timer object
HardwareTimer sampleTimer(TIM1);

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

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
    std::bitset<32> inputs;
    SemaphoreHandle_t mutex;
    QueueHandle_t txQueue;
    QueueHandle_t rxQueue;
    SemaphoreHandle_t CAN_TX_Semaphore;
    Knob knobs[4] = {
        Knob(3, 0, 1, 0, 0, 7),  // Knob 3: Row 3, cols 0,1, volume control (0-7)
        Knob(3, 2, 3, 0, 0, 15), // Knob 2: Row 3, cols 2,3, unused
        Knob(4, 0, 1, 0, 0, 15), // Knob 1: Row 4, cols 0,1, unused
        Knob(4, 2, 3, 0, 0, 15)  // Knob 0: Row 4, cols 2,3, unused
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

    int32_t Vout = 0;
    int numActive = __builtin_popcount(allActive);

    for (int i = 0; i < 12; i++)
    {
        if (allActive & (1 << i))
        {
            phaseAcc[i] += stepSizes[i];
            // Vout += (int32_t)(sinLUT[(phaseAcc[i] >> 20) & 0x3ff]) * 127 / numActive;
            Vout += (phaseAcc[i] >> 24) - 128; // Sawtooth wave
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
    Vout = (Vout > 127) ? 127 : (Vout < -128) ? -128 : Vout;

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

        // Determine which note name to display based on currentStepSize
        uint32_t stepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);

        const char *pressedKey = "None";
        if (stepSize > 0)
        {
            // Find which note matches the current step size
            for (int i = 0; i < 12; i++)
            {
                if (stepSize == stepSizes[i])
                {
                    pressedKey = noteNames[i];
                    break;
                }
            }
        }

        // Get volume value directly from the atomic global
        int8_t volume = __atomic_load_n(&volumeControl, __ATOMIC_RELAXED);

        // Get the latest RX message data under mutex protection
        if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE)
        {
            memcpy(localRxMessage, sysState.RX_Message, 8);
            xSemaphoreGive(sysState.mutex);
        }

        // Update display
        u8g2.clearBuffer();                   // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
        u8g2.drawStr(2, 10, "Pressed key: "); // write something to the internal memory
        u8g2.setCursor(75, 10);
        u8g2.print(pressedKey);

        u8g2.setCursor(2, 20);
        u8g2.print("Volume: ");
        u8g2.print(volume);

        // Display additional knob values if needed
        u8g2.setCursor(2, 30);
        u8g2.print("Knobs: ");
        if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE)
        {
            for (int i = 1; i < 4; i++)
            {
                u8g2.print(sysState.knobs[i].getValue());
                u8g2.print(" ");
            }
            xSemaphoreGive(sysState.mutex);
        }

        // Display the latest RX message
        u8g2.setCursor(80, 30);
        if (localRxMessage[0] != 0) // Only display if we have received something
        {
            u8g2.print((char)localRxMessage[0]); // 'P' or 'R'
            u8g2.print(localRxMessage[1]);       // Octave
            u8g2.print(localRxMessage[2]);       // Note
        }
        else
        {
            u8g2.print("---"); // No message received yet
        }

        u8g2.sendBuffer(); // transfer internal memory to the display

        // Toggle LED for visual feedback
        digitalToggle(LED_BUILTIN);
    }
}

// Task to update volume control value
void updateVolumeTask(void *pvParameters)
{
    const TickType_t xFrequency = 5 / portTICK_PERIOD_MS; // More frequent updates for responsive volume control
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Safely get the volume value from the knob object
        if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE)
        {
            int8_t newVolume = sysState.knobs[0].getValue();
            xSemaphoreGive(sysState.mutex);

            // Update the atomic volume control variable
            __atomic_store_n(&volumeControl, newVolume, __ATOMIC_RELAXED);
        }
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
    CAN_Init(true); // true for loopback, false for normal
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