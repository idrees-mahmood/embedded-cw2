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

// Global variables

// Global variable to store the notes
volatile uint16_t localActiveNotes = 0;
volatile uint16_t remoteActiveNotes = 0;

// Global volatile variable for volume control
// This will be accessed by both the ISR and the knob update task
volatile int8_t volumeControl = 0;
volatile int32_t pitchBend = 0;

// Audio buffer configuration
constexpr uint32_t AUDIO_BUFFER_SIZE = 256; // 11.6 ms at 22050 Hz
static int16_t audioBuffer0[AUDIO_BUFFER_SIZE] = {0};
static int16_t audioBuffer1[AUDIO_BUFFER_SIZE] = {0};
volatile int16_t *activeBuffer = audioBuffer0;
volatile uint32_t bufferPosition = 0;

struct ReverbState
{
    SemaphoreHandle_t mutex;
    static constexpr uint32_t DELAY_SAMPLES = 2200;
    float feedback = 0.0f;
    int16_t delayBuffer[DELAY_SAMPLES] = {0};
    uint32_t writeIndex = 0;
} reverb;
// atomic access, may consider mutex

// Timer object
HardwareTimer sampleTimer(TIM1);

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

/*--------------------------------------Helper Functions----------------------------------------------------*/

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
        Knob(3, 0, 1, 0, 0, 7),  // Knob 3: Row 3, cols 0,1, volume control (0-7)
        Knob(3, 2, 3, 0, 0, 15), // Knob 2: Row 3, cols 2,3, reverb decay (0-15)
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

int32_t generateSawtooth(uint16_t activeNotes)
{
    static uint32_t phaseAcc[12] = {0}; // Phase accumulator, static (stores value between calls)
    int32_t Vout = 0;
    int numActive = __builtin_popcount(activeNotes);

    for (int i = 0; i < 12; i++)
    {
        if (activeNotes & (1 << i))
        {
            phaseAcc[i] += stepSizes[i];
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

    return Vout;
}

int32_t applyReverb(int32_t drySample, float feedback)
{

    xSemaphoreTake(reverb.mutex, portMAX_DELAY);

    uint32_t readIndex = (reverb.writeIndex + ReverbState::DELAY_SAMPLES - 
        (ReverbState::DELAY_SAMPLES % ReverbState::DELAY_SAMPLES)) % 
        ReverbState::DELAY_SAMPLES;

    int32_t delayedSample = reverb.delayBuffer[readIndex];

    int32_t wet = (delayedSample * feedback);

    //low pass filter
    static int32_t lastOutput = 0;
    wet = (wet + lastOutput) / 2;
    lastOutput = wet;

    int32_t output = wet * 0.7 + drySample;

    reverb.writeIndex = (reverb.writeIndex + 1) % ReverbState::DELAY_SAMPLES;

    xSemaphoreGive(reverb.mutex);

    return output;
}

/*------------------------------ISRs-------------------------------------------*/

void sampleISR()
{
    uint32_t pos = __atomic_load_n(&bufferPosition, __ATOMIC_RELAXED);
    if (pos < AUDIO_BUFFER_SIZE)
    {
        int16_t sample = activeBuffer[pos];
        __atomic_store_n(&bufferPosition, pos + 1, __ATOMIC_RELAXED);

        analogWrite(OUTR_PIN, (uint8_t)(sample + 128));
    }
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
        u8g2.print(noteNames[localRxMessage[2]]);

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
                        int8_t decayValue = sysState.knobs[1].getValue();
                        xSemaphoreGive(sysState.mutex);
                        xSemaphoreTake(reverb.mutex, portMAX_DELAY);
                        float feedback = decayValue / 15.0f * 0.9f; // 0-15 -> 0.0-0.9
                        reverb.feedback = feedback;
                        xSemaphoreGive(reverb.mutex);
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

        // Apply pitch bend to the notes

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

void audioProcessingTask(void *pvParameters)
{
    const TickType_t xBufferDuration = pdMS_TO_TICKS(11.6); // ~11.6ms for 256 samples @22kHz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int16_t *workingBuffer = audioBuffer0;

    while (1)
    {
        // Generate samples for the entire buffer
        for (uint32_t i = 0; i < AUDIO_BUFFER_SIZE; ++i)
        {
            uint16_t allActive = __atomic_load_n(&localActiveNotes, __ATOMIC_RELAXED) |
                                 __atomic_load_n(&remoteActiveNotes, __ATOMIC_RELAXED);
            int32_t Vout = generateSawtooth(allActive);

            // Apply pitch bend
            Vout += __atomic_load_n(&pitchBend, __ATOMIC_RELAXED);

            // Apply reverb
            float feedback;
            xSemaphoreTake(reverb.mutex, portMAX_DELAY);
            feedback = reverb.feedback;
            xSemaphoreGive(reverb.mutex);
            Vout = applyReverb(Vout, feedback);

            // Apply volume
            int8_t volume = __atomic_load_n(&volumeControl, __ATOMIC_RELAXED);
            Vout = (volume >= 0 && volume <= 7) ? Vout >> (7 - volume) : 0;

            workingBuffer[i] = static_cast<int16_t>(Vout);
        }

        // Switch buffers atomically
        portENTER_CRITICAL();
        if (workingBuffer == audioBuffer0)
        {
            activeBuffer = (volatile int16_t *)audioBuffer0;
            workingBuffer = audioBuffer1;
        }
        else
        {
            activeBuffer = (volatile int16_t *)audioBuffer1;
            workingBuffer = audioBuffer0;
        }
        bufferPosition = 0; // Reset ISR's buffer position
        portEXIT_CRITICAL();

        // Delay until the next buffer period starts
        vTaskDelayUntil(&xLastWakeTime, xBufferDuration);
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

    // Initialize reverb state
    reverb.mutex = xSemaphoreCreateMutex();
    if (reverb.mutex == NULL)
    {
        Serial.println("ERROR: Reverb mutex creation failed");
        while (1)
            ;
    }

    // Initialize global variables
    __atomic_store_n(&volumeControl, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&pitchBend, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&activeBuffer, (volatile int16_t *)audioBuffer0, __ATOMIC_RELAXED);
    __atomic_store_n(&bufferPosition, 0, __ATOMIC_RELAXED);

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
        decodeTask,       // Function to run
        "decode",         // Task name
        128,              // Stack size in words
        NULL,             // Parameters
        1,                // Priority
        &decodeTaskHandle // Task handle
    );

    xTaskCreate(
        canTxTask, // Function to run
        "canTx",   // Task name
        128,       // Stack size in words
        NULL,      // Parameters
        1,         // Priority
        NULL       // Task handle
    );

    xTaskCreate(
        audioProcessingTask, // Function to run
        "AudioProcess",
        2048, // to account for the audio buffer
        NULL,
        2, // Higher priority than the other tasks
        NULL);

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