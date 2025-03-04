#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>

// Constants
const uint32_t interval = 100; // Display update interval
const char *noteNames[12] = {"C4", "C#4", "D4", "D#4", "E4", "F4",
                             "F#4", "G4", "G#4", "A4", "A#4", "B4"};
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

// Timer object
HardwareTimer sampleTimer(TIM1);

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// Threading
struct {
  std::bitset<32> inputs;
} sysState;

// Task handles
TaskHandle_t scanKeysHandle = NULL;
TaskHandle_t displayUpdateHandle = NULL;

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

void sampleISR()
{
  static uint32_t phaseAcc = 0; // Phase accumulator, static (stores value between calls)
  phaseAcc += currentStepSize;  // Increment phase

  int32_t Vout = (phaseAcc >> 24) - 128; // Convert to sawtooth waveform
  analogWrite(OUTR_PIN, Vout + 128);
  analogWrite(OUTL_PIN, Vout + 128); // Add output to left channel too
}

// Display update task
void displayUpdateTask(void *pvParameters)
{
  int lastKeyPressed = -1;
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Determine which note name to display based on currentStepSize
    uint32_t stepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
    
    const char *pressedKey = "None";
    if (stepSize > 0) {
      // Find which note matches the current step size
      for (int i = 0; i < 12; i++) {
        if (stepSize == stepSizes[i]) {
          pressedKey = noteNames[i];
          break;
        }
      }
    }

    // Update display
    u8g2.clearBuffer();                   // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
    u8g2.drawStr(2, 10, "Pressed key: "); // write something to the internal memory
    u8g2.setCursor(75, 10);
    u8g2.print(pressedKey);
    u8g2.sendBuffer(); // transfer internal memory to the display
    
    // Toggle LED for visual feedback
    digitalToggle(LED_BUILTIN);
  }
}

// Key scanning task
void scanKeysTask(void *pvParameters)
{
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    int lastKeyPressed = -1; // default: no key pressed

    for (uint8_t row = 0; row < 3; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> result = readCols();

      for (uint8_t col = 0; col < 4; col++) {
        int keyIndex = row * 4 + col;
        if (keyIndex < 12 && !result[col]) { // Only check piano keys, active low logic
          lastKeyPressed = keyIndex; // Store this pressed key
        }
      }
    }

    // Update the step size based on the last pressed key
    if (lastKeyPressed != -1) {
      __atomic_store_n(&currentStepSize, stepSizes[lastKeyPressed], __ATOMIC_RELAXED);
    } else {
      __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    }
  }
}

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
  Serial.println("Hello World");

  // Initialize the keyboard
  setOutMuxBit(HKOE_BIT, HIGH); // Enable keyboard output

  // Timer and interrupt set up
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  // Create tasks BEFORE starting the scheduler
  xTaskCreate(
      scanKeysTask,       // Function to run
      "scanKeys",         // Task name
      64,                // Stack size in word
      NULL,               // Parameters
      2,                  // Priority
      &scanKeysHandle);   // Task handle

  xTaskCreate(
      displayUpdateTask,  // Function to run
      "displayUpdate",    // Task name
      256,                // Stack size in words 
      NULL,               // Parameters
      1,                  // Priority
      &displayUpdateHandle); // Task handle

  // Start the scheduler
  Serial.println("Starting FreeRTOS scheduler");
  vTaskStartScheduler();

  // Code should never reach here if scheduler starts properly
  Serial.println("ERROR: FreeRTOS scheduler failed to start");
  while(1); // Infinite loop if scheduler fails
}

void loop()
{
  vTaskDelayUntil();
}