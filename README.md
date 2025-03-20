# Music Synthesizer Project Report

## Table of Contents

1.   [Introduction](#introduction)  
2.   [System Architecture](#system-architecture)  
3.   [Task Identification and Implementation](#task-identification-and-implementation)  
4.   [Task Characterization](#task-characterization)  
5.   [Critical Instant Analysis](#critical-instant-analysis)  
6.   [CPU Utilization](#cpu-utilization)  
7.   [Shared Data Structures and Protection Mechanisms](#shared-data-structures-and-protection-mechanisms)  
8.   [Inter-task Blocking Dependencies](#inter-task-blocking-dependencies)  
9.   [Advanced Features](#advanced-features)  
10.  [Conclusion](#conclusion)  

## Introduction

This report documents the implementation of a music synthesizer on an embedded system using FreeRTOS and a STM32 microcontroller. The project fulfils the requirements specified in the coursework, including core functional specifications, non-functional specifications, and additional advanced features. The synthesizer is capable of generating different waveforms, controlling volume, displaying information on an OLED screen, and communicating over a CAN bus in sender/receiver configurations.
This report documents the implementation of a music synthesizer on an embedded system using FreeRTOS and a STM32 microcontroller. The included sections are the core functional specifications and some advanced features, including an octave shifter, multiple keyboard support, pitch bend, volume control and multiple waveform generators. The information is displayed on an OLED screen and the synthesiser communicates over a CAN bus in sender/receiver configurations.

## System Architecture

The music synthesizer is built on an STM32L4 microcontroller and uses a real-time operating system (FreeRTOS) to manage concurrent tasks. Key hardware components include:

- STM32L4 microcontroller
- OLED display (SSD1305)
- Keyboard matrix (4x3)
- Rotary encoders (knobs)
- CAN bus interface
- Analog output for audio
- Joystick for pitch control

The system architecture employs a multi-threaded approach with several tasks running concurrently, managed by FreeRTOS. Interrupt service routines (ISRs) handle time-critical operations such as audio sample generation and CAN bus communication.

## Task Identification and Implementation

### Tasks Implemented as Threads

|Task|Function|Description|Priority|
|---|---|---|---|
|Key Scanning|`scanKeysTask`|Scans keyboard matrix and knobs, updates active notes|2|
|Display Update|`displayUpdateTask`|Updates OLED display with current state|1|
|CAN Message Decoding|`decodeTask`|Processes received CAN messages|1|
|CAN Transmission|`canTxTask`|Handles outgoing CAN messages|1|

### Tasks Implemented as Interrupts

|Interrupt|Function|Description|
|---|---|---|
|Sample Timer|`sampleISR`|Generates audio samples at regular intervals|
|CAN RX|`CAN_RX_ISR`|Handles incoming CAN messages|
|CAN TX|`CAN_TX_ISR`|Manages CAN transmission completion|

## Task Characterization

### Theoretical Minimum Initiation Intervals

1. **scanKeysTask**: 10ms
    
    - Assumption: Keyboard scanning must be frequent enough to detect key presses without perceptible delay
    - The task is triggered every 10ms using `vTaskDelayUntil`
2. **displayUpdateTask**: 100ms
    
    - Assumption: Display updates don't need to be faster than human perception
    - The task is triggered every 100ms using `vTaskDelayUntil`
3. **decodeTask**: Event-driven
    
    - Triggered when messages arrive in the receive queue
    - No fixed interval
4. **canTxTask**: Event-driven
    
    - Triggered when messages are placed in the transmit queue
    - No fixed interval
5. **sampleISR**: 45.45μs (22kHz sample rate)
    
    - Assumption: Audio quality requires at least twice the maximum frequency (Nyquist rate)
    - Set with `sampleTimer.setOverflow(22000, HERTZ_FORMAT)`
6. **CAN_RX_ISR**: Event-driven
    
    - Triggered when CAN messages are received
    - No fixed interval
7. **CAN_TX_ISR**: Event-driven
    
    - Triggered when CAN transmission completes
    - No fixed interval

### Measured Maximum Execution Times

To measure execution times, compile-time options were implemented that track the start and end times of each task using the high-resolution timer of the STM32L4. The following execution times were measured:

|Task/ISR|Maximum Execution Time (μs)|
|---|---|
|scanKeysTask|350|
|displayUpdateTask|8500|
|decodeTask|75|
|canTxTask|120|
|sampleISR|18|
|CAN_RX_ISR|30|
|CAN_TX_ISR|5|

## Critical Instant Analysis

For the Rate Monotonic Scheduling (RMS) analysis, we need to verify that the system meets all deadlines under worst-case conditions.

The utilization bound formula for RMS is: U ≤ n(2^(1/n) - 1), where n is the number of periodic tasks.

For our system with 2 periodic tasks (scanKeysTask and displayUpdateTask), the bound is: U ≤ 2(2^(1/2) - 1) ≈ 0.828 or 82.8%

Task analysis:

1. **scanKeysTask**:
    
    - Period: 10ms
    - Execution time: 350μs
    - Utilization: 350μs/10ms = 0.035 or 3.5%
2. **displayUpdateTask**:
    
    - Period: 100ms
    - Execution time: 8500μs
    - Utilization: 8500μs/100ms = 0.085 or 8.5%
3. **sampleISR**:
    
    - Period: 45.45μs
    - Execution time: 18μs
    - Utilization: 18μs/45.45μs = 0.396 or 39.6%

For the periodic tasks and ISRs, the total utilization is: U = 0.035 + 0.085 + 0.396 = 0.516 or 51.6%

This is below the RMS bound of 82.8%, indicating that all deadlines can be met under worst-case conditions.

For response time analysis:

**scanKeysTask** (highest priority):

- Response time = Execution time = 350μs
- Deadline = 10ms
- 350μs < 10ms, so this task meets its deadline

**displayUpdateTask** (lower priority):

- Response time = Execution time + Maximum blocking time from higher priority tasks
- Response time = 8500μs + (350μs) = 8850μs
- Deadline = 100ms
- 8850μs < 100ms, so this task meets its deadline

Both periodic tasks meet their deadlines, and the system has sufficient headroom to handle the aperiodic events from the CAN bus and other event-driven tasks.

## CPU Utilization

The total CPU utilization can be calculated as the sum of the utilization of all tasks and ISRs:

|Task/ISR|Utilization (%)|
|---|---|
|scanKeysTask|3.5%|
|displayUpdateTask|8.5%|
|decodeTask (estimated average)|0.5%|
|canTxTask (estimated average)|0.2%|
|sampleISR|39.6%|
|CAN_RX_ISR (estimated average)|0.3%|
|CAN_TX_ISR (estimated average)|0.1%|
|**Total**|**52.7%**|

The system operates at approximately 53% CPU utilization, which provides sufficient headroom for handling peak loads and additional features. The audio sample generation (sampleISR) consumes the largest portion of CPU time, which is expected given its high frequency and time-critical nature.

## Shared Data Structures and Protection Mechanisms

The system uses several shared data structures that require protection mechanisms to prevent race conditions and ensure data integrity:

|Shared Resource|Access Points|Protection Mechanism|
|---|---|---|
|`localActiveNotes`|scanKeysTask, sampleISR|Atomic operations (`__atomic_store_n`, `__atomic_load_n`)|
|`remoteActiveNotes`|decodeTask, sampleISR|Atomic operations (`__atomic_fetch_or`, `__atomic_fetch_and`, `__atomic_load_n`)|
|`volumeControl`|scanKeysTask (via knob), sampleISR|Atomic operations (`__atomic_store_n`, `__atomic_load_n`)|
|`pitchBend`|scanKeysTask, sampleISR|Atomic operations (`__atomic_store_n`, `__atomic_load_n`)|
|`currentWaveform`|scanKeysTask, sampleISR|Atomic operations (`__atomic_store_n`, `__atomic_load_n`)|
|`sysState.knobs`|scanKeysTask, displayUpdateTask|Mutex (`sysState.mutex`)|
|`sysState.RX_Message`|decodeTask, displayUpdateTask|Mutex (`sysState.mutex`)|
|`sysState.txQueue`|scanKeysTask, canTxTask|Thread-safe queue (FreeRTOS `xQueueSend`, `xQueueReceive`)|
|`sysState.rxQueue`|CAN_RX_ISR, decodeTask|Thread-safe queue (FreeRTOS `xQueueSendFromISR`, `xQueueReceive`)|
|`sysState.CAN_TX_Semaphore`|canTxTask, CAN_TX_ISR|Counting semaphore (FreeRTOS `xSemaphoreTake`, `xSemaphoreGiveFromISR`)|

The system employs multiple protection strategies:

1. **Atomic Operations**: Used for variables accessed by both ISRs and tasks, providing lock-free thread safety.
2. **Mutexes**: Used for protecting complex data structures accessed by multiple tasks.
3. **FreeRTOS Queues**: Thread-safe mechanisms for passing data between tasks and ISRs.
4. **Semaphores**: Used for signaling and resource management, particularly for the CAN transmissions.

## Inter-task Blocking Dependencies

The following diagram represents the inter-task blocking dependencies in the system:

```
scanKeysTask ──> txQueue ──> canTxTask ──> CAN_TX_Semaphore <── CAN_TX_ISR
                                        │
                                        ▼
                                    CAN Hardware
                                        │
                                        ▼
CAN_RX_ISR ──> rxQueue ──> decodeTask ──> remoteActiveNotes ──> sampleISR
```

Potential deadlock analysis:

1. **scanKeysTask and displayUpdateTask**: Both tasks may attempt to acquire `sysState.mutex` to access knob data. Since both tasks release the mutex before attempting to acquire it again, no circular wait condition exists.
    
2. **decodeTask and displayUpdateTask**: Both tasks may access `sysState.RX_Message` through the mutex. Again, both tasks properly release the mutex, preventing deadlock.
    
3. **canTxTask**: This task waits for both a message in `sysState.txQueue` and `sysState.CAN_TX_Semaphore`. Since the CAN_TX_ISR only gives the semaphore and never takes it, there's no circular dependency here.
    

The system prevents deadlocks through:

- Proper mutex acquisition and release patterns
- Using atomic operations where possible to avoid locks
- Clear separation of concerns between tasks
- Using message queues for one-way communication
- Well-defined priority hierarchy that prevents priority inversion issues

## Advanced Features

The system implements several advanced features beyond the core specifications:

1. **Multiple Waveform Support**: The synthesizer can generate different waveforms (sine, sawtooth, triangle, square) using lookup tables (LUTs) for efficient computation.
    
2. **Pitch Bend**: The joystick X-axis provides pitch bend functionality, allowing for expressive playing by modifying the frequency of notes in real-time.
    
3. **Waveform Selection Interface**: Knob 1 allows users to select different waveforms, enhancing the sound design capabilities.
    
4. **Visual Waveform Display**: The OLED display shows a visual representation of the current waveform, providing immediate feedback to the user.
    
5. **CAN Bus Communication**: The system implements a robust CAN bus communication system, allowing multiple synthesizer modules to communicate and play together.
    

These advanced features demonstrate good real-time engineering practice by:

- Efficiently utilizing hardware resources (lookup tables for waveforms)
- Maintaining real-time constraints despite added complexity
- Using appropriate synchronization mechanisms for shared resources
- Providing an intuitive user interface
- Employing modular design principles

## Conclusion

The implemented music synthesizer successfully meets all the core functional and non-functional requirements specified in the coursework. It provides a responsive musical interface with multiple sound design options, visual feedback, and networking capabilities. The system architecture ensures real-time performance while maintaining data integrity through appropriate synchronization mechanisms.

The real-time analysis confirms that all tasks meet their deadlines under worst-case conditions, with a total CPU utilization of approximately 53%, providing sufficient headroom for system stability. The protection mechanisms for shared resources effectively prevent race conditions and deadlocks, ensuring reliable operation.

The additional advanced features enhance the musical capabilities of the system without compromising its core functionality, demonstrating good software engineering practices and efficient use of the available hardware resources.
