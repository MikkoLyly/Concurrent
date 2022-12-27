//------------------------------------------------------------------------------
//            Simple time-sliced multithreding on Arduino UNO
//
// This sketch runs multiple tasks concurrently by performing a context switch
// every 1 ms. It uses TIMER2 to generate OS ticks (Pro: Arduino's own timing
// routines like millis() and delay() remain functional. Con: Makes libraries
// based on TIMER2 unusable).
//
// This is a recreational weekend project written for my self to better under-
// stand the basic concepts of time-slicing on 8bit AVR MCUs.
// 
// Written by: Mikko Lyly
// Date:       26 dec 2022
//------------------------------------------------------------------------------
#include <Arduino.h>

//------------------------------------------------------------------------------
// Global variables and typedefs
//------------------------------------------------------------------------------
volatile uint8_t currentTask = 0;                  // Current task ID
volatile uint32_t currentTicks = 0;                // Milliseconds
typedef void(*Task)();                             // Task pointer

//------------------------------------------------------------------------------
// Forward declarations (definitions at the end of this file)
//------------------------------------------------------------------------------
void pushContext() __attribute__((always_inline));
void popContext() __attribute__((always_inline));
void pushAddress(Task) __attribute__((always_inline));
void ownDelay(uint32_t) __attribute__((always_inline));

//------------------------------------------------------------------------------
// Tasks (signature: void taskName(), structure: setup + infinite loop)
//------------------------------------------------------------------------------
#define NOF_TASKS 3

void blinker() {
  Serial.println(F("Blinker starting"));
  
  for(;;) {
    digitalWrite(13, HIGH);
    ownDelay(500);                                 // Own delay
    digitalWrite(13, LOW);
    delay(500);                                    // Arduino's delay
  }
}

void console() {
  Serial.println(F("Console starting"));

  for(;;) {        
    while(Serial.available() > 0) {
      char c = Serial.read();
      Serial.print(c);
    }
  }
}

void plotter() {
  uint8_t counter = 0;
  
  Serial.println(F("Plotter starting"));
  
  for(;;) {
    Serial.print('.');                             // Plots a dot once a second
    ++counter %= 60;                               
    if(counter == 0) Serial.println();             // New line once a minute
    ownDelay(1000);
  }
}

//------------------------------------------------------------------------------
// List of tasks and initial stack pointers.
//
// NOTE: SRAM is organized as follows:
//
// RAMEND (=0x08ff)
//   Stack frame of blinker()
// RAMEND - 0x0100
//   Stack frame of console()
// RAMEND - 0x0200
//   Stack frame of plotter()
// ...
//   Data, bss, heap
// RAMSTART (=0x0100)
//   Registers and ports
// 0x0000
//
// A task needs at least N + 35 bytes to store all local variables (N bytes)
// and the execution context (35 bytes) in stack. There is no protection against
// possible memory access violations.
//------------------------------------------------------------------------------
const Task tasks[] = {blinker, console, plotter};
volatile uint16_t stackPointers[] = {RAMEND, RAMEND - 0x100, RAMEND - 0x200};

//------------------------------------------------------------------------------
// Arduino's setup()
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);                              // Init USART
  Serial.println(F("Initializing"));
  pinMode(13, OUTPUT);                             // On-board LED
   
  //----------------
  // Prepare stacks
  //----------------
  for(uint8_t n = 0; n < NOF_TASKS; ++n) {
    SP = stackPointers[n];                         // Load stack pointer
    pushAddress(tasks[n]);                         // Push task address
    pushContext();                                 // Push (any) context
    stackPointers[n] = SP;                         // Save stack pointer
  }
    
  //--------------------------------
  // Set up timer 2 for 1 kHz ticks
  //--------------------------------
  cli();                                           // Disable interrupts
  TCCR2A = _BV(WGM21);                             // Select CTC mode and
  TCCR2B = _BV(CS22);                              // set prescaler to 64
  OCR2A = 249;                                     // Trigger every 1 ms
  TIMSK2 |= _BV(OCIE2A);                           // Enable Compare A ISR
  sei();                                           // Enable interrupts

  //------------------------
  // Start the initial task
  //------------------------
  SP = stackPointers[currentTask];                 // Load stack pointer
  popContext();                                    // Pop context
  asm("ret");                                      // Start execution
}

//------------------------------------------------------------------------------
// Arduino's loop()
//------------------------------------------------------------------------------
void loop() {                                      // Never reached
}

//------------------------------------------------------------------------------
// Switch task (round-robin)
//------------------------------------------------------------------------------
ISR(TIMER2_COMPA_vect, ISR_NAKED) {
  pushContext();                                   // Push context
  stackPointers[currentTask] = SP;                 // Save stack pointer
  ++currentTicks;                                  // Increment ticks
  ++currentTask %= NOF_TASKS;                      // Rotate task
  SP = stackPointers[currentTask];                 // Load stack pointer
  popContext();                                    // Pop context
  asm("reti");                                     // Resume execution
}

//------------------------------------------------------------------------------
// Push context (33 bytes)
//------------------------------------------------------------------------------
void pushContext() {
  asm("push r0       \n"
      "in   r0, 0x3f \n"                           // Status
      "push r0       \n"
      "push r1       \n"
      "clr r1        \n"                           // Keep gcc happy
      "push r2       \n"
      "push r3       \n"
      "push r4       \n"
      "push r5       \n"
      "push r6       \n"
      "push r7       \n"
      "push r8       \n"
      "push r9       \n"
      "push r10      \n"
      "push r11      \n"
      "push r12      \n"
      "push r13      \n"
      "push r14      \n"
      "push r15      \n"
      "push r16      \n"
      "push r17      \n"            
      "push r18      \n"
      "push r19      \n"
      "push r20      \n"
      "push r21      \n"
      "push r22      \n"
      "push r23      \n"
      "push r24      \n"
      "push r25      \n"
      "push r26      \n"
      "push r27      \n"
      "push r28      \n"
      "push r29      \n"
      "push r30      \n"
      "push r31      \n");
}

//------------------------------------------------------------------------------
// Pop context (33 bytes)
//------------------------------------------------------------------------------
void popContext() {
  asm("pop r31      \n"
      "pop r30      \n"
      "pop r29      \n"
      "pop r28      \n"
      "pop r27      \n"
      "pop r26      \n"
      "pop r25      \n"
      "pop r24      \n"
      "pop r23      \n"
      "pop r22      \n"
      "pop r21      \n"
      "pop r20      \n"
      "pop r19      \n"
      "pop r18      \n"
      "pop r17      \n"
      "pop r16      \n"
      "pop r15      \n"
      "pop r14      \n"
      "pop r13      \n"
      "pop r12      \n"
      "pop r11      \n"
      "pop r10      \n"
      "pop r9       \n"
      "pop r8       \n"
      "pop r7       \n"
      "pop r6       \n"
      "pop r5       \n"
      "pop r4       \n"
      "pop r3       \n"
      "pop r2       \n"
      "pop r1       \n"       
      "pop r0       \n"
      "out 0x3f, r0 \n"                            // Status
      "pop r0       \n");
}

//------------------------------------------------------------------------------
// Push task address (2 bytes)
//------------------------------------------------------------------------------
void pushAddress(Task task) {
  asm("push %A0     \n"                            // Low byte
      "push %B0     \n"                            // High byte
      :: "x" (task));
}

//------------------------------------------------------------------------------
// Delay N milliseconds (it is possible to use Arduino's delay() as well)
//------------------------------------------------------------------------------
void ownDelay(uint32_t n) { 
  uint32_t n0, n1;
  
  cli();                                           // Critical section
  n0 = currentTicks;
  sei();
  
  n1 = n0 + n;
  
  while(n0 < n1) {
    cli();                                         // Critical section
    n0 = currentTicks;
    sei();
  }
}
