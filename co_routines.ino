//------------------------------------------------------------------------------
//         Simple co-routines for Arduino (works on all boards)               
//
// This sketch runs multiple tasks concurrently by keeping track of their
// states over repeated calls from the main event loop. The library consists
// of only five lines of macros. Highly inspired by Protothreads.
//
// This is a small weekend project written for my self to better understand
// the basic concepts of concurrent programming on 8 bit AVR MCUs. There are
// several full featured libraries to accomplish the same and much more.
//
// Written by: Mikko Lyly
// Date:       26 dec 2022
//------------------------------------------------------------------------------
#define coStart() static uint32_t S = 0, T = 0; switch(S) { case 0:;
#define coEnd() } S = __LINE__; return &S;
#define coYield() S = __LINE__; return &S; case __LINE__:;
#define coWaitUntil(X) S = __LINE__; case __LINE__: if(!(X)) return &S;
#define coDelay(X) T = millis(); coWaitUntil((T + X) < millis());

//------------------------------------------------------------------------------
//                                   Tasks
//
// Signature: uint32_t *task(void)
// 
// Structure: Initialization code followed by an inifinite loop must be
// placed between coStart() and coEnd(). Use coYield() to return control to
// the main event loop. Use coWaitUntil() and coDelay() to wait and delay
// in a non-blocking fashion.
//
// NOTE: All local variables which are supposed to retain their values over
// coYield(), coWaitUntil() and coDelay(), should be declared static.
//------------------------------------------------------------------------------
uint32_t *blinker() {
  coStart();                                          // Start

  static uint32_t t = 0;                              // static
  
  Serial.println(F("Blinker starting"));
  
  for(;;) {
    digitalWrite(13, HIGH);
    coDelay(500);                                     // Non-blocking delay
    digitalWrite(13, LOW);
    t = millis();                                     // Another way to delay
    coWaitUntil(millis() > t + 1000);                 // by using coWaitUntil()
  }

  coEnd();                                            // End
}

uint32_t *console() {
  coStart();                                          // Start

  Serial.println(F("Console starting"));
  
  for(;;) {
    coWaitUntil(Serial.available());                  // Non-blocking wait
    char c = Serial.read();
    Serial.print(c);
  }
  
  coEnd();                                            // End
}

uint32_t *dotPlotter() {
  coStart();                                          // Start

  static uint8_t counter = 0;
  static uint32_t t = 0;
  
  Serial.println(F("Dot plotter starting"));
  
  for(;;) {
    Serial.print('.');
    ++counter %= 60;
    if(!counter) Serial.println();
    t = millis();                                     // Yet another way to
    do { coYield(); } while(millis() < t + 1000);     // delay by using coYield()
  }
  
  coEnd();                                            // End
}

//------------------------------------------------------------------------------
// Setup
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);                                 // Init USART
  Serial.println(F("Initializing"));
  pinMode(13, OUTPUT);                                // On board LED                            
}

//------------------------------------------------------------------------------
// Main loop. NOTE: A task can be reset by dereferencing and zeroing the
// state pointer returned by the task. For example:
//
// uint32_t *state = blinker();
// *state = 0;                                        // Reset blinker
//------------------------------------------------------------------------------
void loop() {
  blinker();                                          // Task 1
  console();                                          // Task 2
  dotPlotter();                                       // Task 3
}
