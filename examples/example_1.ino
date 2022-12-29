//------------------------------------------------------------------------------
//            Simple time-sliced multithreading on Arduino UNO
//                [see time_slicing.ino for more details]
//
// Task 1 handles LED blinking
// Task 2 handles serial communication
//
// Written by: Mikko Lyly
// Date:       29 dec 2022
//------------------------------------------------------------------------------
#define pushAddress(task) asm("push %A0 \n push %B0" :: "x" (task));

#define pushContext() asm("push r0\n in r0, 0x3f\n push r0\n push r1\n"        \
"clr r1\n push r2\n push r3\n push r4\n push r5\n push r6\n push r7\n"         \
"push r8\n push r9\n push r10\n push r11\n push r12\n push r13\n push r14\n"   \
"push r15\n push r16\n push r17\n push r18\n push r19\n push r20\n push r21\n" \
"push r22\n push r23\n push r24\n push r25\n push r26\n push r27\n push r28\n" \
"push r29\n push r30\n push r31");

#define popContext() asm("pop r31\n pop r30\n pop r29\n pop r28\n pop r27\n"   \
"pop r26\n pop r25\n pop r24\n pop r23\n pop r22\n pop r21\n pop r20\n"        \
"pop r19\n pop r18\n pop r17\n pop r16\n pop r15\n pop r14\n pop r13\n"        \
"pop r12\n pop r11\n pop r10\n pop r9\n pop r8\n pop r7\n pop r6\n pop r5\n"   \
"pop r4\n pop r3\n pop r2\n pop r1\n pop r0\n out 0x3f, r0\n pop r0");

#define NOF_TASKS 2
volatile uint16_t stackPointers[NOF_TASKS] = {RAMEND, RAMEND - 256};
volatile uint8_t currentTask = 0;
volatile uint32_t currentTicks = 0;
volatile bool ledStatus = false;

void task_1() {
  Serial.println(F("LED blinker starting"));
  
  for(;;) {
    while(ledStatus == false) {}
    digitalWrite(13, HIGH);
    delay(250);
    digitalWrite(13, LOW);
    delay(250);
  }
}

void task_2() {
  Serial.println(F("Serial console starting"));
  Serial.println(F("Type 'on' or 'off' to start or stop blinking"));
  String str = "";

  for(;;) {        
    while(Serial.available() > 0) {
      char c = Serial.read();
      switch(c) {
      case '\n':
        if(str == "on") {
          Serial.println(F("Blinking ON"));
          ledStatus = true;
        } else if(str == "off") {
          Serial.println(F("Blinking OFF"));
          ledStatus = false; 
        } else {
          Serial.println(F("Unknown command"));
        }
        str = "";
        break;
      case '\r':
        break;
      default:
        str += c;
      }
    }
  }
}

void setup() {
  pinMode(13, OUTPUT);
  
  Serial.begin(9600);
  Serial.println(F("Initializing"));

  SP = stackPointers[0];
  pushAddress(task_1);
  pushContext();
  stackPointers[0] = SP;

  SP = stackPointers[1];
  pushAddress(task_2);
  pushContext();
  stackPointers[1] = SP;
  
  cli();
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22);
  OCR2A = 249;
  TIMSK2 |= _BV(OCIE2A);
  sei();

  SP = stackPointers[currentTask];
  popContext();
  asm("ret");
}

void loop() {
}

ISR(TIMER2_COMPA_vect, ISR_NAKED) {
  pushContext();
  stackPointers[currentTask] = SP;
  ++currentTicks;
  ++currentTask %= NOF_TASKS;
  SP = stackPointers[currentTask];
  popContext();
  asm("reti");
}
