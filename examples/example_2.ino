//------------------------------------------------------------------------------
//                   Simple co-routines on Arduino UNO
//                  [see co_routines.ino for more details]
//
// Task 1 handles LED blinking
// Task 2 handles serial communication
//
// Written by: Mikko Lyly
// Date:       29 dec 2022
//------------------------------------------------------------------------------
#define coStart() static uint32_t S = 0, T = 0; switch(S) { case 0:;
#define coEnd() } S = __LINE__; return &S;
#define coYield() S = __LINE__; return &S; case __LINE__:;
#define coWaitUntil(X) S = __LINE__; case __LINE__: if(!(X)) return &S;
#define coDelay(X) T = millis(); coWaitUntil((T + X) < millis());

bool ledStatus = false;

uint32_t *task_1() {
  coStart();
    
  Serial.println(F("LED blinker starting"));
  
  for(;;) {
    coWaitUntil(ledStatus == true);
    digitalWrite(13, HIGH);
    coDelay(250);
    digitalWrite(13, LOW);
    coDelay(250);
  }

  coEnd();
}

uint32_t *task_2() {
  coStart();
  
  Serial.println(F("Serial console starting"));
  Serial.println(F("Type 'on' or 'off' to start or stop blinking"));
  static String str = "";

  for(;;) {
    coWaitUntil(Serial.available());
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

  coEnd();
}

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  Serial.println(F("Initializing"));
}

void loop() {
  task_1();
  task_2();
}
