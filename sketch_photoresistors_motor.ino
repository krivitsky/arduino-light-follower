#define DEBUG false

#define MOTOR_PIN1 8
#define MOTOR_PIN2 9
#define MOTOR_PIN3 10
#define MOTOR_PIN4 11

#define MINIMAL_VOLTAGE_DIFF_MARGIN 10 // if voltage difference between photoresistors is less that this, that we assume it is equal and no adjustment needed
#define SIGNIFICANT_MOVE 30  // difference in voltage difference between photoresistors that is a significant adjustment

#define PHOTO_RESISTOR_LEFT A0
#define PHOTO_RESISTOR_RIGHT A1

#define SPEAKER_PIN 3
#define LED_MANUAL_PIN 4
#define LED_AUTO_PIN 5

#define JOYSTICK_PIN_BUTTON 7
#define JOYSTICK_PIN_X A14
#define JOYSTICK_PIN_Y A15


#define MAX_STEPS_AROUND 100

boolean significant_move = false;
boolean automatic_mode = true;

/*
 * Speaker to encapsulate basic beeping stuff
 */
class Speaker {
  public:
    static const int BEEP_PITCH_LOW = 1500;
    static const int BEEP_PITCH_HIGH = 1000;

    static const int BEEP_BIT_LONG = 8;
    static const int BEEP_BIT_SHORT = 4;

    void attach(int pin) {
      this -> pin = pin;
      pinMode(this -> pin, OUTPUT);
    }

    void beep(int tone_, int beat) {
      long duration = beat * TEMPO;
      
      long elapsed_time = 0;
        while (elapsed_time < duration) { 
        digitalWrite(pin, HIGH);
        delayMicroseconds(tone_ / 2);
        
        digitalWrite(pin, LOW);
        delayMicroseconds(tone_ / 2);
        
        elapsed_time += (tone_);
    } 
  }

  private:
    byte pin;
    const long TEMPO = 10000;
};

/*
 * Motor to contain the stepper motor movements
 */
class Motor {
  public:
    static const int SPEED_FAST = 6; // msec delay between sending HIGH to motor coils, experimental value;
    static const int SPEED_SLOW = 20; // msec delay between sending HIGH to motor coils, experimental value;
    static const int SPEED_DEFAULT = SPEED_FAST;
    
    void attach(byte pin1, byte pin2, byte pin3, int pin4) {
      this -> pin1 = pin1;
      this -> pin2 = pin2;
      this -> pin3 = pin3;
      this -> pin4 = pin4;

      setupPins(); 
    }
    
    void stepForward() {
      stepA(); 
      stepB();
      stepC();
      stepD();
    }
    
    void stepBackward() {
      stepD();
      stepC();
      stepB();
      stepA(); 
    }
    
    void stop() {
      stepping(LOW, LOW, LOW, LOW); 
    }

  private:
    int delayBetweenPulses = SPEED_DEFAULT; 
    byte pin1, pin2, pin3, pin4;

    void setupPins() {
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
      pinMode(pin3, OUTPUT);
      pinMode(pin4, OUTPUT);
    }
  
    void stepA() {
      stepping(HIGH, LOW, LOW, LOW);
    }
    
    void stepB() {
      stepping(LOW, HIGH, LOW, LOW);
    }
    
    void stepC() {
       stepping(LOW, LOW, HIGH, LOW);
    }
    
    void stepD() {
       stepping(LOW, LOW, LOW, HIGH);
    }
    
    void stepping(byte signal1, byte signal2, byte signal3, byte signal4) {
      digitalWrite(pin1, signal1);
      digitalWrite(pin2, signal2);
      digitalWrite(pin3, signal3);
      digitalWrite(pin4, signal4);
      
      wait();
    }
    
    void wait() {
      delay(delayBetweenPulses);
    }    
};

/*
 * Joystick to get user inputs
 */
class Joystick {
  public:
    typedef enum  {
        NONE = 0,
        LEFT = 1,      
        RIGHT = 2,
    } DirectionEnumType; 
    
    Joystick(void (*func)(void)) {
      this -> callback = *func;  
    }

    void attach(byte pinX, byte pinY, byte pinButton) {
      this -> pinX = pinX;
      this -> pinY = pinY;
      this -> pinButton = pinButton;
      setupPins();
    }

    DirectionEnumType readX() {            
      return decodeDirection(analogRead(pinX));
    }

    DirectionEnumType readY() {
      return decodeDirection(analogRead(pinY));
    }

    void checkButtonState() {    
      int buttonState = digitalRead(pinButton);
      
      if (!isButtonPressed && buttonState == LOW) {
        isButtonPressed = true;
        this -> callback();
      } else if (buttonState == HIGH) {
        isButtonPressed = false;
      }
    }

  private:
    byte pinX, pinY, pinButton;
    boolean isButtonPressed = false;
    void (*callback)(void);

    void setupPins() {
      pinMode(pinX, INPUT);
      pinMode(pinY, INPUT);
      pinMode(pinButton, INPUT_PULLUP);
    }

  DirectionEnumType decodeDirection(int value) {
      if (value <= 200) {
        return DirectionEnumType::LEFT;
      } else if (value >= 600) {
        return DirectionEnumType::RIGHT;
      }
      
      return DirectionEnumType::NONE;
    }
};

Speaker speaker;
Motor motor;
Joystick joystick = Joystick(&toggleModeOnButtOnClick);

void setup() {    
  Serial.begin(9600);

  motor.attach(MOTOR_PIN1, MOTOR_PIN2, MOTOR_PIN3, MOTOR_PIN4);
  speaker.attach(SPEAKER_PIN);
  joystick.attach(JOYSTICK_PIN_X, JOYSTICK_PIN_Y, JOYSTICK_PIN_BUTTON);

  pinMode(LED_AUTO_PIN, OUTPUT);
  pinMode(LED_MANUAL_PIN, OUTPUT);
  actualizeLEDs();

  beepReady();
}

void loop() {
  joystick.checkButtonState();
  
  if (automatic_mode) {
    
    readMeasurementsAndMove();
  
  } else {
  
    readJoystickAndMove();
  
  }
}

void readJoystickAndMove() {
  Joystick::DirectionEnumType x = joystick.readX();
  Joystick::DirectionEnumType y = joystick.readY();
  
  if (x == Joystick::DirectionEnumType::LEFT || y == Joystick::DirectionEnumType::LEFT) {
    motor.stepBackward();   
  } else if (x == Joystick::DirectionEnumType::RIGHT || y == Joystick::DirectionEnumType::RIGHT) {
    motor.stepForward(); 
  }
}

void readMeasurementsAndMove() {
  int lValue = analogRead(PHOTO_RESISTOR_LEFT);
  int rValue = analogRead(PHOTO_RESISTOR_RIGHT);

  debugPrint(lValue);
  debugPrint("...");
  debugPrintln(rValue);

  byte diff;

  if (lValue > rValue) {
    diff = lValue - rValue;
  } else {
    diff = rValue - lValue;
  }

  if (diff > SIGNIFICANT_MOVE) {
    significant_move = true;
  }

  debugPrint("diff: ");
  debugPrintln(diff);    

  if (diff > MINIMAL_VOLTAGE_DIFF_MARGIN) {
    if (lValue < rValue ) {
      debugPrintln("backward");
      motor.stepBackward();   
    } else if (lValue > rValue) {
      motor.stepForward();
      debugPrintln("forward"); 
    }
  } else {
    if (significant_move) {
      beepOnce();   
      significant_move = false;
    }
  }
}

void toggleModeOnButtOnClick() {
  automatic_mode ^= true;
  actualizeLEDs();

  beepOnce();  
}

void actualizeLEDs() {
  digitalWrite(LED_AUTO_PIN, (automatic_mode ? HIGH : LOW)); 
  digitalWrite(LED_MANUAL_PIN, (automatic_mode ? LOW : HIGH)); 
}

void beepReady() {
  speaker.beep(Speaker::BEEP_PITCH_HIGH, Speaker::BEEP_BIT_LONG);
  speaker.beep(Speaker::BEEP_PITCH_LOW, Speaker::BEEP_BIT_LONG);
}

void beepOnce() {
  speaker.beep(Speaker::BEEP_PITCH_HIGH, Speaker::BEEP_BIT_SHORT);
}

void debugPrint(String msg) {
  if (DEBUG) {
    Serial.print(msg);    
  }
}

void debugPrint(int msg) {
  if (DEBUG) {
    Serial.print(msg);    
  }
}

void debugPrintln(String msg) {
  if (DEBUG) {
    Serial.println(msg);    
  }
}

void debugPrintln(int msg) {
  if (DEBUG) {
    Serial.println(msg);    
  }
}
