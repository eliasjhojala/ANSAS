
// Amount of trusses
const int N = 2;
class Truss {
  unsigned long currentHeight;
  int upPin, downPin;
  
  public: // Public enums
    enum Mode {
      goToTarget, manualUpDown
    };
    enum State {
      notMoving, movingUp, movingDown
    };
  private:
  Mode operatingMode;
  const static Mode defaultMode = Mode::goToTarget;
  
  State state, targetState;
  const static State defaultState = State::notMoving;
  unsigned long stateLastChanged;
  
  bool killSwitchEngaged;
  unsigned long killSwitchLastReceived;
  
  // DEBUG - true jos "älä vaadi killswitchin päivitystä koko ajan"
  const bool nonStrictKillSwitch = true;
  
  bool killSwitchDataValid() {
    return millis() - killSwitchLastReceived <= killSwitchReceiveInterval || nonStrictKillSwitch;
  }
  
  bool allowMovement = true;
  
  bool mayMoveUp() {
    return !killSwitchEngaged && killSwitchDataValid() && allowMovement;
  }
  
  bool mayMoveDown() {
    return currentHeight >= minHeight && killSwitchDataValid() && allowMovement;
  }
  
  void _setState(State newState, bool force=false) {
    if (newState != state) {
      if (force || millis() - stateLastChanged >= stateChangeThrottle) {
        state = newState;
        stateLastChanged = millis();
      }
    }
  }
  
  void updatePins(State state) {
    // Relays take complement
    digitalWrite(upPin, !(state == State::movingUp));
    digitalWrite(downPin, !(state == State::movingDown));
  }
  
  unsigned long lastLoopCall;
  
  public:
    const static unsigned long
      // ([pääarvo]L<<8 + [fine]) << 16
      maxHeight = (128L<<8 + 0) << 16,
      minHeight = (114L<<8 + 0) << 16,
      
      movementTreshold = 1<<20,
      
      // unit-height / ms
      // kuinka paljon dmx-arvo-korkeus (16bit) muuttuu millisekunnissa * 2^16
      // (ylädmx-aladmx) * [koko matka sekunteina] * 1000 * 2^8 * 2^16
      speedUp = 4697,
      speedDown = 4697,
      
      // ms
      stateChangeThrottle = 1000,
      killSwitchReceiveInterval = 200;
    
    unsigned long targetHeight;
    
    int hChan, hChanFine;
    
    Truss(unsigned long height, int up, int down, int hChan, int hChanFine, Mode mode = defaultMode) :
      currentHeight(height),
      upPin(up),
      downPin(down),
      targetHeight(height),
      operatingMode(mode),
      state(defaultState),
      targetState(defaultState),
      stateLastChanged(0),
      hChan(hChan),
      hChanFine(hChanFine)
    {}
    
    void setup() {
      pinMode(upPin, OUTPUT);
      pinMode(downPin, OUTPUT);
    }
    unsigned long _lTargetHeight;
    bool _movDone = 0;
    State _movDir;
    void loop() {
      // Update position
      unsigned long now = millis();
      if (state == State::movingUp) {
        currentHeight += (now - lastLoopCall) * speedUp;
      } else if (state == State::movingDown) {
        currentHeight -= (now - lastLoopCall) * speedDown;
      }
      lastLoopCall = now;
      
      if (operatingMode == Mode::goToTarget) {
        if (targetHeight != _lTargetHeight) {
          _movDone = 0;
          if (targetHeight > currentHeight && targetHeight - currentHeight >= movementTreshold) {
            _movDir = State::movingUp;
          } else if (currentHeight > targetHeight && currentHeight - targetHeight >= movementTreshold) {
            _movDir = State::movingDown;
          } else {
            _movDir = State::notMoving;
            _movDone = 1;
          }
          _lTargetHeight = targetHeight;
        }
        switch (_movDir) {
          case State::movingUp:
            if (currentHeight >= targetHeight) _movDone = 1;
          break;
          case State::movingDown:
            if (targetHeight >= currentHeight) _movDone = 1;
          break;
        }
        if (!_movDone) {
          if (targetHeight >= currentHeight) {
            targetState = State::movingUp;
          } else if (currentHeight >= targetHeight) {
            targetState = State::movingDown;
          }
        } else targetState = State::notMoving;
      }
      bool allowUp = mayMoveUp(), allowDown = mayMoveDown();
      bool forceStop = (!allowUp && state == State::movingUp) || (!allowDown && state == State::movingDown);
      if (forceStop) {
        if (targetState != state) _setState(targetState, true);
        else _setState(State::notMoving, true);
      } else {
        if (allowUp && targetState == State::movingUp) _setState(targetState);
        else if (allowDown && targetState == State::movingDown) _setState(targetState);
        else _setState(State::notMoving);
      }
      updatePins(state);
    }
    
    void setState(State newState) {
      if (operatingMode == Mode::manualUpDown) {
        targetState = newState;
      }
    }
    
    void setTarget(unsigned int pos) {
      if (operatingMode == Mode::goToTarget) {
        targetHeight = (unsigned long)pos << 16;
      }
    }
    
    void setKillSwitch(bool status) {
      digitalWrite(13, status);
      killSwitchEngaged = status;
      if (status) {
        currentHeight = maxHeight;
      }
      killSwitchLastReceived = millis();
    }
    
    void setAllowMovement(bool val) {
      allowMovement = val;
    }
} trusses[N] = {
  // [alkukorkeus], [ylös rele pin], [alas rele pin], [korkeuskanava], [fine]
  Truss(Truss::minHeight, 22, 23, 452, 453), // 462
  Truss(Truss::minHeight, 24, 25, 482, 483)  // 492
};

#include <DMXSerial.h>


void setup() {
  for (int i = 0; i < N; ++i) {
    trusses[i].setup();
  }
  Serial3.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, 0);
  DMXSerial.init(DMXReceiver);
}

int ksRequestStatus = 0;
bool ksRequestSent = false;
unsigned long ksTime;
void loop() {
  if (ksRequestSent) {
    if (Serial3.available()) {
      digitalWrite(13, 0);
      int in = (int)Serial3.read();
      if ((in & 0x0E) == 0 && (in & 0xE0) == 0) {
        trusses[in >> 4].setKillSwitch(in & 0x01);
      }
      while (Serial3.available()) Serial3.read();
      ksRequestSent = false;
      //ksRequestStatus = (ksRequestStatus + 1) % N;
    }
    if (millis() - ksTime > 200) {
      digitalWrite(13, 0);
      delay(5);
      ksRequestSent = 0; 
    }
  }
  if (!ksRequestSent) {
    digitalWrite(13, 1);
    ksTime = millis();
    Serial3.write(ksRequestStatus << 4 | 0x02);
    ksRequestSent = 1;
  }
  
  unsigned long lastDMX = DMXSerial.noDataSince();
  if (lastDMX < 2000) {
    // analogWrite(13, DMXSerial.read(482));
    for (int i = 0; i < N; ++i) {
      trusses[i].setAllowMovement(true);
      unsigned int h = DMXSerial.read(trusses[i].hChan);
      unsigned int hFine = DMXSerial.read(trusses[i].hChanFine);
      unsigned int fval = h << 8 | hFine;
      trusses[i].setTarget(fval);
    }
  } else {
    for (int i = 0; i < N; ++i) {
      trusses[i].setAllowMovement(false);
    }
  }
  
  // Anna trusseille tavoitekorkeus finenä (yht 16bit -- huom käytä unsigned int):
  // trusses[0].setTarget(korkeus0);
  // trusses[1].setTarget(korkeus1);
  
  // Anna trusseille killSwitchin arvo
  // trusses[0].setKillSwitch(true tai false);
  // ...
  for (int i = 0; i < N; ++i) {
    trusses[i].loop();
  }
}


