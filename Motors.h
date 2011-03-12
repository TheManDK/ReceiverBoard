/******************************************************/
/********************* PWM Motors *********************/
/******************************************************/
class Motors_PWM : public Motors {
private:
  #if defined(AeroQuadMega_v2) || defined(AeroQuadMega_Wii) || defined (AeroQuadMega_CHR6DM)
    #define FRONTMOTORPIN  2
    #define REARMOTORPIN   3
    #define RIGHTMOTORPIN  5
    #define LEFTMOTORPIN   6
    #define LASTMOTORPIN   7
  #else
    #define FRONTMOTORPIN  3
    #define REARMOTORPIN   9
    #define RIGHTMOTORPIN 10
    #define LEFTMOTORPIN  11
    #define LASTMOTORPIN  12
  #endif
  int minCommand;
  byte pin;

 public:
  Motors_PWM() : Motors(){
   // Analog write supports commands from 0-255 => 0 - 100% duty cycle
   // Using 125-250 for motor setting 1000-2000
  }

  void initialize(void) {
    commandAllMotors(1000);
  }

  void write(void) {
    analogWrite(FRONTMOTORPIN, motorCommand[FRONT] / 8);
    analogWrite(REARMOTORPIN,  motorCommand[REAR]  / 8);
    analogWrite(RIGHTMOTORPIN, motorCommand[RIGHT] / 8);
    analogWrite(LEFTMOTORPIN,  motorCommand[LEFT]  / 8);

  }

  void commandAllMotors(int _motorCommand) {   // Sends commands to all motors
    analogWrite(FRONTMOTORPIN, _motorCommand / 8);
    analogWrite(REARMOTORPIN,  _motorCommand / 8);
    analogWrite(RIGHTMOTORPIN, _motorCommand / 8);
    analogWrite(LEFTMOTORPIN,  _motorCommand / 8);
  }
};
