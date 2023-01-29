const int RightDirPin = 10;
const int RightStepPin = 12;
const int LeftDirPin=8;
const int LeftStepPin=6;
const int FrontDirPin=7;
const int FrontStepPin=9;

double w=1.0;
double r=0.5;
double r_wheel=0.029;
double fw_rpm, rw_rpm, lw_rpm;

const int stepsPerRevolution = 200;
int state=1;
unsigned long  current_t ,  deltat;
unsigned long  delayt = 3000000; //3seconds

  
class stepperMotor{
  public:
     
  void stop(void){
    enable = 0;
  }
  
  void start(void){
    enable = 1;
  }
  
  void init(int _pulsePin, int _dirPin, unsigned long _rpm, bool _direction){
    pulsePin      = _pulsePin;
    dirPin        = _dirPin;
    delayTime     = 3*1e6/(20*_rpm);
    direction     = _direction;
      
    togglePulse   = LOW;
    enable        = 0;
      
    pinMode(pulsePin, OUTPUT);
    pinMode(dirPin, OUTPUT);
  }
  
  void control(){
    currentTime = micros();
    digitalWrite(dirPin, direction);

    if(enable == 1){
      if( (currentTime - deltaTime) > delayTime ){
        pulseCount++;
 
        // Each HIGH or LOW is a "pulse"
        // But each step of the motor requires two "pulses"
        if(pulseCount % 2 == 0){
          stepCount++;
        }
  
        togglePulse = togglePulse == LOW ? HIGH : LOW;
        digitalWrite(pulsePin, togglePulse);
        deltaTime = currentTime;
      }
    }
  }
  
  void changeDirection(bool _direction){
    direction = _direction;
  }
  
  unsigned long steps(void){
    return stepCount;
  }
  
  void changeRPM(signed long rpm){
    if(rpm==0) enable=0;
    else{
      enable=1;
      if(rpm<0)direction=HIGH;
      else direction=LOW;
    
      delayTime = 3*1e6/(20*abs(rpm));
    }
  }
    
  public:
  unsigned long delayTime, deltaTime, currentTime;
  unsigned long pulseCount = 0;
  unsigned long stepCount = 0;
  int pulsePin, dirPin;
  bool direction, togglePulse, enable;
};
  
stepperMotor stepperFront , stepperRight , stepperLeft;

  
void setup() {
  Serial.begin(9600);
  digitalWrite(FrontStepPin, LOW);
  digitalWrite(RightStepPin, LOW);
  digitalWrite(LeftStepPin, LOW);

  stepperFront.init(FrontStepPin,FrontDirPin,0,HIGH);
  stepperRight.init(RightStepPin,RightDirPin,0,HIGH);
  stepperLeft.init(LeftStepPin,LeftDirPin,0,HIGH);

  stepperFront.start();
  stepperRight.start();
  stepperLeft.start();
}
  
void loop() {
  current_t = millis();
  double a=double(w*r*9.549297)/r_wheel;
  
  // circular 
  fw_rpm=-a*sin(double(w*current_t/1000));
  rw_rpm=(a/2)*(sin(double(w*current_t/1e3))-1.732*cos(double(w*current_t/1e3)));
  lw_rpm=(a/2)*(sin(w*current_t/1e3)+1.732*cos(w*current_t/1e3));
  
  stepperFront.changeRPM(int(fw_rpm));
  stepperRight.changeRPM(int(rw_rpm));
  stepperLeft.changeRPM(int(lw_rpm));

  stepperLeft.control();
  stepperRight.control();
  stepperFront.control();
}
