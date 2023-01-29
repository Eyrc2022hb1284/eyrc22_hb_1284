const int RightDirPin = 10;
const int RightStepPin = 12;
const int LeftDirPin=8;
const int LeftStepPin=6;
const int FrontDirPin=7;
const int FrontStepPin=9;

int v =70;

const int stepsPerRevolution = 200;
int state=1;
unsigned long  currentt ,  deltat;
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
 
  currentt = micros();
  //l shape 
  if(state==1){
    stepperFront.changeRPM(0);
    stepperRight.changeRPM(-80);
    stepperLeft.changeRPM(80);
   
    if(currentt-deltat>delayt)
    {
      deltat=currentt;
      state++;
    }
  }
  else if (state==2)
  {
    stepperFront.changeRPM(0);
    stepperRight.changeRPM(80);
    stepperLeft.changeRPM(-80);
  
    if(currentt-deltat>delayt)
    {
      deltat=currentt;
      state++;
    }
  }
  else if (state==3){
    stepperFront.changeRPM(-100);
    stepperLeft.changeRPM(50);
    stepperRight.changeRPM(50);

    if(currentt-deltat>delayt)
    {
      deltat=currentt;
      state++;
    }
  }
  else if (state==4){
    
    stepperFront.changeRPM(100);
    stepperLeft.changeRPM(-50);
    stepperRight.changeRPM(-50);

    if(currentt-deltat>delayt)
    {
      deltat=currentt;
      state++;
    }
  }
  else{
    stepperFront.changeRPM(0);
    stepperLeft.changeRPM(0);
    stepperRight.changeRPM(0);

    if(currentt-deltat>delayt)
    {
      deltat=currentt;
      state=1;
    }
  }

  stepperLeft.control();
  stepperRight.control();
  stepperFront.control();

}
