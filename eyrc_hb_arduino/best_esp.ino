#include <WiFi.h>
#include<WiFiUdp.h>

#define BUZZER_PIN 22
#define SERVO_PIN 23

// WiFi credentials
const char* ssid = "X2 Pro";                   
const char* password =  "debrupdeb";  

const char * udpAddress = "192.168.1.100";
unsigned int localUdpPort = 44444;

char msg[255]; 
char* pch; 
int i, arr[]={0,0,0,0};
int pos = 0;
int dutyCycle = 0;
const int PWMFreq = 50;
const int PWMChannel = 0;
const int PWMResolution = 8;

WiFiUDP Udp;


//Pins
const int RightDirPin = 25;
const int RightStepPin = 26;
const int FrontDirPin=16;
const int FrontStepPin=13;
const int LeftDirPin=12;
const int LeftStepPin=14;

//const int servo = 11 ;  //servo pins 1A 

const int stepsPerRevolution = 200;
int state=1, incomingByte=0;
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
  Serial.begin(115200);

  
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  /* Attach the LED PWM Channel to the GPIO Pin */
  ledcAttachPin(SERVO_PIN, PWMChannel);
  ledcWrite(PWMChannel, dutyCycle);
  
  pinMode(BUZZER_PIN, OUTPUT);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    
    digitalWrite(BUZZER_PIN, HIGH);
     delay(100);
     digitalWrite(BUZZER_PIN, LOW); 
     delay(100);
  Serial.println("...");
  }
  Udp.begin(localUdpPort);
  Serial.print("WiFi connected with IP: ");
  digitalWrite(BUZZER_PIN, HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN, LOW);   
  Serial.println(WiFi.localIP().toString().c_str());
  
  digitalWrite(FrontStepPin, LOW);
  digitalWrite(RightStepPin, LOW);
  digitalWrite(LeftStepPin, LOW);

  
  stepperFront.init(FrontStepPin,FrontDirPin,0,HIGH);
  stepperRight.init(RightStepPin,RightDirPin,0,HIGH);
  stepperLeft.init(LeftStepPin,LeftDirPin,0,HIGH);

  
  stepperFront.start();
  stepperRight.start();
  stepperLeft.start();
  
  ledcWrite(PWMChannel,20);

}

void loop() {
  
  Udp.beginPacket(udpAddress,localUdpPort);
  int packetsize = Udp.parsePacket();

  if(packetsize){
    int len =  Udp.read(msg,255);
  
    if(len>0){
      msg[len]=0;
    }
    Serial.println(msg);
    pch = strtok(msg,",");
    i=0;
    while(pch != NULL)
    {
      arr[i]=atoi(pch);
      pch = strtok(NULL , ",");
      i++;
    }  
  }

  stepperFront.changeRPM(arr[0]);
  stepperRight.changeRPM(arr[1]);
  stepperLeft.changeRPM(arr[2]);
  ledcWrite(PWMChannel ,arr[3] + 10);
  
  
  
  
  
  stepperFront.control();
  stepperLeft.control();
  stepperRight.control();

}
