/* 
▪ * Team Id: 1284
▪ * Author List: Aman Sagar , Pratyush Kumar Mohanty , Debrup Panda  
▪ * Filename: best_esp
▪ * Theme: Hola Bot  
▪ * Functions:  stop(void) , start(void) , init(int , int , unsigned long, bool ),control() ,  changeDirection(bool ) ,steps(void) , changeRPM(signed long ),
setup() , loop(). 
▪ * Global Variables: BUZZER_PIN , SERVO_PIN , ssid , password , udpAddress , localUdpPort , msg , pch , i , arr , pos , dutyCycle ,  PWMFreq , PWMChannel
,PWMResolution , Udp ,  RightDirPin , RightStepPin , FrontDirPin, FrontStepPin ,LeftDirPin , LeftStepPin , stepsPerRevolution , state , incomingByte ,
currentt , delayt , deltat 
▪ */ 


//including libraries
#include <WiFi.h>
#include<WiFiUdp.h>


#define BUZZER_PIN 22
#define SERVO_PIN 23

//variables

char msg[255]; 
char* pch; 
int i, arr[]={0,0,0,0};
int pos = 0;
int dutyCycle = 0;
const int PWMFreq = 50;
const int PWMChannel = 0;
const int PWMResolution = 8;

//Pins
const int RightDirPin = 25;
const int RightStepPin = 26;
const int FrontDirPin=16;
const int FrontStepPin=13;
const int LeftDirPin=12;
const int LeftStepPin=14;

// WiFi credentials
const char* ssid = "X2 Pro";                   
const char* password =  "debrupdeb";  

//wifi address && port 
const char * udpAddress = "192.168.1.100";
unsigned int localUdpPort = 44444;

//class used variables 
const int stepsPerRevolution = 200;
int state=1, incomingByte=0;
unsigned long  currentt ,  deltat;
unsigned long  delayt = 3000000; //3seconds


// object 
WiFiUDP Udp;







//Stepper Motor class
class stepperMotor{
  public:
/* 
▪ * Function Name:stop(void)
▪ * Input: void
▪ * Output: NULL
▪ * Logic: Its just helps to enable the stepper motor motion
▪ * Example Call: stop()
▪ */ 

  void stop(void){
    enable = 0;
  }
/* 
▪ * Function Name:start(void) 
▪ * Input: void
▪ * Output: NULL
▪ * Logic: Its just helps to disable the stepper motor motion 
▪ * Example Call: start() 
▪ */ 

  void start(void){
    enable = 1;
  }
  
/* 
▪ * Function Name:init(int , int , unsigned long , bool )
▪ * Input: pulse pin , direction pin  , rpm , direction 
▪ * Output: Its just help to assigned the variable with the passed values 
▪ * Logic: Calculate the delay time and intiate the pins too
▪ * Example Call: stepperFront.init(FrontStepPin,FrontDirPin,160,HIGH); 
▪ */ 

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
/* 
▪ * Function Name:control
▪ * Input: NULL 
▪ * Output: using digitalWrite it makes the pins high and low with different time cycle
▪ * Logic: call the time and for each time ranging from 700 us to 1100 us it makes the pin high and low  
▪ * Example Call: control()
▪ */ 

  void control(){
    //intiate the time
    currentTime = micros();
    digitalWrite(dirPin, direction);

    if(enable == 1){
      if( (currentTime - deltaTime) > delayTime ){
        pulseCount++;
 
        // Each HIGH or LOW is a "pulse"   ___|--|___|--|___
        // But each step of the motor requires two "pulses" for making it a complete cycle 
        if(pulseCount % 2 == 0){
          stepCount++;
        }
  
        togglePulse = togglePulse == LOW ? HIGH : LOW;
        digitalWrite(pulsePin, togglePulse);
        deltaTime = currentTime;
      }
    }
  }
/* 
▪ * Function Name: changeDirection(bool)
▪ * Input: bool value of directrion either high or low  
▪ * Output: NULL
▪ * Logic: Its just help to assign a variable which changes the direcrtion of Stepper motor shaft / Wheel 
▪ * Example Call: changeDirection(LOW)
▪ */ 

  void changeDirection(bool _direction){
    direction = _direction;
  }
  
/* 
▪ * Function Name:changeRPM
▪ * Input: RPM (signed long)
▪ * Output: Its just changed the rpm value of the wheel 
▪ * Logic: Based on the rpm value it generate a time delay between high and low pulses
▪ * Example Call: changeRPM(-80) 
▪ */ 

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

  //variables 
  unsigned long delayTime, deltaTime, currentTime;
  unsigned long pulseCount = 0;
  unsigned long stepCount = 0;
  int pulsePin, dirPin;
  bool direction, togglePulse, enable;
};


 //Class objects
stepperMotor stepperFront , stepperRight , stepperLeft;
/* 
▪ * Function Name:setup
▪ * Input: NULL
▪ * Output: NULL
▪ * Logic:  Setting up all the pins and intiating wifi calls ,starting udp and connecting to the assigned wifi credentials , also intiatlizing motors with zero rpm and also with buzzer and servo  
▪ * Example Call: setup()
▪ */ 

void setup() {
  //Serial Monitor
  Serial.begin(115200);


  //Timer Configuration by specifying the PWM signal’s frequency and duty cycle resolution
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  ledcAttachPin(SERVO_PIN, PWMChannel);
  ledcWrite(PWMChannel, dutyCycle);

  
  pinMode(BUZZER_PIN, OUTPUT);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);


  while (WiFi.status() != WL_CONNECTED) {
     //connecting ,  indication with buzzer sound
    digitalWrite(BUZZER_PIN, HIGH);
     delay(100);
     digitalWrite(BUZZER_PIN, LOW); 
     delay(100);
  Serial.println("...");
  }

  //udp begin 
  Udp.begin(localUdpPort);

  //printing local ip 
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP().toString().c_str());

  //connected ,  indication with buzzer sound
  digitalWrite(BUZZER_PIN, HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN, LOW);   


  //digital Write 
  digitalWrite(FrontStepPin, LOW);
  digitalWrite(RightStepPin, LOW);
  digitalWrite(LeftStepPin, LOW);

  //intializing
  stepperFront.init(FrontStepPin,FrontDirPin,0,HIGH);
  stepperRight.init(RightStepPin,RightDirPin,0,HIGH);
  stepperLeft.init(LeftStepPin,LeftDirPin,0,HIGH);

  //chnging enable to 1
  stepperFront.start();
  stepperRight.start();
  stepperLeft.start();

  //servo default 
  ledcWrite(PWMChannel,20);

}
/* 
▪ * Function Name: loop 
▪ * Input: NULL 
▪ * Output: NULL 
▪ * Logic: recieving values then splitting based on comma "," , and passing the value to the fucntion which changes the rpm of stepper motor (Generalised Approach)
▪ * Example Call: loop()
▪ */ 

void loop() {
  
  //begning recieving over wifi 
  Udp.beginPacket(udpAddress,localUdpPort);
  int packetsize = Udp.parsePacket();

  if(packetsize){
      //reading recieved msg
      int len =  Udp.read(msg,255);
    
      if(len>0){
        msg[len]=0;
    }
    
    //printing the msg
    Serial.println(msg);

    //spliting the values based on "," (comma).
    pch = strtok(msg,",");
    i=0;
    while(pch != NULL)
    {
      
    //changing to integer
      arr[i]=atoi(pch);
      pch = strtok(NULL , ",");
      i++;
    }   
    
  }

  //changing rpm to recieved values
  stepperFront.changeRPM(arr[0]);
  stepperRight.changeRPM(arr[1]);
  stepperLeft.changeRPM(arr[2]);
  //changing angle to recieved values with passing higher pwm signals
  ledcWrite(PWMChannel ,arr[3] + 10);
  
  //calling the function for changing the pulse with time cycle loop
  stepperFront.control();
  stepperLeft.control();
  stepperRight.control();

}
