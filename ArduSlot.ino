//Calibration des REC
  #define NEU_SIGNAL 1480 
  #define MAX_SIGNAL 2150
  #define MIN_SIGNAL 1036
  #define DEADBAND 100

/*
 * Define pins used to provide RC PWM signal to Arduino
 * Pins 8, 9 and 10 are used since they work on both ATMega328 and 
 * ATMega32u4 board. So this code will work on Uno/Mini/Nano/Micro/Leonardo
 * See PinChangeInterrupt documentation for usable pins on other boards
 */
  const byte channel_pin[] = {2, 3, 4};
  volatile unsigned long rising_start[] = {0, 0, 0};
  volatile long channel_length[] = {0, 0, 0};
  signed int thr[] = {0, 0, 0};

//VAR MOTORS
  #include <PinChangeInterrupt.h> //lire les int facilement
  
  int IN1=5;   //moteurs avec L298N
  int IN2=6;
  int IN3=7;
  int IN4=8;
  int ENA=9;
  int ENB=10;

//controle des vitesses
  float speedRatio_A = 1; //pour ajuster en fonction des moteurs, 1 = plein voltage
  float speedRatio_B = 1; 
  boolean revert_A = true; //pour inverser des voies
  boolean revert_B = true;
  byte speedA;
  byte speedB;


void setup() {
  
  //configuration des interruptions
  pinMode(channel_pin[0], INPUT);
  pinMode(channel_pin[1], INPUT);
  pinMode(channel_pin[2], INPUT);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[0]), onRising0, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[1]), onRising1, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[2]), onRising2, CHANGE);
  
  //Serial
  Serial.begin(9600);
  
  //MOTEURS
  pinMode (IN1, OUTPUT); 
  pinMode (IN2, OUTPUT); 
  pinMode (ENA, OUTPUT); 
  pinMode (IN3, OUTPUT); 
  pinMode (IN4, OUTPUT); 
  pinMode (ENB, OUTPUT);
  speedA = 0;
  speedB = 0;
  
} //SETUP FIN

//Fontions de lecture des signaux 
void processPin(byte pin) {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(channel_pin[pin]));

  if(trigger == RISING) {
    rising_start[pin] = micros();
  } else if(trigger == FALLING) {
    channel_length[pin] = micros() - rising_start[pin];
  }
}

void onRising0(void) {
  processPin(0);
}

void onRising1(void) {
  processPin(1);
}

void onRising2(void) {
  processPin(2);
}
//FIN Fontions de lecture des signaux 

//conversion des signaux en int signé (max 255)
signed int convert(int SIGNAL) {
  if (SIGNAL > NEU_SIGNAL+DEADBAND) {
  return min(map(SIGNAL, NEU_SIGNAL, MAX_SIGNAL, 1, 255),255);
  }
  if (SIGNAL < NEU_SIGNAL-DEADBAND) {
  return max(map(SIGNAL, MIN_SIGNAL, NEU_SIGNAL, -255, -1),-255);
  }
  return 0;
}
//fin de conversion


void loop() {

  thr[0] = convert(channel_length[0]);
  thr[1] = convert(channel_length[1]); 
  thr[2] = convert(channel_length[2]); 
  
  /*
  Serial.print(channel_length[0]);
  Serial.print(" | ");
  Serial.print(channel_length[1]);
  Serial.print(" | ");
  Serial.print(channel_length[2]);
  Serial.println("");
  */
  Serial.print(thr[0]);
  Serial.print(" | ");
  Serial.print(thr[1]);
  Serial.print(" | ");
  Serial.print(thr[2]);
  Serial.println("");
 
 
//MOTEUR_A sur piste 1

  if (thr[0] == 0) { //FREIN
  digitalWrite (IN1 , HIGH); 
  digitalWrite (IN2, HIGH);
  analogWrite (ENA, 1); 
  }
  
  if (thr[0] > 0) {  //MARCHE AVANT
  digitalWrite (IN1 , HIGH); 
  digitalWrite (IN2, LOW);
  analogWrite (ENA, min(abs(thr[0])*speedRatio_A,255));
  }
  
  if (thr[0] < 0) { //MARCHE ARRIERE
  digitalWrite (IN1 , LOW); 
  digitalWrite (IN2, HIGH);
  analogWrite (ENA, min(abs(thr[0])*speedRatio_A,255));
  } 

  if (thr[1] > 0 && revert_A == true) {  //MARCHE AVANT mode inversé
  digitalWrite (IN1 , LOW); 
  digitalWrite (IN2, HIGH);
  analogWrite (ENB, min(abs(thr[0])*speedRatio_A,255));
  }
  
  if (thr[1] < 0 && revert_A == true) { //MARCHE ARRIERE mode inversé
  digitalWrite (IN1 , HIGH); 
  digitalWrite (IN2, LOW);
  analogWrite (ENB, min(abs(thr[0])*speedRatio_A,255));
  } 


//MOTEUR_B sur piste 2

  if (thr[1] == 0) { //FREIN
  digitalWrite (IN3 , HIGH); 
  digitalWrite (IN4, HIGH);
  analogWrite (ENB, 1);
  }
  
  if (thr[1] > 0) {  //MARCHE AVANT mode normal
  digitalWrite (IN3 , HIGH); 
  digitalWrite (IN4, LOW);
  analogWrite (ENB, min(abs(thr[1])*speedRatio_B,255));
  }
  
  if (thr[1] < 0) { //MARCHE ARRIERE mode normal
  digitalWrite (IN3 , LOW); 
  digitalWrite (IN4, HIGH);
  analogWrite (ENB, min(abs(thr[1])*speedRatio_B,255));
  } 
  
  if (thr[1] > 0 && revert_B == true) {  //MARCHE AVANT mode inversé
  digitalWrite (IN3 , LOW); 
  digitalWrite (IN4, HIGH);
  analogWrite (ENB, min(abs(thr[1])*speedRatio_B,255));
  }
  
  if (thr[1] < 0 && revert_B == true) { //MARCHE ARRIERE mode inversé
  digitalWrite (IN3 , HIGH); 
  digitalWrite (IN4, LOW);
  analogWrite (ENB, min(abs(thr[1])*speedRatio_B,255));
  } 
 
} //LOOP
