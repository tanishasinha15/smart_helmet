# smart_helmet
Solution code for BITJ hackathon

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

#define VS 10                      //vibration sensor
#define R 2
#define Y 4
#define MQ3 A0
# define buzzer 9
#define triggerPin 7                  
#define echoPin 8              
int gaslevel;

char replybuffer[255];
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
int vs = 10;
int shockVal = HIGH;

void setup() {
  Serial.begin(9600);
  Serial.println("Tech Ponder's UltraSonic Sensor Tutorial");
  pinMode(triggerPin, OUTPUT);                           //defining pins
  pinMode(echoPin, INPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
  pinMode(MQ3, INPUT);
  pinMode(R, OUTPUT);   //red led
  pinMode(Y, OUTPUT);   //yellow led
  pinMode(vs, INPUT);          

  while (!Serial);
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    while (1);
  }
  type = fona.type();
  switch (type) {
      Serial.print(F("Found "));
    case FONA800L:
      Serial.println(F("FONA 800L")); break;
    case FONA800H:
      Serial.println(F("FONA 800H")); break;
    case FONA808_V1:
      Serial.println(F("FONA 808 (v1)")); break;
    case FONA808_V2:
      Serial.println(F("FONA 808 (v2)")); break;
    case FONA3G_A:
      Serial.println(F("FONA 3G (American)")); break;
    case FONA3G_E:
      Serial.println(F("FONA 3G (European)")); break;
    default:
      Serial.println(F("???"));
      break;
  }
  char imei[15] = {0};
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }


}

void loop() {
  int duration, distance;
  digitalWrite(triggerPin, HIGH);
  delay(10);
  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
  delay(1000);
  Serial.print(distance);
  Serial.print("cm");
  Serial.println(" ");
  if (distance < 35)
  {
    digitalWrite(buzzer, HIGH);
    Serial.println("Buzzer On");
  }
  digitalWrite(buzzer, LOW);
  gaslevel = (analogRead(MQ3));
  gaslevel = map(gaslevel, 0, 1023, 0, 255);

  if (gaslevel > 100 && gaslevel <= 300) {
    digitalWrite(R, LOW);            //RED led is off
    _delay_ms(500);
    digitalWrite(Y, HIGH);            //YELLOW led is on
    _delay_ms(500);
  }
  else if (gaslevel > 300 && gaslevel <= 600) {
    digitalWrite(Y, LOW);                 //YELLOW led is off
    _delay_ms(500);
    digitalWrite(R, HIGH);                   //RED led is on
  }
  else
  {
    digitalWrite(R, LOW);                           //red led is off
    digitalWrite(Y, LOW);                           //YELLOW led is off
  }
  Serial.println(gaslevel);
  _delay_ms(100);

  //viberationFun();

}

void viberationFun(){
  shockVal = digitalRead (vs) ;
        int t=0;
        char sendto[11]="YOUR NUMBER";
        char sendto1[11]="YOUR NUMBER 2";
        char message[27]="Accident has been detected";
        if(shockVal == HIGH || shockVal == 1){
          if(t==0){
            Serial.println(shockVal);
        if (!fona.sendSMS(sendto, message) && !fona.sendSMS(sendto1, message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
          t=1;
        }
        delay(1000);
        if(!fona.sendSMS(sendto1, message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
          t=1;
        }
         }
        }else{
          t=0;
        }
}
