#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define J1_X 34
#define J1_Y 35

#define J2_X 32
#define J2_Y 33

#define J3_X 25
#define J3_Y 26

#define FWD_BTN 27
#define BWD_BTN 14
#define LEFT_BTN 12
#define RIGHT_BTN 13

int centerX = 2048;
int centerY = 2048;
int deadBand = 5;

int dataToSend[5] = {0,0,5,5,0};

uint8_t receiverMAC[] = {0xXX,0xXX,0xXX,0xXX,0xXX,0xXX};

int getAngle(int val, int center)
{
  int diff = val - center;
  int angle = (diff * 180) / 4096;

  if (abs(angle) < deadBand) angle = 0;

  return angle;
}

void setup()
{
  Serial.begin(115200);

  pinMode(FWD_BTN, INPUT_PULLUP);
  pinMode(BWD_BTN, INPUT_PULLUP);
  pinMode(LEFT_BTN, INPUT_PULLUP);
  pinMode(RIGHT_BTN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  esp_now_init();

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 10;
  esp_now_add_peer(&peerInfo);
}

void loop()
{
  int j1x = analogRead(J1_X);
  int j1y = analogRead(J1_Y);

  int j2x = analogRead(J2_X);
  int j2y = analogRead(J2_Y);

  int j3x = analogRead(J3_X);
  int j3y = analogRead(J3_Y);

  // ANGLES 
  int angle1 = getAngle(j1x, centerX);
  int angle2 = getAngle(j1y, centerY);

  int angle3 = getAngle(j2x, centerX);
  int angle4 = getAngle(j2y, centerY);

  int angle5 = getAngle(j3x, centerX);
  int angle6 = getAngle(j3y, centerY);

  // J1
  if(angle1>0){
    int J1 = 1;
  }
  else if(angle1<0){
    int J1 = 2;
  }
  else if(angle1==0){
    int J1 = 0;
  }

  if(angle2>0){
    int J1 = 3;
  }
  else if(angle2<0){
    int J1 = 4;
  }
  else if(angle2==0){
    int J1 = 0;
  }

  // J2
  if(angle3>0){
    int J2 = 1;
  }
  else if(angle3<0){
    int J2 = 2;
  }
  else if(angle3==0){
    int J2 = 0;
  }

  if(angle4>0){
    int J2 = 3;
  }
  else if(angle4<0){
    int J2 = 4;
  }
  else if(angle4==0){
    int J2 = 0;
  }

  // J3
  if(angle5>0){
    int J3 = 1;
  }
  else if(angle5<0){
    int J3 = 2;
  }
  else if(angle5==0){
    int J3 = 0;
  }

  if(angle6>0){
    int J3 = 3;
  }
  else if(angle6<0){
    int J3 = 4;
  }
  else if(angle6==0){
    int J3 = 0;
  }

  // PUSHBUTTONS
  if (!digitalRead(FWD_BTN)) dataToSend[0] = 1;
  else if (!digitalRead(BWD_BTN)) dataToSend[0] = 2;
  else if (!digitalRead(LEFT_BTN)) dataToSend[0] = 3;
  else if (!digitalRead(RIGHT_BTN)) dataToSend[0] = 4;
  else dataToSend[0] = 0;

  esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));

  delay(50);
}
