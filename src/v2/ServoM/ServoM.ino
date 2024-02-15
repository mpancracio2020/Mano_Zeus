/*###########################################
## Author's github:    madport             ##
## Author's github:    vbarcena2020        ##
#############################################
*/

#include "ServoM.h"

enum
{
  NUMSERVOS = 6,
  NUMINVALUES = 12*3,
  DIGITSPERVAL = 3,
};

int lpins[NUMSERVOS] = {2, 3, 4, 5, 6, 7};
int rpins[NUMSERVOS] = {8, 9, 10, 11, 12, 13};
int acel = 200;

ServoM lservos[NUMSERVOS] = {
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel*2, -180, 180)
};

ServoM rservos[NUMSERVOS] = {
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel*2, -180, 180)
};

int valsIn[NUMINVALUES];
int stringLenght = NUMINVALUES * DIGITSPERVAL + 1;
int counter = 0;
bool counterStart = false;
String recivedString;


void setup() 
{
  Serial.begin(9600);
  
  for (int i = 0; i < NUMSERVOS; ++i) {
    lservos[i].attach(lpins[i]);
    rservos[i].attach(rpins[i]);
  }
}


void receiveData()
{
  while(Serial.available())
  {
    char c = Serial.read();

    if (c=='$') {counterStart = true; }
    
    if (counterStart)
    {
      if (counter < stringLenght)
      {
        recivedString = String(recivedString+c);
        counter++;
      }
      if (counter >= stringLenght){

        for (int i = 0; i < NUMINVALUES ; i++){
          int num = (i*DIGITSPERVAL) + 1;
          valsIn[i] = recivedString.substring(num,num + DIGITSPERVAL).toInt();
        }
        recivedString = "";
        counter = 0;
        counterStart = false;
        
      }
    }
  }
}

void loop() 
{
  receiveData();
  for (int i = 0; i < NUMSERVOS; i++ ) {
    lservos[i].goTo(valsIn[i]);
    rservos[i].goTo(valsIn[i+NUMSERVOS]);
  }
  
  // for (int i = 0; i < NUMINVALUES; ++i) {
  //   if (valsIn[i] == 1) {lservos[i].goTo(180);} 
  //   else {lservos[i].goTo(0);}
  // }

  // for (int i = 6; i < NUMINVALUES*2; ++i) {
  //   if (valsIn[i + 6] == 1) {rservos[i].goTo(180);} 
  //   else {rservos[i].goTo(0);}
  // }
  
}
