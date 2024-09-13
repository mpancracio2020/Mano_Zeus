/*###########################################
## Author's github:    madport             ##
## Author's github:    vbarcena2020        ##
#############################################
*/

#include "ServoM.h"

enum
{
  NUMSERVOSHAND = 6,
  NUMHANDS = 2,
  NUMARMS = 2,
  NUMINVALUES = (NUMSERVOSHAND*NUMHANDS+NUMARMS),
  DIGITSPERVAL = 3,
};

int lpins[NUMSERVOSHAND] = {2, 3, 4, 5, 6, 7};
int rpins[NUMSERVOSHAND] = {8, 9, 10, 11, 12, 13};
// int armspins = {};
int acel = 200;

ServoM lservos[NUMSERVOSHAND] = {
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel*2, -180, 180)
};

ServoM rservos[NUMSERVOSHAND] = {
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel, -180, 180),
  ServoM(acel*2, -180, 180)
};

// ServoM larm = ServoM(acel*2, -180, 180);
// ServoM rarm = ServoM(acel*2, -180, 180);


int valsIn[NUMINVALUES];
int stringLenght = NUMINVALUES * DIGITSPERVAL + 1;
int counter = 0;
bool counterStart = false;
String recivedString;


void setup() 
{
  Serial.begin(9600);
  
  for (int i = 0; i < NUMSERVOSHAND; ++i) {
    lservos[i].attach(lpins[i]);
    rservos[i].attach(rpins[i]);
  }
  // rarm.attach(armspins[0]);
  // larm.attach(armspins[1]);

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
        for (int i = 0; i < NUMINVALUES ; i++) {
          Serial.print(valsIn[i]);
          Serial.print(",");
        }
        Serial.println();
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
  for (int i = 0; i < NUMSERVOSHAND; i++ ) {
    lservos[i].goTo(valsIn[i]);
    rservos[i].goTo(valsIn[i+NUMSERVOSHAND]);
  }
  // larm.goTo(valsIn[NUMSERVOSHAND*2]);
  // rarm.goTo(valsIn[NUMSERVOSHAND*2+1]);


  // // ------ left_hand positions -------
  // for (int i = 0; i < NUMINVALUES; ++i) {
  //   if (valsIn[i] == 1) {lservos[i].goTo(180);} 
  //   else {lservos[i].goTo(0);}
  // }

  // // ------  right_hand positions -------
  // for (int i = 6; i < NUMINVALUES*2; ++i) {
  //   if (valsIn[i + 6] == 1) {rservos[i].goTo(180);} 
  //   else {rservos[i].goTo(0);}
  // }
  
}
