#include <LiquidCrystal_I2C.h>
#include <BigNumbers_I2C.h>
#include <OneWire.h>                //Multitech Automation farm project v.1
#include <Encoder.h>       
#include <SparkFun_External_EEPROM.h>
#include <MsTimer2.h>
#include "Countimer.h"
 ExternalEEPROM myMem;
 Countimer timer;
 LiquidCrystal_I2C lcd(0x27,20,4);
 BigNumbers_I2C bigNum(&lcd);
 Encoder myEnc(A2, A3);// DT, CLK
 OneWire  ds(2);

 int menu,set,F_reg,flag1=0, flag2=0, flag3=0,time_s = 0,time_m = 0;
 int duty_cycle,duty_cycle2,duty_cycle3;
 int Fan1=6,Fan2=9,Fan3=10,up=5;
 float temp,V_hi1,V_hi2,V_hi3,V_lo1,V_lo2,V_lo3,V_s1,V_s2,V_s3,V_x1,V_x2,V_x3;
 bool w;
 unsigned long times,times1,oldPosition = -999,newPosition;

void setup() {
  lcd.begin();lcd.backlight();myMem.setMemoryType(16);bigNum.begin();
  lcd.setCursor(0,1);lcd.print("Multitech Automation");delay(2000);lcd.clear();
  MsTimer2::set(1, to_Timer);MsTimer2::start();
  timer.setInterval(print_time, 999);
  read_on_time();timer.start();flag3=0;

  pinMode(A1,INPUT_PULLUP);
  pinMode(A0,INPUT_PULLUP);
  pinMode(up, OUTPUT);
  pinMode(Fan1, OUTPUT);
  pinMode(Fan2, OUTPUT);
  pinMode(Fan3, OUTPUT);
  pinMode(13, OUTPUT);
  
  myMem.get(0,V_s1);myMem.get(5,V_lo1);myMem.get(10,V_hi1);myMem.get(15,V_x1);
  myMem.get(20,V_s2);myMem.get(25,V_lo2);myMem.get(30,V_hi2);myMem.get(35,V_x2);
  myMem.get(40,V_s3);myMem.get(45,V_lo3);myMem.get(50,V_hi3);myMem.get(55,V_x3);
}
void print_time(){
   time_s = time_s-1;
  if(time_s<0){time_s=59; time_m = time_m-1;}
  if(time_m<0){time_m=59;}
} 
void loop() {
  timer.run();
  if(millis()-times>1000){times=millis();temp = dsRead(0);}
  if(digitalRead(A1)==LOW){menu++;if(menu>12){menu=0;}times1=millis();w=1;delay(200);}
  if(digitalRead(A0)==LOW){set++;if(set>7){set=0;}times1=millis();delay(100);}

  byte x = 0;
  byte y = 0;
  int last = temp*10;
  bigNum.displayLargeInt(temp, x, y, 2, false);lcd.setCursor(6,0);
  lcd.print(".");lcd.print(last%10);;lcd.print((char)223);
  lcd.setCursor(7,1);lcd.print("V1");
  lcd.setCursor(7,2);lcd.print("V2");
  lcd.setCursor(7,3);lcd.print("V3");

  if(menu==0){lcd.setCursor(14,3);lcd.print("      ");
  }
  if(menu==1){
   lcd.setCursor(14,1);lcd.print("S ");
   lcd.print(V_s1,0);
  }
  if(menu==2){
   lcd.setCursor(14,1);lcd.print("L ");
   lcd.print(V_lo1/10,1);
  }
  if(menu==3){
   lcd.setCursor(14,1);lcd.print("H ");
   lcd.print(V_hi1/10,1);
  }
  if(menu==4){
   lcd.setCursor(14,1);lcd.print("X ");
   lcd.print(V_x1/10,1);
  }
  if(menu==5){
   lcd.setCursor(14,1);lcd.print("      ");
   lcd.setCursor(14,2);lcd.print("S ");
   lcd.print(V_s2,0);
  }
  if(menu==6){
   lcd.setCursor(14,2);lcd.print("L ");
   lcd.print(V_lo2,1);
  }
  if(menu==7){
   lcd.setCursor(14,2);lcd.print("H ");
   lcd.print(V_hi2,1);
  }
  if(menu==8){
   lcd.setCursor(14,2);lcd.print("X ");
   lcd.print(V_x2,1);
  }
  if(menu==9){
   lcd.setCursor(14,2);lcd.print("      ");
   lcd.setCursor(14,3);lcd.print("S ");
   lcd.print(V_s3,0);
  }
  if(menu==10){
   lcd.setCursor(14,3);lcd.print("L ");
   lcd.print(V_lo3,1);
  }
  if(menu==11){
   lcd.setCursor(14,3);lcd.print("H ");
   lcd.print(V_hi3,1);
  }
  if(menu==12){
   lcd.setCursor(14,3);lcd.print("X ");
   lcd.print(V_x3,1);
  }
  switch(menu){
   case 1: F_reg = V_s1;break;
   case 2: F_reg = V_lo1;break;
   case 3: F_reg = V_hi1;break;
   case 4: F_reg = V_x1;break;
   case 5: F_reg = V_s2;break;
   case 6: F_reg = V_lo2;break;
   case 7: F_reg = V_hi2;break;
   case 8: F_reg = V_x2;break;
   case 9: F_reg = V_s3;break;
   case 10: F_reg = V_lo3;break;
   case 11: F_reg = V_hi3;break;
   case 12: F_reg = V_x3;break;      
  } 
 if(newPosition!=oldPosition){oldPosition=newPosition;F_reg=F_reg+newPosition;myEnc.write(0);newPosition=0;times1=millis();w=1;if(F_reg>999){F_reg=999;}if(F_reg<0){F_reg=0;}}
  switch(menu){
   case 1: V_s1 = F_reg;break;
   case 2: V_lo1 = F_reg;break;
   case 3: V_hi1 = F_reg;break;
   case 4: V_x1 = F_reg;break;
   case 5: V_s2 = F_reg;break;
   case 6: V_lo2 = F_reg;break;
   case 7: V_hi2 = F_reg;break;
   case 8: V_x2 = F_reg;break;
   case 9: V_s3 = F_reg;break;
   case 10: V_lo3 = F_reg;break;
   case 11: V_hi3 = F_reg;break;
   case 12: V_x3 = F_reg;break;     
  }
  if(millis()-times1>10000 && w==1){
   myMem.put(0,V_s1);myMem.put(5,V_lo1);myMem.put(10,V_hi1);myMem.put(15,V_x1);
   myMem.put(20,V_s2);myMem.put(25,V_lo2);myMem.put(30,V_hi2);myMem.put(35,V_x2);
   myMem.put(40,V_s3);myMem.put(45,V_lo3);myMem.put(50,V_hi3);myMem.put(55,V_x3);
   w=0;lcd.clear();}

  int value1 = temp*10;
  int value2 = V_lo1*10;
  int value3 = V_hi1*10;
  int value4 = V_lo2*10;
  int value5 = V_hi2*10;
  int value6 = V_lo3*10;
  int value7 = V_hi3*10;

  duty_cycle = map(value1, value2 , value3 , 0, 100);
  duty_cycle2 = map(value1, value4 , value5 , 0, 100);
  duty_cycle3 = map(value1, value6 , value7 , 0, 100);

  if(duty_cycle>100)duty_cycle=100;
  if(duty_cycle<V_s1)duty_cycle=V_s1;
  if(V_s1>100)V_s1=100;
  if(V_s1<0)V_s1=0;
  if(duty_cycle2>100)duty_cycle2=100;
  if(duty_cycle2<V_s2)duty_cycle2=V_s2;
  if(V_s2>100)V_s2=100;
  if(V_s2<0)V_s2=0;
  if(duty_cycle3>100)duty_cycle3=100;
  if(duty_cycle3<V_s3)duty_cycle3=V_s3;
  if(V_s3>100)V_s3=100;
  if(V_s3<0)V_s3=0;

  if(temp*1<V_lo1){analogWrite(Fan1, (V_s1*1.3));}
  else{
    analogWrite(Fan1,(duty_cycle*1.3));
    lcd.setCursor(10,0);
    if(duty_cycle<100)lcd.print(" ");
    lcd.print(duty_cycle);
    if(temp*1>V_x1){
    analogWrite(Fan1,(duty_cycle)=135);}
  } 
  if(temp<V_lo2){analogWrite(Fan2, (V_s2*1.2));}
  else{
    analogWrite(Fan2, (duty_cycle2*1.2));
    lcd.setCursor(10,2);
    if(duty_cycle2<100)lcd.print(" ");
    lcd.print(duty_cycle2);
    if(temp>V_x2){
    analogWrite(Fan2,(duty_cycle2)=255);}
  }
  if(temp<V_lo3){analogWrite(Fan3, (V_s3*1.2));}
  else{
    analogWrite(Fan3, (duty_cycle3*1.2));
    lcd.setCursor(10,3);
    if(duty_cycle3<100)lcd.print(" ");
    lcd.print(duty_cycle3);
    if(temp>V_x3){
    analogWrite(Fan3,(duty_cycle3)=255);}
  }
  if(set==1){timer.stop();flag3=0;analogWrite(up, 0);digitalWrite(13, LOW);
    lcd.setCursor(10,0);lcd.print("stop");
  }
  if(set==2){read_on_time();if (newPosition != oldPosition) {oldPosition = newPosition;  
    time_s=time_s+newPosition;myEnc.write(0);newPosition=0;
    times1=millis();if(time_s>60){time_s=60;}if(time_s<1){time_s=1;}}
    lcd.setCursor(10,0);lcd.print("Of s");update_on_time(); 
  }
  if(set==3){read_on_time();if (newPosition != oldPosition) {oldPosition = newPosition;  
    time_m=time_m+newPosition;myEnc.write(0);newPosition=0;
    times1=millis();if(time_m>60){time_m=60;}if(time_m<0){time_m=0;}}
    lcd.setCursor(10,0);lcd.print("Of m");update_on_time(); 
  }
  if(set==4){read_of_time();if (newPosition != oldPosition) {oldPosition = newPosition;  
    time_s=time_s+newPosition;myEnc.write(0);newPosition=0;
    times1=millis();if(time_s>60){time_s=60;}if(time_s<1){time_s=1;}}
    lcd.setCursor(10,0);lcd.print("On s");update_of_time(); 
  }
  if(set==5){read_of_time();if (newPosition != oldPosition) {oldPosition = newPosition; 
    time_m=time_m+newPosition;myEnc.write(0);newPosition=0;
    times1=millis();if(time_m>60){time_m=60;}if(time_m<0){time_m=0;}}
    lcd.setCursor(10,0);lcd.print("On m");update_of_time(); 
  }
  if(set==6){flag3=0;read_on_time();timer.restart();timer.start();
    lcd.setCursor(10,0);lcd.print("star");
  }
    lcd.setCursor(15,0);
  if(time_m<=9){lcd.print("0");}lcd.print(time_m);lcd.print(":");  
  if(time_s<=9){lcd.print("0");}lcd.print(time_s);
  if(time_s==0 && time_m==0){flag3 = flag3+1; 
  if(flag3>1){flag3=0;}
  if(flag3==0){read_on_time();lcd.setCursor(10,0);lcd.print("Of");
    analogWrite(up, 0);digitalWrite(13, LOW);} 
  if(flag3==1){read_of_time();lcd.setCursor(10,0);lcd.print("On  ");
    analogWrite(up, 135);digitalWrite(13, HIGH);} 
    timer.restart();timer.start();} 
}
float dsRead(byte x) {
  byte data[2], addr[8][8], kol = 0;
  while (ds.search(addr[kol])) {  // search for sensors, determine the address and number of sensors
    kol++;
  } 
  ds.reset_search();  // Reset the sensor search
  ds.reset();         // Initialization, reset the bus
  ds.select(addr[x]); // Access the sensor at the address
  ds.write(0x44, 0);  // Measure temperature with data transfer to memory
  ds.reset();         // Initialization, reset the bus
  ds.select(addr[x]); // Access the sensor at the address
  ds.write(0xBE);     // Access memory
  data[0] = ds.read();// Read memory byte low
  data[1] = ds.read();// Read memory byte high
  float value = ((data[1] << 8) | data[0]) / 16.0; return (float)value;
}
void to_Timer(){newPosition = myEnc.read()/2;}
void update_on_time(){myMem.write(97, time_s);myMem.write(98, time_m);}
void update_of_time(){myMem.write(99, time_s);myMem.write(100, time_m);}
void read_on_time(){time_s =  myMem.read(97);time_m =  myMem.read(98);}
void read_of_time(){time_s =  myMem.read(99);time_m =  myMem.read(100);}