//update robi
#include <LiquidCrystal_I2C.h>
#include <BigNumbers_I2C.h>
#include <OneWire.h>                
#include <Encoder.h>       
#include <SparkFun_External_EEPROM.h>
#include <MsTimer2.h>
#include "Countimer.h"

ExternalEEPROM myMem;
Countimer timer;
Encoder myEnc(A2, A3); // DT, CLK
OneWire ds(2);

// Variabel global
int menu = 0, set = 0, flag1 = 0, flag2 = 0, flag3 = 0, time_s = 0, time_m = 0;
float duty_cycle, duty_cycle2, duty_cycle3; 
const int Fan1 = 6, Fan2 = 9, Fan3 = 10, up = 5;

float F_reg;
float temp = 0.0;
float V_hi1 = 0.0, V_hi2 = 0.0, V_hi3 = 0.0;
float V_lo1 = 0.0, V_lo2 = 0.0, V_lo3 = 0.0;
float V_s1 = 0.0, V_s2 = 0.0, V_s3 = 0.0;
float V_x1 = 0.0, V_x2 = 0.0, V_x3 = 0.0;

bool w = false;
unsigned long times = 0, times1 = 0;
float oldPosition = -999.0, newPosition = 0.0;

LiquidCrystal_I2C lcd(0x27, 20, 4);
BigNumbers_I2C bigNum(&lcd);

void setup() {
  // Inisialisasi LCD
  lcd.begin();
  lcd.backlight();
  bigNum.begin();
  lcd.setCursor(0, 1);
  lcd.print("Multitech Automation");
  delay(2000);
  lcd.clear();

  // Inisialisasi EEPROM
  if (!myMem.begin()) {
    lcd.print("EEPROM Error!");
    while(1);
  }
  myMem.setMemoryType(16);

  // Inisialisasi timer
  MsTimer2::set(1, to_Timer);
  MsTimer2::start();
  timer.setInterval(print_time, 999);
  read_on_time();
  timer.start();
  flag3 = 0;

  // Inisialisasi pin
  pinMode(A1, INPUT_PULLUP);
  pinMode(A0, INPUT_PULLUP);
  pinMode(up, OUTPUT);
  pinMode(Fan1, OUTPUT);
  pinMode(Fan2, OUTPUT);
  pinMode(Fan3, OUTPUT);
  pinMode(13, OUTPUT);
  
  // Baca nilai dari EEPROM
  readEEPROMValues();
}

void loop() {
  timer.run();
  
  // Baca suhu setiap detik
  if(millis() - times > 1000) {
    times = millis();
    temp = dsRead(0);
  }

  // Handle tombol menu
  if(digitalRead(A1) == LOW) {
    menu++;
    if(menu > 12) menu = 0;
    times1 = millis();
    w = true;
    delay(200); // Debounce
  }

  // Handle tombol set
  if(digitalRead(A0) == LOW) {
    set++;
    if(set > 7) set = 0;
    times1 = millis();
    delay(100); // Debounce
  }

  // Tampilkan suhu
  displayTemperature();

  // Update tampilan menu
  updateMenuDisplay();

  // Handle encoder
  handleEncoder();

  // Simpan nilai ke EEPROM jika ada perubahan
  if(millis() - times1 > 10000 && w) {
    saveToEEPROM();
    w = false;
    lcd.clear();
  }

  // Kontrol fan
  controlFans();

  // Handle pengaturan timer
  handleTimerSettings();

  // Tampilkan waktu
  displayTime();

  // Handle timer habis
  if(time_s == 0 && time_m == 0) {
    handleTimerExpired();
  }
}


void readEEPROMValues() {
  myMem.get(0, V_s1);
  myMem.get(5, V_lo1);
  myMem.get(10, V_hi1);
  myMem.get(15, V_x1);
  myMem.get(20, V_s2);
  myMem.get(25, V_lo2);
  myMem.get(30, V_hi2);
  myMem.get(35, V_x2);
  myMem.get(40, V_s3);
  myMem.get(45, V_lo3);
  myMem.get(50, V_hi3);
  myMem.get(55, V_x3);

  // Batasi nilai yang dibaca
  constrainValues();
}

void constrainValues() {
  // Batasi semua nilai dalam range yang valid
  V_s1 = constrain(V_s1, 0.0, 100.0);
  V_s2 = constrain(V_s2, 0.0, 100.0);
  V_s3 = constrain(V_s3, 0.0, 100.0);
  
  V_lo1 = constrain(V_lo1, 0.0, 999.9);
  V_lo2 = constrain(V_lo2, 0.0, 999.9);
  V_lo3 = constrain(V_lo3, 0.0, 999.9);
  
  V_hi1 = constrain(V_hi1, 0.0, 999.9);
  V_hi2 = constrain(V_hi2, 0.0, 999.9);
  V_hi3 = constrain(V_hi3, 0.0, 999.9);
  
  V_x1 = constrain(V_x1, 0.0, 999.9);
  V_x2 = constrain(V_x2, 0.0, 999.9);
  V_x3 = constrain(V_x3, 0.0, 999.9);
}

void saveToEEPROM() {
  // Simpan semua nilai ke EEPROM
  myMem.put(0, V_s1);
  myMem.put(5, V_lo1);
  myMem.put(10, V_hi1);
  myMem.put(15, V_x1);
  myMem.put(20, V_s2);
  myMem.put(25, V_lo2);
  myMem.put(30, V_hi2);
  myMem.put(35, V_x2);
  myMem.put(40, V_s3);
  myMem.put(45, V_lo3);
  myMem.put(50, V_hi3);
  myMem.put(55, V_x3);
}

void displayTemperature() {
  byte x = 0;
  byte y = 0;
  int last = temp * 10;
  bigNum.displayLargeInt(temp, x, y, 2, false);
  lcd.setCursor(6, 0);
  lcd.print(".");
  lcd.print(last % 10);
  lcd.print((char)223);
  
  lcd.setCursor(7, 1); lcd.print("V1");
  lcd.setCursor(7, 2); lcd.print("V2");
  lcd.setCursor(7, 3); lcd.print("V3");
}

void updateMenuDisplay() {
  if(menu == 0) {
    lcd.setCursor(14, 3); lcd.print("      ");
  }
  
  // Update display berdasarkan menu aktif
  switch(menu) {
    case 1: displayValue(1, "S ", V_s1, 0); break;
    case 2: displayValue(1, "L ", V_lo1, 1); break;
    case 3: displayValue(1, "H ", V_hi1, 1); break;
    case 4: displayValue(1, "X ", V_x1, 1); break;
    case 5: displayValue(2, "S ", V_s2, 0); break;
    case 6: displayValue(2, "L ", V_lo2, 1); break;
    case 7: displayValue(2, "H ", V_hi2, 1); break;
    case 8: displayValue(2, "X ", V_x2, 1); break;
    case 9: displayValue(3, "S ", V_s3, 0); break;
    case 10: displayValue(3, "L ", V_lo3, 1); break;
    case 11: displayValue(3, "H ", V_hi3, 1); break;
    case 12: displayValue(3, "X ", V_x3, 1); break;
  }
}

void displayValue(int row, const char* prefix, float value, int decimals) {
  if(menu > 4 && menu < 9) lcd.setCursor(14, 1); lcd.print("      ");
  if(menu > 8) lcd.setCursor(14, 2); lcd.print("      ");
  
  // Tampilkan nilai
  lcd.setCursor(14, row);
  lcd.print(prefix);
  lcd.print(value, decimals);
}

void handleEncoder() {
  if (newPosition != oldPosition) {
    oldPosition = newPosition;

    float stepS = 0.1f; 
    float stepL = 0.1f;  

    // Update nilai berdasarkan menu aktif
    switch (menu) {
      case 1:  V_s1 += (newPosition * stepS); break;
      case 2:  V_lo1 += (newPosition * stepL); break;
      case 3:  V_hi1 += (newPosition * stepL); break;
      case 4:  V_x1 += (newPosition * stepL); break;
      case 5:  V_s2 += (newPosition * stepS); break;
      case 6:  V_lo2 += (newPosition * stepL); break;
      case 7:  V_hi2 += (newPosition * stepL); break;
      case 8:  V_x2 += (newPosition * stepL); break;
      case 9:  V_s3 += (newPosition * stepS); break;
      case 10: V_lo3 += (newPosition * stepL); break;
      case 11: V_hi3 += (newPosition * stepL); break;
      case 12: V_x3 += (newPosition * stepL); break;
    }

    // Reset encoder
    myEnc.write(0);
    newPosition = 0;
    times1 = millis();
    w = true;
    constrainValues();
  }
}

void controlFans() {
  // === FAN 1 ===
  if (temp <= V_lo1) {
    duty_cycle = V_s1;
  } 
  else if (temp >= V_hi1) {
    duty_cycle = 100.0;
  } 
  else {
    duty_cycle = mapFloat(temp, V_lo1, V_hi1, V_s1, 100.0);
  }
  duty_cycle = constrain(duty_cycle, V_s1, 100.0);
  analogWrite(Fan1, round(duty_cycle * 2.55));
  displayPWM(1, duty_cycle);

  // === FAN 2 ===
  if (temp <= V_lo2) {
    duty_cycle2 = V_s2;
  } 
  else if (temp >= V_hi2) {
    duty_cycle2 = 100.0;
  } 
  else {
    duty_cycle2 = mapFloat(temp, V_lo2, V_hi2, V_s2, 100.0);
  }
  duty_cycle2 = constrain(duty_cycle2, V_s2, 100.0);
  analogWrite(Fan2, round(duty_cycle2 * 2.55));
  displayPWM(2, duty_cycle2);

  // === FAN 3 ===
  if (temp <= V_lo3) {
    duty_cycle3 = V_s3;
  } 
  else if (temp >= V_hi3) {
    duty_cycle3 = 100.0;
  } 
  else {
    duty_cycle3 = mapFloat(temp, V_lo3, V_hi3, V_s3, 100.0);
  }
  duty_cycle3 = constrain(duty_cycle3, V_s3, 100.0);
  analogWrite(Fan3, round(duty_cycle3 * 2.55));
  displayPWM(3, duty_cycle3);
}

// Fungsi mapping floating point 
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Menampilkan PWM 
void displayPWM(int row, float pwm_value) {
  lcd.setCursor(10, row);
  if (pwm_value < 100) lcd.print(" ");
  if (pwm_value < 10) lcd.print(" ");
  lcd.print(round(pwm_value));
}


void handleTimerSettings() {
  switch(set) {
    case 1: // Stop
      timer.stop();
      flag3 = 0;
      analogWrite(up, 0);
      digitalWrite(13, LOW);
      lcd.setCursor(10, 0); lcd.print("stop");
      break;
      
    case 2: // Off seconds
      read_on_time();
      if (newPosition != oldPosition) {
        oldPosition = newPosition;  
        time_s = constrain(time_s + newPosition, 1, 60);
        myEnc.write(0);
        newPosition = 0;
        times1 = millis();
      }
      lcd.setCursor(10, 0); lcd.print("Of s");
      update_on_time(); 
      break;
  }
}

void displayTime() {
  lcd.setCursor(15, 0);
  if(time_m <= 9) lcd.print("0");
  lcd.print(time_m);
  lcd.print(":");  
  if(time_s <= 9) lcd.print("0");
  lcd.print(time_s);
}

void handleTimerExpired() {
  flag3++;
  if(flag3 > 1) flag3 = 0;
  
  if(flag3 == 0) {
    read_on_time();
    lcd.setCursor(10, 0); lcd.print("Of");
    analogWrite(up, 0);
    digitalWrite(13, LOW);
  } else {
    read_of_time();
    lcd.setCursor(10, 0); lcd.print("On  ");
    analogWrite(up, 135);
    digitalWrite(13, HIGH);
  }
  
  timer.restart();
  timer.start();
}

// Fungsi-fungsi timer dan sensor
void print_time() {
  time_s--;
  if(time_s < 0) {
    time_s = 59; 
    time_m--;
  }
  if(time_m < 0) {
    time_m = 59;
  }
} 

float dsRead(byte x) {
  byte data[2], addr[8][8], kol = 0;
  while (ds.search(addr[kol])) {
    kol++;
  } 
  ds.reset_search();
  ds.reset();
  ds.select(addr[x]);
  ds.write(0x44, 0);
  ds.reset();
  ds.select(addr[x]);
  ds.write(0xBE);
  data[0] = ds.read();
  data[1] = ds.read();
  return ((data[1] << 8) | data[0]) / 16.0;
}

void to_Timer() {
  newPosition = myEnc.read() / 2;
}

void update_on_time() {
  myMem.write(97, time_s);
  myMem.write(98, time_m);
}

void update_of_time() {
  myMem.write(99, time_s);
  myMem.write(100, time_m);
}

void read_on_time() {
  time_s = myMem.read(97);
  time_m = myMem.read(98);
}

void read_of_time() {
  time_s = myMem.read(99);
  time_m = myMem.read(100);
}