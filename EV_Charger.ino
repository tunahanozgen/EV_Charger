#define GFCI_TEST_PIN         8
#define AC_RELAY_MCU_PIN      9
#define PWM_PIN               10
#define MISO_PIN              12
#define MOSI_PIN              11
#define SCK_PIN               13
#define GFCI_INT_PIN          2
#define INTERNAL_NTC_PIN      A3
#define PP_MCU_READ_PIN       A2
#define PILOT_PWM_MCU_READ_PIN A1
#define CURRENT_ANALOG_PIN    A0
#define EXTERNAL_NTC_PIN      7 // adc7


/////////// PILOT PWM sabitleri
float maxCurrent = 32.0;  //A
float desiredCurrent = 6.0; 
float oldDesiredCurrent = 0; 

////////////////

///NTC sabitleri
const float Vcc = 5.0;
const int R_fixed = 10000;  // 10k sabit direnç
const int adcMax = 1023;

const float BETA = 3950;    // NTC'nin beta değeri
const float T0 = 298.15;    // 25°C in Kelvin
const float R0 = 10000;     // 25°C'deki direnç (ohm)
// Temperature veriables
float internalTemperature = 0.0; // kart sıcaklığı
float externalTemperature = 0.0; // araç şarj fişi sıcaklığı
unsigned long lastTemperatureReadMillis = millis();
#define TEMP_SENS_INTERVAL  1000
/////////////////


void setup() {
  Serial.begin(115200);
  Serial.println("--EV Charger DEMO--");
  setTimer(); // 1khz frekans ayarlama işlemi
  
  pinMode(PILOT_PWM_MCU_READ_PIN, INPUT);
  pinMode(PP_MCU_READ_PIN, INPUT);


  maxCurrent = readPPCableCurrent(); // read max current?
  Serial.println("Kablo akım kapasitesi: " + String(maxCurrent));

  //for debug
  if(maxCurrent < 32) maxCurrent = 32;
}

void loop() {
  
  if(millis() - lastTemperatureReadMillis > TEMP_SENS_INTERVAL){
    internalTemperature = readthermistor(INTERNAL_NTC_PIN);
    externalTemperature = readthermistor(EXTERNAL_NTC_PIN);
    //Serial.println("T_internal: " + String(internalTemperature) + " | T_external: " + String(externalTemperature));
    readPilotPWM();
    lastTemperatureReadMillis = millis();
  }
  
  if(desiredCurrent != oldDesiredCurrent){
    setChargingCurrent(desiredCurrent); // akımı ayarla
    oldDesiredCurrent = desiredCurrent; // eski ve güncel akımı eşitliyoruz
  }
  

  while(Serial.available()){
    String cmd = Serial.readStringUntil('#');
    String key = split(cmd,'=',0);
    String value = split(cmd,'=',1);
  
    if(key == "current") desiredCurrent = value.toInt();

    Serial.println("desiredCurrent: " + String(desiredCurrent) + " oldDesiredCurrent: " + String(oldDesiredCurrent));
  }
}

float readthermistor(byte thermistor_pin){

  int adcValue = analogRead(thermistor_pin);
  float Vntc = (adcValue / (float)adcMax) * Vcc;

  // NTC direncini hesapla
  float R_ntc = (R_fixed * Vntc) / (Vcc - Vntc);

  // Kelvin cinsinden sıcaklık
  float tempK = 1.0 / ( (1.0 / T0) + (1.0 / BETA) * log(R_ntc / R0) );
  float tempC = tempK - 273.15;
  /*
  Serial.print("Sıcaklık: ");
  Serial.print(tempC);
  Serial.println(" °C");
  */
  return tempC;
}
void setTimer(){
  // Timer1'i 1kHz PWM için ayarla (16MHz / 8 prescaler / 2000 top = 1kHz)
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM13);             // Mode 8: PWM, Phase and Frequency Correct, TOP = ICR1
  TCCR1A |= (1 << COM1B1);            // Non-inverting PWM on OC1B (D10)
  TCCR1B |= (1 << CS11);              // Prescaler = 8

  ICR1 = 2000;                        // TOP = 2000 ➝ 16MHz / 8 / 2000 = 1kHz
}

// Akım değerine göre PWM duty ayarla
void setChargingCurrent(float currentA) {
  // Güvenlik için sınırla
  if (currentA < 6.0) currentA = 6.0;
  if (currentA > maxCurrent) currentA = maxCurrent;

  float dutyPercent = currentA * 0.6; // IEC 61851 formülü

  // PWM duty hesapla
  uint16_t dutyCount = (uint16_t)((dutyPercent / 100.0) * ICR1);
  OCR1B = dutyCount;

  Serial.print("Akım limiti: ");
  Serial.print(currentA);
  Serial.print(" A, PWM: ");
  Serial.print(dutyPercent);
  Serial.println(" %");
}
String split(const String &data, char separator, int index) {
  int start = 0;
  int end = -1;
  int found = 0;

  int len = data.length();
  for (int i = 0; i <= len; i++) {
    if (data[i] == separator || i == len) {
      if (found == index) {
        return data.substring(start, i);  // direkt döndür, diğerlerine bakma
      }
      found++;
      start = i + 1;
    }
  }

  return "";
}
void readPilotPWM(){
  unsigned long lowTime  = pulseIn(PILOT_PWM_MCU_READ_PIN, LOW, 20000);
  unsigned long highTime = pulseIn(PILOT_PWM_MCU_READ_PIN, HIGH, 20000);

  if (highTime == 0 || lowTime == 0) {
    Serial.println("PWM sinyali yok.");
    return;
  }

  unsigned long period = highTime + lowTime;
  float duty = (highTime * 100.0) / period;

  Serial.print("Duty: ");
  Serial.print(duty);
  Serial.println(" %");


}
float readPPCableCurrent() {
  int adcValue = analogRead(PP_MCU_READ_PIN);
  float voltage = (adcValue / (float)adcMax) * Vcc;

  // Karar aralıkları: Voltaj değerine göre karşılık gelen kablo akımı
  
  // Kablo takılı değilse — neredeyse 5V okunur
  if (voltage > 4.85) return 0.0;       // PP hattı boşta, kablo takılı değil
  else if (voltage > 4.70) return 63.0;
  else if (voltage > 4.55) return 32.0;
  else if (voltage > 4.35) return 20.0;
  else if (voltage > 4.10) return 13.0;
  else return 6.0; // Örn: 330Ω gibi özel direnç varsa (opsiyonel senaryo)
}