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
const float maxCurrent = 32.0;  //A
float desiredCurrent = 16.0;    
////////////////

///NTC sabitleri
const float Vcc = 5.0;
const int R_fixed = 10000;  // 10k sabit direnç
const int adcMax = 1023;

const float BETA = 3950;    // NTC'nin beta değeri
const float T0 = 298.15;    // 25°C in Kelvin
const float R0 = 10000;     // 25°C'deki direnç (ohm)
// Temperature veriables
float internalTemperature = 0.0;
/////////////////


void setup() {
  Serial.begin(115200);
  setTimer();
  setChargingCurrent(desiredCurrent); // akımı ayarla
}

void loop() {
  internalTemperature = readthermistor(INTERNAL_NTC_PIN);
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