/*
    Example code compatible with the Lambda Shield 2 for Arduino Uno WiFi Rev 2 and smartphone apps.
    
    Copyright (C) 2020 Bylund Automotive AB
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    Contact information of author:
    http://www.bylund-automotive.com/
    
    info@bylund-automotive.com
    
    Version history:
    2020-04-13        v1.0.0        First release.
    2020-05-01        v1.0.1        Bug fixes.
    2026-11-27        v1.0.2        Modified for Arduino Uno R4 Wifi
*/

// Eingebundene Bibliotheken
#include <ArduinoBLE.h>
#include <SPI.h>

// BLE-Service und Charakteristik definieren (funktioniert nativ auf dem Uno R4 WiFi)
BLEService LambdaService("C664F3C0-81B3-4E7B-A2F6-EA96181A2E71");
BLEUnsignedCharCharacteristic OxygenLevelChar("C83C027E-F105-4E0D-AA8F-801AC553D5C1", BLERead | BLENotify);

// CJ125 Register definieren
#define CJ125_IDENT_REG_REQUEST         0x4800  /* Identify request, gives revision of the chip. */
#define CJ125_DIAG_REG_REQUEST          0x7800  /* Diagnostic request, gives the current status. */
#define CJ125_INIT_REG1_REQUEST         0x6C00  /* Requests the first init register. */
#define CJ125_INIT_REG2_REQUEST         0x7E00  /* Requests the second init register. */
#define CJ125_INIT_REG1_MODE_CALIBRATE  0x569D  /* Sets the first init register in calibration mode. */
#define CJ125_INIT_REG1_MODE_NORMAL_V8  0x5688  /* Sets the first init register in operation mode. V=8 amplification. */
#define CJ125_INIT_REG1_MODE_NORMAL_V17 0x5689  /* Sets the first init register in operation mode. V=17 amplification. */
#define CJ125_DIAG_REG_STATUS_OK        0x28FF  /* Response when everything is ok. */
#define CJ125_DIAG_REG_STATUS_NOPOWER   0x2855  /* Response when power is low. */
#define CJ125_DIAG_REG_STATUS_NOSENSOR  0x287F  /* Response when no sensor is connected. */
#define CJ125_INIT_REG1_STATUS_0        0x2888  /* Response when V=8 amplification is in use. */
#define CJ125_INIT_REG1_STATUS_1        0x2889  /* Response when V=17 amplification is in use. */

// Pin-Belegung (identisch auf dem Uno R4 WiFi)
#define LED_STATUS_POWER                7       /* Power Status LED */
#define LED_STATUS_HEATER               6       /* Heater Status LED */
#define CJ125_NSS_PIN                   10      /* SPI Chip Select */
#define HEATER_OUTPUT_PIN               5       /* PWM Ausgang für Sondenheizung */
#define UB_ANALOG_INPUT_PIN             A2      /* Analogeingang Versorgungsspannung */
#define UR_ANALOG_INPUT_PIN             A1      /* Analogeingang Sondentemperatur */
#define UA_ANALOG_INPUT_PIN             A0      /* Analogeingang Lambdawert */

// Einstellbare Parameter
#define UBAT_MIN                        150     /* Mindestspannung (ADC-Wert) an Ubat */
#define hardwareId                      0x01    /* Hardware-ID für die App */

// SPI-Einstellungen für den Uno R4 (125 kHz entspricht dem alten DIV128 bei 16 MHz)
SPISettings cj125SPISettings(125000, MSBFIRST, SPI_MODE1);

// Globale Variablen
int adcValue_UA = 0;
int adcValue_UR = 0;
int adcValue_UB = 0;
int adcValue_UA_Optimal = 0;
int adcValue_UR_Optimal = 0;
int HeaterOutput = 0;
uint16_t CJ125_Status = 0;
int HeaterStatus = 0;

// PID-Regler Variablen
int dState = 0;
int iState = 0;
const int iMax = 250;
const int iMin = -250;
const float pGain = 120.0;
const float iGain = 0.8;
const float dGain = 10.0;

// Funktion für die SPI-Kommunikation mit dem CJ125 (angepasst auf SPI Transactions)
uint16_t COM_SPI(uint16_t TX_data) {
  SPI.beginTransaction(cj125SPISettings);
  digitalWrite(CJ125_NSS_PIN, LOW);

  byte highByte = SPI.transfer(TX_data >> 8);
  byte lowByte = SPI.transfer(TX_data & 0xFF);

  digitalWrite(CJ125_NSS_PIN, HIGH);
  SPI.endTransaction();

  uint16_t Response = ((uint16_t)highByte << 8) | lowByte;
  return Response;
}

// PID-Temperaturregelung
int CalculateHeaterOutput(int input) {
  int error = adcValue_UR_Optimal - input;
  int position = input;

  float pTerm = -pGain * error;

  iState += error;
  if (iState > iMax) iState = iMax;
  if (iState < iMin) iState = iMin;

  float iTerm = -iGain * iState;
  float dTerm = -dGain * (dState - position);
  dState = position;

  int RegulationOutput = (int)(pTerm + iTerm + dTerm);

  if (RegulationOutput > 255) RegulationOutput = 255;
  if (RegulationOutput < 0) RegulationOutput = 0;

  return RegulationOutput;
}

// Analogeingänge und CJ125-Status aktualisieren
void UpdateInputs() {
  // Der Uno R4 nutzt standardmäßig 10-Bit ADC-Auflösung (0-1023), kompatibel zum Originalcode
  adcValue_UA = analogRead(UA_ANALOG_INPUT_PIN);
  adcValue_UR = analogRead(UR_ANALOG_INPUT_PIN);
  adcValue_UB = analogRead(UB_ANALOG_INPUT_PIN);

  // Werte über Luft-Sauerstoffgehalt begrenzen
  if (adcValue_UA > 854) adcValue_UA = 854;

  // CJ125 Diagnose-Status abfragen
  CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);
}

// Daten per BLE an die Smartphone-App senden
void UpdateBLEOutput() {
  BLE.poll(); // Auf dem Uno R4 wichtig, um den BLE-Stack regelmäßig zu bedienen

  if (BLE.connected()) {
    uint32_t checksum = HeaterStatus + hardwareId + CJ125_Status + adcValue_UA + adcValue_UR + adcValue_UB;

    String txString = String(HeaterStatus) + "," +
                      String(hardwareId) + "," +
                      String(CJ125_Status) + "," +
                      String(adcValue_UA) + "," +
                      String(adcValue_UR) + "," +
                      String(adcValue_UB) + "," +
                      String(checksum) + "\n";

    // Zeichenweise übertragen (identisch zum App-Protokoll)
    for (unsigned int i = 0; i < txString.length(); i++) {
      OxygenLevelChar.writeValue((uint8_t)txString.charAt(i));
    }
  }
}

void setup() {
  Serial.begin(9600);

  // Digitale Pins zuerst sauber initialisieren
  pinMode(CJ125_NSS_PIN, OUTPUT);
  digitalWrite(CJ125_NSS_PIN, HIGH); // Chip Select im Ruhezustand HIGH
  pinMode(LED_STATUS_POWER, OUTPUT);
  pinMode(LED_STATUS_HEATER, OUTPUT);
  pinMode(HEATER_OUTPUT_PIN, OUTPUT);

  // SPI starten
  SPI.begin();

  // BLE auf dem Uno R4 WiFi initialisieren
  if (!BLE.begin()) {
    Serial.println("Fehler beim Starten von BLE!");
    while (1);
  }

  BLE.setDeviceName("Arduino");
  BLE.setLocalName("Lambda Sensor");
  BLE.setAdvertisedService(LambdaService);
  LambdaService.addCharacteristic(OxygenLevelChar);
  BLE.addService(LambdaService);
  BLE.advertise();
}

void loop() {
  UpdateInputs();
  UpdateBLEOutput();

  // Heizung ausschalten bei Unterspannung oder Sensorfehler
  if (adcValue_UB < UBAT_MIN || CJ125_Status != CJ125_DIAG_REG_STATUS_OK) {
    HeaterStatus = 0;
    analogWrite(HEATER_OUTPUT_PIN, 0);

    digitalWrite(LED_STATUS_POWER, LOW);
    digitalWrite(LED_STATUS_HEATER, LOW);
  }

  // Sonde aufheizen (Kalibrierung -> Kondensationsphase -> Ramp-Up)
  if (adcValue_UR >= 250 && adcValue_UB >= UBAT_MIN && CJ125_Status == CJ125_DIAG_REG_STATUS_OK) {
    digitalWrite(LED_STATUS_POWER, HIGH);
    digitalWrite(LED_STATUS_HEATER, LOW);

    // Kalibrierungsmodus aktivieren
    COM_SPI(CJ125_INIT_REG1_MODE_CALIBRATE);
    delay(500);

    // Optimalwerte speichern
    adcValue_UA_Optimal = analogRead(UA_ANALOG_INPUT_PIN);
    adcValue_UR_Optimal = analogRead(UR_ANALOG_INPUT_PIN);

    // Normalbetrieb aktivieren (V=17 Verstärkung)
    COM_SPI(CJ125_INIT_REG1_MODE_NORMAL_V17);

    HeaterStatus = 1;

    // Versorgungsspannung berechnen
    float SupplyVoltage = (((float)adcValue_UB / 1023.0 * 5.0) / 10000.0) * 110000.0;

    // 1. Kondensationsphase: 2V für 5 Sekunden
    int CondensationPWM = (int)((2.0 / SupplyVoltage) * 255.0);
    analogWrite(HEATER_OUTPUT_PIN, CondensationPWM);

    int t = 0;
    while (t < 500 && adcValue_UB > UBAT_MIN) {
      UpdateInputs();
      UpdateBLEOutput();
      t += 1;
      delay(10);
    }

    // 2. Ramp-Up Phase: +0.4V / s ab 8.5V bis 13V
    float UHeater = 8.5;
    while (UHeater < 13.0 && adcValue_UB > UBAT_MIN && adcValue_UR > adcValue_UR_Optimal) {
      UpdateInputs();
      UpdateBLEOutput();

      CondensationPWM = (int)((UHeater / SupplyVoltage) * 255.0);
      if (CondensationPWM > 255) CondensationPWM = 255;

      analogWrite(HEATER_OUTPUT_PIN, CondensationPWM);

      UHeater += 0.004;
      delay(10);
    }
  }

  // PID-Regelung im Betriebszustand
  if (adcValue_UR < 250 && adcValue_UB >= UBAT_MIN && CJ125_Status == CJ125_DIAG_REG_STATUS_OK) {
    HeaterOutput = CalculateHeaterOutput(adcValue_UR);
    analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);

    digitalWrite(LED_STATUS_POWER, HIGH);
    digitalWrite(LED_STATUS_HEATER, HIGH);

    HeaterStatus = 2;
  }
}
