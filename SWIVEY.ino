#include <Arduino.h>
#include <Servo.h>
#include "pin.h"
#include "PID.h"
#include "PID_VC.h"
#include "pressureSensor.h"
#include "vpws.h"
#include "booting.h"
#include "ie_ratio.h"
#include "flowSensor.h"
#include "uart.h"
#include "Exvalve.h"

// NEXTION MAPDATA

int mappressure = 0;
int mappressuregrafik = 0;
int mapflow = 0;
int lastflow;
int mapvolume = 0;
int mapvolumev = 0;
int mapvolumep = 0;
int mapvolumex = 0;

int mapI = 0;
int mapE = 0;
int mapPIP = 0;
int mapPEEP = 0;
int mapVT = 0;
int mapIT = 0;
int mapITV = 0;
int mapRR = 0;
int mapIF = 0;
int mapIFPx = 0;
int mapF = 0;
int mapFIO2 = 0;
int mappressure1 = 0;

int activePageId = -1;  // ID halaman saat ini
bool calibrationDone = false;
bool systemShutdown = false;  // Flag untuk menandai apakah kalibrasi telah dilakukan
bool RDone = false;
bool SDone = false;


double PIP = 20.0;                  // Target tekanan dalam cmH2O
const unsigned long interval = 20;  // Interval sampling dalam milidetik
double PEEP = 5;                    // Tekanan PEEP dalam cmH2O
bool modePC = false;                // Flag untuk mengecek status sistem
bool modeVC = false;
float PIPmonitor = 0;
bool lastisInspiration = false;

unsigned long phaseStartTime = 0;
bool isInspiration = true;

int RR = 10;  // Rasio pernapasan (breaths per minute)
float inspR = 1;
float expR = 2;

double inspirationTime;  // Durasi inspirasi dalam ms
double expirationTime;   // Durasi ekspirasi dalam ms

// ROTARY
const int dtPin = 13;   // Pin DT dari rotary encoder
const int clkPin = 14;  // Pin CLK dari rotary encoder
const int swPin = 15;   // Pin tombol rotary encoder (SW)
int lastClkState;
int counter = 0;
int oxygen = 21;
int inspiratoryFlow;

// VC
double setVolume = 0.5;             // Target volume (L)
double V_aktual = 0.0;              // Volume aktual udara yang diberikan (liter)
const unsigned long sampling = 20;  // harus >100 karna flow nya baca tiap 100ms
double flowPEEP = 0;                // besar flow saat untuk peep 5 cmH2O
bool skipFirstCycle = true;
double adjustedFlow = 0.0;  // Flow yang sudah disesuaikan

float Q_min = 5.0;   // Laju aliran minimum
float Q_max = 20.0;  // Laju aliran maksimum

float outvpws;


void handleCommand41() {
  digitalWrite(relayPin, HIGH);
  pinMode(relayPin, INPUT);
  digitalWrite(relayPin, LOW);
  pinMode(relayPin, OUTPUT);

  Serial.println("ESC Off");
  delay(1000);
  esc.writeMicroseconds(MAX_PULSE_LENGTH);
  delay(6000);
  digitalWrite(relayPin, HIGH);
  pinMode(relayPin, INPUT);
  Serial.println("ESC On");
  delay(6000);
  esc.writeMicroseconds(MIN_PULSE_LENGTH);

  modePC = false;
  modeVC = false;
}

void handleCommand30() {
  Serial.println("Running");
  modePC = true;
  modeVC = false;
}

void handleCommand52() {
  Serial.println("Running");
  modePC = false;
  modeVC = true;
}

void handleCommand31() {
  Serial.println("Shutting down system");
  esc.writeMicroseconds(static_cast<int>(PEEP));
  digitalWrite(in3, LOW);  // Tutup katup inspirasi
  digitalWrite(in4, LOW);  // Tutup katup ekspirasi
  analogWrite(enB, 0);     // Matikan motor
  vpwsoff();
  modePC = false;  // Sistem berhenzti
  modeVC = false;
}

void readRotaryEncoder() {
  int currentClkState = digitalRead(clkPin);
  if (currentClkState != lastClkState) {
    if (digitalRead(dtPin) != currentClkState) {
      counter++;  // Putar ke kanan
    } else {
      counter--;  // Putar ke kiri
    }
    // Perbarui nilai PEEP jika berada di halaman Kontrol PEEP

    if (activePageId == 0x11) {
      PEEP = constrain(PEEP + (counter * 1), 5, 15);  // Batas nilai PEEP (0-20)
      counter = 0;                                    // Reset rotary value setelah perubahan
      Serial.print("KONTROL PEEP: ");
      Serial.println(PEEP);


      // Kirim nilai PEEP ke Nextion
      // String peepCommand = "kontrolPEEP.val=" + String(static_cast<int>(PEEP * 10));  // Kirim dalam format yang sesuai
      // uart_puts(uart0, peepCommand.c_str());
      // uart();
    }

    else if (activePageId == 0x10) {  // KONTROL OXYGEN
      oxygen = constrain(oxygen + (counter * 1), 21, 100);
      counter = 0;
      Serial.print("KONTROL OXYGEN: ");
      Serial.println(oxygen);


    }

    else if (activePageId == 0x31) {         // KONTROL RR
      RR = constrain(RR + counter, 10, 40);  // Update RR and constrain
      counter = 0;
      Serial.print("KONTROL RR: ");
      Serial.println(RR);


    }

    else if (activePageId == 0x13) {          // KONTROL PIP
      PIP = constrain(PIP + counter, 1, 30);  // Update PIP
      counter = 0;
      Serial.print("KONTROL PIP: ");
      Serial.println(PIP);


    }

    else if (activePageId == 0x14) {             // KONTROL RASIO I
      inspR = constrain(inspR + counter, 1, 5);  // Update RASIO I
      counter = 0;
      Serial.print("KONTROL INSPIRATION TIME: ");
      Serial.println(inspR);


    } else if (activePageId == 0x15) {         // KONTROL RASIO E
      expR = constrain(expR + counter, 1, 5);  // Update RASIO E
      counter = 0;
      Serial.print("KONTROL EXPIRATION TIME: ");
      Serial.println(expR);

    }

    else if (activePageId == 0x17) {                                // KONTROL VT
      setVolume = constrain(setVolume + (counter * 0.05), 0.3, 0.6);  // Update setVolume
      counter = 0;
      Serial.print("KONTROL VT: ");
      Serial.println(setVolume);
    }

    lastClkState = currentClkState;
  }
}

void checkPageChange() {

  // Memeriksa apakah ada cukup data yang tersedia (header dan data ID)
  if (uart_is_readable_within_us(uart0, 100)) {
    if (uart_getc(uart0) == 0x23) {     // Memeriksa apakah header cocok dengan "23"
      if (uart_getc(uart0) == 0x02) {   // Memastikan panjang data
        int pageId = uart_getc(uart0);  // Membaca ID halaman
        if (pageId != activePageId) {   // Jika ID halaman berubah
          activePageId = pageId;
          Serial.print("Halaman aktif: ");
          Serial.println(activePageId);

          calibrationDone = false;  // Reset flag kalibrasi saat halaman berubah
          RDone = false;
          SDone = false;


          switch (activePageId) {
            case 0x00: Serial.println("Halaman: splash"); break;
            case 0x01: Serial.println("Halaman: main0"); break;
            case 0x03: Serial.println("Halaman: main"); break;
            case 0x10: Serial.println("KONTROL OXYGEN"); break;
            case 0x11: Serial.println("KONTROL PEEP"); break;
            case 0x12:
              Serial.println("STOP");
              handleCommand31();
              break;
            case 0x13: Serial.println("KONTROL PIP "); break;
            case 0x14: Serial.println("KONTROL INSPIRASI "); break;
            case 0x15: Serial.println("KONTROL EXPIRASI "); break;
            case 0x16: Serial.println("KONTROL IF"); break;
            case 0x17: Serial.println("KONTROL VT"); break;
            case 0x24: Serial.println("MODE PC"); break;
            case 0x25: Serial.println("MODE VC"); break;
            case 0x40: Serial.println("kalibrasi cuma sekali"); break;
            case 0x41:
              Serial.println("KALIBRASI");
              handleCommand41();
              break;

            case 0x30:
              Serial.println("RUN");
              handleCommand30();
              break;

            case 0x52:
              Serial.println("RUN");
              handleCommand52();
              break;

            case 0x31: Serial.println("RR KONTROL"); break;

            default: Serial.println("Halaman Tidak Diketahui"); break;
          }
        }
      }
    }
  }
}

void hmi() {
  mapPEEP = map(PEEP, 0, 400, 0, 400);
  String PEEPP = "pc.PEEP.val=" + String(mapPEEP);
  uart_puts(uart0, PEEPP.c_str());
  uart();

  String PEEPPv = "vc.PEEP.val=" + String(mapPEEP);
  uart_puts(uart0, PEEPPv.c_str());
  uart();

  mapFIO2 = oxygen;
  String FIO2P = "pc.FIO2.val=" + String(mapFIO2);
  uart_puts(uart0, FIO2P.c_str());
  uart();

  String FIO2Pv = "vc.FIO2.val=" + String(mapFIO2);
  uart_puts(uart0, FIO2Pv.c_str());
  uart();

  mapRR = map(RR, 0, 400, 0, 400);
  String RRE = "pc.RR.val=" + String(mapRR);
  uart_puts(uart0, RRE.c_str());
  uart();

  String RREv = "vc.RR.val=" + String(mapRR);
  uart_puts(uart0, RREv.c_str());
  uart();

  mapPIP = map(PIP, 0, 400, 0, 400);

  String PIPP = "pc.PIP.val=" + String(mapPIP);
  uart_puts(uart0, PIPP.c_str());
  uart();

  String PIPPv = "vc.PIP.val=" + String(mapPIP);
  uart_puts(uart0, PIPPv.c_str());
  uart();


  mapI = map(inspR, 0, 400, 0, 400);
  String inspirasiP = "pc.inspirasi.val=" + String(mapI);
  uart_puts(uart0, inspirasiP.c_str());
  uart();
  String inspirasiPv = "vc.inspirasi.val=" + String(mapI);
  uart_puts(uart0, inspirasiPv.c_str());
  uart();

  String inspirasi2 = "ie.inspirasi.val=" + String(mapI);
  uart_puts(uart0, inspirasi2.c_str());
  uart();

  mapE = map(expR, 0, 400, 0, 400);
  String expirasiP = "pc.expirasi.val=" + String(mapE);
  uart_puts(uart0, expirasiP.c_str());
  uart();

  String expirasiPv = "vc.expirasi.val=" + String(mapE);
  uart_puts(uart0, expirasiPv.c_str());
  uart();

  String expirasiP2 = "ie.expirasi.val=" + String(mapE);
  uart_puts(uart0, expirasiP2.c_str());
  uart();

  mapVT = map(setVolume, 0, 400, 0, 400);
  String VTP = "pc.VT.val=" + String(mapVT);
  uart_puts(uart0, VTP.c_str());
  uart();

  mapVT = (setVolume * 1000);
  String VT = "vc.VT.val=" + String(mapVT);
  uart_puts(uart0, VT.c_str());
  uart();
}

void waveform() {
  // PRESSURE CONTROL

  String PSPCv = "pc.PS.val=" + String(mappressure);
  uart_puts(uart0, PSPCv.c_str());
  uart();

  String FPCv = "pc.FLOW.val=" + String(mapF);
  uart_puts(uart0, FPCv.c_str());
  uart();


  float insp;
  mapIT = map(PIPmonitor, 0, 400, 0, 400);  // DATA
  String ITP = "pc.IT.val=" + String(mapIT);
  uart_puts(uart0, ITP.c_str());
  uart();
}

void setup() {

  Serial2.setRX(9);
  Serial2.setTX(8);
  Serial2.begin(9600);

  //  Serial2.setRX(9);
  //  Serial2.setTX(8);
  //  Serial2.begin(9600);

  //  uart_init(uart1, 9600);
  //  gpio_set_function(9, GPIO_FUNC_UART);
  //  gpio_set_function(8, GPIO_FUNC_UART);
  //  uart_set_format(uart1, 8, 1, UART_PARTY_NONE);
  //  uart_set_fifo_enabled(uart1, true);


  pinMode(dtPin, INPUT);
  pinMode(clkPin, INPUT);
  pinMode(swPin, OUTPUT);
  digitalWrite(swPin, LOW);
  lastClkState = digitalRead(clkPin);

  // NEXTION
  gpio_set_function(0, GPIO_FUNC_UART);
  gpio_set_function(1, GPIO_FUNC_UART);
  uart_init(uart0, 250000);
  Serial.begin(250000);  // Memulai Serial Monitor untuk debugging
  Serial.println("Menunggu data dari Nextion...");
  // NEXTION

  bootsetup();
  inisialisasiPID();
  initializePID();
  sensorflowinit();
  // vpws();
  lastTime = millis();

  pinMode(22, OUTPUT);
  pinMode(20, OUTPUT);
  digitalWrite(22, LOW);
  digitalWrite(20, LOW);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  analogWriteFreq(60000);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  digitalWrite(relayPin, LOW);
  pinMode(relayPin, OUTPUT);
}


void loop() {
  hmi();
  sensorflow();
  readRotaryEncoder();
  checkPageChange();  // Memeriksa apakah ada perubahan halaman
  delay(10);          // Menambahkan jeda agar tidak terlalu cepat

  unsigned long currentTime = millis();

  float rat = inspR + expR;
  float resp = (60 / RR) * 1000;
  inspirationTime = (float(inspR) / float(rat)) * resp;
  expirationTime = resp - inspirationTime;

  double setFlow = ((setVolume * 1000 * 60) / inspirationTime);


  if (modePC) {
    if (isInspiration && currentTime - phaseStartTime >= inspirationTime) {
      isInspiration = false;
      phaseStartTime = currentTime;
    } else if (!isInspiration && currentTime - phaseStartTime >= expirationTime) {
      isInspiration = true;
      phaseStartTime = currentTime;
      integralV = 0;
    }

    if (isInspiration) {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enB, 200);
      PIPmonitor = 0.0;
      float dt = (currentTime - lastTime) / 1000.0;
      V_aktual += aFlow1 * (dt / 60.0);

    } else {
      V_aktual = 0;
      runExvalve();
    }


    if (currentTime - lastTime >= interval) {
      waveform();

      int measuredPressure = readPressure();
      targetPressure = isInspiration ? PIP : PEEP;
      double pidOutput = computePID(targetPressure, measuredPressure);
      PIPmonitor = max(measuredPressure, PIPmonitor);
      int throttle = constrain(static_cast<int>(minThrottle + pidOutput), minThrottle, maxThrottle);
      esc.writeMicroseconds(throttle);

      outvpws = map(oxygen, 21, 100, 0, 255);

      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, outvpws);


      lastTime = currentTime;
      Serial.print("pressure: ");
      Serial.print(measuredPressure);
      Serial.print(", ");
      Serial.print("flowI: ");
      Serial.print(aFlow1);
      Serial.print(", ");
      Serial.print("Inspirasi: ");
      Serial.print(inspirationTime);
      Serial.print(", ");
      Serial.print("Ekspirasi: ");
      Serial.println(expirationTime);


      mappressure = map(measuredPressure, 0, 400, 0, 400);  // Data grafik pressure
      String PSPC = "pc.PS.val=" + String(mappressure);
      uart_puts(uart0, PSPC.c_str());
      uart();

      mappressure1 = map(measuredPressure * 2, 0, 400, 0, 400);  // Grafik pressure
      String pr = "add 5,0," + String(mappressure1);
      uart_puts(uart0, pr.c_str());
      uart();

      mapF = map(aFlow1, 0, 400, 0, 400);  // DATA flow grafik
      String FPC = "pc.FLOW.val=" + String(mapF);
      uart_puts(uart0, FPC.c_str());
      uart();

      mapIFPx = map(aFlow1, 0, 1000, 0, 1000);

      mapvolumep = map(V_aktual * 50, 0, 1000, 0, 1000);    // Grafik
      mapvolumex = V_aktual * 1000;  // Grafik



      if (abs(aFlow1) >= 1) {
        mapflow = map(aFlow1 + 30, 0, 400, 0, 400);  // Grafik Flow
      } else {
        mapflow = lastflow;
      }

      String flow = "add 6,0," + String(mapIFPx);
      uart_puts(uart0, flow.c_str());
      uart();

      if (mapflow != 50) {
        lastflow = mapflow;
      }
      String VOLUMEp = "pc.VOL.val=" + String(mapvolumex);
      uart_puts(uart0, VOLUMEp.c_str());
      uart();

      Serial.print(V_aktual * 1000);
      String volumep = "add 7,0," + String(mapvolumep);
      uart_puts(uart0, volumep.c_str());
      uart();

      String VTP = "pc.VT.val=" + String(mapvolumex);
      uart_puts(uart0, VTP.c_str());
      uart();

      String IFP = "pc.IF.val=" + String(mapIFPx);
      uart_puts(uart0, IFP.c_str());
      uart();
    }
  }


  if (modeVC) {

    

    if (isInspiration && V_aktual >= setVolume) {
      isInspiration = false;
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enB, 0);

      esc.writeMicroseconds(static_cast<int>(flowPEEP));  // Atur throttle ke PEEP untuk mempertahankan tekanan

      double rasio = (setVolume) / V_aktual;
      adjustedFlow = ((setVolume * 1000 * 60) / inspirationTime) * rasio;

      // Batas flow
      if (adjustedFlow < Q_min) adjustedFlow = Q_min;
      if (adjustedFlow > Q_max) adjustedFlow = Q_max;

      setFlow = adjustedFlow;
    
    }

    if (isInspiration && currentTime - phaseStartTime >= inspirationTime) {
      isInspiration = false;
      phaseStartTime = currentTime;
    } else if (!isInspiration && currentTime - phaseStartTime >= expirationTime) {
      isInspiration = true;
      phaseStartTime = currentTime;
      integralV = 0;
    }


    if (isInspiration) {
      digitalWrite(in3, HIGH);  // Katup inspirasi terbuka
      digitalWrite(in4, LOW);   // Katup ekspirasi tertutup
      analogWrite(enB, 255);
      PIPmonitor = 0.0;

    } else {
      if (V_aktual < setVolume) {  // Penyesuaian flow hanya jika volume aktual belum tercapai
        double rasio = setVolume / V_aktual;
        adjustedFlow = ((setVolume * 1000 * 60) / inspirationTime) * rasio;

        // Batas flow
        if (adjustedFlow < Q_min) adjustedFlow = Q_min;
        if (adjustedFlow > Q_max) adjustedFlow = Q_max;

        setFlow = adjustedFlow;
      }

      runExvalve();
    }

    if (currentTime - lastTime >= sampling) {

      waveform();

      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, outvpws);

      int measuredPressure = readPressure();

      if (isInspiration) {

        float dt = (currentTime - lastTime) / 1000.0;
        V_aktual += aFlow1 * (dt / 60.0);

        double pidOutput = calculatePID(setFlow, aFlow1);
        pidOutput = constrain(pidOutput, 50, 200);

        PIPmonitor = max(measuredPressure, PIPmonitor);


        int throttle = constrain(static_cast<int>(minThrottle + pidOutput), 1000, 1700);
        esc.writeMicroseconds(throttle);

        lastTime = currentTime;

      } else {
        double setFlow = flowPEEP;
        float dt = (currentTime - lastTime) / 1000.0;
        V_aktual = 0;


        double pidOutput = calculatePID(setFlow, aFlow1);
        pidOutput = constrain(pidOutput, 50, 200);

        int throttle = constrain(static_cast<int>(minThrottle + pidOutput), 1000, 1700);
        esc.writeMicroseconds(throttle);

        lastTime = currentTime;
      }

      Serial.print("Pressure :");
      Serial.print(measuredPressure);
      Serial.print(", ");
      Serial.print("Actual Volume :");
      Serial.print(V_aktual * 1000);
      Serial.print(", ");
      Serial.print("flow1 :");
      Serial.print(aFlow1);
      Serial.print(", ");
      Serial.print("Target Flow :");
      Serial.println(setFlow);


      mappressure = map(measuredPressure, 0, 400, 0, 400);  // Data grafik pressure
      String PSPC = "pc.PS.val=" + String(mappressure);
      uart_puts(uart0, PSPC.c_str());
      uart();

      mappressure1 = map(measuredPressure * 2, 0, 400, 0, 400);  // Grafik pressure
      String pr = "add 5,0," + String(mappressure1);
      uart_puts(uart0, pr.c_str());
      uart();

      mapF = map(aFlow1, 0, 400, 0, 400);  // DATA flow grafik
      String FPC = "pc.FLOW.val=" + String(mapF);
      uart_puts(uart0, FPC.c_str());
      uart();

      if (abs(aFlow1) >= 1) {
        mapflow = map(aFlow1 + 30, 0, 400, 0, 400);  // Grafik Flow
      } else {
        mapflow = lastflow;
      }

      String flow = "add 6,0," + String(mapflow);
      uart_puts(uart0, flow.c_str());
      uart();

      if (mapflow != 50) {
        lastflow = mapflow;
      }

      mapvolume = map(V_aktual * 1000, 0, 1000, 0, 1000);  //data
      mapvolumev = map(V_aktual * 150, 0, 1000, 0, 1000);  // Grafik
      mapIF = map(aFlow1, 0, 1000, 0, 1000);


      // VOLUME CONTROL
      String prv = "add 15,0," + String(mappressure1);
      uart_puts(uart0, prv.c_str());
      uart();

      String flowv = "add 16,0," + String(mapflow);
      uart_puts(uart0, flowv.c_str());
      uart();

      String volumev = "add 17,0," + String(mapvolumev);
      uart_puts(uart0, volumev.c_str());
      uart();

      // VOLUME DATA
      String PRESSUREV = "vc.PS.val=" + String(mappressure);
      uart_puts(uart0, PRESSUREV.c_str());
      uart();

      String FLOWV1 = "vc.FLOW.val=" + String(mapIF);
      uart_puts(uart0, FLOWV1.c_str());
      uart();

      String VOLUMEV = "vc.VOL.val=" + String(mapvolume);
      uart_puts(uart0, VOLUMEV.c_str());
      uart();





      String IFv = "vc.IF.val=" + String(mapIF);
      uart_puts(uart0, IFv.c_str());
      uart();



      float insp;
      mapITV = map(PIPmonitor, 0, 400, 0, 400);  // DATA
      String ITv = "vc.IT.val=" + String(mapITV);
      uart_puts(uart0, ITv.c_str());
      uart();
    }
  }
}
