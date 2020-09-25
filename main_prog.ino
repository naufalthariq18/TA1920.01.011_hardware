/*  EL4091 TUGAS AKHIR II - TA1920.01.011
 *  PENGEMBANGAN METERAN DIGITAL BERBASIS APLIKASI MENGGUNAKAN LoRa
 *
 *  Anggota:
 *  1. Alija Rasyidi Daud (13214124) - Pengembangan Database dan Aplikasi User
 *  2. Muhammad Naufal Thariq (13216010) - Pengembangan Meteran
 *  3. Vincent Oktavian Kaulika (13216115) - Pengembangan Komunikasi antara Meteran dan Server
 *
 *  Program ini merupakan program utama yang mengimplementasikan fungsi-fungsi
 *  meteran sebagai tercantum pada dokumen TA yang terkait. Kode ini diimplementasikan
 *  pada ESP32. Wiring diagram yang digunakan dapat diakses pada dokumen terkait.
 */

/* PENGGUNAAN LIBRARY */
#include <string.h>
#include <PZEM004Tv30.h>
#include <Keypad.h>
#include <Keypad_I2C.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_PN532.h>
#include <LoRa.h>
#include "RTClib.h"
#include "AES.h"
#include "base64.h"
#include "mbedtls/md.h"

/* KONFIGURASI TASK HANDLE RTOS */
TaskHandle_t xScanKeypad = NULL;
TaskHandle_t xLoRaHandlerTx = NULL;
TaskHandle_t xLoRaHandlerRx = NULL;


/* PENDEFINISIAN PIN UNTUK KOMUNIKASI SPI PN532 */
#define PN532_SCK   (27)
#define PN532_MISO  (26)
#define PN532_MOSI  (25)
#define PN532_SS    (5)

/* IDENTIFICATION CODE METERAN (JANGAN DIUBAH, SANGAT SENSITIF) */
#define SCRTCD 3159
#define SCRTRC 428954425

/* DEKLARASI STRUKTUR DATA UNTUK PENGOLAHAN DATA DALAM PEMBUATAN PAYLOAD */
union buffer2Byte {
    byte buffer[2];
    uint16_t number;
};

union buffer4Byte {
    byte buffer[4];
    unsigned long numberLong;
    float numberFloat;
};

struct mockDateTime {
    uint8_t day;
    uint8_t month;
    union buffer2Byte year;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

union buffer4Byte convert;
union buffer2Byte convert2;

/* VARIABEL GLOBAL METERAN SECARA UMUM*/
float E_tot;                    //  Konsumsi energi residensi terukur
float E_all;                    //  Alokasi energi untuk residensi
unsigned long startTimeReport;  //  Digunakan untuk perhitungan interval waktu antara pengukuran sampel daya/tegangan/arus
bool isNearOver = 0;            //  Bernilai 1 jika 16 A < I < 20 A (arus residensi)
unsigned char isOn;             //  Penjelasan terdapat di fungsi changeMode()
unsigned char R_stat;           //  Penjelasan terdapat di fungsi changeMode()
char **prev_token = NULL;
int numToken = 0;
bool isKeypadOn = 0;            //  Bernilai 1 jika terdapat masukan keypad dari user
unsigned char isTampered;       //  Bernilai 1 jika meteran ter-tampered (magnetic switch terbuka)
bool isReportEnergy = 0;
bool isTimeSync = 0;
int counterTimeSync = 0;
char payload[37];               //  String payload yang telah diterima (setelah dekrispi)
bool isFromHandlerRx = 0;
unsigned char transmissionCode;
bool isACK = 1;
byte mock_key[16];              //  Mock key dalam pengembangan key change mechanism
uint16_t packet_time;
uint16_t packet_condition;
unsigned char packet_opcode;
float packet_energy;
bool isLCDChanged = 0;

/* VARIABEL GLOBAL TERKAIT PENGIRIMAN DATA KE SERVER */
byte opCode_send;           //  OPCODE yang akan dikirim ke server
unsigned char n_send;       //  Kode n yang akan dikirim ke server
unsigned char R_send;       //  Kode R yang akan dikirim ke server
char byteStream_send[16];   //  Byte stream 16 byte untuk slot NFC pada payload

/* VARIABEL GLOBAL TERKAIT PENERIMAAN DATA DARI SERVER */
char *rcvd_data = NULL;              //  String received data dari MQTT
char **prev_rcvd_data = NULL;        //  String received data sebelumnya (untuk memastikan tidak diproses dua kali)
unsigned char rcvd_buffer_mode = 0;
byte opCode_rec;
unsigned char n_rec;
unsigned char R_rec;
struct mockDateTime timeRec;
union buffer4Byte E_tot_rec;
union buffer4Byte E_all_rec;
char byteStream_rec[16];


/* VARIABEL TERKAIT TIMER INTERRUPT */
volatile int interruptCounter;
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/* Variabel global terkait packaging payload sebelum/setelah pengiriman */
char *node_id = "<4567>";  //From LG01 via web Local Channel settings on MQTT.Please refer <> dataformat in here.
uint8_t datasend[128];
char b64data[200];
char b64payload[200];
byte hmacResult[32];

/* KONFIGURASI PENGGUNAAN KEYPAD */
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};
byte rowPins[ROWS] = {4, 5, 6, 7};
byte colPins[COLS] = {0, 1, 2, 3};

/* INISIALISASI NAMA HARI UNTUK RTC */
char daysOfTheWeek[7][12] = {"Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"};

/*  INISIALISASI PIN UNTUK MAGNETIC SENSOR */
const int magnetSensor = 33;

/* INISIALISASI PIN UNTUK SOLID STATE RELAY */
const int ssr = 4;

/* INISIALISASI PIN UNTUK MODUL LORA */
const int csPin = 13;          // LoRa radio chip select (NSS)
const int resetPin = 12;        // LoRa radio reset (RST)
const int irqPin = 2;          // change for your board; must be a hardware interrupt pin (DIO0)

// Our AES key. Note that is the same that is used on the Node-Js side but as hex bytes.
byte key[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// The unitialized Initialization vector
byte my_iv[N_BLOCK] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


/* DEKLARASI KELAS UNTUK BERKOMUNIKASI DENGAN BERBAGAI MODUL */
Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);
Keypad_I2C keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS, 0x20);
PZEM004Tv30 pzem(&Serial2);
LiquidCrystal_I2C lcd(0x3F, 16, 2);
RTC_DS3231 rtc;
AES aes;

/* ISR yang dijalankan setiap detik */
void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

/* FUNCTION PROTOTYPES */
void interruptHandler();
void checkCondition();
void measureEnergy();
void receiveSignal();
void receiveNFC();
void setNumLCD(int row, int col, int num);
void changeMode(int n, int r);
void updatePulsa();
int extractDigit(int n);
void verifyCode();
void statusPrompt();
void readNFC(uint8_t uid[], uint8_t uidLength);
void overwriteEEPROM(char *key_write, float num, unsigned char isOn_write, unsigned char R_stat_write, int mode);
byte i2c_eeprom_read_byte(int deviceaddress, unsigned int eeaddress);
void i2c_eeprom_write_byte(int deviceaddress, unsigned int eeaddress, byte data);
void i2c_eeprom_write_page(int deviceaddress, unsigned int eeaddresspage, byte* data, byte length);
void tampered(int n);
void verifyRecoveryCode();
void onReceive(int packeySize);
void decrypt(String b64data_rcvd, String IV_base64, int lsize);
int getHMAC(char *payload);
void parsePayload();
void printHexChar(const byte *data, const uint32_t numBytes);
uint8_t getrnd();
void gen_iv(byte *iv);
void setup_aes();
void envelop_data();
void encrypt_payload(char *msg, int b64payloadlen);
void sendData(uint8_t *datasend_buffer, bool condition);

 /* SETUP AWAL */
void setup() {

     delay(3000);

     /* Inisialisasi Serial Communication */
     Serial.begin(115200);
     Serial2.begin(9600, SERIAL_8N1, 16, 17);
     while (!Serial);
     Serial.printf("Serial Communication initialization successful!\n");

     // if analog input pin 36 is unconnected, random analog
     // noise will cause the call to randomSeed() to generate
     // different seed numbers each time the sketch runs.
     // randomSeed() will then shuffle the random function.
     randomSeed(analogRead(36));

     /* LORA INITIALIZATION */
     LoRa.setPins(csPin, resetPin, irqPin);
     Serial.println(F("Start MQTT Example"));
     if (!LoRa.begin(923600000))   //923600000 is frequency
     {
         Serial.println("Starting LoRa failed!");
         while (1);
     }
     // Setup Spreading Factor (6 ~ 12)
     LoRa.setSpreadingFactor(9);

     // Setup BandWidth, option: 7800,10400,15600,20800,31250,41700,62500,125000,250000,500000
     //Lower BandWidth for longer distance.
     LoRa.setSignalBandwidth(125000);

     // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8)
     LoRa.setCodingRate4(5);
     LoRa.setSyncWord(0x34);
     LoRa.onReceive(onReceive);
     Serial.println("LoRa initialization successful!");

     /* Inisialisasi kondisi LCD */
     lcd.begin(21, 22);
     lcd.backlight();
     lcd.clear();
     Serial.printf("LCD initialization successful!\n");

     /* Inisialisasi konfigurasi keypad */
     Wire.begin();
     keypad.begin();
     keypad.setDebounceTime(50);
     Serial.printf("Keypad initialization successful!\n");

     /* Inisialisasi komunikasi dengan NFC */
     nfc.begin();
     uint32_t versiondata = nfc.getFirmwareVersion();
     if(!versiondata) {
         Serial.printf("Cannot find PN53X board for NFC scanning!\n");
         while(1);
     }
     nfc.SAMConfig();
     Serial.printf("NFC scanner initialization successful!\n");

     /* Inisialisasi variabel global berdasarkan nilai pada EEPROM RTC */
     Serial.printf("Fetching starting parameters from EEPROM\n");
     byte b = i2c_eeprom_read_byte(0x57, 0);
     isTampered = (unsigned char) (b & 8) >> 3;
     isOn = (unsigned char) (b & 4) >> 2;
     R_stat = (unsigned char) b & 3;

     // Exceptions
     if(isOn && R_stat != 0 && R_stat != 1) {
         isOn = 0;
         R_stat = 0;
         delay(100);
         overwriteEEPROM(NULL, 0, isOn, R_stat, 2);
     }

     for(unsigned int i = 0; i < 4; i++) {
         convert.buffer[i] = i2c_eeprom_read_byte(0x57, i + 5);
     }
     E_all = convert.numberFloat;
     if(E_all < 0) {
         E_all = 0;
         delay(100);
         overwriteEEPROM(NULL, E_all, 0, 0, 1);
     }

     for(unsigned int i = 0; i < 4; i++) {
         convert.buffer[i] = i2c_eeprom_read_byte(0x57, i + 1);
     }
     E_tot = convert.numberFloat;
     if(E_tot >= E_all) {
         E_tot = E_all;
         delay(100);
         overwriteEEPROM(NULL, E_tot, 0, 0, 0);
         if(isOn) {
             isOn = 0;
             R_stat = 1;
             delay(100);
             overwriteEEPROM(NULL, 0, isOn, R_stat, 2);
         }
     } else if(!isOn && R_stat == 1) {
         isOn = 1;
         R_stat = 0;
         delay(100);
         overwriteEEPROM(NULL, 0, isOn, R_stat, 2);
     }

     if(!isOn) isLCDChanged = 1;

     Serial.printf("isTampered = %u\n", isTampered);
     Serial.printf("isOn = %u\n", isOn);
     Serial.printf("R_stat = %u\n", R_stat);
     Serial.printf("E_tot = %f\n", E_tot);
     Serial.printf("E_all = %f\n", E_all);

     for(unsigned char i = 0; i < 16; i++) {
         mock_key[i] = i2c_eeprom_read_byte(0x57, i + 9);
     }

     for(unsigned char i = 0; i < 2; i++) {
         convert2.buffer[i] = i2c_eeprom_read_byte(0x57, i + 25);
     }
     uint16_t raw_timePacket = convert2.number;
     packet_time = raw_timePacket & 4095;
     packet_condition = (raw_timePacket >> 12) & 3;
     packet_opcode = (unsigned char) ((raw_timePacket >> 14) & 3);
     for(unsigned char i = 0; i < 4; i++) {
       convert.buffer[i] = i2c_eeprom_read_byte(0x57, i + 27);
     }
     packet_energy = convert.numberFloat;

     /* FOR DEBUGGING PURPOSES ONLY */
     // delay(500);
     // Serial.printf("Stored key: "); printHexChar(mock_key, sizeof(mock_key));
     // delay(500);
     Serial.printf("Stored time: %u:%u\n", packet_time / 60, packet_time % 60);
     Serial.printf("Stored condition: %u\n", packet_condition);
     Serial.printf("Stored opcode: %u\n", packet_opcode);

     Serial.printf("Starting parameters are already set!\n");

     /* Inisialisasi Magnetic Switch MC38 untuk Anti-Tampering */
     pinMode(magnetSensor, INPUT);
     Serial.printf("Magnetic Sensor initialization successful!\n");

     /* Inisialisasi Relay Tegangan Tinggi (SSR-40DA) */
     pinMode(ssr, OUTPUT);
     int state = digitalRead(magnetSensor);
     if(isOn && !state && !isTampered) {
         digitalWrite(ssr, HIGH);
         startTimeReport = millis();
     } else {
         digitalWrite(ssr, LOW);
         if(isTampered) isLCDChanged = 1;
     }
     Serial.printf("Solid State Relay initialization successful!\n");

     /* Inisialisasi RTC */
     if (! rtc.begin()) {
         Serial.println("Couldn't find RTC");
         while (1);
     }
     if (rtc.lostPower()) {
         Serial.println("RTC lost power. Readjusting the time.");
         rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
     } else {
         delay(100);
         DateTime now = rtc.now();
         Serial.printf("{%02d:%02d:%02d, ", now.hour(), now.minute(), now.second());
         Serial.printf("%02d-%02d-20%02d}\n", now.day(), now.month(), now.year() - 2000);

         bool timeCondition = now.hour() < 24 && now.minute() < 60 && now.second() < 60 && now.day() < 32 && now.month() < 13;
         if(!timeCondition) {
             Serial.println("RTC is incorrect, recalibration with server time is needed.\n");
         }
     }
     Serial.printf("RTC initialization successful!\n");

     timer = timerBegin(0, 80, true);
     timerAttachInterrupt(timer, &onTimer, true);
     timerAlarmWrite(timer, 1000000, true);
     timerAlarmEnable(timer);
     Serial.printf("Timer initialization successful!\n");

     /* INISIALISASI GENERATOR PAYLOAD */
     opCode_send = 0;
     n_send = 0;
     R_send = 0;
     for(char i = 0; i < 16; i += 4) {
         byteStream_send[i] = 0;
         byteStream_send[i + 1] = 0;
         byteStream_send[i + 2] = 0;
         byteStream_send[i + 3] = 0;
     }

     prev_rcvd_data = (char **) malloc(5 * sizeof(char *));
     for(unsigned char i = 0; i < 5; i++) {
         prev_rcvd_data[i] = (char *) malloc(40 * sizeof(char));
         for(unsigned char j = 0; j < 40; j++) {
             prev_rcvd_data[i][j] = '0';
         }
     }

     Serial.printf("Payload Mechanism initialization successful!\n");

     /* DEFINISI TASK RTOS (Dokumentasi masing-masing task terdapat di Dokumentasi
        implementasi task) */
    xTaskCreatePinnedToCore(
    ScanKeypad
    ,  "Scan Keypad"
    ,  8192
    ,  NULL
    ,  1
    ,  &xScanKeypad
    ,  0);

    xTaskCreatePinnedToCore(
    ScanNFC
    ,  "Scan NFC"
    ,  8192
    ,  NULL
    ,  1
    ,  NULL
    ,  1);

    xTaskCreatePinnedToCore(
    ScanLoRa
    ,  "Scan LoRa"
    ,  8192
    ,  NULL
    ,  1
    ,  NULL
    ,  1);
    Serial.printf("RTOS initialization successful!\n\n");

    Serial.printf("Meter is set!\n\n");

    /* Attempt untuk receive paket data dari server */
    LoRa.receive();
 }

void loop() {
    vTaskDelay(10);
}

/*  TASK: ScanKeypad()
 *  Fungsi utama terdiri dari scan keypad
 */
void ScanKeypad(void *pvParameters)
{
    (void) pvParameters;

    for (;;)
    {
        if(interruptCounter > 0) {
            interruptHandler();
        }

        receiveSignal();

        vTaskDelay(300);
    }
}

/*  TASK: ScanNFC()
 *  Fungsi ini terdiri dari scan NFC
 */
void ScanNFC(void *pvParameters)
{
    (void) pvParameters;

    for (;;)
    {
        receiveNFC();

        vTaskDelay(700);
    }
}

/*  TASK: ScanLoRa()
 *  Fungsi ini terdiri dari scan LoRa
 */
void ScanLoRa(void *pvParameters)
{
    (void) pvParameters;

    unsigned long timeLeftToUpdate = millis();
    bool check = 1;
    bool check1 = 1;
    bool check2 = 1;

    for (;;)
    {
        LoRa.receive();

        if(millis() - timeLeftToUpdate > 10000) {
            if(check && !xLoRaHandlerRx) {
                if(packet_opcode == 0 && packet_condition) {
                    Serial.print("Initializing ReportEnergy at core 1\n");
                    // Pembuatan task baru
                    xTaskCreatePinnedToCore(
                        ReportEnergy
                        ,  "Report Energy"
                        ,  8192
                        ,  NULL
                        ,  2
                        ,  NULL
                        ,  1);
                        check = 0;
                } else if(packet_opcode == 1){
                    if(packet_condition == 2) {
                        if(check2) {
                            Serial.printf("Sending [n,R] = [%d,%d] to server\n", isOn, R_stat);

                            opCode_send = 1;
                            n_send = (unsigned char) (isOn + 2 * isTampered);
                            R_send = (unsigned char) R_stat;
                            transmissionCode = 2;
                            Serial.println("Initializing LoRaHandlerTx at core 1");
                            xTaskCreatePinnedToCore(
                            LoRaHandlerTx
                            ,  "LoRa Handler Transmit"
                            ,  8192
                            ,  NULL
                            ,  3
                            ,  &xLoRaHandlerTx
                            ,  1);
                            check2 = 0;
                        }
                        timeLeftToUpdate = millis();
                    }

                    if(packet_condition == 1) {
                        if(check1) {
                            opCode_send = 4;
                            transmissionCode = 2;
                            Serial.print("Initializing LoRaHandlerTx at core 1\n");
                            xTaskCreatePinnedToCore(
                            LoRaHandlerTx
                            ,  "LoRa Handler Transmit"
                            ,  8192
                            ,  NULL
                            ,  3
                            ,  &xLoRaHandlerTx
                            ,  1);
                            check1 = 0;
                        }
                        timeLeftToUpdate = millis();
                    }

                    if(!packet_condition) {
                        check = 0;
                    }
                } else if(packet_opcode == 2 && packet_condition == 1) {
                    opCode_send = 1;
                    n_send = (unsigned char) (isOn + 2 * isTampered);
                    R_send = (unsigned char) R_stat;
                    transmissionCode = 2;

                    Serial.print("Initializing LoRaHandlerTx at core 1\n");
                    xTaskCreatePinnedToCore(
                    LoRaHandlerTx
                    ,  "LoRa Handler Transmit"
                    ,  8192
                    ,  NULL
                    ,  3
                    ,  &xLoRaHandlerTx
                    ,  1);
                    check = 0;
                }
            }
        }

        vTaskDelay(500);
    }
}

/*  TASK: LoRaHandlerRx()
 *  Handler pengolahan data dan konfigurasi meteran dari server
 */
void LoRaHandlerRx(void *pvParameters) {
    Serial.printf("Starting LoRaHandlerRx procedure.\n");
    byte local_opCode_rec = opCode_rec;
    bool needACK = 1;

    Serial.printf("Recalibrating time with server time.\n");
    rtc.adjust(DateTime(timeRec.year.number, timeRec.month, timeRec.day, timeRec.hour, timeRec.minute, timeRec.second));
    Serial.printf("Time recalibrated: ");
    Serial.printf("{%02d:%02d:%02d, ", timeRec.hour, timeRec.minute, timeRec.second);
    Serial.printf("%02d-%02d-%04d}\n", timeRec.day, timeRec.month, timeRec.year.number);

    switch(local_opCode_rec) {
        case 0x04  :
                        isACK = 1;
                        needACK = 0;
                        if(isReportEnergy && packet_opcode == 0 && packet_condition == 2) {
                            packet_condition = 1;
                            vTaskDelay(100);
                            overwriteEEPROM(NULL, 0, 0, 0, 4);
                            isReportEnergy = 0;
                            Serial.printf("Energy Report successful!\n");
                        } else if(packet_opcode == 1 && packet_condition) {
                            packet_condition--;
                            vTaskDelay(100);
                            overwriteEEPROM(NULL, 0, 0, 0, 4);
                        } else if(packet_opcode == 2 && packet_condition) {
                            packet_condition = 0;
                            vTaskDelay(100);
                            overwriteEEPROM(NULL, 0, 0, 0, 4);
                        }
                        break;
        case 0x05  :{
                        Serial.printf("Receiving [n,R] = [%u,%u] from server\n", n_rec, R_rec);
                        bool condition = !(packet_opcode == 1 && (packet_condition == 1 || packet_condition == 0) && n_rec == isOn && R_rec == R_stat);
                        if(condition) {
                            packet_opcode = 1;
                            packet_condition = 2;
                            vTaskDelay(100);
                            overwriteEEPROM(NULL, 0, 0, 0, 4);
                            isFromHandlerRx = 1;
                            changeMode((int) n_rec, (int) R_rec);
                            isFromHandlerRx = 0;
                        }
                        break;
                    }
        case 0x06  :{
                        vTaskDelay(100);
                        overwriteEEPROM(byteStream_rec, 0, 0, 0, 3);
                        /* SOME LINES ARE FOR DEBUGGING PURPOSES ONLY */
                        Serial.printf("Previous key: "); printHexChar(mock_key, sizeof(mock_key));
                        for(unsigned char i = 0; i < 16; i++) {
                            mock_key[i] = i2c_eeprom_read_byte(0x57, i + 9);
                        }
                        bool isSame = 1;
                        for(unsigned char i = 0; i < 16; i++) {
                            if(mock_key[i] != byteStream_rec[i]) {
                                isSame = 0;
                            }
                        }
                        while(!isSame) {
                            Serial.printf("Written key is not the same. Rewriting key.\n");
                            vTaskDelay(1000);
                            overwriteEEPROM(byteStream_rec, 0, 0, 0, 3);
                            for(unsigned char i = 0; i < 16; i++) {
                                mock_key[i] = i2c_eeprom_read_byte(0x57, i + 9);
                            }
                            isSame = 1;
                            for(unsigned char i = 0; i < 16; i++) {
                                if(mock_key[i] != byteStream_rec[i]) {
                                    isSame = 0;
                                }
                            }
                        }
                        Serial.printf("Private key change is successful!\n");
                        Serial.printf("New key: "); printHexChar(mock_key, sizeof(mock_key));
                        break;
                    }
        case 0x08  :{
                        if(!((!packet_opcode && packet_condition == 1) || (!packet_opcode && packet_condition == 2))) {
                            uint16_t packet_time_rec = (uint16_t) (timeRec.minute * 60 + timeRec.second);
                            if(packet_time != packet_time_rec) {
                                packet_opcode = 0;
                                packet_condition = 2;
                                packet_time = packet_time_rec;
                                packet_energy = E_all_rec.numberFloat;
                                vTaskDelay(100);
                                overwriteEEPROM(NULL, 0, 0, 0, 4);
                            }
                        }

                        if(packet_opcode == 0 && packet_condition == 1) {
                            needACK = 0;
                        }

                        if(!packet_opcode && packet_condition) {
                            Serial.print("Initializing ReportEnergy at core 1\n");
                            // Pembuatan task baru
                            xTaskCreatePinnedToCore(
                                ReportEnergy
                                ,  "Report Energy"
                                ,  8192
                                ,  NULL
                                ,  2
                                ,  NULL
                                ,  1);
                                vTaskDelay(7000);
                        }
                        break;
                    }
        default :
                        Serial.printf("This message should NOT have been printed!\n");
                        break;
    }

    if(needACK && !isReportEnergy) {
        vTaskDelay(10000);
        if(packet_opcode == 1 && packet_condition == 2) {
            while(packet_condition == 2) {
                vTaskDelay(10000);
            }
        }
        opCode_send = 4;
        transmissionCode = 2;
        Serial.print("Initializing LoRaHandlerTx at core "); Serial.println(xPortGetCoreID());
        xTaskCreatePinnedToCore(
        LoRaHandlerTx
        ,  "LoRa Handler Transmit"
        ,  8192
        ,  NULL
        ,  3
        ,  &xLoRaHandlerTx
        ,  xPortGetCoreID());
    }

    Serial.printf("Ending LoRaHandlerRx procedure.\n");
    vTaskDelete(NULL);
}

/*  TASK: LoRaHandlerTx()
 *  Handler pembuatan payload dan pengiriman ke server
 *
 *  Transmission code:
 *  0   : default
 *  1   : waiting for time sync confirmation
 *  2   : waiting for acknowledgement from server
 */
void LoRaHandlerTx(void *pvParameters) {
    Serial.printf("Starting LoRaHandlerTx procedure.\n");

    unsigned char localConditionCode = transmissionCode;
    byte local_opCode_send = opCode_send;
    unsigned char local_n_send = n_send;
    unsigned char local_R_send = R_send;
    unsigned char first_send;
    float E_tot_send = E_tot;
    float E_all_send = E_all;

    if(local_opCode_send == 3) {
        isReportEnergy = 1;
        isLCDChanged = 1;
    }

    unsigned char countACK = 0;
    while(!isACK && countACK < 10) {
        Serial.printf("Counter: %u/10\n", countACK + 1);
        Serial.printf("Waiting for server acknowledgement from previous data packet.\n");
        countACK++;
        vTaskDelay(10000);
    }
    if(!isACK) {
        ESP.restart();
    }

    vTaskDelay(100);
    DateTime now = rtc.now();
    bool timeCondition = now.hour() < 24 && now.minute() < 60 && now.second() < 60 && now.day() < 32 && now.month() < 13;
    unsigned char countTimer = 0;
    while(!timeCondition && countTimer < 10) {
        if(countTimer == 0) Serial.println("RTC is incorrect, recalibration with server time is needed.\n");
        countTimer++;
        vTaskDelay(100);
        now = rtc.now();
        timeCondition = now.hour() < 24 && now.minute() < 60 && now.second() < 60 && now.day() < 32 && now.month() < 13;
    }

    if(!timeCondition) {
        isTimeSync = 0;
        counterTimeSync = 0;
    } else {
        isTimeSync = 1;
    }

    first_send = 0x00;
    first_send |= local_opCode_send;
    first_send = (first_send << 2) | local_n_send;
    first_send = (first_send << 2) | local_R_send;
    payload[0] = first_send;

    union buffer4Byte meterID_send;
    meterID_send.numberLong = (unsigned long) SCRTCD;
    for(unsigned char i = 0; i < 4; i++) {
        payload[1 + i] = (unsigned char) meterID_send.buffer[i];
    }

    if(isTimeSync) {
        payload[5] = (unsigned char) now.day();
        payload[6] = (unsigned char) now.month();
        union buffer2Byte year_send;
        year_send.number = (uint16_t) now.year();
        payload[7] = (unsigned char) year_send.buffer[0];
        payload[8] = (unsigned char) year_send.buffer[1];
        payload[9] = (unsigned char) now.hour();
        if(isReportEnergy && packet_condition) {
            payload[10] = (unsigned char) (packet_time / 60);
            payload[11] = (unsigned char) (packet_time % 60);
        }
        payload[10] = (unsigned char) now.minute();
        payload[11] = (unsigned char) now.second();
    } else {
        for(unsigned char i = 5; i < 12; i++) {
            payload[i] = 0;
        }
    }

    union buffer4Byte placeEnergy;
    placeEnergy.numberFloat = E_tot_send;
    for(unsigned char i = 0; i < 4; i++) {
        payload[12 + i] = (unsigned char) placeEnergy.buffer[i];
    }
    placeEnergy.numberFloat = E_all_send;
    for(unsigned char i = 0; i < 4; i++) {
        payload[16 + i] = (unsigned char) placeEnergy.buffer[i];
    }

    for(unsigned char i = 0; i < 16; i++) {
        payload[20 + i] = byteStream_send[i];
    }

    // Printing payload components
    Serial.printf("Prepared payload components:\n");
    Serial.printf("Opcode\t\t\t= "); printHexChar(&local_opCode_send, sizeof(local_opCode_send));
    Serial.printf("n\t\t\t= %u - ", n_send); printHexChar(&local_n_send, sizeof(local_n_send));
    Serial.printf("R\t\t\t= %u - ", R_send); printHexChar(&local_R_send, sizeof(local_R_send));
    Serial.printf("Byte-0\t\t\t= "); printHexChar(&first_send, sizeof(first_send));
    Serial.printf("Meter ID\t\t= %lu - ", meterID_send.numberLong); printHexChar(meterID_send.buffer, sizeof(meterID_send.buffer));
    Serial.printf("Time\t\t\t= ");
    Serial.printf("{%02u:%02u:%02u, ", now.hour(), now.minute(), now.second());
    Serial.printf("%02u-%02u-%04u}\n", now.day(), now.month(), now.year());
    Serial.printf("NFC Code\t\t= "); printHexChar((byte *) byteStream_send, sizeof(byteStream_send));

    payload[36] = '\0';


    Serial.printf("\nPREPARED PAYLOAD:\n"); printHexChar((byte *) payload, sizeof(payload));

    if(sizeof(payload) == 37) {
        Serial.printf("Starting data packet preparation.\n");
        envelop_data();
        Serial.printf("Data packet preparation finished. Sending data packet.\n");
        sendData(NULL, 0);
        isACK = 0;

        if(localConditionCode == 1) {
            vTaskDelay(10000);
            now = rtc.now();
            bool timeCondition = now.hour() < 24 && now.minute() < 60 && now.second() < 60 && now.day() < 32 && now.month() < 13;
            if(timeCondition) {
                isTimeSync = 1;
                Serial.printf("Time synchronization successful!\n");
            } else {
                Serial.printf("Time synchronization failed. Restarting system.\n");
                ESP.restart();
            }
        } else if(localConditionCode == 2) {
            Serial.print("Initializing AssertACK at core "); Serial.println(xPortGetCoreID());
            xTaskCreatePinnedToCore(
            AssertACK
            ,  "Acknowledgement Assertion"
            ,  8192
            ,  NULL
            ,  2
            ,  NULL
            ,  xPortGetCoreID());
        } else {
            isACK = 1;
            vTaskDelay(5000);
        }
        if(xLoRaHandlerRx) vTaskResume(xLoRaHandlerRx);
    } else {
        Serial.printf("Payload generation failed.\n");
    }

    Serial.printf("Ending LoRaHandlerTx procedure.\n");


    vTaskDelete(NULL);
}

/*  TASK: AssertACK()
 *  Handler untuk memastikan server menerima sinyal dari meteran
 *  dengan dikirimnya sinyal ACK dari server.
 */
void AssertACK(void *pvParameters) {
    Serial.printf("Starting AssertACK procedure.\n");

    uint8_t datasend_buffer[128];
    unsigned char countACK = 0;
    unsigned char counterLoop = 0;

    for(int i = 0; i < sizeof(datasend); i++) {
        datasend_buffer[i] = datasend[i];
    }

    Serial.printf("Starting counter to wait for ACK.\n");
    vTaskDelay(1000);
    while(!isACK && countACK < 10) {
        if(counterLoop > 10) {
            countACK++;
            Serial.printf("Counter: %u/10\n", countACK);
            Serial.println("Resending data packet: " + String((char *) datasend_buffer));
            sendData(datasend_buffer, 1);
            counterLoop = 0;
        } else {
            counterLoop++;
        }

        vTaskDelay(1000);
    }

    if(isACK) {
        Serial.printf("Acknowledgement detected.\n");
    } else {
        Serial.printf("Counter: %u/10\n", countACK);
        Serial.printf("No ACK detected.\n");
        ESP.restart();
    }

    Serial.printf("Ending AssertACK procedure.\n");

    vTaskDelete(NULL);
}

/*  TASK: ReportEnergy()
 *  Handler untuk pelaporan penggunaan energi ke server
 */
void ReportEnergy(void *pvParameters) {
    Serial.printf("Starting ReportEnergy procedure.\n");

    if(packet_condition == 2) {
        opCode_send = 4;
        transmissionCode = 2;
        Serial.print("Initializing LoRaHandlerTx at core "); Serial.println(xPortGetCoreID());
        xTaskCreatePinnedToCore(
        LoRaHandlerTx
        ,  "LoRa Handler Transmit"
        ,  8192
        ,  NULL
        ,  3
        ,  &xLoRaHandlerTx
        ,  xPortGetCoreID());

        vTaskDelay(10000);

        Serial.printf("Sending E_tot = %.2f and E_all = %.2f to server\n", E_tot, E_all);

        opCode_send = 3;
        transmissionCode = 2;
        Serial.println("Initializing LoRaHandlerTx at core 1");
        xTaskCreatePinnedToCore(
        LoRaHandlerTx
        ,  "LoRa Handler Transmit"
        ,  8192
        ,  NULL
        ,  3
        ,  &xLoRaHandlerTx
        ,  1);

        unsigned char countACK = 0;
        while(countACK < 10 && isReportEnergy) {
            Serial.printf("Counter: %u/10\n", countACK + 1);
            Serial.printf("Waiting for server acknowledgement from energy report.\n");
            countACK++;
            vTaskDelay(10000);
        }
    }

    if(packet_condition == 1) {
        float E_hasilbayar = packet_energy;

        float E_all_new = E_all + E_hasilbayar - E_tot;
        bool statement = (E_all_new > E_all && !((!isOn && !R_stat) || (!isOn && R_stat == 3))) || (!isOn && R_stat == 2);
        if(statement) {
            changeMode(1, 0);
        }

        E_all = E_all_new;
        vTaskDelay(100);
        overwriteEEPROM(NULL, E_all, 0, 0, 1);
        E_tot = 0;
        vTaskDelay(100);
        overwriteEEPROM(NULL, E_tot, 0, 0, 0);
        packet_condition = 0;
        vTaskDelay(100);
        overwriteEEPROM(NULL, 0, 0, 0, 4);

        if(packet_energy > 0) {
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("PULSA");
            lcd.setCursor(0,1);
            lcd.print("DITAMBAHKAN");
            isKeypadOn = 1;
            vTaskDelay(3000);
            isKeypadOn = 0;
        }
    }

    isReportEnergy = 0;
    Serial.printf("Ending ReportEnergy procedure.\n");

    vTaskDelete(NULL);
}

/*  TASK: CheckMeter()
 *  Pengecekan rutin meteran.
 */
void CheckMeter(void *pvParameters) {

    checkCondition();
    measureEnergy();

    vTaskDelete(NULL);
}

/* Handler ketika interrupt terjadi (BUKAN ISR) */
void interruptHandler() {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);

    xTaskCreatePinnedToCore(
    CheckMeter
    ,  "Check Meter"
    ,  8192
    ,  NULL
    ,  2
    ,  NULL
    ,  0);
}

/*  checkCondition()
 *  Mengecek kondisi meteran (kode isOn dan energi yang digunakan).
 *  Meteran akan dimatikan jika dimatikan secara sengaja atau jika energi yang
 *  digunakan lebih besar dari alokasi energi residensi tersebut.
 */
void checkCondition() {
    int state = digitalRead(magnetSensor);
    if(!isTampered && !state) {
        if(isOn && E_tot > E_all) {
            changeMode(0,1);
            E_tot = E_all;
            vTaskDelay(100);
            overwriteEEPROM(NULL, E_tot, 0, 0, 0);

        }
    } else {
        if(!isTampered) {
            tampered(1);
        }
    }
}

/*  measureEnergy()
 *  1. Pengukuran daya dan arus serta perhitungan energi berdasarkan hasil pengukuran.
 *     - Jika arus melebihi 20 A, meteran dimatikan demi keselamatan kerja.
 *     - Jika arus antara 16-20 A, meteran mengirimkan peringatan ke user mengenai batas arus.
 *  2. Update penggunaan energi pada console (untuk troubleshooting) dan LCD
 */
void measureEnergy() {
    /* PENGUKURAN ARUS DAN DAYA RESIDENSI */
    float measuredP;
    float measuredI;
    float measuredV;
    if(isOn && !isTampered) {
        measuredP = pzem.power();
        measuredI = pzem.current();
        measuredV = pzem.voltage();
        if(!isnan(measuredP) && !isnan(measuredI) && measuredI < 40) {
            E_tot += measuredP * (float) (millis() - startTimeReport) / 1000;
            startTimeReport = millis();
            vTaskDelay(100);
            overwriteEEPROM(NULL, E_tot, 0, 0, 0);
        }
        if(isnan(measuredI)) {
            measuredI = 0;
        }
        if(isnan(measuredV)) {
            measuredV = 220;
        }
    } else {
        measuredP = 0;
        measuredI = 0;
        measuredV = 220;
    }

    /* PENGECEKAN KONDISI ARUS RESIDENSI */
    bool condition = measuredI > 20 && measuredI < 40 &&
                        measuredV > 242 && measuredV < 198;
    if(condition) {
            changeMode(0,2);
    } else if(measuredI > 16) {
        if(!isNearOver) {
            isNearOver = 1;
            /* SEND WARNING */
            Serial.printf("Sending [n,R] = [1,2] to server\n");
            /* Kirim kode [n,R] = [2,0] ke server */
            opCode_send = 1;
            n_send = 1;
            R_send = 2;
            transmissionCode = 2;
            Serial.println("Initializing LoRaHandlerTx at core 1");
            xTaskCreatePinnedToCore(
            LoRaHandlerTx
            ,  "LoRa Handler Transmit"
            ,  8192
            ,  NULL
            ,  3
            ,  &xLoRaHandlerTx
            ,  1);
        }
    } else {
        isNearOver = 0;
    }

    /* UPDATE PENGGUNAAN ENERGI PADA CONSOLE DAN LCD */
    if(!isKeypadOn) {
        Serial.printf("isTampered = %d\tisOn = %d\tR_stat = %d\tE_all = %.2f\tE_tot = %.2f\tE_sisa = %.2f\tI = %.2f\tV = %.2f\n",
                        isTampered, isOn, R_stat, E_all, E_tot, E_all - E_tot, measuredI, measuredV);
        if(!isTampered) {
            if(isReportEnergy) {
                if(isLCDChanged) {
                    lcd.clear();
                    lcd.setCursor(0,0); lcd.print("PELAPORAN");
                    lcd.setCursor(0,1); lcd.print("KONSUMSI ENERGI");
                    isLCDChanged = 0;
                }
            } else if(isOn || !isOn && isLCDChanged) {
                lcd.clear();
                lcd.setCursor(0,0); lcd.print("Status = ");
                lcd.print(isOn);
                lcd.print(R_stat);
                lcd.setCursor(0,1); lcd.print("Pulsa  = ");
                setNumLCD(9,1, E_all - E_tot);
                isLCDChanged = 0;
            }
        } else if(isLCDChanged){
            lcd.clear();
            lcd.setCursor(0,0); lcd.print("PERUSAKAN METER");
            lcd.setCursor(0,1); lcd.print("TERDETEKSI!");
            isLCDChanged = 0;
        }
    }
}

/*  receiveSignal()
 *  Penerimaan sinyal dari keypad. BEBERAPA BAGIAN FUNGSI INI BERSIFAT TENTATIF.
 *  NOTES: Untuk sementara digunakan keypad untuk mensimulasikan stimulus sinyal
 *         dari server sebelum porting modul LoRa.
 */
void receiveSignal() {
    char keyEntered = keypad.getKey();
    if(keyEntered != NO_KEY && keyEntered == 'A') {
         updatePulsa();
    // } else if(keyEntered != NO_KEY && keyEntered == 'D') {
    //      statusPrompt();
    } else
    if(keyEntered != NO_KEY && keyEntered == 'C') {
         packet_condition = 0;
         overwriteEEPROM(NULL, 0, 0, 0, 4);
    } else
    if(keyEntered != NO_KEY && keyEntered == '#') {
        if(isTampered) {
            verifyRecoveryCode();
        } else {
            verifyCode();
        }
    }
    // TO BE DELETED AFTER DEVELOPMENT
    else if(keyEntered != NO_KEY && keyEntered == 'B') {
        ESP.restart();
    }
}

/*  receiveNFC()
 *  Verifikasi dan penambahan alokasi energi listrik residensi jika user melakukan
 *  scanning NFC tag pada modul NFC.
 */
void receiveNFC() {
    uint8_t success;
    uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
    uint8_t uidLength;
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
    if(success && !isTampered) {
        // Verifikasi dan penambahan alokasi energi jika NFC terbaca
        readNFC(uid, uidLength);
    } else if(success) {
        Serial.printf("NFC read but meter is tampered\n");
    }
}

/*  setNumLCD()
 *  Handler untuk mencetak bilangan pada LCD dan mencegah mencetak bilangan yang
 *  memiliki digit lebih besar daripada alokasi blok LCD yang tersedia.
 *  Input:
 *  - row: baris pada lCD yang hendak dicetak bilangan num
 *  - col: kolom yang mencetak digit pertama bilangan num
 *  - num: bilangan yang hendak dicetak di LCD
 */
void setNumLCD(int row, int col, int num) {
    lcd.setCursor(row, col);
    if(num < 0) lcd.print(0);
    else if(num < 10000000) lcd.print(num);
    else lcd.print("OVF    ");
}

/*  changeMode()
 *  Mengubah kondisi meteran yang diparameterisasi oleh n dan R (Mengubah
 *  ke state isOn dan R_stat yang baru)
 *
 *  [isOn, R_stat]
 *  [0,0]: Pemadaman listrik karena request pelanggan
 *  [0,1]: Pemadaman listrik karena tidak cukup pulsa energi
 *  [0,2]: Pemadaman listrik karena arus terlalu tinggi (> 20 A)
 *  [0,3]: Pemadaman listrik karena request penyedia listrik
 *  [1,0]: Penyalaan listrik dengan alasan selain request penyedia listrik
 *  [1,1]: Penyalaan listrik karena request penyedia listrik
 *
 *  Input:
 *  - row: baris pada lCD yang hendak dicetak bilangan num
 *  - r: State R_stat tujuan
 *  - n: bernilai 1 jika meteran sedang dinyalakan sebelum perubahan state
 */
void changeMode(int n, int r) {
    if(!n && isOn == 1) {
        isOn = 0;
        R_stat = r;
        vTaskDelay(100);
        overwriteEEPROM(NULL, 0, isOn, R_stat, 2);
        digitalWrite(ssr, LOW);
        isLCDChanged = 1;
        Serial.printf("Sending [n,R] = [0,%d] to server\n", r);

        opCode_send = 1;
        n_send = 2 * isTampered;
        R_send = (unsigned char) r;
        transmissionCode = 2;
        Serial.println("Initializing LoRaHandlerTx at core 1");
        xTaskCreatePinnedToCore(
        LoRaHandlerTx
        ,  "LoRa Handler Transmit"
        ,  8192
        ,  NULL
        ,  3
        ,  &xLoRaHandlerTx
        ,  1);
        if(isFromHandlerRx) vTaskSuspend(NULL);
    } else if(n == 1 && !isOn) {
        bool statement = (!r && R_stat != 3) || (r == 1);
        if(statement) {
              isOn = 1;
              R_stat = r;
              vTaskDelay(100);
              overwriteEEPROM(NULL, 0, isOn, R_stat, 2);
              (0, isOn, R_stat, 2);
              if(!isTampered) {
                  digitalWrite(ssr, HIGH);
                  startTimeReport = millis();
              }
              Serial.printf("Sending [n,R] = [1,%d] to server\n", r);

              /* Kirim kode [n,R] = [1,R] ke server */
              opCode_send = 1;
              n_send = 1 + 2 * isTampered;
              R_send = (unsigned char) r;
              transmissionCode = 2;

              Serial.println("Initializing LoRaHandlerTx at core 1");
              xTaskCreatePinnedToCore(
              LoRaHandlerTx
              ,  "LoRa Handler Transmit"
              ,  8192
              ,  NULL
              ,  3
              ,  &xLoRaHandlerTx
              ,  1);
              if(isFromHandlerRx) vTaskSuspend(NULL);
        } else {
            packet_condition--;
        }
    } else if(!n && !isOn) {
        if(r == 1 || r == 3) {
            R_stat = r;
            vTaskDelay(100);
            overwriteEEPROM(NULL, 0, isOn, R_stat, 2);
            Serial.printf("Sending [n,R] = [0,%d] to server\n", r);
            /* Kirim kode [n,R] = [0,R] ke server */
            opCode_send = 1;
            n_send =  2 * isTampered;
            R_send = (unsigned char) r;
            transmissionCode = 2;
            Serial.println("Initializing LoRaHandlerTx at core 1");
            xTaskCreatePinnedToCore(
            LoRaHandlerTx
            ,  "LoRa Handler Transmit"
            ,  8192
            ,  NULL
            ,  3
            ,  &xLoRaHandlerTx
            ,  1);
            if(isFromHandlerRx) vTaskSuspend(NULL);
        } else if(!r) {
            if(R_stat == 3) {
                Serial.printf("Sending [n,R] = [0,%d] to server\n", r);
                /* Kirim kode [n,R] = [0,R] ke server */
                opCode_send = 1;
                n_send = 2 * isTampered;
                R_send = (unsigned char) r;
                transmissionCode = 2;
                Serial.println("Initializing LoRaHandlerTx at core 1");
                xTaskCreatePinnedToCore(
                LoRaHandlerTx
                ,  "LoRa Handler Transmit"
                ,  8192
                ,  NULL
                ,  3
                ,  &xLoRaHandlerTx
                ,  1);
                if(isFromHandlerRx) vTaskSuspend(NULL);
            } else {
                R_stat = r;
                vTaskDelay(100);
                overwriteEEPROM(NULL, 0, isOn, R_stat, 2);
                Serial.printf("Sending [n,R] = [0,%d] to server\n", r);
                /* Kirim kode [n,R] = [0,R] ke server */
                opCode_send = 1;
                n_send = 2 * isTampered;
                R_send = (unsigned char) r;
                transmissionCode = 2;
                Serial.println("Initializing LoRaHandlerTx at core 1");
                xTaskCreatePinnedToCore(
                LoRaHandlerTx
                ,  "LoRa Handler Transmit"
                ,  8192
                ,  NULL
                ,  3
                ,  &xLoRaHandlerTx
                ,  1);
                if(isFromHandlerRx) vTaskSuspend(NULL);
            }
        }
    } else if(isFromHandlerRx) {
        // Jika menyala dan perintahnya tetap menyala
        Serial.printf("Sending [n,R] = [1,%d] to server\n", r);
        /* Kirim kode [n,R] = [1,R] ke server */
        opCode_send = 1;
        n_send = 1 + 2 * isTampered;
        R_send = (unsigned char) r;
        transmissionCode = 2;
        Serial.println("Initializing LoRaHandlerTx at core 1");
        xTaskCreatePinnedToCore(
        LoRaHandlerTx
        ,  "LoRa Handler Transmit"
        ,  8192
        ,  NULL
        ,  3
        ,  &xLoRaHandlerTx
        ,  1);
        vTaskSuspend(NULL);
    } else {
        packet_condition--;
    }
}

/*  updatePulsa()
 *  Handler simulasi (dgn keypad) untuk update E_all, dipanggil jika terdapat sinyal dari server untuk
 *  penambahan pulsa energi.
 *  NOTES: HANYA UNTUK DEBUGGING SAJA
 */
void updatePulsa() {
    // /* IMPLEMENTASI MENGGUNAKAN KEYPAD */
    // isKeypadOn = 1;
    // Serial.printf("Sending E_tot = %.2f and E_all = %.2f to server\n", E_tot, E_all);
    // /* Kirim E_tot dan E_all ke server */
    // /* Menerima E_all_updated dari server */
    // lcd.clear();
    // lcd.setCursor(0,0); lcd.print("Sisa energi:");
    // Serial.printf("Allocated energy : ");
    //
    // int E_all_updated = 0;
    //
    // char kpad;
    // while((kpad = keypad.getKey()) == NO_KEY) {
    //     if(interruptCounter > 0) {
    //         interruptHandler();
    //     } else {
    //         vTaskDelay(1);
    //     }
    // }
    // while(keypad.getKey() != NO_KEY) {
    //     if(interruptCounter > 0) {
    //         interruptHandler();
    //     } else {
    //         vTaskDelay(1);
    //     }
    // }
    // while(kpad != 'A') {
    //     if(!(kpad == 'B' || kpad == 'C' || kpad == 'D' || kpad == '#' || kpad == '*')) {
    //         E_all_updated = E_all_updated * 10 + kpad - 48;
    //         Serial.printf("%d", kpad - 48);
    //     }
    //
    //     setNumLCD(0,1, E_all_updated);
    //     while((kpad = keypad.getKey()) == NO_KEY) {
    //         if(interruptCounter > 0) {
    //             interruptHandler();
    //         } else {
    //             vTaskDelay(1);
    //         }
    //     }
    //     while(keypad.getKey() != NO_KEY) {
    //         if(interruptCounter > 0) {
    //             interruptHandler();
    //         } else {
    //             vTaskDelay(1);
    //         }
    //     }
    // }
    // Serial.printf("\n");
    //
    // /* PENAMBAHAN PULSA */
    // bool statement = (E_all_updated > E_all && !((!isOn && !R_stat) || (!isOn && R_stat == 3))) || (!isOn && R_stat == 2);
    // if(statement) {
    //     changeMode(1,0);
    // }
    // /* Notes: something weird happened here */
    // E_all = E_all_updated;
    // vTaskDelay(100);
    // overwriteEEPROM(NULL, E_all, 0, 1, 1);
    // E_tot = 0;
    // vTaskDelay(100);
    // overwriteEEPROM(NULL, E_tot, 0, 0, 0);
    //
    // isKeypadOn = 0;
}

/*  extractDigit()
 *  Digunakan oleh verifyCode() untuk mengambil digit terbesar suatu bilangan n
 *  Input:
 *  - n: Bilangan yang hendak ditentukan digit terbesarnya.
 */
int extractDigit(int n) {
    if(n < 0) n = -n;
    while(n >= 10) n /= 10;
    return n;
}

/*  verifyCode()
 *  Verifikasi kode pembayaran yang dimasukkan pada keypad. Penambahan pulsa sesuai
 *  dengan algoritma penentuan penambahan pulsa berdasarkan kode yang dimasukkan.
 */
void verifyCode() {
    /* INISIALISASI VARIABEL LOKAL */
    isKeypadOn = 1;
    unsigned char token[10];
    char kpad;
    int i = 0;
    int sameCount = 0;

    /* PROMPT INPUT KODE "TOKEN" */
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Token:");
    Serial.printf("Token: ");

    while((kpad = keypad.getKey()) == NO_KEY) {
        if(interruptCounter > 0) {
            interruptHandler();
        } else {
            vTaskDelay(1);
        }
    }
    while(keypad.getKey() != NO_KEY) {
        if(interruptCounter > 0) {
            interruptHandler();
        } else {
            vTaskDelay(1);
        }
    }
    while(kpad != '#') {
        if(!(kpad == 'B' || kpad == 'C' || kpad == 'D' || kpad == 'A' || kpad == '*') && i < 10) {
            token[i] = kpad - 48;
            setNumLCD(i, 1, token[i]);
            i++;
            Serial.printf("%d", kpad - 48);
        }

        while((kpad = keypad.getKey()) == NO_KEY) {
            if(interruptCounter > 0) {
                interruptHandler();
            } else {
                vTaskDelay(1);
            }
        }
        while(keypad.getKey() != NO_KEY) {
            if(interruptCounter > 0) {
                interruptHandler();
            } else {
                vTaskDelay(1);
            }
        }
    }
    Serial.printf("\n");
    while(i < 10) {
        token[i] = 0;
        i++;
    }

    for(unsigned char i = 0; i < 10; i++) {
        bool sameCheckCond = 0;
        for(unsigned char j = 0; j < numToken; j++) {
            sameCheckCond |= (token[i] == prev_token[j][i]);
        }
        if(sameCheckCond) {
            sameCount++;
        }
    }

    lcd.clear();
    lcd.setCursor(0,0);

    if(sameCount == 10) {
        Serial.printf("Token has been used!\n");
        lcd.print("GAGAL!");
    } else {
        /* VERIFIKASI TOKEN DAN PENAMBAHAN PULSA */
        int a = token[0] + token[1] + token[2];
        int b = token[3] * token[4] + token[5];
        int c = token[6] - token[7] * token[8];
        int d = b + a * token[9] - c;
        int result = 1000 * extractDigit(a) + 100 * extractDigit(b) + 10 * extractDigit(c) + extractDigit(d);

        int ref = SCRTCD;
        int refFactor = 1;
        bool isMatch = 0;
        while(refFactor < 5) {
            if(ref > 9999) ref %= 10000;
            if(ref == result) {
                lcd.print("BERHASIL");
                Serial.printf("Success!\n");
                changeMode(1,0);
                switch(refFactor) {
                    case 1  : E_all += 10000; break;
                    case 2  : E_all += 20000; break;
                    case 3  : E_all += 50000; break;
                    case 4  : E_all += 100000; break;
                }
                vTaskDelay(100);
                overwriteEEPROM(NULL, E_all, 0, 1, 1);
                isMatch = 1;

                numToken++;
                if(!prev_token) {
                    prev_token = (char **) malloc(sizeof(char *));
                } else {
                    prev_token = (char **) realloc(prev_token, numToken * sizeof(char *));
                }
                prev_token[numToken - 1] = (char *) malloc(10 * sizeof(char));
                for(unsigned char i = 0; i < 10; i++) {
                    prev_token[numToken - 1][i] = token[i];
                }
            }
            ref += SCRTCD;
            refFactor++;
        }

        if(!isMatch) {
            lcd.print("GAGAL!");
            Serial.printf("Failed!\n");
        }

    }
    unsigned long currentTime = millis();
    while((millis() - currentTime) < 3000) {
        if(interruptCounter > 0) {
            interruptHandler();
        }
    }


    isKeypadOn = 0;
}

/*  statusPrompt()
 *  Handler perubahan status meteran dari sinyal server
 *  NOTES: HANYA DIGUNAKAN UNTUK DEBUGGING (dengan keypad)
 */
void statusPrompt() {
    // /* SIMULASI SINYAL SERVER MENGGUNAKAN KEYPAD */
    // isKeypadOn = 1;
    // lcd.clear();
    // lcd.setCursor(0,0);
    // lcd.print("[n,R] = ");
    // Serial.printf("[n,R] = ");
    //
    // char statCode[2];
    // char kpad;
    // int i = 0;
    //
    // while((kpad = keypad.getKey()) == NO_KEY) {
    //     if(interruptCounter > 0) {
    //         interruptHandler();
    //     } else {
    //         vTaskDelay(1);
    //     }
    // }
    // while(keypad.getKey() != NO_KEY) {
    //     if(interruptCounter > 0) {
    //         interruptHandler();
    //     } else {
    //         vTaskDelay(1);
    //     }
    // }
    // while(kpad != 'D') {
    //     if(!(kpad == 'A' ||kpad == 'B' || kpad == 'C' || kpad == '#' || kpad == '*') && i < 2) {
    //         statCode[i] = kpad - 48;
    //         setNumLCD(8 + i, 0, statCode[i]);
    //         i++;
    //         Serial.printf("%d", kpad - 48);
    //     }
    //
    //     while((kpad = keypad.getKey()) == NO_KEY) {
    //         if(interruptCounter > 0) {
    //             interruptHandler();
    //         } else {
    //             vTaskDelay(1);
    //         }
    //     }
    //     while(keypad.getKey() != NO_KEY) {
    //         if(interruptCounter > 0) {
    //             interruptHandler();
    //         } else {
    //             vTaskDelay(1);
    //         }
    //     }
    // }
    // Serial.printf("\n");
    // while(i < 2) {
    //     statCode[i] = 5;
    //     i++;
    // }
    // lcd.setCursor(0,1);
    //
    // /* HANDLER PERUBAHAN STATUS METERAN (TENTATIF) */
    // if(!(statCode[0] == 5 || statCode[1] == 5)) {
    //     Serial.printf("Receiving [n,R] = [%d,%d] from server\n", statCode[0], statCode[1]);
    //     lcd.print("Status received!");
    //     changeMode(statCode[0], statCode[1]);
    // } else {
    //     Serial.printf("Failed to receive [n,R] code from server\n");
    //     lcd.print("Status failed!");
    // }
    //
    // unsigned long currentTime = millis();
    // while((millis() - currentTime) < 3000) {
    //     if(interruptCounter > 0) {
    //         interruptHandler();
    //     }
    // }
    // isKeypadOn = 0;
}

/*  readNFC()
 *  Pengiriman kode NFC ke server
 *  Input:
 *  - uid: UID dari pembacaan NFC
 *  - uidLength: panjang kode UID dari pembacaan NFC
 *
 *  NOTES: Untuk sementara, penambahan pulsa dilakukan di fungsi ini sebelum
 *         porting LoRa dilakukan.
 */
void readNFC(uint8_t uid[], uint8_t uidLength) {
    /* PEMBERITAHUAN BAHWA NFC TELAH TERDETEKSI */
    isKeypadOn = 1;
    Serial.printf("NFC card detected!\n");
    lcd.clear();
    lcd.setCursor(0,0);
    Serial.printf("Authenticating card ...\n");

    /* PENGIRIMAN KODE NFC KE SERVER */
    if(uidLength == 4) {
        uint8_t keya[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        uint8_t success = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keya);

        if(success) {
            Serial.printf("Card authenticated!\n");
            uint8_t data[16];
            success = nfc.mifareclassic_ReadDataBlock(4, data);
            if(success) {
                vTaskSuspend(xScanKeypad);
                lcd.print("NFC berhasil!   ");
                vTaskResume(xScanKeypad);

                Serial.printf("Sending NFC code ");
                for(uint8_t i = 0; i < 16; i++) {
                    if(data[i] <= 0xF) {
                        Serial.printf("0");
                    }
                    Serial.print(data[i], HEX);
                    if(i != 15) {
                        Serial.printf(" ");
                    }
                }
                Serial.printf(" to server.\n");

                opCode_send = 2;
                for(unsigned char i = 0; i < 16; i++) {
                    byteStream_send[i] = (char) data[i];
                }
                transmissionCode = 2;
                Serial.print("Initializing LoRaHandlerTx at core "); Serial.println(xPortGetCoreID());
                xTaskCreatePinnedToCore(
                LoRaHandlerTx
                ,  "LoRa Handler Transmit"
                ,  8192
                ,  NULL
                ,  3
                ,  &xLoRaHandlerTx
                ,  xPortGetCoreID());
            } else {
                Serial.printf("Cannot read NFC data contents.\n");
                lcd.print("NFC gagal dibaca");
            }
        } else {
            Serial.printf("Authentication failed.\n");
            lcd.print("NFC gagal dibaca");
        }
    } else {
        Serial.printf("Authentication failed.\n");
        lcd.print("NFC gagal dibaca");
    }

    vTaskDelay(3000);
    isKeypadOn = 0;
}

/*  overwriteEEPROM()
 *  Overwrite nilai suatu parameter di EEPROM jika nilai tersebut berbeda dengan
 *  nilai yang tersimpan di EEPROM.
 *  INPUT:
 *  - num: floating point (E_tot atau E_all) yang hendak disimpan di EEPROM
 *  - isOn_write: isOn yang hendak disimpan di EEPROM
 *  - R_stat_write: R_stat yang hendak disimpan di EEPROM
 *  - mode: Terdapat 4 mode.
 *          0: E_tot hendak disimpan, isOn dan R_stat bersifat don't care
 *          1: E_all hendak disimpan, isOn dan R_stat bersifat don't care
 *          2: isOn dan R_stat hendak disimpan, num bersifat don't care
 *          3: key_write hendak disimpan. num, usOn, R_stat bersifat don't care.
 *          4: data terkait pelaporan energi dan perubahan status disimpan.
 */
void overwriteEEPROM(char *key_write, float num, unsigned char isOn_write, unsigned char R_stat_write, int mode) {
    byte b;
    byte key_buffer[16];
    uint16_t packet_time_buffer;
    uint16_t packet_condition_buffer;
    unsigned char packet_opcode_buffer;
    switch (mode) {
        case 0:
                    for(unsigned int i = 0; i < 4; i++) {
                        convert.buffer[i] = i2c_eeprom_read_byte(0x57, i + 1);
                    }
                    // Representasi biner kedua bilangan tersebut harus beda agar diubah
                    if(num != convert.numberFloat) {
                        convert.numberFloat = num;
                        i2c_eeprom_write_page(0x57, 1, (byte *) convert.buffer, sizeof(convert.buffer));
                    } else {
                        Serial.printf("EEPROM data is the same!\n");
                    }
                    break;
        case 1:
                    for(unsigned int i = 0; i < 4; i++) {
                        convert.buffer[i] = i2c_eeprom_read_byte(0x57, i + 5);
                    }
                    // Representasi biner kedua bilangan tersebut harus beda agar diubah
                    if(num != convert.numberFloat) {
                        convert.numberFloat = num;
                        i2c_eeprom_write_page(0x57, 5, (byte *) convert.buffer, sizeof(convert.buffer));
                    } else {
                        Serial.printf("EEPROM data is the same!\n");
                    }
                    break;
        case 2: {
                    b = i2c_eeprom_read_byte(0x57, 0);
                    byte writeByte = 0x00;
                    writeByte |= isTampered;
                    writeByte |= (writeByte << 1) | isOn_write;
                    writeByte = (writeByte << 2) | R_stat_write;
                    if(writeByte != b) {
                        i2c_eeprom_write_byte(0x57, 0, writeByte);
                    } else {
                        Serial.printf("EEPROM data is the same!\n");
                    }
                    break;
                }
        case 3: {
                    for(unsigned int i = 0; i < 16; i++) {
                        key_buffer[i] = i2c_eeprom_read_byte(0x57, i + 9);
                    }
                    int sameCount = 0;
                    for(unsigned char i = 0; i < 16; i++) {
                        if((char) key_buffer[i] == key_write[i]) {
                            sameCount++;
                        }
                    }
                    if(sameCount < 16) {
                        i2c_eeprom_write_page(0x57, 9, (byte *) key_write, sizeof(byteStream_rec));
                    } else {
                        Serial.printf("EEPROM data is the same!\n");
                    }
                    break;
                }
        case 4:
                    for(unsigned int i = 0; i < 2; i++) {
                        convert2.buffer[i] = i2c_eeprom_read_byte(0x57, i + 25);
                    }
                    packet_time_buffer = convert2.number & 4095;
                    packet_condition_buffer = (convert2.number >> 12) & 3;
                    packet_opcode_buffer = (unsigned char) ((convert2.number >> 14) & 3);
                    bool sameCondition = packet_time != packet_time_buffer || packet_condition != packet_condition_buffer
                                        || packet_opcode != packet_opcode_buffer;
                    if(sameCondition) {
                        convert2.number = packet_time;
                        convert2.number |= (packet_condition << 12);
                        convert2.number |= ((uint16_t) packet_opcode << 14);
                        i2c_eeprom_write_page(0x57, 25, (byte *) convert2.buffer, sizeof(convert2.buffer));
                        vTaskDelay(200);
                    } else {
                        Serial.printf("EEPROM data is the same!\n");
                    }
                    for(unsigned int i = 0; i < 4; i++) {
                        convert.buffer[i] = i2c_eeprom_read_byte(0x57, i + 27);
                    }
                    // Representasi biner kedua bilangan tersebut harus beda agar diubah
                    if(packet_energy != convert.numberFloat) {
                        convert.numberFloat = packet_energy;
                        i2c_eeprom_write_page(0x57, 27, (byte *) convert.buffer, sizeof(convert.buffer));
                    } else {
                        Serial.printf("EEPROM data is the same!\n");
                    }
                    break;
    }

    vTaskDelay(200);
}

/*  i2c_eeprom_read_byte()
 *  Pembacaan suatu byte pada EEPROM di address tertentu
 *  INPUT:
 *  - devicaddress: I2C address device (0x57)
 *  - eeaddress: address di EEPROM yang hendak dibaca
 *
 *  OUTPUT: nilai byte yang terbaca
 *
 *  REFERENSI: https://lastminuteengineers.com/ds3231-rtc-arduino-tutorial/
 */
byte i2c_eeprom_read_byte(int deviceaddress, unsigned int eeaddress) {
    byte rdata = 0xFF;
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(deviceaddress,1);
    if (Wire.available()) rdata = Wire.read();
    return rdata;
}

/*  i2c_eeprom_write_byte()
 *  Penyimpanan suatu data (satu byte) pada EEPROM di address tertentu
 *  INPUT:
 *  - devicaddress: I2C address device (0x57)
 *  - eeaddress: address di EEPROM yang hendak dibaca
 *  - data: byte data yang hendak disimpan
 *
 *  REFERENSI: https://lastminuteengineers.com/ds3231-rtc-arduino-tutorial/
 */
void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data ) {
    int rdata = data;
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.write(rdata);
    Wire.endTransmission();
}

/*  i2c_eeprom_write_page()
 *  Penyimpanan suatu byte stream pada EEPROM di address tertentu
 *  INPUT:
 *  - devicaddress: I2C address device (0x57)
 *  - eeaddress: address di EEPROM yang hendak dibaca
 *  - data: stream data yang hendak disimpan
 *  - length: panjang byte data (dalam byte)
 *
 *  REFERENSI: https://lastminuteengineers.com/ds3231-rtc-arduino-tutorial/
 *  WARNING: address is a page address, 6-bit end will wrap around
 *           also, data can be maximum of about 30 bytes, because the Wire library has a
 *           buffer of 32 bytes
 */
void i2c_eeprom_write_page(int deviceaddress, unsigned int eeaddresspage, byte* data, byte length) {
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddresspage >> 8)); // MSB
    Wire.write((int)(eeaddresspage & 0xFF)); // LSB
    byte c;
    for ( c = 0; c < length; c++)
        Wire.write(data[c]);
    Wire.endTransmission();
}

/*  tampered()
 *  Perubahan kondisi meteran ketika tampered atau recovery code sudah dimasukkan
 *  INPUT:
 *  - n : bernilai 1 jika tampered, bernilai 0 jika recovery code berhasil dimasukkan
 */
 void tampered(int n) {
    if(n) {

        isTampered = 1;
        isLCDChanged = 1;
        overwriteEEPROM(NULL, 0, isOn, R_stat, 2);
        digitalWrite(ssr, LOW);
        Serial.printf("Meter tampered!\n");

        Serial.printf("Sending tampering status code to server\n");
        packet_opcode = 2;
        packet_condition = 1;
        vTaskDelay(100);
        overwriteEEPROM(NULL, 0, 0, 0, 4);
        opCode_send = 1;
        n_send = (unsigned char) (isOn + 2 * isTampered);
        R_send = (unsigned char) R_stat;
        transmissionCode = 2;

        Serial.print("Initializing LoRaHandlerTx at core 1\n");
        xTaskCreatePinnedToCore(
        LoRaHandlerTx
        ,  "LoRa Handler Transmit"
        ,  8192
        ,  NULL
        ,  3
        ,  &xLoRaHandlerTx
        ,  1);
    } else {
        isTampered = 0;
        overwriteEEPROM(NULL, 0, isOn, R_stat, 2);
        Serial.printf("Meter recovered!\n");

        Serial.printf("Sending restoration status code to server\n");
        packet_opcode = 2;
        packet_condition = 1;
        vTaskDelay(100);
        overwriteEEPROM(NULL, 0, 0, 0, 4);
        opCode_send = 1;
        n_send = (unsigned char) (isOn + 2 * isTampered);
        R_send = (unsigned char) R_stat;
        transmissionCode = 2;

        Serial.print("Initializing LoRaHandlerTx at core 1\n");
        xTaskCreatePinnedToCore(
        LoRaHandlerTx
        ,  "LoRa Handler Transmit"
        ,  8192
        ,  NULL
        ,  3
        ,  &xLoRaHandlerTx
        ,  1);

        int state = digitalRead(magnetSensor);
        if(isOn && !state) {
            digitalWrite(ssr, HIGH);
            startTimeReport = millis();
        }
    }
}

/*  verifyRecoveryCode()
 *  Verifikasi recovery code yang dimasukkan pada keypad dengan recovery code
 *  yang benar. Tampering mechanism akan dimatikan jika recovery code yang
 *  dimasukkan benar.
 */
void verifyRecoveryCode() {
    char inputCode[9];
    uint32_t calcCode = 0;
    char kpad;
    int i = 0;

    isKeypadOn = 1;

    /* Prompt input kode */
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Kode Pemulihan:");
    Serial.printf("Recovery code: ");

    while((kpad = keypad.getKey()) == NO_KEY) {
        if(interruptCounter > 0) {
            interruptHandler();
        } else {
            vTaskDelay(1);
        }
    }
    while(keypad.getKey() != NO_KEY) {
        if(interruptCounter > 0) {
            interruptHandler();
        } else {
            vTaskDelay(1);
        }
    }
    while(kpad != '#') {
        if(!(kpad == 'B' || kpad == 'C' || kpad == 'D' || kpad == 'A' || kpad == '*') && i < 9) {
            inputCode[i] = kpad - 48;
            setNumLCD(i, 1, inputCode[i]);
            i++;
            Serial.printf("%d", kpad - 48);
        }

        while((kpad = keypad.getKey()) == NO_KEY) {
            if(interruptCounter > 0) {
                interruptHandler();
            } else {
                vTaskDelay(1);
            }
        }
        while(keypad.getKey() != NO_KEY) {
            if(interruptCounter > 0) {
                interruptHandler();
            } else {
                vTaskDelay(1);
            }
        }
    }
    Serial.printf("\n");
    while(i < 9) {
        inputCode[i] = 0;
        i++;
    }

    /* Verifikasi Recovery Code */
    for(unsigned char j = 0; j < 9; j++) {
        calcCode = 10 * calcCode + inputCode[j];
    }

    uint32_t ref = SCRTRC;
    if(calcCode == ref) {
        tampered(0);
    }

    isKeypadOn = 0;
}

/*  onReceive()
 *  Handler ketika meteran menerima paket data dari server.
 */
void onReceive(int packetSize) {
    // received a packet
    // Serial.printf("\n==============================================\n");
    Serial.print("\nReceived packet : ");
    rcvd_data = (char*)malloc(packetSize * sizeof(char));

    // read packet
    for (int i = 0; i < packetSize; i++) {
      rcvd_data[i]= (char)LoRa.read();
      Serial.print(rcvd_data[i]);
    }
    Serial.print("\n\r");

    char *iv  = "AAAAAAAAAAAAAAAAAAAAAA==";
    decrypt(rcvd_data, String(iv), 108);
    free(rcvd_data);
}

/*  decrypt()
 *  Dekripsi payload ketika diterima dari server.
 */
void decrypt(String b64data_rcvd, String IV_base64, int lsize) {
    char data_decoded[200];
    char iv_decoded[200];
    byte out[200];
    char temp[200];

    int sameCount = 0;

    b64data_rcvd.toCharArray(temp, 200);
    base64_decode(data_decoded, temp, b64data_rcvd.length());
    IV_base64.toCharArray(temp, 200);
    base64_decode(iv_decoded, temp, IV_base64.length());
    base64_encode( b64data, (char *)iv_decoded, N_BLOCK);
    // Serial.println("\nIV b64: " + String(b64data));

    aes.do_aes_decrypt((byte *)data_decoded, lsize, out, key, 128, (byte *)iv_decoded);

    char message[100];

    base64_decode(message, (char *)out, 80);

    // Serial.print("Out: "+ String(message));

    char b64payload[80];
    char HMAC[7]="\0";
    char rcvd_HMAC[7]="\0";

    for (int i=6;i<80;i++) {
        b64payload[i-6]=message[i];
    }

    for (int i=0;i<6;i++) {
        HMAC[i]=message[i];
    }

    // Serial.print("\nPayload: "+ String(b64payload));
    // Serial.print("\nHMAC: "+String(HMAC));
    getHMAC(b64payload);

    for(int i= 0; i< 3; i++) {
        char temp[3];
        sprintf(temp, "%02x", (int)hmacResult[i]);
        strcat(rcvd_HMAC,temp);
    }

    if(strcmp(rcvd_HMAC, HMAC) == 0) {
        base64_decode(payload, b64payload, sizeof(b64payload));

        for(unsigned char i = 0; i < sizeof(payload); i++) {
            bool sameCheckCond = 0;
            for(unsigned char j = 0; j < 5; j++) {
                sameCheckCond |= (rcvd_data[i] == prev_rcvd_data[j][i]);
            }
            if(sameCheckCond) {
                sameCount++;
            }
        }

        if(sameCount == sizeof(payload)) {
            Serial.printf("Data packet has already been processed. Decryption and parsing terminated.\n");
        } else {
            for(unsigned char i = 0; i < sizeof(payload); i++) {
                prev_rcvd_data[rcvd_buffer_mode][i] = rcvd_data[i];
            }
            rcvd_buffer_mode = (rcvd_buffer_mode + 1) % 5;

            Serial.printf("\nRECEIVED PAYLOAD:\n"); printHexChar((byte *) payload, sizeof(payload));
            parsePayload();
        }
    } else {
        Serial.print(strcmp(rcvd_HMAC,HMAC));
        Serial.print("\n");
        Serial.print(String(rcvd_HMAC));
        Serial.print("\n");
        Serial.print(String(HMAC));
        Serial.print("\n");
    }
}

/*  getHMAc()
 *  Perhitungan HMAC dari payload.
 */
int getHMAC(char *payload){

  char *key = "secretKey";

  mbedtls_md_context_t ctx;
  mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;

  const size_t payloadLength = strlen(payload);
  const size_t keyLength = strlen(key);

  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
  mbedtls_md_hmac_starts(&ctx, (const unsigned char *) key, keyLength);
  mbedtls_md_hmac_update(&ctx, (const unsigned char *) payload, payloadLength);
  mbedtls_md_hmac_finish(&ctx, hmacResult);
  mbedtls_md_free(&ctx);

  // Serial.print("\nHash: ");

  for(int i= 0; i< sizeof(hmacResult); i++){
      char str[3];

      sprintf(str, "%02x", (int)hmacResult[i]);
      // Serial.print(str);
  }
}

/*  parsePayload()
 *  Parsing data pada payload untuk mengubah parameter meteran.
 */
void parsePayload() {
    // Objective: to parse the payload that has been generated with generatePayload()

    // Translating the parsed code
    // Serial.printf("----------------------------------------------\n");
    // Serial.printf("PARSING PAYLOAD\n");

    // Obtain the components of the payload
    // Serial.printf("Number of bytes on payload = %u\n", sizeof(payload));
    if(sizeof(payload) > 36) {
        unsigned char first_rec = payload[0];

        R_rec = first_rec & 0x3;
        n_rec = (first_rec >> 2) & 0x3;
        opCode_rec = first_rec >> 4;

        union buffer4Byte meterID_rec;
        for(unsigned char i = 0; i < 4; i++) {
            meterID_rec.buffer[i] = (byte) payload[1 + i];
        }

        timeRec.day = (uint8_t) payload[5];
        timeRec.month = (uint8_t) payload[6];
        timeRec.year.buffer[0] = (byte) payload[7];
        timeRec.year.buffer[1] = (byte) payload[8];
        timeRec.hour = (uint8_t) payload[9];
        timeRec.minute = (uint8_t) payload[10];
        timeRec.second = (uint8_t) payload[11];

        for(unsigned char i = 0; i < 4; i++) {
            E_tot_rec.buffer[i] = (byte) payload[12 + i];
        }
        for(unsigned char i = 0; i < 4; i++) {
            E_all_rec.buffer[i] = (byte) payload[16 + i];
        }

        for(unsigned char i = 0; i < 16; i++) {
            byteStream_rec[i] = payload[20 + i];
        }

        // Printing payload components
        // Serial.printf("Opcode\t\t\t= "); printHexChar(&opCode_rec, sizeof(opCode_rec));
        // Serial.printf("n\t\t\t= %u - ", n_rec); printHexChar(&n_rec, sizeof(n_rec));
        // Serial.printf("R\t\t\t= %u - ", R_rec); printHexChar(&R_rec, sizeof(R_rec));
        // Serial.printf("Byte-0\t\t\t= "); printHexChar(&first_rec, sizeof(first_rec));
        // Serial.printf("Meter ID\t\t= %lu - ", meterID_rec.numberLong); printHexChar(meterID_rec.buffer, sizeof(meterID_rec.buffer));
        // Serial.printf("Day\t\t\t= %u - ", timeRec.day); printHexChar(&(timeRec.day), sizeof(timeRec.day));
        // Serial.printf("Month\t\t\t= %u - ", timeRec.month); printHexChar(&(timeRec.month), sizeof(timeRec.month));
        // Serial.printf("Year\t\t\t= %u - ", timeRec.year.number); printHexChar(timeRec.year.buffer, sizeof(timeRec.year.buffer));
        // Serial.printf("Hour\t\t\t= %u - ", timeRec.hour); printHexChar(&(timeRec.hour), sizeof(timeRec.hour));
        // Serial.printf("Minute\t\t\t= %u - ", timeRec.minute); printHexChar(&(timeRec.minute), sizeof(timeRec.minute));
        // Serial.printf("Second\t\t\t= %u - ", timeRec.second); printHexChar(&(timeRec.second), sizeof(timeRec.second));
        // Serial.printf("Total Energy\t\t= %f - ", E_tot_rec.numberFloat); printHexChar(E_tot_rec.buffer, sizeof(E_tot_rec.buffer));
        // Serial.printf("Allocated Energy\t= %f - ", E_all_rec.numberFloat); printHexChar(E_all_rec.buffer, sizeof(E_all_rec.buffer));
        // Serial.printf("NFC Code\t\t= "); printHexChar((byte *) byteStream_rec, sizeof(byteStream_rec));

        // Translating the parsed code
        // Serial.printf("----------------------------------------------\n");
        // Serial.printf("TRANSLATING PAYLOAD\n");

        if(meterID_rec.numberLong == SCRTCD) {
            switch(opCode_rec) {
                case 0x00 :
                            Serial.printf("Code: NULL - no state parameters will be changed\n");
                            break;
                case 0x01 :
                case 0x02 :
                case 0x03 :
                            Serial.printf("Invalid opcode, should be used for endpoint to server transmission!\n");
                            break;
                case 0x04 :
                            Serial.printf("Code: ACKNOWLEDGEMENT\n");
                            Serial.printf("Received parameters:\n");
                            Serial.printf("Opcode\t\t\t= "); printHexChar(&opCode_rec, sizeof(opCode_rec));
                            Serial.printf("Meter ID\t\t= %lu\n", meterID_rec.numberLong);
                            Serial.printf("Time\t\t\t= ");
                            Serial.printf("{%02u:%02u:%02u, ", timeRec.hour, timeRec.minute, timeRec.second);
                            Serial.printf("%02u-%02u-%04u}\n", timeRec.day, timeRec.month, timeRec.year.number);
                            Serial.print("Initializing LoRaHandlerRx at core "); Serial.println(xPortGetCoreID());
                            // Pembuatan task baru
                            xTaskCreatePinnedToCore(
                            LoRaHandlerRx
                            ,  "LoRa Handler Receive"
                            ,  8192
                            ,  NULL
                            ,  2
                            ,  &xLoRaHandlerRx
                            ,  xPortGetCoreID());
                            break;
                case 0x05 :
                            Serial.printf("Code: PERUBAHAN STATUS METERAN (SERVER->METER)\n");
                            Serial.printf("Received parameters:\n");
                            Serial.printf("Opcode\t\t\t= "); printHexChar(&opCode_rec, sizeof(opCode_rec));
                            Serial.printf("n\t\t\t= %u\n", n_rec);
                            Serial.printf("R\t\t\t= %u\n", R_rec);
                            Serial.printf("Meter ID\t\t= %lu\n", meterID_rec.numberLong);
                            Serial.printf("Time\t\t\t= ");
                            Serial.printf("{%02u:%02u:%02u, ", timeRec.hour, timeRec.minute, timeRec.second);
                            Serial.printf("%02u-%02u-%04u}\n", timeRec.day, timeRec.month, timeRec.year.number);
                            Serial.print("Initializing LoRaHandlerRx at core "); Serial.println(xPortGetCoreID());
                            // Pembuatan task baru
                            xTaskCreatePinnedToCore(
                            LoRaHandlerRx
                            ,  "LoRa Handler Receive"
                            ,  8192
                            ,  NULL
                            ,  2
                            ,  &xLoRaHandlerRx
                            ,  xPortGetCoreID());
                            break;
                case 0x06 :
                            Serial.printf("Code: PRIVATE KEY CHANGE (SERVER->METER)\n");
                            Serial.printf("Received parameters:\n");
                            Serial.printf("Opcode\t\t\t= "); printHexChar(&opCode_rec, sizeof(opCode_rec));
                            Serial.printf("Meter ID\t\t= %lu\n", meterID_rec.numberLong);
                            Serial.printf("Time\t\t\t= ");
                            Serial.printf("{%02u:%02u:%02u, ", timeRec.hour, timeRec.minute, timeRec.second);
                            Serial.printf("%02u-%02u-%04u}\n", timeRec.day, timeRec.month, timeRec.year.number);
                            Serial.printf("New Key\t\t= "); printHexChar((byte *) byteStream_rec, sizeof(byteStream_rec));
                            Serial.print("Initializing LoRaHandlerRx at core "); Serial.println(xPortGetCoreID());
                            // Pembuatan task baru
                            xTaskCreatePinnedToCore(
                            LoRaHandlerRx
                            ,  "LoRa Handler Receive"
                            ,  8192
                            ,  NULL
                            ,  2
                            ,  &xLoRaHandlerRx
                            ,  xPortGetCoreID());
                            break;
                case 0x07 :
                            Serial.printf("Invalid opcode!\n");
                            break;
                case 0x08 :
                            Serial.printf("Code: REPORT TRIGGER AND UPDATE ALLOCATION (SERVER->METER)\n");
                            Serial.printf("Received parameters:\n");
                            Serial.printf("Opcode\t\t\t= "); printHexChar(&opCode_rec, sizeof(opCode_rec));
                            Serial.printf("Meter ID\t\t= %lu\n", meterID_rec.numberLong);
                            Serial.printf("Time\t\t\t= ");
                            Serial.printf("{%02u:%02u:%02u, ", timeRec.hour, timeRec.minute, timeRec.second);
                            Serial.printf("%02u-%02u-%04u}\n", timeRec.day, timeRec.month, timeRec.year.number);
                            Serial.printf("Added energy\t\t= %f\n", E_all_rec.numberFloat);
                            Serial.print("Initializing LoRaHandlerRx at core "); Serial.println(xPortGetCoreID());
                            // Pembuatan task baru
                            xTaskCreatePinnedToCore(
                            LoRaHandlerRx
                            ,  "LoRa Handler Receive"
                            ,  8192
                            ,  NULL
                            ,  2
                            ,  &xLoRaHandlerRx
                            ,  xPortGetCoreID());
                            break;
                default :
                            Serial.printf("Code: INVALID OPCODE\n");
            }
        } else {
            Serial.printf("Payload not intended for this endpoint.\n");
        }


    } else {
        // Things went wrong when the received payload is less than 37 bytes
        Serial.printf("Payload corrupted!\n");
    }
}

/*  printHexChar()
 *  Print hexadecimal dengan format yang readable.
 */
void printHexChar(const byte *data, const uint32_t numBytes) {
    uint32_t szPos;
    Serial.printf("0x");
    for (szPos = 0; szPos < numBytes; szPos++) {
        // Append leading 0 for small values
        if (data[szPos] <= 0xF)
            Serial.printf("0");
            Serial.printf("%X", data[szPos]);
        if ((numBytes > 1) && (szPos != numBytes - 1)) {
            Serial.printf(" ");
        }
    }
    Serial.printf("\n");
}

/* Random generator */
uint8_t getrnd() {
    uint8_t really_random = *(volatile uint8_t *)0x3FF20E44;
    return really_random;
}

// Generate a random initialization vector
void gen_iv(byte  *iv) {
    for (int i = 0 ; i < N_BLOCK ; i++ ) {
        iv[i]= (byte) getrnd();
    }
}

/* Setup AES */
void setup_aes() {
  aes.set_key(key, sizeof(key)); // Get the globally defined key
}

/* Encrypt data dan aplikasi HMAC */
void envelop_data(){
  char data[128] = "\0";
  char with_hmac[128] = "\0";
  for(int i = 0; i < 50; i++)
  {
     data[i] = node_id[i];
  }
  int b64payloadlen = base64_encode(b64payload, payload, sizeof(payload));
  Serial.print("\nb64payload: "+String(b64payload));
  getHMAC(b64payload);
  for(int i= 0; i< 3; i++){
      char temp[3];
      sprintf(temp, "%02x", (int)hmacResult[i]);
      strcat(with_hmac,temp);
  }
  strcat(with_hmac,b64payload);
  encrypt_payload(with_hmac,b64payloadlen+6);
  strcat(data,b64data);

  strcpy((char *)datasend,data);
}

/* Enkripsi data */
void encrypt_payload(char *msg, int b64payloadlen){
    byte cipher[1000];
    byte iv [N_BLOCK] ;
    char decoded[37];
    Serial.println("Let's encrypt:");

    // Our message to encrypt. Static for this example.
    //String msg = "{\"data\":{\"value\":300}, \"SEQN\":700 , \"msg\":\"IT WORKS!!\" }";
//    String msg = String(payload);
    aes.set_key( key , sizeof(key));  // Get the globally defined key
    gen_iv( my_iv );                  // Generate a random IV

    // Print the IV
    base64_encode( b64data, (char *)my_iv, N_BLOCK);
    Serial.println(" IV b64: " + String(b64data));

//    Serial.println(" Message: " + msg );

    int b64len = base64_encode(b64data, (char *)msg, b64payloadlen);
    Serial.println (" Message in B64: " + String(b64data) );
    Serial.println (" The length is:  " + String(b64len) );

    // For sanity check purpose
//    base64_decode( decoded , b64data , b64len );
//    Serial.printf("Decoded:\n"); printHexChar((byte *) decoded, 37);

    // Encrypt! With AES128, our key and IV, CBC and pkcs7 padding
    aes.do_aes_encrypt((byte *)b64data, b64len , cipher, key, 128, my_iv);

    Serial.println("Encryption done!");

    Serial.println("Cipher size: " + String(aes.get_size()));

    b64len = base64_encode(b64data, (char *)cipher, aes.get_size() );
    Serial.println ("Encrypted data in base64: " + String(b64data) );
    Serial.println ("The length is:  " + String(b64len) );

    Serial.println("Done...");
}

/*  sendData()
 *  Kirim data ke MQTT
 *  INPUT:
 *  - datasend_buffer: buffer datasend yang akan dikirim jika condition = 1
 *  - condition: Terdapat 2 kondisi.
 *          0: datasend yang dikirim adalah datasend dari payload
 *          1: datasend yang dikirim adalah datasend dari datasend_buffer
 *             (di task LoRaHandlerTx)
 */
void sendData(uint8_t *datasend_buffer, bool condition)
{
    LoRa.beginPacket();
    if(condition) {
        LoRa.print((char *) datasend_buffer);
    } else {
        LoRa.print((char *) datasend);
    }
    LoRa.endPacket();
    Serial.println("Packet Sent");
}
