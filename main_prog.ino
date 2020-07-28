/*  EL4092 TUGAS AKHIR II - TA1920.01.011
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
#include "RTClib.h"
#include "AES.h"
#include "base64.h"
#include <LoRa.h>
#include "mbedtls/md.h"

/* PENDEFINISIAN PENGGUNAAN CORE UNTUK RTOS */
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

/* PENDEFINISIAN PIN UNTUK KOMUNIKASI SPI */
#define PN532_SCK   (27)
#define PN532_MISO  (26)
#define PN532_MOSI  (25)
#define PN532_SS    (5)

/* SECRET CODE METERAN (JANGAN DIUBAH, SANGAT SENSITIF) */
#define SCRTCD 3159
#define SCRTRC 428954425

/* VARIABEL GLOBAL METERAN */
float E_tot;                //  Penggunaan energi residensi terukur
float E_all;                //  Alokasi energi untuk residensi
unsigned char isTampered = 0;   //  Bernilai 1 jika meteran ter-tampered (magnetic switch terbuka)
unsigned char isOn;         //  Penjelasan terdapat di fungsi changeMode()
unsigned char R_stat;       //  Penjelasan terdapat di fungsi changeMode()
bool isNearOver = 0;        //  Bernilai 1 jika 16 A < I < 20 A (arus residensi)
bool isKeypadOn = 0;        //  Bernilai 1 jika terdapat masukan keypad dari user
int counter = 0;            //  Counter untuk refresh LCD setiap 1 menit (upaya pencegahan data corruption)
char *rcvd_data;            //  String received data dari MQTT
byte opCode;                //  OPCODE yang akan dikirim ke server
unsigned char n;            //  Kode n yang akan dikirim ke server
unsigned char R;            //  Kode R yang akan dikirim ke server
unsigned char first;        //  Byte-0 dalam payload
char byteStreamCode[16];    //  Byte stream 16 byte untuk slot NFC pada payload
char payload[37];           //  String payload yang akan dikirim/telah diterima (setelah enkripsi/dekrispi)

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

/* INISIALISASI NAMA HARI UNTUK RTC */
char daysOfTheWeek[7][12] = {"Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"};

/* DEKLARASI STRUKTUR DATA PENGOLAHAN DATA DI EEPROM DAN PEMBUATAN PAYLOAD */
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

/* DEKLARASI KELAS UNTUK BERKOMUNIKASI DENGAN BERBAGAI MODUL */
Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);
Keypad_I2C keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS, 0x20);
PZEM004Tv30 pzem(&Serial2);
LiquidCrystal_I2C lcd(0x3F, 16,2);
RTC_DS3231 rtc;
AES aes;

/* FUNCTION PROTOTYPES */
void setNumLCD(int row, int col, int num);
void changeMode(int n, int r);
void updatePulsa();
int extractDigit(int n);
void verifyCode();
void statusPrompt();
void readNFC(uint8_t uid[], uint8_t uidLength);
void readTime();
void overwriteEEPROM(float num, unsigned char cond, int mode);
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

/* SETUP AWAL */
void setup() {

    delay(3000);
    /* Inisialisasi Serial Communication */
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, 16, 17);
    while(!Serial);
    Serial.printf("Serial Communication initialization succeeded!\n");

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
    Serial.println("LoRa initialization succeeded!");

    /* Inisialisasi kondisi LCD */
    lcd.begin(21, 22);
    lcd.backlight();
    Serial.printf("LCD initialization succeeded!\n");

    /* Inisialisasi konfigurasi keypad */
    Wire.begin();   // just to make sure :)
    keypad.begin();
    keypad.setDebounceTime(50);
    Serial.printf("Keypad initialization succeeded!\n");

    /* Inisialisasi komunikasi dengan NFC */
    nfc.begin();
    uint32_t versiondata = nfc.getFirmwareVersion();
    if(!versiondata) {
        Serial.printf("Cannot find PN53X board for NFC scanning!\n");
        while(1);
    }
    nfc.SAMConfig();
    Serial.printf("NFC scanner initialization succeeded!\n");

    /* Inisialisasi variabel global berdasarkan nilai pada EEPROM RTC */
    Serial.printf("Fetching starting parameters from EEPROM\n");
    byte b = i2c_eeprom_read_byte(0x57, 0);
    // isTampered = (unsigned char) (b & 8) >> 3;
    isOn = (unsigned char) (b & 4) >> 2;
    R_stat = (unsigned char) b & 3;

    for(unsigned int i = 0; i < 4; i++) {
        convert.buffer[i] = i2c_eeprom_read_byte(0x57, i + 1);
    }
    E_tot = convert.numberFloat;

    for(unsigned int i = 0; i < 4; i++) {
        convert.buffer[i] = i2c_eeprom_read_byte(0x57, i + 5);
    }
    E_all = convert.numberFloat;
    Serial.printf("Starting parameters are already set!\n");

    /* Inisialisasi Magnetic Switch MC38 untuk Anti-Tampering */
    pinMode(magnetSensor, INPUT);
    Serial.printf("Magnetic Sensor initialization succeeded!\n");

    /* Inisialisasi Relay Tegangan Tinggi (SSR-40DA) */
    pinMode(ssr, OUTPUT);
    int state = digitalRead(magnetSensor);
    if(isOn && !state && !isTampered) {
        digitalWrite(ssr, HIGH);
    } else {
        digitalWrite(ssr, LOW);
    }
    Serial.printf("Solid State Relay initialization succeeded!\n");

    /* Inisialisasi RTC */
    if (! rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (1);
    }
    if (rtc.lostPower()) {
        Serial.println("RTC lost power, lets set the time!");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    Serial.printf("RTC initialization succeeded!\n");

    /*  DEFINISI TASK RTOS (Dokumentasi masing-masing task terdapat di Dokumentasi
        implementasi task) */
    xTaskCreatePinnedToCore(
    MainFunction
    ,  "Main Function"
    ,  8192
    ,  NULL
    ,  3
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
    ReportEnergy
    ,  "Report Energy"
    ,  8192
    ,  NULL
    ,  2
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
    ReceiveSignal
    ,  "Receive Signal"
    ,  8192
    ,  NULL
    ,  3
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
    ReceiveNFC
    ,  "Receive NFC"
    ,  8192
    ,  NULL
    ,  3
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

    Serial.printf("RTOS initialization succeeded!\n");
    Serial.printf("Meter is set!\n\n");

    /* Attempt untuk receive paket data dari server */
    LoRa.onReceive(onReceive);
    LoRa.receive();
}

/* SEMUA IMPLEMENTASI DILAKUKAN DI TASK RTOS */
void loop() {}

/*  TASK: MainFunction()
 *  Mengecek kondisi meteran (kode isOn dan energi yang digunakan).
 *  Meteran akan dimatikan jika dimatikan secara sengaja atau jika energi yang
 *  digunakan lebih besar dari alokasi energi residensi tersebut.
 */
void MainFunction(void *pvParameters)
{
    (void) pvParameters;

    for (;;)
    {
        int state = digitalRead(magnetSensor);
        if(!isTampered && !state) {
            /* DN: something weird happened here */
            if(isOn && E_tot > E_all) {
                changeMode(0,1);
                E_all = 0;
                overwriteEEPROM(E_all, 1, 0);
                E_tot = 0;
                overwriteEEPROM(E_tot, 0, 0);

            } else {
                vTaskDelay(400);
            }
        } else {
            if(!isTampered) {
                tampered(1);
            }
        }
    }
}

/*  TASK: ReportEnergy()
 *  1. Pengukuran daya dan arus serta perhitungan energi berdasarkan hasil pengukuran.
 *     - Jika arus melebihi 20 A, meteran dimatikan demi keselamatan kerja.
 *     - Jika arus antara 16-20 A, meteran mengirimkan peringatan ke user mengenai batas arus.
 *  2. Update penggunaan energi pada console (untuk troubleshooting) dan LCD
 */
void ReportEnergy(void *pvParameters)
{
    (void) pvParameters;

    for (;;)
    {
        /* PENGUKURAN ARUS DAN DAYA RESIDENSI */
        float measuredP;
        float measuredI;
        float measuredV;
        if(isOn && !isTampered) {
            measuredP = pzem.power();
            measuredI = pzem.current();
            measuredV = pzem.voltage();
            if(!isnan(measuredP) && !isnan(measuredI) && measuredI < 40) {
                E_tot += measuredP;
                overwriteEEPROM(E_tot, 0, 0);
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
                readTime();
                Serial.printf("Sending [n,R] = [2,0] to server\n");
                /* Kirim kode [n,R] = [2,0] ke server */
            }
        } else {
            isNearOver = 0;
        }

        /* UPDATE PENGGUNAAN ENERGI PADA CONSOLE DAN LCD */
        if(!isKeypadOn) {
            Serial.printf("isTampered = %d\tisOn = %d\tR_stat = %d\tE_all = %.2f\tE_tot = %.2f\tE_sisa = %.2f\tI = %.2f\tV = %.2f\n",
                            isTampered, isOn, R_stat, E_all, E_tot, E_all - E_tot, measuredI, measuredV);
            if(!isTampered) {
                lcd.clear();
                lcd.setCursor(0,0); lcd.print("Status = ");
                lcd.print(isOn);
                lcd.print(R_stat);
                lcd.setCursor(0,1); lcd.print("Sisa   = ");
                setNumLCD(9,1, E_all - E_tot);
            } else {
                lcd.clear();
                lcd.setCursor(0,0); lcd.print("ATTEMPTED");
                lcd.setCursor(0,1); lcd.print("TAMPERING");
            }
        }

        if(counter >= 60) {
            lcd.backlight();
            counter = 0;
        } else {
            counter++;
        }

        /* Receive transmisi paket data dari server */
        LoRa.receive();

        vTaskDelay(850);
    }
}

/*  TASK: ReceiveSignal()
 *  Penerimaan sinyal dari keypad. BEBERAPA BAGIAN TASK INI BERSIFAT TENTATIF.
 *  NOTES: Untuk sementara digunakan keypad untuk mensimulasikan stimulus sinyal
 *         dari server sebelum porting modul LoRa.
 */
void ReceiveSignal(void *pvParameters)
{
    (void) pvParameters;

    for (;;)
    {
        char keyEntered = keypad.getKey();
        if(keyEntered != NO_KEY && keyEntered == 'A') {
             updatePulsa();
        // } else if(keyEntered != NO_KEY && keyEntered == 'D') {
        //      statusPrompt();
        // } else if(keyEntered != NO_KEY && keyEntered == '#') {
        } else if(keyEntered != NO_KEY && keyEntered == '7') {
            if(isTampered) {
                verifyRecoveryCode();
            } else {
                verifyCode();
            }
        }
        vTaskDelay(100);

    }
}

/*  TASK: ReceiveNFC()
 *  Verifikasi dan penambahan alokasi energi listrik residensi jika user melakukan
 *  scanning NFC tag pada modul NFC.
 */
void ReceiveNFC(void *pvParameters)
{
    (void) pvParameters;

    for (;;)
    {
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
        vTaskDelay(1000);

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
        // Serial.printf("CHECKPOINT 3A\n");
        isOn = 0;
        // Serial.printf("CHECKPOINT 3B\n");
        // overwriteEEPROM(0, (isTampered << 1) | (isOn & 1), 2);
        overwriteEEPROM(0, isOn, 2);
        // Serial.printf("CHECKPOINT 3C\n");
        R_stat = r;
        // Serial.printf("CHECKPOINT 3D\n");
        overwriteEEPROM(0, R_stat, 3);
        // Serial.printf("CHECKPOINT 3E\n");
        digitalWrite(ssr, LOW);
        // Serial.printf("CHECKPOINT 3F\n");
        readTime();
        Serial.printf("Sending [n,R] = [0,%d] to server\n", r);
        // Serial.printf("CHECKPOINT 3G\n");
        /* Kirim kode [n,R] = [0,R] ke server */
    } else if(n == 1 && !isOn) {
        bool statement = (!r && R_stat != 3) || (r == 1);
        if(statement) {
              isOn = 1;
              // overwriteEEPROM(0, (isTampered << 1) | (isOn & 1), 2);
              overwriteEEPROM(0, isOn, 2);
              R_stat = r;
              overwriteEEPROM(0, R_stat, 3);
              if(!isTampered) digitalWrite(ssr, HIGH);
              readTime();
              Serial.printf("Sending [n,R] = [1,%d] to server\n", r);
              /* Kirim kode [n,R] = [1,R] ke server */
        }
    } else if(!n && !isOn) {
        if(r == 1 || r == 3) {
            R_stat = r;
            overwriteEEPROM(0, R_stat, 3);
            readTime();
            Serial.printf("Sending [n,R] = [0,%d] to server\n", r);
            /* Kirim kode [n,R] = [0,R] ke server */
        } else if(!r) {
            if(R_stat == 3) {
                readTime();
                Serial.printf("Sending [n,R] = [0,%d] to server\n", r);
                /* Kirim kode [n,R] = [0,R] ke server */
            } else {
                R_stat = r;
                overwriteEEPROM(0, R_stat, 3);
                readTime();
                Serial.printf("Sending [n,R] = [0,%d] to server\n", r);
                /* Kirim kode [n,R] = [0,R] ke server */
            }
        }
    }
}

/*  updatePulsa()
 *  Handler untuk update E_all, dipanggil jika terdapat sinyal dari server untuk
 *  penambahan pulsa energi.
 *  NOTES: MASIH TERDAPAT KODE YANG DIIMPLEMENTASIKAN MENGGUNAKAN KEYPAD. Kode
 *         hendak dihapus setelah porting LoRa.
 */
void updatePulsa() {
    /* IMPLEMENTASI MENGGUNAKAN KEYPAD */
    isKeypadOn = 1;
    readTime();
    Serial.printf("Sending E_tot = %d and E_all = %d to server\n", E_tot, E_all);
    /* Kirim E_tot dan E_all ke server */
    /* Menerima E_all_updated dari server */
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Sisa energi:");
    Serial.printf("Allocated energy : ");

    int E_all_updated = 0;

    char kpad;
    while((kpad = keypad.getKey()) == NO_KEY) {
        vTaskDelay(1);
    }
    while(keypad.getKey() != NO_KEY) {
        vTaskDelay(1);
    }
    while(kpad != 'A') {
        if(!(kpad == 'B' || kpad == 'C' || kpad == 'D' || kpad == '#' || kpad == '*')) {
            E_all_updated = E_all_updated * 10 + kpad - 48;
            Serial.printf("%d", kpad - 48);
        }

        setNumLCD(0,1, E_all_updated);
        while((kpad = keypad.getKey()) == NO_KEY) {
            vTaskDelay(1);
        }
        while(keypad.getKey() != NO_KEY) {
            vTaskDelay(1);
        }
    }
    Serial.printf("\n");

    /* PENAMBAHAN PULSA */
    bool statement = (E_all_updated > E_all && !((!isOn && !R_stat) || (!isOn && R_stat == 3))) || (!isOn && R_stat == 2);
    if(statement) {
        changeMode(1,0);
    }
    /* DN: something weird happened here */
    E_all = E_all_updated;
    overwriteEEPROM(E_all, 0, 1);
    E_tot = 0;
    overwriteEEPROM(E_tot, 0, 0);

    isKeypadOn = 0;
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
    // /* INISIALISASI VARIABEL LOKAL */
    // isKeypadOn = 1;
    // char token[10];
    // char kpad;
    // int i = 0;
    //
    // /* PROMPT INPUT KODE "TOKEN" */
    // lcd.clear();
    // lcd.setCursor(0,0); lcd.print("Token:");
    // Serial.printf("Token: ");
    //
    // while((kpad = keypad.getKey()) == NO_KEY) {
    //     vTaskDelay(1);
    // }
    // while(keypad.getKey() != NO_KEY) {
    //     vTaskDelay(1);
    // }
    // while(kpad != '#') {
    //     if(!(kpad == 'B' || kpad == 'C' || kpad == 'D' || kpad == 'A' || kpad == '*') && i < 10) {
    //         token[i] = kpad - 48;
    //         setNumLCD(i, 1, token[i]);
    //         i++;
    //         Serial.printf("%d", kpad - 48);
    //     }
    //
    //     while((kpad = keypad.getKey()) == NO_KEY) {
    //         vTaskDelay(1);
    //     }
    //     while(keypad.getKey() != NO_KEY) {
    //         vTaskDelay(1);
    //     }
    // }
    // Serial.printf("\n");
    // while(i < 10) {
    //     token[i] = 0;
    //     i++;
    // }
    //
    // /* VERIFIKASI TOKEN DAN PENAMBAHAN PULSA */
    // int a = token[0] + token[1] + token[2];
    // int b = token[3] * token[4] + token[5];
    // int c = token[6] - token[7] * token[8];
    // int d = b + a * token[9] - c;
    // int result = 1000 * extractDigit(a) + 100 * extractDigit(b) + 10 * extractDigit(c) + extractDigit(d);
    //
    // lcd.clear();
    // lcd.setCursor(0,0);
    // int ref = SCRTCD;
    // int refFactor = 1;
    // bool isMatch = 0;
    // while(refFactor < 5) {
    //     if(ref > 9999) ref %= 10000;
    //     if(ref == result) {
    //         lcd.print("BERHASIL");
    //         Serial.printf("Success!\n");
    //         changeMode(1,0);
    //         switch(refFactor) {
    //             case 1  : E_all += 10000; break;
    //             case 2  : E_all += 20000; break;
    //             case 3  : E_all += 50000; break;
    //             case 4  : E_all += 100000; break;
    //         }
    //         overwriteEEPROM(E_all, 0, 1);
    //         isMatch = 1;
    //     }
    //     ref += SCRTCD;
    //     refFactor++;
    // }
    //
    // if(!isMatch) {
    //     lcd.print("GAGAL!");
    //     Serial.printf("Failed!\n");
    // }
    isKeypadOn = 1;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("BERHASIL");
    readTime();
    Serial.printf("Token success! (still waiting for the keypad tho)\n");
    changeMode(1,0);
    E_all -= 100000;
    overwriteEEPROM(E_all, 0, 1);
    vTaskDelay(3000);

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
    //     vTaskDelay(1);
    // }
    // while(keypad.getKey() != NO_KEY) {
    //     vTaskDelay(1);
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
    //         vTaskDelay(1);
    //     }
    //     while(keypad.getKey() != NO_KEY) {
    //         vTaskDelay(1);
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
    // readTime();
    // if(!(statCode[0] == 5 || statCode[1] == 5)) {
    //     Serial.printf("Receiving [n,R] = [%d,%d] from server\n", statCode[0], statCode[1]);
    //     lcd.print("Status received!");
    //     changeMode(statCode[0], statCode[1]);
    // } else {
    //     Serial.printf("Failed to receive [n,R] code from server\n");
    //     lcd.print("Status failed!");
    // }
    //
    //
    // vTaskDelay(3000);
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
                lcd.print("NFC berhasil!   ");
                lcd.setCursor(0,1);
                lcd.print("+100000");

                readTime();
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

                /* ADD ENERGY ACCORDING TO THE CARD, SEND TO SERVER */

                /* THIS IS NOT SUPPOSED TO HAPPEN, WRITTEN HERE FOR EASIER HARDWARE DEBUGGING */
                E_all += 100000;
                overwriteEEPROM(E_all, 0, 1);
                changeMode(1, 0);
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

/*  readTime()
 *  Pembacaan waktu saat pemanggilan fungsi dari RTC.
 *  NOTES: Untuk sementara digunakan untuk print di console.
 */
void readTime() {
    DateTime now = rtc.now();
    // Serial.printf("CHECKPOINT 1A\n");

    Serial.printf("{%02d:%02d:%02d, ", now.hour(), now.minute(), now.second());
    Serial.printf("%02d-%02d-20%02d} ", now.day(), now.month(), now.year() - 2000);
    // Serial.printf("CHECKPOINT 1B\n");
}

/*  overwriteEEPROM()
 *  Overwrite nilai suatu parameter di EEPROM jika nilai tersebut berbeda dengan
 *  nilai yang tersimpan di EEPROM.
 *  INPUT:
 *  - num: floating point (E_tot atau E_all) yang hendak disimpan di EEPROM
 *  - cond: isOn atau R_stat yang hendak disimpan di EEPROM
 *  - mode: Terdapat 4 mode.
 *          0: E_tot hendak disimpan, cond bersifat don't care
 *          1: E_all hendak disimpan, cond bersifat don't care
 *          2: isOn hendak disimpan, num bersifat don't care
 *          3: R_stat hendak disimpan, num bersifat don't care
 */
void overwriteEEPROM(float num, unsigned char cond, int mode) {
    byte b;
    switch (mode) {
        case 0:
                for(unsigned int i = 0; i < 4; i++) {
                    convert.buffer[i] = i2c_eeprom_read_byte(0x57, i + 1);
                }
                // Representasi biner kedua bilangan tersebut harus beda agar diubah
                if(num != convert.numberFloat) {
                    convert.numberFloat = num;
                    i2c_eeprom_write_page(0x57, 1, (byte *) convert.buffer, sizeof(convert.buffer));
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
                }
                break;
        case 2:
                b = i2c_eeprom_read_byte(0x57, 0);
                if(cond != ((b & 12) >> 2)) {
                    b = (cond << 2) | (b & 3);
                    i2c_eeprom_write_byte(0x57, 0, b);
                }
                break;
        case 3:
                b = i2c_eeprom_read_byte(0x57, 0);
                if(cond != (b & 3)) {
                    b = cond | (b & 12);
                    i2c_eeprom_write_byte(0x57, 0, b);
                }
                break;
    }
    vTaskDelay(100);
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
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("ATTEMPTED");
        lcd.setCursor(0,1); lcd.print("TAMPERING");

        isTampered = 1;
        // overwriteEEPROM(0, (isTampered << 1) | (isOn & 1), 2);
        digitalWrite(ssr, LOW);
        Serial.printf("Meter tampered!\n");
        readTime();
        Serial.printf("Sending [n,R] = [3,1] to server\n");
    } else {
        isTampered = 0;
        // overwriteEEPROM(0, (isTampered << 1) | (isOn & 1), 2);
        Serial.printf("Meter recovered!\n");
        readTime();
        Serial.printf("Sending [n,R] = [3,0] to server\n");

        int state = digitalRead(magnetSensor);
        if(isOn && !state) {
            digitalWrite(ssr, HIGH);
        }
    }
}

/*  verifyCode()
 *  Verifikasi recovery code yang dimasukkan pada keypad dengan recovery code
 *  yang benar. Tampering mechanism akan dimatikan jika recovery code yang
 *  dimasukkan benar.
 */
void verifyRecoveryCode() {
    // char inputCode[9];
    // uint32_t calcCode = 0;
    // char kpad;
    // int i = 0;
    //
    // isKeypadOn = 1;
    //
    // /* Prompt input kode */
    // lcd.clear();
    // lcd.setCursor(0,0); lcd.print("Recovery Code:");
    // Serial.printf("Recovery code: ");
    //
    // while((kpad = keypad.getKey()) == NO_KEY) {
    //     vTaskDelay(1);
    // }
    // while(keypad.getKey() != NO_KEY) {
    //     vTaskDelay(1);
    // }
    // while(kpad != '#') {
    //     if(!(kpad == 'B' || kpad == 'C' || kpad == 'D' || kpad == 'A' || kpad == '*') && i < 9) {
    //         inputCode[i] = kpad - 48;
    //         setNumLCD(i, 1, inputCode[i]);
    //         i++;
    //         Serial.printf("%d", kpad - 48);
    //     }
    //
    //     while((kpad = keypad.getKey()) == NO_KEY) {
    //         vTaskDelay(1);
    //     }
    //     while(keypad.getKey() != NO_KEY) {
    //         vTaskDelay(1);
    //     }
    // }
    // Serial.printf("\n");
    // while(i < 9) {
    //     inputCode[i] = 0;
    //     i++;
    // }
    //
    // /* Verifikasi Recovery Code */
    // for(unsigned char j = 0; j < 9; j++) {
    //     calcCode = 10 * calcCode + inputCode[j];
    // }
    //
    // uint32_t ref = SCRTRC;
    // if(calcCode == ref) {
    //     tampered(0);
    // }
    //
    // isKeypadOn = 0;
    tampered(0);
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
        Serial.printf("\nRECEIVED PAYLOAD:\n"); printHexChar((byte *) payload, sizeof(payload));
        parsePayload();
    } else {
        Serial.print(strcmp(rcvd_HMAC,HMAC));
        Serial.print("\n");
        Serial.print(String(rcvd_HMAC));
        Serial.print("\n");
        Serial.print(String(HMAC));
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

        byte opCode_rec;
        unsigned char n_rec, R_rec;
        R_rec = first_rec & 0x3;
        n_rec = (first_rec >> 2) & 0x3;
        opCode_rec = first_rec >> 4;

        union buffer4Byte meterID_rec;
        for(unsigned char i = 0; i < 4; i++) {
            meterID_rec.buffer[i] = (byte) payload[1 + i];
        }

        struct mockDateTime timeRec;
        timeRec.day = (uint8_t) payload[5];
        timeRec.month = (uint8_t) payload[6];
        timeRec.year.buffer[0] = (byte) payload[7];
        timeRec.year.buffer[1] = (byte) payload[8];
        timeRec.hour = (uint8_t) payload[9];
        timeRec.minute = (uint8_t) payload[10];
        timeRec.second = (uint8_t) payload[11];

        union buffer4Byte E_tot_rec, E_all_rec;
        for(unsigned char i = 0; i < 4; i++) {
            E_tot_rec.buffer[i] = (byte) payload[12 + i];
        }
        for(unsigned char i = 0; i < 4; i++) {
            E_all_rec.buffer[i] = (byte) payload[16 + i];
        }

        char nfcCode_rec[16];
        for(unsigned char i = 0; i < 16; i++) {
            nfcCode_rec[i] = payload[20 + i];
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
        // Serial.printf("NFC Code\t\t= "); printHexChar((byte *) nfcCode_rec, sizeof(nfcCode_rec));

        // Translating the parsed code
        // Serial.printf("----------------------------------------------\n");
        // Serial.printf("TRANSLATING PAYLOAD\n");

        if(meterID_rec.numberLong == SCRTCD) {
            switch(opCode_rec) {
                case 0x00 : Serial.printf("Code: NULL - no state parameters will be changed\n");
                            break;
                case 0x01 : Serial.printf("Invalid opcode, should be used for endpoint to server transmission!\n");
                            break;
                case 0x02 : Serial.printf("Invalid opcode, should be used for endpoint to server transmission!\n");
                            break;
                case 0x03 : Serial.printf("Invalid opcode, should be used for endpoint to server transmission!\n");
                            break;
                case 0x04 : Serial.printf("Code: ACKNOWLEDGEMENT\n");
                            Serial.printf("Received parameters:\n");
                            Serial.printf("Opcode\t\t\t= "); printHexChar(&opCode_rec, sizeof(opCode_rec));
                            Serial.printf("Meter ID\t\t= %lu\n", meterID_rec.numberLong);
                            Serial.printf("Time\t\t\t= ");
                            Serial.printf("{%02u:%02u:%02u, ", timeRec.hour, timeRec.minute, timeRec.second);
                            Serial.printf("%02u-%02u-%04u}\n", timeRec.day, timeRec.month, timeRec.year.number);
                            break;
                case 0x05 : Serial.printf("Code: PERUBAHAN STATUS METERAN (SERVER->METER)\n");
                            // Serial.printf("CHECKPOINT 1\n");
                            readTime();
                            // Serial.printf("CHECKPOINT 2\n");
                            Serial.printf("Receiving [n,R] = [%u,%u] from server\n", n_rec, R_rec);
                            // Serial.printf("CHECKPOINT 3\n");
                            changeMode((int) n_rec, (int) R_rec);
                            // Serial.printf("CHECKPOINT 4\n");
                            // Serial.printf("Received parameters:\n");
                            // Serial.printf("Opcode\t\t\t= "); printHexChar(&opCode_rec, sizeof(opCode_rec));
                            // Serial.printf("n\t\t\t= %u\n", n_rec);
                            // Serial.printf("R\t\t\t= %u\n", R_rec);
                            // Serial.printf("Meter ID\t\t= %lu\n", meterID_rec.numberLong);
                            // Serial.printf("Time\t\t\t= ");
                            // Serial.printf("{%02u:%02u:%02u, ", timeRec.hour, timeRec.minute, timeRec.second);
                            // Serial.printf("%02u-%02u-%04u}\n", timeRec.day, timeRec.month, timeRec.year.number);
                            break;
                case 0x06 : Serial.printf("Code: PRIVATE KEY CHANGE (SERVER->METER)\n");
                            Serial.printf("Received parameters:\n");
                            Serial.printf("Opcode\t\t\t= "); printHexChar(&opCode_rec, sizeof(opCode_rec));
                            Serial.printf("Meter ID\t\t= %lu\n", meterID_rec.numberLong);
                            Serial.printf("Time\t\t\t= ");
                            Serial.printf("{%02u:%02u:%02u, ", timeRec.hour, timeRec.minute, timeRec.second);
                            Serial.printf("%02u-%02u-%04u}\n", timeRec.day, timeRec.month, timeRec.year.number);
                            Serial.printf("New Key\t\t= "); printHexChar((byte *) nfcCode_rec, sizeof(nfcCode_rec));
                            break;
                case 0x07 : Serial.printf("Code: TRIGGER PELAPORAN KONSUMSI ENERGI (SERVER->METER)\n");
                            Serial.printf("Received parameters:\n");
                            Serial.printf("Opcode\t\t\t= "); printHexChar(&opCode_rec, sizeof(opCode_rec));
                            Serial.printf("Meter ID\t\t= %lu\n", meterID_rec.numberLong);
                            Serial.printf("Time\t\t\t= ");
                            Serial.printf("{%02u:%02u:%02u, ", timeRec.hour, timeRec.minute, timeRec.second);
                            Serial.printf("%02u-%02u-%04u}\n", timeRec.day, timeRec.month, timeRec.year.number);
                            break;
                case 0x08 : Serial.printf("Code: UPDATE ALLOCATION (SERVER->METER)\n");
                            Serial.printf("Received parameters:\n");
                            Serial.printf("Opcode\t\t\t= "); printHexChar(&opCode_rec, sizeof(opCode_rec));
                            Serial.printf("Meter ID\t\t= %lu\n", meterID_rec.numberLong);
                            Serial.printf("Time\t\t\t= ");
                            Serial.printf("{%02u:%02u:%02u, ", timeRec.hour, timeRec.minute, timeRec.second);
                            Serial.printf("%02u-%02u-%04u}\n", timeRec.day, timeRec.month, timeRec.year.number);
                            Serial.printf("Added energy\t= %f\n", E_all_rec.numberFloat);
                            break;
                default :   Serial.printf("Code: INVALID OPCODE\n");
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
