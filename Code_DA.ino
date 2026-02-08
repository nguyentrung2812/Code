#include <DHTesp.h>
#include <SPI.h>
#include <SD.h>

// ===== LCD ST7735 =====
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// ===== SHT30 (I2C) =====
#include <Wire.h>
#include <Adafruit_SHT31.h>

// ===== DHT & MQ135 & PMS =====
const int PIN_DHT = 21;
const int PIN_MQ135_ADC = 34;
const int PMS_RX_PIN = 16;
const int PMS_TX_PIN = 17;

// ===== SHT30 I2C pins =====
const int I2C_SDA = 32;
const int I2C_SCL = 33;
const uint8_t SHT30_ADDR = 0x44; // thử 0x45 nếu init fail

// ===== SPI (VSPI) =====
const int PIN_SCK  = 18;
const int PIN_MISO = 19;
const int PIN_MOSI = 23;

// ===== MicroSD =====
const int SD_CS = 13;

// ===== LCD ST7735 (SPI) =====
const int TFT_CS  = 14;
const int TFT_DC  = 27;
const int TFT_RST = 22;

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

// ===== CẤU HÌNH =====
const float ADC_REF = 3.3;
const int ADC_MAX = 4095;

const uint32_t SENSOR_PERIOD_MS = 3000; // chu kỳ in/log/lcd (3s)
const uint32_t MQ_SAMPLE_MS = 500;
const uint8_t  MQ_AVG_SAMPLES = 10;
const uint32_t MQ_WARMUP_MS = 3UL * 60UL * 1000UL;

// ===== ĐỐI TƯỢNG =====
DHTesp dht;
HardwareSerial pmsSerial(2);
Adafruit_SHT31 sht30 = Adafruit_SHT31();

// ===== BIẾN =====
uint32_t t_last_tick = 0;
uint32_t t_last_mq = 0;

float mq_running_avg = 0;
bool mq_avg_inited = false;

uint16_t pm1_atm = 0, pm25_atm = 0, pm10_atm = 0;
uint32_t pm_last_ok_ms = 0;

bool sht_ok = false;
float sht_T = NAN, sht_H = NAN;

bool dht_ok = false;
float dht_T = NAN, dht_H = NAN;

String logFilename = "";

// ===== LCD offset =====
static const int DX = -6;
static const int DY = 3;

// ===================== PROTOTYPE =====================
float readMQ135Voltage(bool &ok);
bool readPMS7003(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10);

String createNewLogFile();
bool sdAppendLine(uint32_t ms,
                  float dhtT, float dhtH,
                  float shtT, float shtH,
                  uint16_t PM1, uint16_t PM25, uint16_t PM10,
                  float MQV);

String fmtVal(float v, uint8_t dec = 1);
void drawCenteredLine(int y, const String &s, uint16_t color, uint8_t textSize);
void lcdShow(float dhtT, float dhtH,
             float shtT, float shtH,
             bool pms_fresh, bool warming);

// ======================================================
// =================== HÀM ĐỌC CẢM BIẾN ==================
// ======================================================

float readMQ135Voltage(bool &ok) {
  int raw = analogRead(PIN_MQ135_ADC);
  float v_adc = (raw * 1.0f / ADC_MAX) * ADC_REF;
  ok = true;
  return v_adc;
}

bool readPMS7003(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10) {
  const uint16_t FRAME_BODY = 30;

  while (pmsSerial.available() >= (2 + FRAME_BODY)) {
    if (pmsSerial.peek() != 0x42) {
      pmsSerial.read();
      continue;
    }
    pmsSerial.read();

    if (pmsSerial.peek() != 0x4D) continue;
    pmsSerial.read();

    uint8_t frame[FRAME_BODY];
    if (pmsSerial.readBytes(frame, FRAME_BODY) != FRAME_BODY) return false;

    uint16_t sum = 0x42 + 0x4D;
    for (int i = 0; i < 28; i++) sum += frame[i];

    uint16_t cs = (uint16_t(frame[28]) << 8) | frame[29];
    if (sum != cs) continue;

    pm1  = (uint16_t(frame[8])  << 8) | frame[9];
    pm25 = (uint16_t(frame[10]) << 8) | frame[11];
    pm10 = (uint16_t(frame[12]) << 8) | frame[13];
    return true;
  }
  return false;
}

// ======================================================
// ======================= SD LOG ========================
// ======================================================

String createNewLogFile() {
  int index = 1;
  char fname[32];

  while (true) {
    snprintf(fname, sizeof(fname), "/data%d.csv", index);
    if (!SD.exists(fname)) {
      File f = SD.open(fname, FILE_WRITE);
      if (f) {
        f.println("millis,DHT_T_C,DHT_H_pct,SHT_T_C,SHT_H_pct,PM1,PM2_5,PM10,MQ135_V");
        f.close();
        return String(fname);
      } else {
        Serial.println("Lỗi: không tạo được file!");
        return "";
      }
    }
    index++;
  }
}

bool sdAppendLine(uint32_t ms,
                  float dhtT, float dhtH,
                  float shtT, float shtH,
                  uint16_t PM1, uint16_t PM25, uint16_t PM10,
                  float MQV) {
  File f = SD.open(logFilename, FILE_APPEND);
  if (!f) return false;

  f.print(ms); f.print(',');
  if (isnan(dhtT)) f.print(""); else f.print(dhtT, 1); f.print(',');
  if (isnan(dhtH)) f.print(""); else f.print(dhtH, 1); f.print(',');
  if (isnan(shtT)) f.print(""); else f.print(shtT, 1); f.print(',');
  if (isnan(shtH)) f.print(""); else f.print(shtH, 1); f.print(',');
  f.print(PM1);  f.print(',');
  f.print(PM25); f.print(',');
  f.print(PM10); f.print(',');
  f.println(MQV, 3);

  f.close();
  return true;
}

// ======================================================
// ======================= LCD UI ========================
// ======================================================

String fmtVal(float v, uint8_t dec) {
  if (isnan(v)) return "--";
  return String((double)v, (unsigned int)dec);
}

void drawCenteredLine(int y, const String &s, uint16_t color, uint8_t textSize) {
  int16_t x1, y1;
  uint16_t w, h;

  tft.setTextSize(textSize);
  tft.setTextColor(color);
  tft.setTextWrap(false);
  tft.getTextBounds(s, 0, y, &x1, &y1, &w, &h);

  int x = (tft.width() - (int)w) / 2;
  if (x < 2) x = 2;

  tft.setCursor(x + DX, y + DY);
  tft.print(s);
}

void lcdShow(float dhtT, float dhtH,
             float shtT, float shtH,
             bool pms_fresh, bool warming) {

  const int margin = 4;
  tft.fillScreen(ST77XX_BLACK);
  tft.drawRect(DX + 0, DY + 0, tft.width(), tft.height(), ST77XX_WHITE);
  tft.drawRect(DX + margin, DY + margin,
               tft.width() - 2 * margin,
               tft.height() - 2 * margin,
               ST77XX_RED);

  int y = DY + margin + 6;
  const int lh = 12;

  String s1 = "DHT T:" + fmtVal(dhtT, 1) + "C H:" + fmtVal(dhtH, 1) + "%";
  String s2 = "SHT T:" + fmtVal(shtT, 1) + "C H:" + fmtVal(shtH, 1) + "%";
  String s3 = "MQ: " + String(mq_running_avg, 3) + "V" + (warming ? " (warm)" : "");

  drawCenteredLine(y, s1, ST77XX_WHITE, 1); y += lh;
  drawCenteredLine(y, s2, ST77XX_CYAN, 1);  y += lh;
  drawCenteredLine(y, s3, ST77XX_GREEN, 1); y += lh;

  y += 6;
  drawCenteredLine(y, "PMS (atm)", ST77XX_YELLOW, 1); y += lh;

  if (pms_fresh) {
    drawCenteredLine(y, "PM1 : " + String(pm1_atm), ST77XX_WHITE, 1); y += lh;
    drawCenteredLine(y, "PM2.5: " + String(pm25_atm), ST77XX_WHITE, 1); y += lh;
    drawCenteredLine(y, "PM10: " + String(pm10_atm), ST77XX_WHITE, 1); y += lh;
  } else {
    drawCenteredLine(y, "No PMS data", ST77XX_RED, 1);
  }
}

// ======================================================
// ======================== SETUP ========================
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);

  dht.setup(PIN_DHT, DHTesp::DHT22);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  // LCD init
  tft.setSPISpeed(4000000);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.println("Booting...");

  // I2C + SHT30
  Wire.begin(I2C_SDA, I2C_SCL);
  sht_ok = sht30.begin(SHT30_ADDR);
  if (!sht_ok) Serial.println("SHT30 init FAILED! (try 0x45)");
  else Serial.println("SHT30 init OK.");

  // SD init
  digitalWrite(TFT_CS, HIGH);
  delay(10);

  if (!SD.begin(SD_CS, SPI, 4000000)) {
    Serial.println("SD init FAILED!");
    tft.setTextColor(ST77XX_RED);
    tft.println("SD FAIL");
  } else {
    Serial.println("SD init OK.");
    logFilename = createNewLogFile();
    Serial.print("Ghi vao file: ");
    Serial.println(logFilename);
    tft.setTextColor(ST77XX_GREEN);
    tft.println("SD OK");
  }

  Serial.println("== ESP32 + DHT22 + SHT30 + MQ135 + PMS7003 + MicroSD + ST7735 ==");
}

// ======================================================
// ========================= LOOP ========================
// ======================================================

void loop() {
  uint32_t now = millis();

  // PMS
  if (readPMS7003(pm1_atm, pm25_atm, pm10_atm)) {
    pm_last_ok_ms = now;
  }

  // MQ135 avg
  if (now - t_last_mq >= MQ_SAMPLE_MS) {
    t_last_mq = now;
    bool ok;
    float v_adc = readMQ135Voltage(ok);
    if (ok) {
      if (!mq_avg_inited) mq_running_avg = v_adc;
      else {
        const float alpha = 1.0f / MQ_AVG_SAMPLES;
        mq_running_avg = mq_running_avg * (1 - alpha) + v_adc * alpha;
      }
      mq_avg_inited = true;
    }
  }

  // Tick
  if (now - t_last_tick >= SENSOR_PERIOD_MS) {
    t_last_tick = now;

    // DHT22
    TempAndHumidity th = dht.getTempAndHumidity();
    int dht_status = dht.getStatus();
    if (dht_status == 0) {
      dht_ok = true;
      dht_T = th.temperature;
      dht_H = th.humidity;
    } else {
      dht_ok = false;
      dht_T = NAN;
      dht_H = NAN;
    }

    // SHT30
    if (sht_ok) {
      float t = sht30.readTemperature();
      float h = sht30.readHumidity();
      if (!isnan(t) && !isnan(h)) {
        sht_T = t;
        sht_H = h;
      } else {
        sht_T = NAN;
        sht_H = NAN;
      }
    } else {
      sht_T = NAN;
      sht_H = NAN;
    }

    bool warming = (now < MQ_WARMUP_MS);
    bool pms_fresh = (now - pm_last_ok_ms) < 5000;

    // Serial
    Serial.print("DHT=");
    if (dht_ok) Serial.printf("%.1fC %.1f%%", dht_T, dht_H);
    else Serial.print("no data");

    Serial.printf(" | MQ=%.3fV %s",
                  mq_running_avg,
                  warming ? "(warming...)" : "");

    Serial.print(" | SHT=");
    if (!isnan(sht_T) && !isnan(sht_H))
      Serial.printf("%.1fC %.1f%%", sht_T, sht_H);
    else
      Serial.print("no data");

    if (pms_fresh) {
      Serial.printf(" | PM1=%u PM2.5=%u PM10=%u\n",
                    pm1_atm, pm25_atm, pm10_atm);
    } else {
      Serial.println(" | PMS: no data");
    }

    // LCD
    lcdShow(dht_T, dht_H, sht_T, sht_H, pms_fresh, warming);

    // SD log
    if (!logFilename.isEmpty()) {
      sdAppendLine(now,
                   dht_T, dht_H,
                   sht_T, sht_H,
                   pms_fresh ? pm1_atm  : 0,
                   pms_fresh ? pm25_atm : 0,
                   pms_fresh ? pm10_atm : 0,
                   mq_running_avg);
    }
  }
}
