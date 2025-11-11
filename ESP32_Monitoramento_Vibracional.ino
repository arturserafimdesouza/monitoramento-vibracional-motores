/*
  ESP32 + MPU6050 (Adafruit) + FFT (ArduinoFFT) + MQTT (PubSubClient)
  Fs: 400 Hz | N: 256 (∆f ≈ 1.5625 Hz)
  Tópicos: tcc/vibration/summary (JSON), tcc/vibration/debug
  Ligações: SCL->D22, SDA->D21, ADO -> GND ,VCC->3V3, GND->GND
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <arduinoFFT.h>
#include <math.h>

//====================== CONFIG ======================
#define WIFI_SSID   "IGOR_2G"
#define WIFI_PASS   "6975940704"

#define MQTT_BROKER "192.168.0.55"
#define MQTT_PORT   1883
#define MQTT_USER   ""
#define MQTT_PASS   ""
#define MQTT_CLIENTID "esp32-vibe-01"

#define MQTT_TOPIC_SUMMARY "tcc/vibration/summary"
#define MQTT_TOPIC_DEBUG   "tcc/vibration/debug"

#define FFT_AXIS 'X'          // 'X' | 'Y' | 'Z'
#define SAMPLES  256
#define FS       400.0

#define USE_G_UNITS false     // true => valores em g, false => m/s^2

#define I2C_SDA 21
#define I2C_SCL 22
#define DEBUG_SERIAL 1

// ---- Thresholds (ajuste conforme teu caso) ----
// vibração (RMS após remover média)
#define THRESH_RMS_DYN_ON   0.20
#define THRESH_RMS_DYN_OFF  0.12
// inclinação lenta (delta da média entre blocos)
#define THRESH_MEAN_DEL_ON  0.40
#define THRESH_MEAN_DEL_OFF 0.25
// amplitude do bloco (max - min)
#define THRESH_RANGE_ON     1.00
#define THRESH_RANGE_OFF    0.70
// giroscópio (RMS do módulo em deg/s)
#define THRESH_GYRO_RMS_ON  6.0
#define THRESH_GYRO_RMS_OFF 3.0

// quantos blocos consecutivos para alternar o estado
#define DEBOUNCE_BLOCKS 3
//=================================================

Adafruit_MPU6050 mpu;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

double vReal[SAMPLES], vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, FS);

float baseX=0, baseY=0, baseZ=0;

double lastMean=0.0, lastMin=0.0, lastMax=0.0;
double prevMean=0.0;
double rms_raw=0.0;      // antes de remover média
double gyro_rms=0.0;     // RMS do módulo do gyro (deg/s)

bool moving=false;
uint8_t cnt_on=0, cnt_off=0;

static inline float toUnits(float a_ms2){
  return USE_G_UNITS ? (a_ms2/9.80665f) : a_ms2;
}

//-------------------- WiFi/MQTT --------------------
void connectWiFi(){
#if DEBUG_SERIAL
  Serial.printf("Conectando ao WiFi: %s\n", WIFI_SSID);
#endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint16_t tries=0;
  while (WiFi.status()!=WL_CONNECTED){
    delay(500);
#if DEBUG_SERIAL
    Serial.print('.');
#endif
    if(++tries>120) ESP.restart();
  }
#if DEBUG_SERIAL
  Serial.print("\nWiFi OK, IP: "); Serial.println(WiFi.localIP());
#endif
}

void connectMQTT(){
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setBufferSize(2048);
  while(!mqttClient.connected()){
#if DEBUG_SERIAL
    Serial.printf("MQTT %s:%d ...\n", MQTT_BROKER, MQTT_PORT);
#endif
    bool ok = String(MQTT_USER).length()>0 ?
      mqttClient.connect(MQTT_CLIENTID, MQTT_USER, MQTT_PASS) :
      mqttClient.connect(MQTT_CLIENTID);
    if(ok){
#if DEBUG_SERIAL
      Serial.println("MQTT conectado!");
#endif
      mqttClient.publish("tcc/vibration/status","online",true);
    }else{
#if DEBUG_SERIAL
      Serial.printf("Falha MQTT rc=%d\n", mqttClient.state());
#endif
      delay(1500);
    }
  }
}

//-------------------- Calibração --------------------
void calibrateBaseline(uint16_t n=300){
#if DEBUG_SERIAL
  Serial.println("Calibrando baseline...");
#endif
  double sx=0,sy=0,sz=0;
  sensors_event_t a,g,t;
  for(uint16_t i=0;i<n;i++){
    mpu.getEvent(&a,&g,&t);
    sx+=a.acceleration.x;
    sy+=a.acceleration.y;
    sz+=a.acceleration.z;
    delay(5);
  }
  baseX = sx/n;
  baseY = sy/n;
  baseZ = sz/n;
#if DEBUG_SERIAL
  Serial.printf("Baseline X/Y/Z: %.3f %.3f %.3f (m/s^2)\n", baseX, baseY, baseZ);
#endif
}

//---------------- Amostragem + métricas -----------
void sampleBlock(uint16_t n, double fs, char axis){
  const uint32_t period_us = (uint32_t)(1000000.0/fs + 0.5);
  uint32_t tNext = micros();

  double sum=0.0, sumsq=0.0;
  double vmin= 1e9, vmax=-1e9;
  double gyro_sumsq=0.0;

  sensors_event_t a,g,t;

  for(uint16_t i=0;i<n;i++){
    mpu.getEvent(&a,&g,&t);
    float ax=a.acceleration.x-baseX;
    float ay=a.acceleration.y-baseY;
    float az=a.acceleration.z-baseZ;

    float s;
    switch(axis){
      case 'Y': s = ay; break;
      case 'Z': s = az; break;
      case 'X':
      default:  s = ax; break;
    }
    if (USE_G_UNITS) s/=9.80665f;

    // guarda para FFT (por enquanto com DC; removemos depois)
    vReal[i]=s; vImag[i]=0.0;

    // métricas do bloco (antes do detrend)
    sum   += s;
    sumsq += (double)s*s;
    if (s<vmin) vmin=s;
    if (s>vmax) vmax=s;

    // módulo do gyro em deg/s (a biblioteca já dá em rad/s? -> Adafruit_MPU6050 dá em rad/s por padrão!
    // Converter para deg/s para uso intuitivo)
    float gx = g.gyro.x * 57.2957795f;
    float gy = g.gyro.y * 57.2957795f;
    float gz = g.gyro.z * 57.2957795f;
    float gn = sqrtf(gx*gx + gy*gy + gz*gz);
    gyro_sumsq += (double)gn*gn;

    // temporização
    tNext += period_us;
    while ((int32_t)(micros()-tNext) < 0) { /* wait */ }
  }

  lastMean = sum/n;
  rms_raw  = sqrt(sumsq/n);
  lastMin  = vmin;
  lastMax  = vmax;
  gyro_rms = sqrt(gyro_sumsq/n);

  // remove DC para FFT e RMS dinâmico
  for(uint16_t i=0;i<n;i++) vReal[i]-=lastMean;
}

double computeRMS(const double *buf, uint16_t n){
  double acc=0.0; for(uint16_t i=0;i<n;i++) acc += buf[i]*buf[i];
  return sqrt(acc/n);
}

void computeFFT_andPeak(double &peakFreq, double &peakAmp){
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  uint16_t k=1; double pv=vReal[1];
  for(uint16_t i=2; i<SAMPLES/2; i++){
    if(vReal[i]>pv){ pv=vReal[i]; k=i; }
  }
  peakFreq = (k*FS)/SAMPLES;
  peakAmp  = pv; // magnitude relativa (com janela; comparar entre blocos)
}

//--------- Atualiza estado de movimento (histerese) ----------
bool updateMoving(double rms_dyn, double dmean, double range, double gyro){
  bool trigger_on  = (rms_dyn >= THRESH_RMS_DYN_ON)  ||
                     (fabs(dmean) >= THRESH_MEAN_DEL_ON) ||
                     (range >= THRESH_RANGE_ON) ||
                     (gyro >= THRESH_GYRO_RMS_ON);

  bool trigger_off = (rms_dyn <  THRESH_RMS_DYN_OFF) &&
                     (fabs(dmean) < THRESH_MEAN_DEL_OFF) &&
                     (range < THRESH_RANGE_OFF) &&
                     (gyro < THRESH_GYRO_RMS_OFF);

  if (moving){
    if (trigger_off) { cnt_off++; cnt_on=0; }
    else { cnt_off=0; }
    if (cnt_off >= DEBOUNCE_BLOCKS) { moving=false; cnt_off=0; }
  } else {
    if (trigger_on) { cnt_on++; cnt_off=0; }
    else { cnt_on=0; }
    if (cnt_on >= DEBOUNCE_BLOCKS) { moving=true; cnt_on=0; }
  }
  return moving;
}

void setup(){
#if DEBUG_SERIAL
  Serial.begin(115200);
  delay(200);
#endif
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if(!mpu.begin()){
#if DEBUG_SERIAL
    Serial.println("Falha ao iniciar MPU6050!");
#endif
    while(true){ delay(1000); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  delay(120);

  connectWiFi();
  connectMQTT();
  calibrateBaseline(300);

#if DEBUG_SERIAL
  Serial.println("Setup pronto.");
#endif
}

void loop(){
  if (WiFi.status()!=WL_CONNECTED) connectWiFi();
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  // 1) Amostra + métricas do bloco
  sampleBlock(SAMPLES, FS, FFT_AXIS);

  // 2) RMS dinâmico (após remover média)
  double rms_dyn = computeRMS(vReal, SAMPLES);
  double dmean   = lastMean - prevMean;
  double range   = lastMax - lastMin;

  // 3) FFT
  double peakFreq=0.0, peakAmp=0.0;
  computeFFT_andPeak(peakFreq, peakAmp);

  // 4) Atualiza estado de movimento
  bool isMoving = updateMoving(rms_dyn, dmean, range, gyro_rms);

  // 5) Publica JSON
  char payload[768];
  snprintf(payload, sizeof(payload),
    "{"
      "\"device\":\"%s\","
      "\"axis\":\"%c\","
      "\"fs\":%.1f,"
      "\"samples\":%d,"
      "\"units\":\"%s\","
      "\"mean\":%.6f,"
      "\"dmean\":%.6f,"
      "\"min\":%.6f,"
      "\"max\":%.6f,"
      "\"range\":%.6f,"
      "\"rms_raw\":%.6f,"
      "\"rms_dyn\":%.6f,"
      "\"gyro_rms_dps\":%.3f,"
      "\"moving\":%s,"
      "\"peak_freq\":%.2f,"
      "\"peak_amp\":%.6f"
    "}",
    MQTT_CLIENTID,
    FFT_AXIS,
    FS,
    SAMPLES,
    USE_G_UNITS ? "g" : "m/s^2",
    lastMean,
    dmean,
    lastMin,
    lastMax,
    range,
    rms_raw,
    rms_dyn,
    gyro_rms,
    isMoving?"true":"false",
    peakFreq,
    peakAmp
  );

  bool ok = mqttClient.publish(MQTT_TOPIC_SUMMARY, payload);

#if DEBUG_SERIAL
  Serial.printf(
    "MQTT summary -> %s | axis=%c fs=%.1f n=%d "
    "mean=%.3f dmean=%.3f range=%.3f rms_raw=%.3f rms_dyn=%.3f gyro=%.2f "
    "pf=%.2fHz pa=%.3f moving=%d\n",
    ok?"OK":"FALHA",
    FFT_AXIS, FS, SAMPLES,
    lastMean, dmean, range, rms_raw, rms_dyn, gyro_rms,
    peakFreq, peakAmp, isMoving?1:0
  );
#endif

  prevMean = lastMean;
  delay(40);
}
