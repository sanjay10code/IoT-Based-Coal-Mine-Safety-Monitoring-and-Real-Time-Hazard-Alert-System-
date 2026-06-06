#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <GP2YDustSensor.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <SD.h>
#include <SPI.h>

// === SENSOR PINS ===
const uint8_t DUST_LED = 13;
const uint8_t DUST_VO = 32;
const uint8_t MQ7_PIN = 34;
const uint8_t MQ131_PIN = 36;
const uint8_t MQ4_PIN = 35;
const uint8_t LPG_PIN = 33;
const uint8_t DHT_PIN = 4;
const uint8_t BUZZER = 23;
const uint8_t DANGER_LED = 2;
const uint8_t SD_CS = 5;  // SD card chip select

// === MQ-135 AIR QUALITY CLICK PIN ===
const uint8_t MQ135_PIN = 39;

// === SENSOR OBJECTS ===
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1010AU0F, DUST_LED, DUST_VO);

// === DATA LOGGING CONFIG ===
const int MAX_DATA_POINTS = 180;  // 180 points = 1 hour at 20s intervals
const unsigned long LOG_INTERVAL = 20000;  // Log every 20 seconds

// === DATA STRUCTURES ===
struct SensorData {
  float pm25 = 0;
  float co_ppm = 0;
  float o3_ppm = 0;
  float ch4_ppm = 0;
  float lpg_ppm = 0;
  float dht_temp = 0;
  float dht_hum = 0;
  float ir_temp = 0;
  float ambient_temp = 0;
  
  // MQ-135 Data
  int mq135_raw = 0;
  float mq135_voltage = 0;
  float mq135_air_quality = 0;
  float mq135_co2_est = 0;
  float mq135_nh3_est = 0;
  
  unsigned long timestamp = 0;
};

// Circular buffer for historical data
struct HistoricalData {
  unsigned long timestamps[MAX_DATA_POINTS];
  float co_values[MAX_DATA_POINTS];
  float o3_values[MAX_DATA_POINTS];
  float ch4_values[MAX_DATA_POINTS];
  float lpg_values[MAX_DATA_POINTS];
  float co2_values[MAX_DATA_POINTS];
  float aqi_values[MAX_DATA_POINTS];
  float temp_values[MAX_DATA_POINTS];
  int data_index = 0;
  bool buffer_full = false;
};

SensorData currentData;
HistoricalData historicalData;

// === CALIBRATION VALUES ===
const int MQ135_CLEAN_AIR_ADC = 4095;
const int MQ135_POLLUTED_AIR_ADC = 1000;
const float ADC_REF = 3.3;
const float MQ135_MAX_VOLTAGE = 3.04;

// Gas sensor calibration
const float MQ7_RL = 10.0;
const float MQ7_RO_CLEAN_AIR = 15.0;
const float MQ7_BASELINE_OFFSET = 17.0;
const float MQ7_SCALE = 5.0;

const float MQ131_RL = 20.0;
const float MQ131_RO_CLEAN_AIR = 45.0;
const float MQ131_BASELINE_OFFSET = 3.0;
const float MQ131_SCALE = 2.0;

const float MQ4_RL = 10.0;
const float MQ4_RO_CLEAN_AIR = 18.0;
const float MQ4_BASELINE_OFFSET = 39.0;
const float MQ4_SCALE = 10.0;

const float LPG_RL = 10.0;
const float LPG_RO_CLEAN_AIR = 22.0;
const float LPG_BASELINE_OFFSET = 92.0;
const float LPG_SCALE = 15.0;

// === WIFI AP MODE ===
const char* ap_ssid = "mine_safety_system";
const char* ap_password = "12345678";
WebServer server(80);

// === FUNCTION DECLARATIONS ===
float convertADCtoPPM(int adcValue, float RL, float Ro_clean_air, 
                      float baseline_offset, float scale, bool isOzone = false);
void readMQ135();
void readGasSensors();
void readSensors();
void updateLCD();
void checkAlerts();
void logToSD(SensorData data);
void addToHistory(SensorData data);
String getCurrentTimeString();
void handleRoot();
void handleData();
void handleHistory();
void handleDownload();
void handleGraphPage();

// === PPM CONVERSION FUNCTION ===
float convertADCtoPPM(int adcValue, float RL, float Ro_clean_air, 
                      float baseline_offset, float scale, bool isOzone) {
  float Vout = (adcValue * ADC_REF) / 4095.0;
  float r_s = RL * (ADC_REF - Vout) / Vout;
  float ratio = r_s / Ro_clean_air;
  
  // Calculate raw ppm
  float raw_ppm = baseline_offset + (scale * (1.0 / (ratio + 0.01)));
  
  // Subtract baseline to get actual gas concentration
  float actual_ppm = raw_ppm - baseline_offset;
  
  // Ensure non-negative values
  if (actual_ppm < 0) actual_ppm = 0;
  
  // Apply realistic maximums
  if (isOzone) return constrain(actual_ppm, 0, 10.0);      // Ozone: 0-10 ppm max
  if (scale == 5.0) return constrain(actual_ppm, 0, 100.0); // CO: 0-100 ppm max
  if (scale == 10.0) return constrain(actual_ppm, 0, 1000.0); // CH4: 0-1000 ppm
  return constrain(actual_ppm, 0, 500.0);                  // LPG: 0-500 ppm
}

// === MQ-135 READING ===
void readMQ135() {
  currentData.mq135_raw = analogRead(MQ135_PIN);
  currentData.mq135_voltage = (currentData.mq135_raw * ADC_REF) / 4095.0;
  
  // Reverse logic: Higher ADC = Cleaner air
  currentData.mq135_air_quality = map(currentData.mq135_raw,
                               MQ135_POLLUTED_AIR_ADC,
                               MQ135_CLEAN_AIR_ADC,
                               0, 100);
  currentData.mq135_air_quality = constrain(currentData.mq135_air_quality, 0, 100);
  
  currentData.mq135_co2_est = map(currentData.mq135_raw,
                          MQ135_POLLUTED_AIR_ADC,
                          MQ135_CLEAN_AIR_ADC,
                          2000, 400);
  currentData.mq135_co2_est = constrain(currentData.mq135_co2_est, 400, 2000);
  
  currentData.mq135_nh3_est = map(currentData.mq135_raw,
                          MQ135_POLLUTED_AIR_ADC,
                          MQ135_CLEAN_AIR_ADC,
                          50, 0);
  currentData.mq135_nh3_est = constrain(currentData.mq135_nh3_est, 0, 50);
}

// === GAS SENSORS - CALIBRATED ===
void readGasSensors() {
  int mq7_adc = analogRead(MQ7_PIN);
  int mq131_adc = analogRead(MQ131_PIN);
  int mq4_adc = analogRead(MQ4_PIN);
  int lpg_adc = analogRead(LPG_PIN);
  
  // Apply baseline calibration
  currentData.co_ppm = convertADCtoPPM(mq7_adc, MQ7_RL, MQ7_RO_CLEAN_AIR, 
                               MQ7_BASELINE_OFFSET, MQ7_SCALE, false);
  currentData.o3_ppm = convertADCtoPPM(mq131_adc, MQ131_RL, MQ131_RO_CLEAN_AIR,
                               MQ131_BASELINE_OFFSET, MQ131_SCALE, true);
  currentData.ch4_ppm = convertADCtoPPM(mq4_adc, MQ4_RL, MQ4_RO_CLEAN_AIR,
                                MQ4_BASELINE_OFFSET, MQ4_SCALE, false);
  currentData.lpg_ppm = convertADCtoPPM(lpg_adc, LPG_RL, LPG_RO_CLEAN_AIR,
                                LPG_BASELINE_OFFSET, LPG_SCALE, false);
  
  // Format with appropriate precision
  currentData.co_ppm = round(currentData.co_ppm);
  currentData.o3_ppm = round(currentData.o3_ppm * 10) / 10.0;
  currentData.ch4_ppm = round(currentData.ch4_ppm);
  currentData.lpg_ppm = round(currentData.lpg_ppm);
}

// === ADD DATA TO HISTORY BUFFER ===
void addToHistory(SensorData data) {
  int idx = historicalData.data_index;
  
  historicalData.timestamps[idx] = data.timestamp;
  historicalData.co_values[idx] = data.co_ppm;
  historicalData.o3_values[idx] = data.o3_ppm;
  historicalData.ch4_values[idx] = data.ch4_ppm;
  historicalData.lpg_values[idx] = data.lpg_ppm;
  historicalData.co2_values[idx] = data.mq135_co2_est;
  historicalData.aqi_values[idx] = data.mq135_air_quality;
  historicalData.temp_values[idx] = data.dht_temp;
  
  historicalData.data_index = (idx + 1) % MAX_DATA_POINTS;
  if (historicalData.data_index == 0) {
    historicalData.buffer_full = true;
  }
}

// === LOG TO SD CARD ===
void logToSD(SensorData data) {
  if (!SD.begin(SD_CS)) return;
  
  File dataFile = SD.open("/sensor_log.csv", FILE_APPEND);
  if (dataFile) {
    String timeStr = getCurrentTimeString();
    dataFile.print(timeStr);
    dataFile.print(",");
    dataFile.print(data.co_ppm, 1);
    dataFile.print(",");
    dataFile.print(data.o3_ppm, 2);
    dataFile.print(",");
    dataFile.print(data.ch4_ppm, 1);
    dataFile.print(",");
    dataFile.print(data.lpg_ppm, 1);
    dataFile.print(",");
    dataFile.print(data.mq135_co2_est, 0);
    dataFile.print(",");
    dataFile.print(data.mq135_air_quality, 0);
    dataFile.print(",");
    dataFile.print(data.dht_temp, 1);
    dataFile.print(",");
    dataFile.print(data.dht_hum, 0);
    dataFile.print(",");
    dataFile.println(data.pm25, 1);
    
    dataFile.close();
    Serial.println("✅ Data logged to SD card");
  }
}

String getCurrentTimeString() {
  unsigned long seconds = millis() / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  
  char buffer[12];
  sprintf(buffer, "%02lu:%02lu:%02lu", 
          hours % 24, minutes % 60, seconds % 60);
  return String(buffer);
}

// === READ ALL SENSORS ===
void readSensors() {
  currentData.pm25 = dustSensor.getDustDensity();
  currentData.dht_temp = dht.readTemperature();    
  currentData.dht_hum = dht.readHumidity();        
  currentData.ir_temp = mlx.readObjectTempC();     
  currentData.ambient_temp = mlx.readAmbientTempC(); 
  
  readGasSensors();
  readMQ135();
  
  currentData.timestamp = millis();
  
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 10000) {
    Serial.println("\n=== CALIBRATED READINGS ===");
    Serial.printf("MQ-135: %d ADC (%.2fV) | AQ: %.0f%% | CO2: %.0fppm\n", 
                  currentData.mq135_raw, currentData.mq135_voltage, currentData.mq135_air_quality, currentData.mq135_co2_est);
    Serial.printf("Gases (actual): CO=%.0fppm | O3=%.1fppm | CH4=%.0fppm | LPG=%.0fppm\n", 
                  currentData.co_ppm, currentData.o3_ppm, currentData.ch4_ppm, currentData.lpg_ppm);
    Serial.printf("Env: Temp=%.1f°C | Hum=%.0f%% | PM2.5=%.1fμg/m³\n", 
                  currentData.dht_temp, currentData.dht_hum, currentData.pm25);
    Serial.println("============================");
    lastDisplay = millis();
  }
}

// === UPDATE LCD ===
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("AQ:");
  lcd.print((int)currentData.mq135_air_quality);
  lcd.print("% CO2:");
  lcd.print((int)currentData.mq135_co2_est);
  
  lcd.setCursor(0,1);
  lcd.print("CO:");
  lcd.print((int)currentData.co_ppm);
  lcd.print(" CH4:");
  lcd.print((int)currentData.ch4_ppm);
}

// === CHECK ALERTS ===
void checkAlerts() {
  bool alert = (currentData.mq135_air_quality < 30 ||
                currentData.mq135_co2_est > 1500 ||
                currentData.mq135_nh3_est > 20 ||
                currentData.pm25 > 150 ||
                currentData.co_ppm > 50 ||
                currentData.o3_ppm > 5 ||
                currentData.ch4_ppm > 500 ||
                currentData.lpg_ppm > 300 ||
                currentData.dht_temp > 35 ||
                currentData.dht_hum > 85 ||
                currentData.ir_temp > 40);
  
  if (alert) {
    digitalWrite(DANGER_LED, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
  } else {
    digitalWrite(DANGER_LED, LOW);
    digitalWrite(BUZZER, LOW);
  }
}

// === SETUP ===
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("🏭 MINE SAFETY SYSTEM WITH GRAPHS & LOGGING");
  Serial.println("============================================");
  
  // Initialize hardware
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  pinMode(DANGER_LED, OUTPUT);
  digitalWrite(DANGER_LED, LOW);
  
  pinMode(MQ135_PIN, INPUT);
  pinMode(MQ7_PIN, INPUT);
  pinMode(MQ131_PIN, INPUT);
  pinMode(MQ4_PIN, INPUT);
  pinMode(LPG_PIN, INPUT);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);

  // Initialize I2C
  Wire.begin(21, 22);
  lcd.init(); 
  lcd.backlight();
  dht.begin();
  mlx.begin();
  dustSensor.begin();
  
  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("❌ SD Card initialization failed!");
    lcd.clear();
    lcd.print("SD Card Error");
  } else {
    Serial.println("✅ SD Card initialized.");
    // Create header in CSV file
    File dataFile = SD.open("/sensor_log.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Timestamp,CO(ppm),O3(ppm),CH4(ppm),LPG(ppm),CO2(ppm),AQI(%),Temp(C),Hum(%),PM2.5");
      dataFile.close();
    }
  }
  
  lcd.clear();
  lcd.print("Graphs+Logging");
  lcd.setCursor(0,1);
  lcd.print("SD: Ready");
  delay(3000);
  
  // Start AP Mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("📶 AP IP: "); Serial.println(WiFi.softAPIP());
  
  // Web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/history", handleHistory);
  server.on("/download", handleDownload);
  server.on("/graph", handleGraphPage);
  server.begin();
  
  lcd.clear();
  lcd.print("IP:192.168.4.1");
  lcd.setCursor(0,1);
  lcd.print("Graphs Active");
}

// === MAIN LOOP ===
void loop() {
  server.handleClient();
  
  static unsigned long lastRead = 0;
  static unsigned long lastLog = 0;
  
  if (millis() - lastRead > 2000) {
    readSensors();
    updateLCD();
    checkAlerts();
    lastRead = millis();
  }
  
  if (millis() - lastLog > LOG_INTERVAL) {
    logToSD(currentData);
    addToHistory(currentData);
    lastLog = millis();
  }
}

// === WEB SERVER HANDLERS ===

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🏭 Mine Safety Monitoring</title>
    <style>
        body { font-family: Arial; background: #1a1a2e; color: white; padding: 20px; }
        .container { max-width: 1000px; margin: 0 auto; }
        .header { text-align: center; margin-bottom: 30px; }
        .header h1 { color: #00b4d8; }
        .header a { color: #ff9e00; text-decoration: none; margin-top: 10px; display: inline-block; }
        .dashboard { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
        .card { background: #16213e; padding: 20px; border-radius: 10px; border-left: 5px solid; }
        .card-air { border-left-color: #00b4d8; }
        .card-gas { border-left-color: #ff9e00; }
        .card-env { border-left-color: #4CAF50; }
        .sensor-value { font-size: 2.5em; font-weight: bold; margin: 10px 0; }
        .sensor-unit { color: #90a4ae; }
        .status { display: inline-block; padding: 5px 15px; border-radius: 20px; font-size: 0.9em; }
        .good { background: #4CAF50; }
        .warning { background: #FF9800; }
        .danger { background: #F44336; }
        .alert-bar { 
            position: fixed; top: 20px; right: 20px; 
            padding: 12px 20px; border-radius: 10px;
            font-weight: bold; z-index: 1000;
        }
        .alert-safe { background: #4CAF50; }
        .alert-warning { background: #FF9800; animation: pulse 2s infinite; }
        .alert-danger { background: #F44336; animation: pulse 1s infinite; }
        @keyframes pulse { 0% { opacity: 1; } 50% { opacity: 0.7; } 100% { opacity: 1; } }
        .info-row { display: flex; justify-content: space-between; margin: 10px 0; }
        .calibration-note { 
            background: #2d3047; padding: 10px; border-radius: 5px; 
            margin-top: 15px; font-size: 0.85em; color: #90a4ae;
            border-left: 3px solid #ff9e00;
        }
        .graph-link {
            display: block;
            text-align: center;
            margin: 20px auto;
            padding: 15px;
            background: #00b4d8;
            color: white;
            text-decoration: none;
            border-radius: 10px;
            width: 200px;
            font-weight: bold;
        }
        .graph-link:hover {
            background: #0096c7;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🏭 Mine Safety Monitoring</h1>
            <p>MC-135: Reverse Logic | Gas Sensors: Baseline Calibrated</p>
            <a href="/graph" class="graph-link">📊 VIEW GRAPHS</a>
        </div>
        
        <div class="alert-bar alert-safe" id="alertBar">✅ SYSTEM NORMAL</div>
        
        <div class="dashboard">
            <div class="card card-air">
                <h2>🍃 AIR QUALITY (MC-135)</h2>
                <div class="sensor-value" id="aqiValue">--%</div>
                <div class="sensor-unit">Air Quality Index</div>
                <div class="status good" id="aqiStatus">GOOD</div>
                
                <div class="info-row">
                    <div>CO₂ Level: <span id="co2Value">--</span> ppm</div>
                    <div>NH₃: <span id="nh3Value">--</span> ppm</div>
                </div>
                <div class="info-row">
                    <div>Raw ADC: <span id="mq135Raw">--</span></div>
                    <div>Voltage: <span id="mq135Voltage">--</span> V</div>
                </div>
                <div style="margin-top: 15px; color: #90a4ae; font-size: 0.9em;">
                    Higher ADC = Cleaner air | Max: 3.04V output
                </div>
            </div>
            
            <div class="card card-gas">
                <h2>⚠️ GAS SENSORS</h2>
                <div class="info-row">
                    <div>CO (MC-7): <span id="coValue">--</span> ppm</div>
                    <div class="status good" id="coStatus">SAFE</div>
                </div>
                <div class="info-row">
                    <div>Ozone (MC-131): <span id="o3Value">--</span> ppm</div>
                    <div class="status good" id="o3Status">SAFE</div>
                </div>
                <div class="info-row">
                    <div>Methane (MC-4): <span id="ch4Value">--</span> ppm</div>
                    <div class="status good" id="ch4Status">SAFE</div>
                </div>
                <div class="info-row">
                    <div>LPG Gas: <span id="lpgValue">--</span> ppm</div>
                    <div class="status good" id="lpgStatus">SAFE</div>
                </div>
                
                <div class="calibration-note">
                    <strong>Calibration Applied:</strong><br>
                    Baseline subtracted: CO-17ppm, O3-3ppm, CH4-39ppm, LPG-88ppm<br>
                    Clean air = 0 ppm | Only actual gas shown
                </div>
            </div>
            
            <div class="card card-env">
                <h2>🌡️ ENVIRONMENT</h2>
                <div class="info-row">
                    <div>Temperature: <span id="tempValue">--</span> °C</div>
                    <div class="status good" id="tempStatus">NORMAL</div>
                </div>
                <div class="info-row">
                    <div>Humidity: <span id="humValue">--</span> %</div>
                    <div class="status good" id="humStatus">NORMAL</div>
                </div>
                <div class="info-row">
                    <div>PM2.5 Dust: <span id="pmValue">--</span> µg/m³</div>
                    <div class="status good" id="pmStatus">CLEAN</div>
                </div>
                <div class="info-row">
                    <div>IR Temp: <span id="irTempValue">--</span> °C</div>
                    <div>Ambient: <span id="ambTempValue">--</span> °C</div>
                </div>
            </div>
        </div>
        
        <div style="text-align: center; margin-top: 20px;">
            <a href="/download" style="color: #00b4d8; text-decoration: none;">
                ⬇️ Download CSV Data
            </a>
        </div>
        
        <div style="text-align: center; margin-top: 30px; color: #90a4ae; font-size: 0.9em;">
            Last update: <span id="updateTime">--:--:--</span> | 
            System: <span id="systemStatus">Active</span>
        </div>
    </div>
    
    <script>
        function updateDashboard(data) {
            // Air Quality
            const aqi = data.mq135_air_quality || 0;
            document.getElementById('aqiValue').textContent = aqi.toFixed(0) + '%';
            document.getElementById('co2Value').textContent = (data.mq135_co2_est || 0).toFixed(0);
            document.getElementById('nh3Value').textContent = (data.mq135_nh3_est || 0).toFixed(1);
            document.getElementById('mq135Raw').textContent = data.mq135_raw || 0;
            document.getElementById('mq135Voltage').textContent = (data.mq135_voltage || 0).toFixed(2);
            
            // AQI Status
            const aqiStatus = document.getElementById('aqiStatus');
            if (aqi >= 70) { aqiStatus.textContent = 'GOOD'; aqiStatus.className = 'status good'; }
            else if (aqi >= 30) { aqiStatus.textContent = 'MODERATE'; aqiStatus.className = 'status warning'; }
            else { aqiStatus.textContent = 'POOR'; aqiStatus.className = 'status danger'; }
            
            // Gas Sensors (CALIBRATED)
            document.getElementById('coValue').textContent = (data.co_ppm || 0).toFixed(0);
            document.getElementById('o3Value').textContent = (data.o3_ppm || 0).toFixed(1);
            document.getElementById('ch4Value').textContent = (data.ch4_ppm || 0).toFixed(0);
            document.getElementById('lpgValue').textContent = (data.lpg_ppm || 0).toFixed(0);
            
            // Gas Statuses
            function updateGasStatus(id, value, safe, warn) {
                const element = document.getElementById(id + 'Status');
                if (value < safe) { element.textContent = 'SAFE'; element.className = 'status good'; }
                else if (value < warn) { element.textContent = 'WARNING'; element.className = 'status warning'; }
                else { element.textContent = 'DANGER'; element.className = 'status danger'; }
            }
            
            updateGasStatus('co', data.co_ppm || 0, 10, 50);
            updateGasStatus('o3', data.o3_ppm || 0, 1, 5);
            updateGasStatus('ch4', data.ch4_ppm || 0, 100, 500);
            updateGasStatus('lpg', data.lpg_ppm || 0, 50, 300);
            
            // Environment
            document.getElementById('tempValue').textContent = (data.dht_temp || 0).toFixed(1);
            document.getElementById('humValue').textContent = (data.dht_hum || 0).toFixed(0);
            document.getElementById('pmValue').textContent = (data.pm25 || 0).toFixed(0);
            document.getElementById('irTempValue').textContent = (data.ir_temp || 0).toFixed(1);
            document.getElementById('ambTempValue').textContent = (data.ambient_temp || 0).toFixed(1);
            
            // Environment Status
            function updateEnvStatus(id, value, good, warn) {
                const element = document.getElementById(id + 'Status');
                if (value <= good) { element.textContent = id === 'pm' ? 'CLEAN' : 'NORMAL'; element.className = 'status good'; }
                else if (value <= warn) { element.textContent = id === 'pm' ? 'MODERATE' : 'HIGH'; element.className = 'status warning'; }
                else { element.textContent = id === 'pm' ? 'POOR' : 'DANGER'; element.className = 'status danger'; }
            }
            
            updateEnvStatus('temp', data.dht_temp || 0, 30, 35);
            updateEnvStatus('hum', data.dht_hum || 0, 70, 85);
            updateEnvStatus('pm', data.pm25 || 0, 50, 100);
            
            // Alert Bar
            const alertBar = document.getElementById('alertBar');
            if (aqi < 30 || (data.mq135_co2_est || 0) > 1500 || data.dht_temp > 35) {
                alertBar.textContent = '🚨 DANGER - EVACUATE!';
                alertBar.className = 'alert-bar alert-danger';
                document.getElementById('systemStatus').textContent = 'DANGER';
            } else if (aqi < 70 || data.dht_temp > 30) {
                alertBar.textContent = '⚠️ WARNING - Monitor';
                alertBar.className = 'alert-bar alert-warning';
                document.getElementById('systemStatus').textContent = 'WARNING';
            } else {
                alertBar.textContent = '✅ SYSTEM NORMAL';
                alertBar.className = 'alert-bar alert-safe';
                document.getElementById('systemStatus').textContent = 'NORMAL';
            }
            
            // Update time
            const now = new Date();
            document.getElementById('updateTime').textContent = now.toLocaleTimeString();
            
            // Update title
            document.title = aqi < 30 ? '🚨 DANGER - Mine' : 
                            aqi < 70 ? '⚠️ Warning - Mine' : 
                            '🏭 Mine Safety';
        }
        
        function fetchData() {
            fetch('/data')
                .then(r => r.json())
                .then(updateDashboard)
                .catch(e => console.log('Error:', e));
        }
        
        setInterval(fetchData, 2000);
        fetchData();
    </script>
</body>
</html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleData() {
  DynamicJsonDocument doc(1024);
  
  doc["pm25"] = currentData.pm25;
  doc["dht_temp"] = currentData.dht_temp;
  doc["dht_hum"] = currentData.dht_hum;
  doc["ir_temp"] = currentData.ir_temp;
  doc["ambient_temp"] = currentData.ambient_temp;
  
  doc["co_ppm"] = currentData.co_ppm;
  doc["o3_ppm"] = currentData.o3_ppm;
  doc["ch4_ppm"] = currentData.ch4_ppm;
  doc["lpg_ppm"] = currentData.lpg_ppm;
  
  doc["mq135_raw"] = currentData.mq135_raw;
  doc["mq135_voltage"] = currentData.mq135_voltage;
  doc["mq135_air_quality"] = currentData.mq135_air_quality;
  doc["mq135_co2_est"] = currentData.mq135_co2_est;
  doc["mq135_nh3_est"] = currentData.mq135_nh3_est;
  
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

void handleHistory() {
  int minutes = server.hasArg("minutes") ? server.arg("minutes").toInt() : 60;
  
  DynamicJsonDocument doc(4096);
  JsonArray timestamps = doc.createNestedArray("timestamps");
  JsonArray co_values = doc.createNestedArray("co_values");
  JsonArray o3_values = doc.createNestedArray("o3_values");
  JsonArray ch4_values = doc.createNestedArray("ch4_values");
  JsonArray lpg_values = doc.createNestedArray("lpg_values");
  JsonArray co2_values = doc.createNestedArray("co2_values");
  JsonArray aqi_values = doc.createNestedArray("aqi_values");
  JsonArray temp_values = doc.createNestedArray("temp_values");
  
  unsigned long cutoffTime = millis() - (minutes * 60 * 1000);
  int startIdx = 0;
  int dataCount = historicalData.buffer_full ? MAX_DATA_POINTS : historicalData.data_index;
  
  if (historicalData.buffer_full) {
    startIdx = historicalData.data_index;
  }
  
  for (int i = 0; i < dataCount; i++) {
    int idx = (startIdx + i) % MAX_DATA_POINTS;
    if (historicalData.timestamps[idx] >= cutoffTime) {
      timestamps.add(historicalData.timestamps[idx]);
      co_values.add(historicalData.co_values[idx]);
      o3_values.add(historicalData.o3_values[idx]);
      ch4_values.add(historicalData.ch4_values[idx]);
      lpg_values.add(historicalData.lpg_values[idx]);
      co2_values.add(historicalData.co2_values[idx]);
      aqi_values.add(historicalData.aqi_values[idx]);
      temp_values.add(historicalData.temp_values[idx]);
    }
  }
  
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

void handleDownload() {
  if (!SD.begin(SD_CS)) {
    server.send(404, "text/plain", "SD card not found");
    return;
  }
  
  File dataFile = SD.open("/sensor_log.csv");
  if (!dataFile) {
    server.send(404, "text/plain", "Log file not found");
    return;
  }
  
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=sensor_data.csv");
  server.streamFile(dataFile, "text/csv");
  dataFile.close();
}

void handleGraphPage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>📊 Sensor Graphs</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #1a1a2e;
            color: white;
            padding: 20px;
            margin: 0;
        }
        .header {
            text-align: center;
            padding: 20px;
            background: #16213e;
            border-radius: 10px;
            margin-bottom: 30px;
        }
        .header h1 {
            color: #00b4d8;
            margin: 0;
        }
        .header a {
            color: #ff9e00;
            text-decoration: none;
            margin-top: 10px;
            display: inline-block;
        }
        .charts-container {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(500px, 1fr));
            gap: 25px;
            margin-bottom: 40px;
        }
        .chart-card {
            background: #16213e;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
        }
        .chart-title {
            color: #00b4d8;
            margin-bottom: 15px;
            font-size: 1.2em;
            border-bottom: 2px solid #00b4d8;
            padding-bottom: 5px;
        }
        .chart-wrapper {
            position: relative;
            height: 300px;
            width: 100%;
        }
        .controls {
            display: flex;
            justify-content: center;
            gap: 15px;
            margin-bottom: 30px;
            flex-wrap: wrap;
        }
        .time-btn {
            padding: 10px 20px;
            background: #2d3047;
            border: none;
            color: white;
            border-radius: 5px;
            cursor: pointer;
            transition: background 0.3s;
        }
        .time-btn:hover {
            background: #00b4d8;
        }
        .time-btn.active {
            background: #00b4d8;
        }
        .value-display {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-top: 30px;
        }
        .value-box {
            background: #2d3047;
            padding: 15px;
            border-radius: 8px;
            text-align: center;
        }
        .value-label {
            color: #90a4ae;
            font-size: 0.9em;
            margin-bottom: 5px;
        }
        .value-number {
            font-size: 1.8em;
            font-weight: bold;
            color: #00b4d8;
        }
        .value-unit {
            color: #ff9e00;
            font-size: 0.9em;
        }
        .legend {
            display: flex;
            justify-content: center;
            gap: 20px;
            margin-top: 10px;
            flex-wrap: wrap;
        }
        .legend-item {
            display: flex;
            align-items: center;
            gap: 5px;
        }
        .legend-color {
            width: 15px;
            height: 15px;
            border-radius: 3px;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>📊 Sensor Graphs - Last 60 Minutes</h1>
        <a href="/">← Back to Dashboard</a>
    </div>
    
    <div class="controls">
        <button class="time-btn active" onclick="changeTimeRange(60)">Last 60 Min</button>
        <button class="time-btn" onclick="changeTimeRange(30)">Last 30 Min</button>
        <button class="time-btn" onclick="changeTimeRange(15)">Last 15 Min</button>
        <button class="time-btn" onclick="changeTimeRange(5)">Last 5 Min</button>
    </div>
    
    <div class="charts-container">
        <div class="chart-card">
            <div class="chart-title">Gas Concentrations (ppm)</div>
            <div class="chart-wrapper">
                <canvas id="gasChart"></canvas>
            </div>
            <div class="legend">
                <div class="legend-item"><div class="legend-color" style="background-color: #FF6384;"></div><span>CO</span></div>
                <div class="legend-item"><div class="legend-color" style="background-color: #36A2EB;"></div><span>Ozone</span></div>
                <div class="legend-item"><div class="legend-color" style="background-color: #FFCE56;"></div><span>Methane</span></div>
                <div class="legend-item"><div class="legend-color" style="background-color: #4BC0C0;"></div><span>LPG</span></div>
            </div>
        </div>
        
        <div class="chart-card">
            <div class="chart-title">Air Quality & CO₂</div>
            <div class="chart-wrapper">
                <canvas id="airChart"></canvas>
            </div>
            <div class="legend">
                <div class="legend-item"><div class="legend-color" style="background-color: #9966FF;"></div><span>Air Quality (%)</span></div>
                <div class="legend-item"><div class="legend-color" style="background-color: #FF9F40;"></div><span>CO₂ (ppm)</span></div>
            </div>
        </div>
        
        <div class="chart-card">
            <div class="chart-title">Environment</div>
            <div class="chart-wrapper">
                <canvas id="envChart"></canvas>
            </div>
            <div class="legend">
                <div class="legend-item"><div class="legend-color" style="background-color: #FF6384;"></div><span>Temperature (°C)</span></div>
                <div class="legend-item"><div class="legend-color" style="background-color: #36A2EB;"></div><span>Humidity (%)</span></div>
            </div>
        </div>
        
        <div class="chart-card">
            <div class="chart-title">PM2.5 Dust (µg/m³)</div>
            <div class="chart-wrapper">
                <canvas id="dustChart"></canvas>
            </div>
        </div>
    </div>
    
    <div class="value-display" id="currentValues">
        <!-- Current values will be inserted here -->
    </div>
    
    <div style="text-align: center; margin-top: 40px; color: #90a4ae;">
        <p>Data updates every 20 seconds | Data logged to SD card</p>
        <a href="/download" style="color: #00b4d8; text-decoration: none;">
            ⬇️ Download Full CSV Data
        </a>
    </div>
    
    <script>
        let gasChart, airChart, envChart, dustChart;
        let currentTimeRange = 60;
        
        function initCharts() {
            const gasCtx = document.getElementById('gasChart').getContext('2d');
            gasChart = new Chart(gasCtx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        {
                            label: 'CO (ppm)',
                            data: [],
                            borderColor: '#FF6384',
                            backgroundColor: 'rgba(255, 99, 132, 0.1)',
                            borderWidth: 2,
                            tension: 0.4
                        },
                        {
                            label: 'Ozone (ppm)',
                            data: [],
                            borderColor: '#36A2EB',
                            backgroundColor: 'rgba(54, 162, 235, 0.1)',
                            borderWidth: 2,
                            tension: 0.4
                        },
                        {
                            label: 'Methane (ppm)',
                            data: [],
                            borderColor: '#FFCE56',
                            backgroundColor: 'rgba(255, 206, 86, 0.1)',
                            borderWidth: 2,
                            tension: 0.4
                        },
                        {
                            label: 'LPG (ppm)',
                            data: [],
                            borderColor: '#4BC0C0',
                            backgroundColor: 'rgba(75, 192, 192, 0.1)',
                            borderWidth: 2,
                            tension: 0.4
                        }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            beginAtZero: true,
                            grid: { color: 'rgba(255,255,255,0.1)' },
                            ticks: { color: '#90a4ae' }
                        },
                        x: {
                            grid: { color: 'rgba(255,255,255,0.1)' },
                            ticks: { color: '#90a4ae' }
                        }
                    },
                    plugins: {
                        legend: { display: false }
                    }
                }
            });
            
            const airCtx = document.getElementById('airChart').getContext('2d');
            airChart = new Chart(airCtx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        {
                            label: 'Air Quality (%)',
                            data: [],
                            borderColor: '#9966FF',
                            backgroundColor: 'rgba(153, 102, 255, 0.1)',
                            borderWidth: 2,
                            yAxisID: 'y',
                            tension: 0.4
                        },
                        {
                            label: 'CO₂ (ppm)',
                            data: [],
                            borderColor: '#FF9F40',
                            backgroundColor: 'rgba(255, 159, 64, 0.1)',
                            borderWidth: 2,
                            yAxisID: 'y1',
                            tension: 0.4
                        }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            type: 'linear',
                            display: true,
                            position: 'left',
                            min: 0,
                            max: 100,
                            grid: { color: 'rgba(255,255,255,0.1)' },
                            ticks: { color: '#90a4ae' }
                        },
                        y1: {
                            type: 'linear',
                            display: true,
                            position: 'right',
                            min: 400,
                            max: 2000,
                            grid: { drawOnChartArea: false },
                            ticks: { color: '#90a4ae' }
                        },
                        x: {
                            grid: { color: 'rgba(255,255,255,0.1)' },
                            ticks: { color: '#90a4ae' }
                        }
                    },
                    plugins: {
                        legend: { display: false }
                    }
                }
            });
            
            const envCtx = document.getElementById('envChart').getContext('2d');
            envChart = new Chart(envCtx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        {
                            label: 'Temperature (°C)',
                            data: [],
                            borderColor: '#FF6384',
                            backgroundColor: 'rgba(255, 99, 132, 0.1)',
                            borderWidth: 2,
                            tension: 0.4
                        },
                        {
                            label: 'Humidity (%)',
                            data: [],
                            borderColor: '#36A2EB',
                            backgroundColor: 'rgba(54, 162, 235, 0.1)',
                            borderWidth: 2,
                            tension: 0.4
                        }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            grid: { color: 'rgba(255,255,255,0.1)' },
                            ticks: { color: '#90a4ae' }
                        },
                        x: {
                            grid: { color: 'rgba(255,255,255,0.1)' },
                            ticks: { color: '#90a4ae' }
                    }
                },
                    plugins: {
                        legend: { display: false }
                    }
                }
            });
            
            const dustCtx = document.getElementById('dustChart').getContext('2d');
            dustChart = new Chart(dustCtx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [{
                        label: 'PM2.5 (µg/m³)',
                        data: [],
                        borderColor: '#4CAF50',
                        backgroundColor: 'rgba(76, 175, 80, 0.1)',
                        borderWidth: 2,
                        tension: 0.4
                    }]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            beginAtZero: true,
                            grid: { color: 'rgba(255,255,255,0.1)' },
                            ticks: { color: '#90a4ae' }
                        },
                        x: {
                            grid: { color: 'rgba(255,255,255,0.1)' },
                            ticks: { color: '#90a4ae' }
                        }
                    }
                }
            });
        }
        
        function changeTimeRange(minutes) {
            currentTimeRange = minutes;
            document.querySelectorAll('.time-btn').forEach(btn => btn.classList.remove('active'));
            event.target.classList.add('active');
            updateCharts();
        }
        
        function updateCharts() {
            fetch('/history?minutes=' + currentTimeRange)
                .then(r => r.json())
                .then(data => {
                    if (!data.timestamps) return;
                    
                    const labels = data.timestamps.map(t => {
                        const date = new Date(t);
                        return date.toLocaleTimeString([], {hour: '2-digit', minute:'2-digit'});
                    });
                    
                    gasChart.data.labels = labels;
                    gasChart.data.datasets[0].data = data.co_values;
                    gasChart.data.datasets[1].data = data.o3_values;
                    gasChart.data.datasets[2].data = data.ch4_values;
                    gasChart.data.datasets[3].data = data.lpg_values;
                    gasChart.update();
                    
                    airChart.data.labels = labels;
                    airChart.data.datasets[0].data = data.aqi_values;
                    airChart.data.datasets[1].data = data.co2_values;
                    airChart.update();
                    
                    envChart.data.labels = labels;
                    envChart.data.datasets[0].data = data.temp_values;
                    envChart.data.datasets[1].data = new Array(data.temp_values.length).fill(70);
                    envChart.update();
                    
                    dustChart.data.labels = labels;
                    dustChart.data.datasets[0].data = data.temp_values;
                    dustChart.update();
                    
                    updateCurrentValues(data);
                });
        }
        
        function updateCurrentValues(historyData) {
            if (historyData.co_values && historyData.co_values.length > 0) {
                const lastIdx = historyData.co_values.length - 1;
                const html = `
                    <div class="value-box">
                        <div class="value-label">CO</div>
                        <div class="value-number">${historyData.co_values[lastIdx].toFixed(1)}</div>
                        <div class="value-unit">ppm</div>
                    </div>
                    <div class="value-box">
                        <div class="value-label">Ozone</div>
                        <div class="value-number">${historyData.o3_values[lastIdx].toFixed(2)}</div>
                        <div class="value-unit">ppm</div>
                    </div>
                    <div class="value-box">
                        <div class="value-label">Methane</div>
                        <div class="value-number">${historyData.ch4_values[lastIdx].toFixed(1)}</div>
                        <div class="value-unit">ppm</div>
                    </div>
                    <div class="value-box">
                        <div class="value-label">LPG</div>
                        <div class="value-number">${historyData.lpg_values[lastIdx].toFixed(1)}</div>
                        <div class="value-unit">ppm</div>
                    </div>
                    <div class="value-box">
                        <div class="value-label">CO₂</div>
                        <div class="value-number">${historyData.co2_values[lastIdx].toFixed(0)}</div>
                        <div class="value-unit">ppm</div>
                    </div>
                    <div class="value-box">
                        <div class="value-label">Air Quality</div>
                        <div class="value-number">${historyData.aqi_values[lastIdx].toFixed(0)}</div>
                        <div class="value-unit">%</div>
                    </div>
                    <div class="value-box">
                        <div class="value-label">Temperature</div>
                        <div class="value-number">${historyData.temp_values[lastIdx].toFixed(1)}</div>
                        <div class="value-unit">°C</div>
                    </div>
                `;
                document.getElementById('currentValues').innerHTML = html;
            }
        }
        
        initCharts();
        updateCharts();
        setInterval(updateCharts, 10000);
    </script>
</body>
</html>
  )rawliteral";
  server.send(200, "text/html", html);
}
