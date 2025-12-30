#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// WiFi bilgileri
const char* ssid = "tameresp";
const char* password = "tameresp";

// Servo motor tanımlamaları
Servo panServo;  // Yatay hareket
Servo tiltServo; // Dikey hareket

// Pin tanımlamaları
const int PAN_PIN = 41;
const int TILT_PIN = 42;

// MG995 için mikrosaniye değerleri
const int SERVO_MIN_US = 1000;   // 0 derece
const int SERVO_MAX_US = 2000;  // 180 derece
const int SERVO_CENTER_US = 1500; // 90 derece

// Servo pozisyonları (float derece olarak)
float panPosition = 114.0;   // Merkez pozisyon (derece)
float tiltPosition = 14.0;  // Merkez pozisyon (derece)



// Web server
WebServer server(80);

// Derece -> Mikrosaniye dönüşüm fonksiyonu
int degreesToMicroseconds(float degrees) {
  // 0-180 derece aralığını 500-2500 mikrosaniye aralığına map'le
  degrees = constrain(degrees, 0, 180);
  return (int)(SERVO_MIN_US + (degrees / 180.0) * (SERVO_MAX_US - SERVO_MIN_US));
}
// Mikrosaniye olarak pozisyonlar (hassas kontrol için)
int panMicros = SERVO_CENTER_US;
int tiltMicros = degreesToMicroseconds(150.0);

// Mikrosaniye -> Derece dönüşüm fonksiyonu
float microsecondsToDegrees(int microseconds) {
  microseconds = constrain(microseconds, SERVO_MIN_US, SERVO_MAX_US);
  return ((float)(microseconds - SERVO_MIN_US) / (SERVO_MAX_US - SERVO_MIN_US)) * 180.0;
}

// Hassas servo hareketi (mikrosaniye ile)
void setServoMicros(Servo &servo, int microseconds) {
  microseconds = constrain(microseconds, SERVO_MIN_US, SERVO_MAX_US);
  servo.writeMicroseconds(microseconds);
}

// Hassas servo hareketi (float derece ile)
void setServoDegrees(Servo &servo, float degrees) {
  int microseconds = degreesToMicroseconds(degrees);
  servo.writeMicroseconds(microseconds);
}

void setup() {
  Serial.begin(115200);
  
  // Servo motorları başlat
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  
  panServo.setPeriodHertz(50); // MG995 için standart 50Hz
  tiltServo.setPeriodHertz(50);
  
  panServo.attach(PAN_PIN, SERVO_MIN_US, SERVO_MAX_US);
  tiltServo.attach(TILT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  
  // Başlangıç pozisyonu (merkez)
  setServoDegrees(panServo, panPosition);
  setServoDegrees(tiltServo, tiltPosition);
  
  // WiFi bağlantısı
  WiFi.begin(ssid, password);
  Serial.print("WiFi'ye bağlanıyor");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi bağlandı!");
  Serial.print("IP adresi: ");
  Serial.println(WiFi.localIP());
  
  // Web server rotaları
  server.on("/", handleRoot);
  server.on("/control", HTTP_POST, handleControl);
  server.on("/control_micros", HTTP_POST, handleControlMicros);
  server.on("/move", HTTP_GET, handleMove);
  server.on("/center", HTTP_GET, handleCenter);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/calibrate", HTTP_GET, handleCalibrate);
  
  server.begin();
  Serial.println("Web server başlatıldı");
  Serial.println("MG995 Hassas Kontrol Sistemi Aktif");
  Serial.println("Mikrosaniye aralığı: 500-2500μs");
}

void loop() {
  server.handleClient();
}

// Ana sayfa
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>MG995 Hassas Pan-Tilt Kontrolu</title>";
  html += "<meta charset='UTF-8'>";
  html += "<style>body{font-family:Arial;text-align:center;margin:50px;background:#f0f0f0;}";
  html += ".control-panel{margin:20px;padding:20px;background:white;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1);}";
  html += "button{padding:15px 25px;margin:5px;font-size:16px;background:#4CAF50;color:white;border:none;border-radius:5px;cursor:pointer;}";
  html += "button:hover{background:#45a049;}";
  html += ".position{font-size:18px;margin:20px;padding:15px;background:#e8f5e9;border-radius:5px;}";
  html += ".precision-control{margin:20px;padding:15px;background:#fff3e0;border-radius:5px;}";
  html += "input[type='number']{padding:8px;margin:5px;width:100px;border:1px solid #ddd;border-radius:3px;}";
  html += ".info{font-size:14px;color:#666;margin:10px;}";
  html += "</style></head><body>";
  html += "<h1>🎯 MG995 Hassas Pan-Tilt Kontrolu</h1>";
  
  // Pozisyon bilgileri
  html += "<div class='position'>";
  html += "<p><strong>Mevcut Pozisyon:</strong></p>";
  html += "<p>Pan: <span id='panPos'>90.0</span>° (<span id='panMicros'>1500</span>μs)</p>";
  html += "<p>Tilt: <span id='tiltPos'>150.0</span>° (<span id='tiltMicros'>1944</span>μs)</p>";
  html += "</div>";
  
  // Hassas kontrol paneli
  html += "<div class='precision-control'>";
  html += "<h3>⚡ Hassas Pozisyon Kontrolü</h3>";
  html += "<p>Pan (derece): <input type='number' id='panDegreeInput' min='0' max='180' step='0.1' value='90.0'>";
  html += "<button onclick='setPanDegrees()'>Ayarla</button></p>";
  html += "<p>Tilt (derece): <input type='number' id='tiltDegreeInput' min='0' max='180' step='0.1' value='150.0'>";
  html += "<button onclick='setTiltDegrees()'>Ayarla</button></p>";
  html += "<hr>";
  html += "<p>Pan (μs): <input type='number' id='panMicrosInput' min='500' max='2500' step='1' value='1500'>";
  html += "<button onclick='setPanMicros()'>Ayarla</button></p>";
  html += "<p>Tilt (μs): <input type='number' id='tiltMicrosInput' min='500' max='2500' step='1' value='1944'>";
  html += "<button onclick='setTiltMicros()'>Ayarla</button></p>";
  html += "</div>";
  
  // Standart kontroller
  html += "<div class='control-panel'>";
  html += "<h3>🎮 Manuel Kontrol</h3>";
  html += "<button onclick='moveCamera(\"up\")'>↑ YUKARI</button><br>";
  html += "<button onclick='moveCamera(\"left\")'>← SOL</button>";
  html += "<button onclick='moveCamera(\"center\")'>⊕ MERKEZ</button>";
  html += "<button onclick='moveCamera(\"right\")'>SAĞ →</button><br>";
  html += "<button onclick='moveCamera(\"down\")'>↓ AŞAĞI</button>";
  html += "</div>";
  
  // Kalibrasyon
  html += "<div class='control-panel'>";
  html += "<h3>🔧 Kalibrasyon</h3>";
  html += "<button onclick='testRange()'>Test Aralığı</button>";
  html += "<button onclick='calibrate()'>Kalibre Et</button>";
  html += "</div>";
  
  html += "<div class='info'>MG995 Servo: 500-2500μs PWM aralığı | 50Hz frekans</div>";
  
  // JavaScript
  html += "<script>";
  
  // Hassas kontrol fonksiyonları
  html += "function setPanDegrees(){";
  html += "var deg=document.getElementById('panDegreeInput').value;";
  html += "fetch('/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},";
  html += "body:'pan='+deg+'&tilt='+document.getElementById('tiltDegreeInput').value})";
  html += ".then(response=>response.json()).then(data=>updateDisplay(data));}";
  
  html += "function setTiltDegrees(){";
  html += "var deg=document.getElementById('tiltDegreeInput').value;";
  html += "fetch('/control',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},";
  html += "body:'pan='+document.getElementById('panDegreeInput').value+'&tilt='+deg})";
  html += ".then(response=>response.json()).then(data=>updateDisplay(data));}";
  
  html += "function setPanMicros(){";
  html += "var us=document.getElementById('panMicrosInput').value;";
  html += "fetch('/control_micros',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},";
  html += "body:'pan_us='+us+'&tilt_us='+document.getElementById('tiltMicrosInput').value})";
  html += ".then(response=>response.json()).then(data=>updateDisplay(data));}";
  
  html += "function setTiltMicros(){";
  html += "var us=document.getElementById('tiltMicrosInput').value;";
  html += "fetch('/control_micros',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},";
  html += "body:'pan_us='+document.getElementById('panMicrosInput').value+'&tilt_us='+us})";
  html += ".then(response=>response.json()).then(data=>updateDisplay(data));}";
  
  // Hareket fonksiyonları
  html += "function moveCamera(direction){";
  html += "fetch('/move?dir='+direction).then(response=>response.json()).then(data=>updateDisplay(data));}";
  
  html += "function calibrate(){";
  html += "fetch('/calibrate').then(response=>response.json()).then(data=>alert('Kalibrasyon: '+JSON.stringify(data)));}";
  
  html += "function testRange(){";
  html += "alert('500μs (0°) -> 1500μs (90°) -> 2500μs (180°) test ediliyor...');";
  html += "setTimeout(()=>setPanMicros(500),0);";
  html += "setTimeout(()=>setPanMicros(1500),2000);";
  html += "setTimeout(()=>setPanMicros(2500),4000);";
  html += "setTimeout(()=>setPanMicros(1500),6000);}";
  
  // Display güncelleme
  html += "function updateDisplay(data){";
  html += "if(data.pan!==undefined){";
  html += "document.getElementById('panPos').textContent=data.pan.toFixed(1);";
  html += "document.getElementById('panDegreeInput').value=data.pan.toFixed(1);";
  html += "document.getElementById('panMicros').textContent=data.pan_us;";
  html += "document.getElementById('panMicrosInput').value=data.pan_us;}";
  html += "if(data.tilt!==undefined){";
  html += "document.getElementById('tiltPos').textContent=data.tilt.toFixed(1);";
  html += "document.getElementById('tiltDegreeInput').value=data.tilt.toFixed(1);";
  html += "document.getElementById('tiltMicros').textContent=data.tilt_us;";
  html += "document.getElementById('tiltMicrosInput').value=data.tilt_us;}}";
  
  // Otomatik güncelleme
  html += "function updatePosition(){";
  html += "fetch('/status').then(response=>response.json()).then(data=>updateDisplay(data));}";
  html += "setInterval(updatePosition,1000);";
  
  html += "</script></body></html>";
  
  server.send(200, "text/html; charset=utf-8", html);
}

// Hareket kontrolü (iyileştirilmiş hassasiyet)
void handleMove() {
  String direction = server.arg("dir");
  float stepSize = 0.2; // Her harekette 0.5 derece (hassas hareket)
  
  if (direction == "left") {
    panPosition = constrain(panPosition - stepSize, 0, 180);
  }
  else if (direction == "right") {
    panPosition = constrain(panPosition + stepSize, 0, 180);
  }
  else if (direction == "up") {
    tiltPosition = constrain(tiltPosition + stepSize, 0, 180);
  }
  else if (direction == "down") {
    tiltPosition = constrain(tiltPosition - stepSize, 0, 180);
  }
  else if (direction == "center") {
    panPosition = 90.0;
    tiltPosition = 90.0;
  }
  
  // Servoya yaz
  panMicros = degreesToMicroseconds(panPosition);
  tiltMicros = degreesToMicroseconds(tiltPosition);
  setServoMicros(panServo, panMicros);
  setServoMicros(tiltServo, tiltMicros);
  
  // JSON yanıt
  String response = "{\"pan\":" + String(panPosition, 1) + 
                   ",\"tilt\":" + String(tiltPosition, 1) +
                   ",\"pan_us\":" + String(panMicros) +
                   ",\"tilt_us\":" + String(tiltMicros) + "}";
  server.send(200, "application/json", response);
  
  Serial.print("Hareket: " + direction);
  Serial.print(" | Pan: " + String(panPosition, 1) + "° (" + String(panMicros) + "μs)");
  Serial.println(" | Tilt: " + String(tiltPosition, 1) + "° (" + String(tiltMicros) + "μs)");
}

// Merkeze alma
void handleCenter() {
  panPosition = 114.0;
  tiltPosition = 14.0;
  panMicros = SERVO_CENTER_US;
  tiltMicros = SERVO_CENTER_US;
  
  setServoMicros(panServo, panMicros);
  setServoMicros(tiltServo, tiltMicros);
  
  String response = "{\"pan\":" + String(panPosition, 1) + 
                   ",\"tilt\":" + String(tiltPosition, 1) +
                   ",\"pan_us\":" + String(panMicros) +
                   ",\"tilt_us\":" + String(tiltMicros) + "}";
  server.send(200, "application/json", response);
}

// Hassas pozisyon kontrolü (float derece ile)
void handleControl() {
  if (server.hasArg("pan") && server.hasArg("tilt")) {
    float newPan = server.arg("pan").toFloat();
    float newTilt = server.arg("tilt").toFloat();
    
    // Güvenli aralıkta tut
    panPosition = constrain(newPan, 0, 180);
    tiltPosition = constrain(newTilt, 0, 180);
    
    // Mikrosaniye değerlerini hesapla
    panMicros = degreesToMicroseconds(panPosition);
    tiltMicros = degreesToMicroseconds(tiltPosition);
    
    // Servoya yaz
    setServoMicros(panServo, panMicros);
    setServoMicros(tiltServo, tiltMicros);
    
    String response = "{\"status\":\"ok\"" +
                     String(",\"pan\":") + String(panPosition, 1) + 
                     ",\"tilt\":" + String(tiltPosition, 1) +
                     ",\"pan_us\":" + String(panMicros) +
                     ",\"tilt_us\":" + String(tiltMicros) + "}";
    server.send(200, "application/json", response);
    
    Serial.print("Hassas pozisyon - Pan: " + String(panPosition, 2) + "° (" + String(panMicros) + "μs)");
    Serial.println(" | Tilt: " + String(tiltPosition, 2) + "° (" + String(tiltMicros) + "μs)");
  } else {
    server.send(400, "application/json", "{\"error\":\"Missing parameters\"}");
  }
}

// Mikrosaniye tabanlı hassas kontrol (yeni endpoint)
void handleControlMicros() {
  if (server.hasArg("pan_us") && server.hasArg("tilt_us")) {
    int newPanMicros = server.arg("pan_us").toInt();
    int newTiltMicros = server.arg("tilt_us").toInt();
    
    // Güvenli aralıkta tut
    panMicros = constrain(newPanMicros, SERVO_MIN_US, SERVO_MAX_US);
    tiltMicros = constrain(newTiltMicros, SERVO_MIN_US, SERVO_MAX_US);
    
    // Derece değerlerini hesapla
    panPosition = microsecondsToDegrees(panMicros);
    tiltPosition = microsecondsToDegrees(tiltMicros);
    
    // Servoya yaz
    setServoMicros(panServo, panMicros);
    setServoMicros(tiltServo, tiltMicros);
    
    String response = "{\"status\":\"ok\"" +
                     String(",\"pan\":") + String(panPosition, 2) + 
                     ",\"tilt\":" + String(tiltPosition, 2) +
                     ",\"pan_us\":" + String(panMicros) +
                     ",\"tilt_us\":" + String(tiltMicros) + "}";
    server.send(200, "application/json", response);
    
    Serial.print("Mikrosaniye kontrol - Pan: " + String(panMicros) + "μs (" + String(panPosition, 2) + "°)");
    Serial.println(" | Tilt: " + String(tiltMicros) + "μs (" + String(tiltPosition, 2) + "°)");
  } else {
    server.send(400, "application/json", "{\"error\":\"Missing parameters\"}");
  }
}

// Durum bilgisi
void handleStatus() {
  String response = "{\"pan\":" + String(panPosition, 2) + 
                   ",\"tilt\":" + String(tiltPosition, 2) +
                   ",\"pan_us\":" + String(panMicros) +
                   ",\"tilt_us\":" + String(tiltMicros) + "}";
  server.send(200, "application/json", response);
}

// Kalibrasyon fonksiyonu
void handleCalibrate() {
  Serial.println("Kalibrasyon başlıyor...");
  
  // Test: Minimum pozisyon
  setServoMicros(panServo, SERVO_MIN_US);
  setServoMicros(tiltServo, SERVO_MIN_US);
  delay(1000);
  
  // Test: Merkez pozisyon
  setServoMicros(panServo, SERVO_CENTER_US);
  setServoMicros(tiltServo, SERVO_CENTER_US);
  delay(1000);
  
  // Test: Maksimum pozisyon
  setServoMicros(panServo, SERVO_MAX_US);
  setServoMicros(tiltServo, SERVO_MAX_US);
  delay(1000);
  
  // Merkeze dön
  setServoMicros(panServo, SERVO_CENTER_US);
  setServoMicros(tiltServo, SERVO_CENTER_US);
  
  panPosition = 114.0;
  tiltPosition = 14.0;
  panMicros = SERVO_CENTER_US;
  tiltMicros = SERVO_CENTER_US;
  
  String response = "{\"status\":\"calibration_complete\","
                   "\"min_us\":" + String(SERVO_MIN_US) + ","
                   "\"max_us\":" + String(SERVO_MAX_US) + ","
                   "\"center_us\":" + String(SERVO_CENTER_US) + "}";
  
  server.send(200, "application/json", response);
  Serial.println("Kalibrasyon tamamlandı!");
}
