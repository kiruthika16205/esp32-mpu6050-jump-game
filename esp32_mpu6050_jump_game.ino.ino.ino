/*
 * ESP32 Wireless MPU6050 - Jump Game Edition
 * Optimized for ultra-sensitive jump detection
 * 
 * WIRING:
 * MPU6050: VCC→3.3V, GND→GND, SDA→GPIO21, SCL→GPIO22
 * LDR: LDR→GPIO34→59kΩ→GND
 */

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>

// ==================== CONFIGURATION ====================
// Choose your WiFi mode:
#define USE_ACCESS_POINT true  // true = ESP32 creates WiFi, false = Connect to home WiFi

// Access Point Settings (ESP32 creates its own WiFi)
const char* ap_ssid = "MPU6050_Sensor";
const char* ap_password = "sensor123";

// Home WiFi Settings (if USE_ACCESS_POINT = false)
const char* wifi_ssid = "Kiki";        // Change this to your WiFi name
const char* wifi_password = "moykotyonok"; // Change this to your WiFi password

// Hardware pins
int MPU_ADDR = 0x68; // Will auto-detect 0x68 or 0x69
const int LDR_PIN = 34;
// =======================================================

WebSocketsServer webSocket = WebSocketsServer(81);
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 50; // 20Hz update rate for smooth gameplay

bool mpuConnected = false;
bool wifiConnected = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n╔═══════════════════════════════════════════╗");
  Serial.println("║   ESP32 MPU6050 JUMP GAME CONTROLLER     ║");
  Serial.println("╚═══════════════════════════════════════════╝\n");
  
  // ============== TEST 1: I2C INITIALIZATION ==============
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("TEST 1: I2C Bus Initialization");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  Wire.begin(21, 22);
  Wire.setClock(400000); // 400kHz for fast response
  pinMode(LDR_PIN, INPUT);
  delay(100);
  
  Serial.println("✓ I2C initialized on GPIO 21 (SDA), GPIO 22 (SCL)");
  Serial.println("✓ I2C speed: 400 kHz");
  Serial.println();
  
  // ============== TEST 2: I2C DEVICE SCAN ==============
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("TEST 2: Scanning I2C Bus for Devices");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  int deviceCount = 0;
  bool mpuFound = false;
  
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("✓ Device found at address 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      deviceCount++;
      
      if (addr == 0x68 || addr == 0x69) {
        MPU_ADDR = addr;
        mpuFound = true;
        Serial.print("  → This is MPU6050! Using address 0x");
        Serial.println(addr, HEX);
      }
    }
  }
  
  Serial.print("\nTotal I2C devices found: ");
  Serial.println(deviceCount);
  
  if (!mpuFound) {
    Serial.println("\n❌ ERROR: MPU6050 NOT FOUND!");
    Serial.println("\n🔧 TROUBLESHOOTING:");
    Serial.println("  1. Check VCC → 3.3V");
    Serial.println("  2. Check GND → GND");
    Serial.println("  3. Check SDA → GPIO 21");
    Serial.println("  4. Check SCL → GPIO 22");
    Serial.println("  5. Try different jumper wires");
    Serial.println("  6. Verify MPU6050 module isn't damaged");
    Serial.println("\n⚠️ SYSTEM HALTED - Fix wiring and restart");
    while(1) {
      delay(1000);
    }
  }
  Serial.println();
  
  // ============== TEST 3: MPU6050 COMMUNICATION ==============
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("TEST 3: MPU6050 Communication Test");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  // Read WHO_AM_I register
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  byte whoami = Wire.read();
  
  Serial.print("WHO_AM_I register: 0x");
  Serial.println(whoami, HEX);
  
  if (whoami == 0x68 || whoami == 0x71) {
    Serial.println("✓ MPU6050 identity confirmed!");
  } else {
    Serial.println("⚠️ Unexpected WHO_AM_I value (but may still work)");
  }
  
  // Wake up MPU6050
  Serial.print("Waking up MPU6050... ");
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Wake up
  byte error = Wire.endTransmission(true);
  
  if (error == 0) {
    Serial.println("✓ Success!");
    mpuConnected = true;
  } else {
    Serial.println("❌ Failed!");
    Serial.println("⚠️ SYSTEM HALTED");
    while(1) { delay(1000); }
  }
  
  delay(100);
  
  // Configure MPU6050 for high sensitivity
  Serial.println("⚙️ Configuring MPU6050 for jump detection...");
  
  // Set accelerometer range to ±2g (most sensitive)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x00); // ±2g range
  Wire.endTransmission(true);
  
  // Set gyroscope range to ±250°/s (most sensitive)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x00); // ±250°/s range
  Wire.endTransmission(true);
  
  Serial.println("✓ High sensitivity mode enabled!");
  Serial.println();
  
  // ============== TEST 4: MPU6050 DATA READ TEST ==============
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("TEST 4: Reading MPU6050 Data (Test Sample)");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  for (int i = 0; i < 3; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);
    
    if (Wire.available() >= 14) {
      ax = Wire.read() << 8 | Wire.read();
      ay = Wire.read() << 8 | Wire.read();
      az = Wire.read() << 8 | Wire.read();
      Wire.read(); Wire.read(); // Skip temp
      gx = Wire.read() << 8 | Wire.read();
      gy = Wire.read() << 8 | Wire.read();
      gz = Wire.read() << 8 | Wire.read();
      
      float accelX = ax / 16384.0;
      float accelY = ay / 16384.0;
      float accelZ = az / 16384.0;
      
      Serial.print("Sample ");
      Serial.print(i + 1);
      Serial.print(": AX=");
      Serial.print(accelX, 2);
      Serial.print(" AY=");
      Serial.print(accelY, 2);
      Serial.print(" AZ=");
      Serial.print(accelZ, 2);
      Serial.print(" (Jump detection ready!)");
      Serial.println();
      
      if (i == 2) {
        if (abs(accelZ) > 0.5) {
          Serial.println("✓ Sensor readings look good!");
        } else {
          Serial.println("⚠️ Warning: Unusual readings (but may be okay)");
        }
      }
    } else {
      Serial.println("❌ Failed to read data!");
    }
    delay(200);
  }
  Serial.println();
  
  // ============== TEST 5: LDR TEST ==============
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("TEST 5: LDR Light Sensor Test");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  int ldrValue = analogRead(LDR_PIN);
  int lightLevel = map(ldrValue, 0, 4095, 0, 100);
  
  Serial.print("LDR Raw Value: ");
  Serial.print(ldrValue);
  Serial.print(" → Light Level: ");
  Serial.print(lightLevel);
  Serial.println("%");
  
  if (ldrValue > 50 && ldrValue < 4090) {
    Serial.println("✓ LDR working properly!");
  } else if (ldrValue <= 50) {
    Serial.println("⚠️ LDR reading very low - check connections");
  } else {
    Serial.println("⚠️ LDR reading very high - may be disconnected");
  }
  Serial.println();
  
  // ============== TEST 6: WIFI SETUP ==============
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("TEST 6: WiFi Initialization");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  if (USE_ACCESS_POINT) {
    Serial.println("Mode: Access Point (ESP32 creates WiFi)");
    Serial.print("Creating WiFi AP: ");
    Serial.println(ap_ssid);
    
    WiFi.softAP(ap_ssid, ap_password);
    delay(500);
    IPAddress IP = WiFi.softAPIP();
    
    Serial.println("✓ Access Point created successfully!");
    Serial.print("  SSID: ");
    Serial.println(ap_ssid);
    Serial.print("  Password: ");
    Serial.println(ap_password);
    Serial.print("  IP Address: ");
    Serial.println(IP);
    wifiConnected = true;
  } else {
    Serial.println("Mode: Station (Connect to existing WiFi)");
    Serial.print("Connecting to: ");
    Serial.println(wifi_ssid);
    
    WiFi.begin(wifi_ssid, wifi_password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("✓ Connected to WiFi!");
      Serial.print("  IP Address: ");
      Serial.println(WiFi.localIP());
      wifiConnected = true;
    } else {
      Serial.println("❌ WiFi connection failed!");
      Serial.println("⚠️ Check SSID and password");
      Serial.println("⚠️ SYSTEM HALTED");
      while(1) { delay(1000); }
    }
  }
  Serial.println();
  
  // ============== TEST 7: WEBSOCKET SERVER ==============
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("TEST 7: WebSocket Server Initialization");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  Serial.println("✓ WebSocket server started on port 81");
  Serial.println();
  
  // ============== ALL TESTS PASSED ==============
  Serial.println("╔═══════════════════════════════════════════╗");
  Serial.println("║     ✓✓✓ ALL SYSTEMS OPERATIONAL ✓✓✓     ║");
  Serial.println("╚═══════════════════════════════════════════╝");
  Serial.println();
  Serial.println("🎮 JUMP GAME READY!");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("📱 HOW TO CONNECT:");
  if (USE_ACCESS_POINT) {
    Serial.print("   1. Connect phone/laptop to WiFi: ");
    Serial.println(ap_ssid);
    Serial.print("   2. Password: ");
    Serial.println(ap_password);
    Serial.print("   3. Open browser to: http://");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.print("   1. Make sure device is on same WiFi: ");
    Serial.println(wifi_ssid);
    Serial.print("   2. Open browser to: http://");
    Serial.println(WiFi.localIP());
  }
  Serial.println("   4. Enter IP in game and click Connect");
  Serial.println();
  Serial.println("🎯 GAME CONTROLS:");
  Serial.println("   • Tilt LEFT/RIGHT: Move ball horizontally");
  Serial.println("   • Tilt BACKWARD (up): JUMP!");
  Serial.println("   • Tilt FORWARD: Move down");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
  
  delay(1000);
}

void loop() {
  webSocket.loop();
  
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentTime;
    
    // Read MPU6050
    int16_t ax, ay, az, gx, gy, gz;
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    byte error = Wire.endTransmission(false);
    
    if (error != 0) {
      Serial.println("⚠️ MPU6050 read error!");
      return;
    }
    
    Wire.requestFrom(MPU_ADDR, 14, true);
    
    if (Wire.available() >= 14) {
      ax = Wire.read() << 8 | Wire.read();
      ay = Wire.read() << 8 | Wire.read();
      az = Wire.read() << 8 | Wire.read();
      Wire.read(); Wire.read(); // Skip temperature
      gx = Wire.read() << 8 | Wire.read();
      gy = Wire.read() << 8 | Wire.read();
      gz = Wire.read() << 8 | Wire.read();
      
      // Convert to g's and degrees/sec
      float accelX = ax / 16384.0;  // ±2g range
      float accelY = ay / 16384.0;
      float accelZ = az / 16384.0;
      float gyroX = gx / 131.0;     // ±250°/s range
      float gyroY = gy / 131.0;
      float gyroZ = gz / 131.0;
      
      // Read LDR
      int ldrValue = analogRead(LDR_PIN);
      int lightLevel = map(ldrValue, 0, 4095, 0, 100);
      
      // Create JSON data for game
      String jsonData = "{";
      jsonData += "\"ax\":" + String(accelX, 2) + ",";
      jsonData += "\"ay\":" + String(accelY, 2) + ",";
      jsonData += "\"az\":" + String(accelZ, 2) + ",";
      jsonData += "\"gx\":" + String(gyroX, 1) + ",";
      jsonData += "\"gy\":" + String(gyroY, 1) + ",";
      jsonData += "\"gz\":" + String(gyroZ, 1) + ",";
      jsonData += "\"light\":" + String(lightLevel);
      jsonData += "}";
      
      // Broadcast to all connected clients
      if (webSocket.connectedClients() > 0) {
        webSocket.broadcastTXT(jsonData);
      }
      
      // Print to Serial (every 10th reading = ~2 times/second)
      static int printCounter = 0;
      printCounter++;
      if (printCounter >= 10) {
        Serial.print("🎮 AX:");
        Serial.print(accelX, 2);
        Serial.print(" AY:");
        Serial.print(accelY, 2);
        Serial.print(" AZ:");
        Serial.print(accelZ, 2);
        Serial.print(" | GX:");
        Serial.print(gyroX, 1);
        Serial.print(" GY:");
        Serial.print(gyroY, 1);
        Serial.print(" GZ:");
        Serial.print(gyroZ, 1);
        Serial.print(" | Light:");
        Serial.print(lightLevel);
        Serial.print("% | Players:");
        Serial.println(webSocket.connectedClients());
        printCounter = 0;
      }
    }
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("❌ [%u] Player Disconnected\n", num);
      Serial.print("Active players: ");
      Serial.println(webSocket.connectedClients());
      break;
      
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("✅ [%u] NEW PLAYER CONNECTED from %d.%d.%d.%d\n", 
                      num, ip[0], ip[1], ip[2], ip[3]);
        Serial.print("Total players now: ");
        Serial.println(webSocket.connectedClients());
        
        // Send welcome message
        String welcome = "{\"status\":\"connected\",\"message\":\"Jump Game Ready! Tilt backward to jump!\"}";
        webSocket.sendTXT(num, welcome);
      }
      break;
      
    case WStype_TEXT:
      Serial.printf("📨 [%u] Message: %s\n", num, payload);
      break;
  }
}