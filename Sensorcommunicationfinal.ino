/*#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// MPU6050 and orientation variables
Adafruit_MPU6050 mpu;
float prevGyroX = 0.0;
float prevGyroY = 0.0;
float prevGyroZ = 0.0;
float prevWheelSpeed = 0.0; // Variable to store the previous wheel speed
float threshold = 0.01; // Threshold for significant change in data

// Incremental rotary encoder variables
volatile int counter = 0;        // Counter for encoder pulses
int prevCounter = 0;             // To store the last counter value for speed calculation
unsigned long prevTime = 0;      // Time of the last speed calculation
float speedRad = 0;              // Calculated angular speed in radians per second

// PPR for your rotary encoder (replace with the correct value)
const int PPR = 600; // Example: 360 pulses per revolution (replace with your encoder's value)

// Define encoder pins
#define ENCODER_PIN_A 15
#define ENCODER_PIN_B 16

// Define BLE characteristics UUIDs
#define SERVICE_UUID        "968feb27-db87-45c2-b981-4db7619e3564"
#define CHARACTERISTIC_UUID "4327e688-5ec6-41d6-b63b-df58b84e9a99"

// BLE setup
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

void setupBLE() {
  BLEDevice::init("ESP32-Kayak");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));
  pCharacteristic = pService->createCharacteristic(
                      BLEUUID(CHARACTERISTIC_UUID),
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902()); // Add the BLE2902 descriptor

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
  Serial.println("BLE service started");
}

// Interrupt Service Routine for Signal A
void IRAM_ATTR ai0() {
  if (digitalRead(ENCODER_PIN_B) == LOW) { // Read Signal B to determine direction
    counter++;
  } else {
    counter--;
  }
}

// Interrupt Service Routine for Signal B
void IRAM_ATTR ai1() {
  if (digitalRead(ENCODER_PIN_A) == LOW) { // Read Signal A to determine direction
    counter--;
  } else {
    counter++;
  }
}

void setup(void) {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Configure encoder pins and attach interrupts
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), ai1, RISING);

  setupBLE(); // Initialize BLE

  Serial.print("BLE Device Address: ");
  Serial.println(BLEDevice::getAddress().toString().c_str());

  Serial.println("System Initialized");
}

void loop() {
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate speed every 100 ms (or adjust as needed)
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= 100) {
    noInterrupts(); // Disable interrupts to read counter safely
    int currentCounter = counter;
    interrupts();   // Re-enable interrupts

    // Calculate the time interval in seconds
    float timeInterval = (currentTime - prevTime) / 1000.0;

    // Calculate pulses per second (pulses per second = pulse count / time)
    float pulsesPerSecond = (currentCounter - prevCounter) / timeInterval;

    // Convert pulses per second to angular speed in radians per second
    speedRad = (pulsesPerSecond * 2.0 * PI) / PPR;

    prevCounter = currentCounter;
    prevTime = currentTime;

   // Serial.print("Speed: ");
    //Serial.print(speedRad);
    // Serial.println(" rad/s");
  }

  // Check if the current orientation or wheel speed differs from the previous readings by more than the threshold
  bool orientationChanged = abs(g.gyro.x - prevGyroX) > threshold ||
                            abs(g.gyro.y - prevGyroY) > threshold ||
                            abs(g.gyro.z - prevGyroZ) > threshold;

  bool speedChanged = abs(speedRad - prevWheelSpeed) > threshold;

  bool dataChanged = orientationChanged || speedChanged;

  // Package data (orientation and speed) into a simplified JSON-like format without field names
  String dataPacket = "{";
  dataPacket += String(g.gyro.x, 2) + "," + String(g.gyro.y, 2) + "," + String(g.gyro.z, 2) + "," + String(speedRad, 2);
  dataPacket += "}";

  // Print data packet to Serial Monitor
  Serial.print("Data Packet: ");
  Serial.println(dataPacket);

  // Send the data packet over BLE if the data has changed
  if (dataChanged) {
    // Update the previous orientation and speed values
    prevGyroX = g.gyro.x;
    prevGyroY = g.gyro.y;
    prevGyroZ = g.gyro.z;
    prevWheelSpeed = speedRad;

    // Send the data packet over BLE
    if (deviceConnected) {
      pCharacteristic->setValue(dataPacket.c_str());
      pCharacteristic->notify();
      Serial.println("Data sent over BLE");
    }
  }

  delay(100); // Adjust the delay as needed
}*/
///////////////////////////////////////////////////////////////////////////////////
/*#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// MPU6050 and orientation variables
Adafruit_MPU6050 mpu;
float prevGyroX = 0.0;
float prevGyroY = 0.0;
float prevGyroZ = 0.0;
float prevWheelSpeed = 0.0; // Variable to store the previous wheel speed
float threshold = 0.01; // Threshold for significant change in data

// Complementary filter variables
float angleX = 0.0;
float angleY = 0.0;
float alpha = 0.95; // Complementary filter coefficient

// Incremental rotary encoder variables
volatile int counter = 0;        // Counter for encoder pulses
int prevCounter = 0;             // To store the last counter value for speed calculation
unsigned long prevTime = 0;      // Time of the last speed calculation
float speedRad = 0;              // Calculated angular speed in radians per second

// PPR for your rotary encoder (replace with the correct value)
const int PPR = 600; // Example: 360 pulses per revolution (replace with your encoder's value)

// Define encoder pins
#define ENCODER_PIN_A 15
#define ENCODER_PIN_B 16

// Define BLE characteristics UUIDs
#define SERVICE_UUID        "968feb27-db87-45c2-b981-4db7619e3564"
#define CHARACTERISTIC_UUID "4327e688-5ec6-41d6-b63b-df58b84e9a99"

// BLE setup
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

void setupBLE() {
  BLEDevice::init("ESP32-Kayak");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));
  pCharacteristic = pService->createCharacteristic(
                      BLEUUID(CHARACTERISTIC_UUID),
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902()); // Add the BLE2902 descriptor

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
  Serial.println("BLE service started");
}

// Interrupt Service Routine for Signal A
void IRAM_ATTR ai0() {
  if (digitalRead(ENCODER_PIN_B) == LOW) { // Read Signal B to determine direction
    counter++;
  } else {
    counter--;
  }
}

// Interrupt Service Routine for Signal B
void IRAM_ATTR ai1() {
  if (digitalRead(ENCODER_PIN_A) == LOW) { // Read Signal A to determine direction
    counter--;
  } else {
    counter++;
  }
}

void setup(void) {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Configure encoder pins and attach interrupts
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), ai1, RISING);

  setupBLE(); // Initialize BLE

  Serial.print("BLE Device Address: ");
  Serial.println(BLEDevice::getAddress().toString().c_str());

  Serial.println("System Initialized");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  if (currentTime - prevTime >= 100) {
    noInterrupts();
    int currentCounter = counter;
    interrupts();

    float timeInterval = (currentTime - prevTime) / 1000.0;
    float pulsesPerSecond = (currentCounter - prevCounter) / timeInterval;
    speedRad = (pulsesPerSecond * 2.0 * PI) / PPR;

    prevCounter = currentCounter;
    prevTime = currentTime;
  }

  // Apply complementary filter
  float accelAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float accelAngleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  angleX = alpha * (angleX + g.gyro.x * 0.1) + (1 - alpha) * accelAngleX;
  angleY = alpha * (angleY + g.gyro.y * 0.1) + (1 - alpha) * accelAngleY;

  bool dataChanged = abs(g.gyro.x - prevGyroX) > threshold ||
                     abs(g.gyro.y - prevGyroY) > threshold ||
                     abs(g.gyro.z - prevGyroZ) > threshold ||
                     abs(speedRad - prevWheelSpeed) > threshold;

  String dataPacket = "{" + String(angleX, 2) + "," + String(angleY, 2) + "," + String(g.gyro.z, 2) + "," + String(speedRad, 2) + "}";

  Serial.print("Data Packet: ");
  Serial.println(dataPacket);

  if (dataChanged && deviceConnected) {
    prevGyroX = g.gyro.x;
    prevGyroY = g.gyro.y;
    prevGyroZ = g.gyro.z;
    prevWheelSpeed = speedRad;
    pCharacteristic->setValue(dataPacket.c_str());
    pCharacteristic->notify();
    Serial.println("Data sent over BLE");
  }
  delay(100);
}*/




 //Final code using msp43
 /*





#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// MPU6050 and orientation variables
Adafruit_MPU6050 mpu;
float prevGyroX = 0.0, prevGyroY = 0.0, prevGyroZ = 0.0;
float prevWheelSpeed = 0.0;
float threshold = 0.01;

// Complementary filter variables
float angleX = 0.0, angleY = 0.0;
float alpha = 0.95; // Complementary filter coefficient

// Encoder variables
volatile int counter = 0;
int prevCounter = 0;
unsigned long prevTime = 0;
float speedRad = 0;
const int PPR = 600; // Pulses per revolution

// Define encoder pins
#define ENCODER_PIN_A 15
#define ENCODER_PIN_B 16

// Define BLE characteristics UUIDs
#define SERVICE_UUID        "968feb27-db87-45c2-b981-4db7619e3564"
#define CHARACTERISTIC_UUID "4327e688-5ec6-41d6-b63b-df58b84e9a99"

// BLE setup
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

void setupBLE() {
  BLEDevice::init("ESP32-Kayak");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));
  pCharacteristic = pService->createCharacteristic(
                      BLEUUID(CHARACTERISTIC_UUID),
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902()); 

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
  Serial.println("BLE service started");
}

// Interrupt Service Routine for Encoder
void IRAM_ATTR ai0() {
  if (digitalRead(ENCODER_PIN_B) == LOW) {
    counter++;
  } else {
    counter--;
  }
}
void IRAM_ATTR ai1() {
  if (digitalRead(ENCODER_PIN_A) == LOW) {
    counter--;
  } else {
    counter++;
  }
}

void setup(void) {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Configure encoder pins and attach interrupts
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), ai1, RISING);

  setupBLE();
  Serial.print("BLE Device Address: ");
  Serial.println(BLEDevice::getAddress().toString().c_str());
  Serial.println("System Initialized");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate time difference
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;  
  prevTime = currentTime;

  // Calculate angular speed
  noInterrupts();
  int currentCounter = counter;
  interrupts();

  float pulsesPerSecond = (currentCounter - prevCounter) / dt;
  speedRad = (pulsesPerSecond * 2.0 * PI) / PPR;
  prevCounter = currentCounter;

  // Apply complementary filter
  float accelAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float accelAngleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  // Convert gyroscope data to degrees per second
  float gyroX_dps = g.gyro.x / 65.5;  // Assuming ±500°/s range
  float gyroY_dps = g.gyro.y / 65.5;

  // Apply complementary filter
  angleX = alpha * (angleX + gyroX_dps * dt) + (1 - alpha) * accelAngleX;
  angleY = alpha * (angleY + gyroY_dps * dt) + (1 - alpha) * accelAngleY;

  // Check if data changed significantly
  bool dataChanged = abs(g.gyro.x - prevGyroX) > threshold ||
                     abs(g.gyro.y - prevGyroY) > threshold ||
                     abs(g.gyro.z - prevGyroZ) > threshold ||
                     abs(speedRad - prevWheelSpeed) > threshold;

  String dataPacket = "{" + String(angleX, 2) + "," + String(angleY, 2) + "," + String(g.gyro.z, 2) + "," + String(speedRad, 2) + "}";

  Serial.print("Data Packet: ");
  Serial.println(dataPacket);

  if (dataChanged && deviceConnected) {
    prevGyroX = g.gyro.x;
    prevGyroY = g.gyro.y;
    prevGyroZ = g.gyro.z;
    prevWheelSpeed = speedRad;
    pCharacteristic->setValue(dataPacket.c_str());
    pCharacteristic->notify();
    Serial.println("Data sent over BLE");
  }
  delay(100);
}*/

// after use icm20948 imu senores

/*#include <Wire.h>
#include <Adafruit_ICM20948.h>  // Adafruit ICM-20948 library
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ICM-20948 objects
Adafruit_ICM20948 imu1;
Adafruit_ICM20948 imu2;

// Complementary filter variables
float angleX = 0.0, angleY = 0.0;
float alpha = 0.95; // Complementary filter coefficient

// Encoder variables
volatile int counter = 0;
int prevCounter = 0;
unsigned long prevTime = 0;
float speedRad = 0;
const int PPR = 600; // Pulses per revolution

// Define encoder pins
#define ENCODER_PIN_A 15
#define ENCODER_PIN_B 16

// Define BLE UUIDs
#define SERVICE_UUID        "968feb27-db87-45c2-b981-4db7619e3564"
#define CHARACTERISTIC_UUID "4327e688-5ec6-41d6-b63b-df58b84e9a99"

// BLE setup
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

// BLE connection callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

void setupBLE() {
  BLEDevice::init("ESP32-Kayak");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));
  pCharacteristic = pService->createCharacteristic(
                      BLEUUID(CHARACTERISTIC_UUID),
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
  Serial.println("BLE service started");
}

// Interrupt Service Routine for Encoder
void IRAM_ATTR ai0() {
  if (digitalRead(ENCODER_PIN_B) == LOW) {
    counter++;
  } else {
    counter--;
  }
}

void IRAM_ATTR ai1() {
  if (digitalRead(ENCODER_PIN_A) == LOW) {
    counter--;
  } else {
    counter++;
  }
}

void setupIMU(Adafruit_ICM20948 &imu, uint8_t address) {
  if (!imu.begin_I2C(address)) {
    Serial.println("IMU not found, check wiring!");
    while (1);
  }
  Serial.println("IMU initialized successfully!");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Setup both IMUs
  setupIMU(imu1, 0x68);
  setupIMU(imu2, 0x69);

  // Configure encoder pins
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), ai1, RISING);

  setupBLE();
  Serial.println("System Initialized");
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Read both IMUs
  sensors_event_t accel1, gyro1, temp1;
  sensors_event_t accel2, gyro2, temp2;

  imu1.getEvent(&accel1, &gyro1, &temp1);
  imu2.getEvent(&accel2, &gyro2, &temp2);

  // Calculate accelerometer angles
  float accelAngleX1 = atan2(accel1.acceleration.y, accel1.acceleration.z) * 180 / PI;
  float accelAngleY1 = atan2(-accel1.acceleration.x, sqrt(accel1.acceleration.y * accel1.acceleration.y + accel1.acceleration.z * accel1.acceleration.z)) * 180 / PI;
  
  float accelAngleX2 = atan2(accel2.acceleration.y, accel2.acceleration.z) * 180 / PI;
  float accelAngleY2 = atan2(-accel2.acceleration.x, sqrt(accel2.acceleration.y * accel2.acceleration.y + accel2.acceleration.z * accel2.acceleration.z)) * 180 / PI;

  // Merge accelerometer angles from both IMUs
  float accelAngleX = (accelAngleX1 + accelAngleX2) / 2.0;
  float accelAngleY = (accelAngleY1 + accelAngleY2) / 2.0;

  // Gyroscope readings (convert raw data to degrees/sec)
  float gyroX = (gyro1.gyro.x + gyro2.gyro.x) / 2.0;
  float gyroY = (gyro1.gyro.y + gyro2.gyro.y) / 2.0;
  float gyroZ = (gyro1.gyro.z + gyro2.gyro.z) / 2.0;

  // Apply complementary filter
  angleX = alpha * (angleX + gyroX * dt) + (1 - alpha) * accelAngleX;
  angleY = alpha * (angleY + gyroY * dt) + (1 - alpha) * accelAngleY;

  // Calculate wheel speed
  noInterrupts();
  int currentCounter = counter;
  interrupts();

  float pulsesPerSecond = (currentCounter - prevCounter) / dt;
  speedRad = (pulsesPerSecond * 2.0 * PI) / PPR;
  prevCounter = currentCounter;

  // Create data packet
  String dataPacket = "{" + String(angleX, 2) + "," + String(angleY, 2) + "," + String(gyroZ, 2) + "," + String(speedRad, 2) + "}";

  Serial.print("Data Packet: ");
  Serial.println(dataPacket);

  // Send BLE notification if connected
  if (deviceConnected) {
    pCharacteristic->setValue(dataPacket.c_str());
    pCharacteristic->notify();
    Serial.println("Data sent over BLE");
  }
  delay(100);
}

*/
/////////////////////////////////////



/*#include <Wire.h>
#include <Adafruit_ICM20948.h>

// Create IMU objects
Adafruit_ICM20948 imu1;
Adafruit_ICM20948 imu2;

// Function to initialize sensors
void setupIMUs() {
    Serial.println("Initializing IMUs...");
    if (!imu1.begin_I2C(0x68)) {
        Serial.println("IMU 1 (0x68) not found! Check wiring.");
    } else {
        Serial.println("IMU 1 (0x68) initialized.");
    }
    
    if (!imu2.begin_I2C(0x69)) {
        Serial.println("IMU 2 (0x69) not found! Check wiring.");
    } else {
        Serial.println("IMU 2 (0x69) initialized.");
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize I2C with lower clock speed for stability
    Wire.begin(21, 22);
    Wire.setClock(100000);  // Reduce speed if issues occur

    setupIMUs();
}

void loop() {
    sensors_event_t accel1, gyro1, temp1;
    sensors_event_t accel2, gyro2, temp2;

    // Read IMU 1 data
    if (imu1.getEvent(&accel1, &gyro1, &temp1)) {
        Serial.print("IMU 1 - Accel X: "); Serial.print(accel1.acceleration.x, 2);
        Serial.print(" | Gyro Z: "); Serial.println(gyro1.gyro.z, 2);
    } else {
        Serial.println("IMU 1 read error! Resetting...");
        setupIMUs();
    }

    // Read IMU 2 data
    if (imu2.getEvent(&accel2, &gyro2, &temp2)) {
        Serial.print("IMU 2 - Accel X: "); Serial.print(accel2.acceleration.x, 2);
        Serial.print(" | Gyro Z: "); Serial.println(gyro2.gyro.z, 2);
    } else {
        Serial.println("IMU 2 read error! Resetting...");
        setupIMUs();
    }

    delay(500);
}
*/

//////////////////////////////
/*#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// Encoder variables
volatile int counter = 0;
#define ENCODER_PIN_A 15
#define ENCODER_PIN_B 16

// BLE UUIDs
#define SERVICE_UUID "968feb27-db87-45c2-b981-4db7619e3564"
#define CHARACTERISTIC_UUID "4327e688-5ec6-41d6-b63b-df58b84e9a99"

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

// BLE connection callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

void setupBLE() {
  BLEDevice::init("ESP32-Encoder");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));
  pCharacteristic = pService->createCharacteristic(
                      BLEUUID(CHARACTERISTIC_UUID),
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
  Serial.println("BLE service started");
}

// Interrupt Service Routine for Encoder
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENCODER_PIN_B) == LOW) {
    counter++;
  } else {
    counter--;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, RISING);
  setupBLE();
}

void loop() {
  static int lastCounter = 0;
  if (counter != lastCounter) {
    Serial.print("Encoder Count: ");
    Serial.println(counter);
    if (deviceConnected) {
      String data = String(counter);
      pCharacteristic->setValue(data.c_str());
      pCharacteristic->notify();
      Serial.println("Data sent over BLE");
    }
    lastCounter = counter;
  }
  delay(100);
}*/
/*#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ICM-20948 objects
Adafruit_ICM20948 imu1;
Adafruit_ICM20948 imu2;

// Complementary filter variables
float angleX = 0.0, angleY = 0.0;
float alpha = 0.95; // Complementary filter coefficient

// Encoder variables
volatile int counter = 0;
int prevCounter = 0;
unsigned long prevTime = 0;
float speedRad = 0;
const int PPR = 600; // Pulses per revolution

// Define encoder pins
#define ENCODER_PIN_A 15
#define ENCODER_PIN_B 16

// Define BLE UUIDs
#define SERVICE_UUID        "968feb27-db87-45c2-b981-4db7619e3564"
#define CHARACTERISTIC_UUID "4327e688-5ec6-41d6-b63b-df58b84e9a99"

// BLE setup
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

// BLE connection callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

void setupBLE() {
  BLEDevice::init("ESP32-Kayak");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));
  pCharacteristic = pService->createCharacteristic(
                      BLEUUID(CHARACTERISTIC_UUID),
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
  Serial.println("BLE service started");
}

// Interrupt Service Routine for Encoder
void IRAM_ATTR ai0() {
  if (digitalRead(ENCODER_PIN_B) == HIGH) {
    counter++;  // Clockwise rotation
  } else {
    counter--;  // Counterclockwise rotation
  }
}

void IRAM_ATTR ai1() {
  if (digitalRead(ENCODER_PIN_A) == HIGH) {
    counter--;  // Counterclockwise rotation
  } else {
    counter++;  // Clockwise rotation
  }
}

void setupIMU(Adafruit_ICM20948 &imu, uint8_t address) {
  if (!imu.begin_I2C(address)) {
    Serial.println("IMU not found, check wiring!");
    while (1);
  }
  Serial.println("IMU initialized successfully!");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Setup both IMUs
  setupIMU(imu1, 0x68);
  setupIMU(imu2, 0x69);

  // Configure encoder pins
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), ai0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), ai1, CHANGE);

  setupBLE();
  Serial.println("System Initialized");
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Read both IMUs
  sensors_event_t accel1, gyro1, temp1;
  sensors_event_t accel2, gyro2, temp2;
  imu1.getEvent(&accel1, &gyro1, &temp1);
  imu2.getEvent(&accel2, &gyro2, &temp2);

  // Calculate accelerometer angles
  float accelAngleX = (atan2(accel1.acceleration.y, accel1.acceleration.z) + atan2(accel2.acceleration.y, accel2.acceleration.z)) * 90 / PI;
  float accelAngleY = (atan2(-accel1.acceleration.x, sqrt(accel1.acceleration.y * accel1.acceleration.y + accel1.acceleration.z * accel1.acceleration.z)) +
                        atan2(-accel2.acceleration.x, sqrt(accel2.acceleration.y * accel2.acceleration.y + accel2.acceleration.z * accel2.acceleration.z))) * 90 / PI;
  accelAngleX /= 2.0;
  accelAngleY /= 2.0;

  // Gyroscope readings
  float gyroX = (gyro1.gyro.x + gyro2.gyro.x) / 2.0;
  float gyroY = (gyro1.gyro.y + gyro2.gyro.y) / 2.0;
  float gyroZ = (gyro1.gyro.z + gyro2.gyro.z) / 2.0;

  // Apply complementary filter
  angleX = alpha * (angleX + gyroX * dt) + (1 - alpha) * accelAngleX;
  angleY = alpha * (angleY + gyroY * dt) + (1 - alpha) * accelAngleY;

  // Calculate wheel speed
  noInterrupts();
  int currentCounter = counter;
  interrupts();

  float pulsesPerSecond = (currentCounter - prevCounter) / dt;
  speedRad = (pulsesPerSecond * 2.0 * PI) / PPR;
  prevCounter = currentCounter;

  // Create data packet
  String dataPacket = "{" + String(angleX, 2) + "," + String(angleY, 2) + "," + String(gyroZ, 2) + "," + String(speedRad, 2) + "}";

  Serial.print("Data Packet: ");
  Serial.println(dataPacket);

  // Send BLE notification if connected
  if (deviceConnected) {
    pCharacteristic->setValue(dataPacket.c_str());
    pCharacteristic->notify();
    Serial.println("Data sent over BLE");
  }
  delay(100);
}*/

#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <TouchScreen.h>

// TFT Display pins
#define TFT_CS   5
#define TFT_DC   26
#define TFT_MOSI 23
#define TFT_CLK  18
#define TFT_RST  17
#define TFT_MISO 19

// Touch Screen pins
#define YP 33  // must be an analog pin
#define XM 25  // must be an analog pin
#define YM 32  // must be an analog pin
#define XP 27  // must be an analog pin

// Touch screen calibration
#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// IMU objects
Adafruit_ICM20948 imu1;
Adafruit_ICM20948 imu2;
bool imu1Active = false;
bool imu2Active = false;
bool imuWorking = false;

// Complementary filter variables
float angleX = 0.0, angleY = 0.0;
float alpha = 0.98;

// Encoder variables
volatile int counter = 0;
int prevCounter = 0;
unsigned long prevTime = 0;
float speedRad = 0;
const int PPR = 600;
bool encoderWorking = false;

// Encoder pins
#define ENCODER_PIN_A 15
#define ENCODER_PIN_B 16

// BLE UUIDs
#define SERVICE_UUID        "968feb27-db87-45c2-b981-4db7619e3564"
#define CHARACTERISTIC_UUID "4327e688-5ec6-41d6-b63b-df58b84e9a99"

// BLE setup
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

// Calorie calculation
float caloriesBurned = 0.0;
float userWeight = 70.0;  // Default weight
bool settingWeight = false;
const float CALORIES_PER_STROKE_PER_KG = 0.0015;

// Touch system variables
unsigned long lastTouchTime = 0;
const unsigned long WEIGHT_CONFIRM_TIMEOUT = 10000; // 10 seconds
const float WEIGHT_INCREMENT = 0.5f;

// Button dimensions
#define BUTTON_WIDTH 80
#define BUTTON_HEIGHT 50
#define BUTTON_SPACING 10
#define BUTTON_Y_POS 180

// Forward declarations
void updateDisplay();
void handleTouch();
void calculateCalories();
void drawWeightSettingButtons();
void checkSensorStatus();

// BLE connection callbacks
class MyServerCallbacks : public BLEServerCallbacks {
void onConnect(BLEServer* pServer) {
deviceConnected = true;
Serial.println("Device connected");
updateDisplay();
}
void onDisconnect(BLEServer* pServer) {
deviceConnected = false;
Serial.println("Device disconnected");
updateDisplay();
}
};

void setupBLE() {
BLEDevice::init("ESP32-Kayak");
pServer = BLEDevice::createServer();
pServer->setCallbacks(new MyServerCallbacks());

BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));
pCharacteristic = pService->createCharacteristic(
BLEUUID(CHARACTERISTIC_UUID),
BLECharacteristic::PROPERTY_NOTIFY
);
pCharacteristic->addDescriptor(new BLE2902());

pService->start();
BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
pAdvertising->start();
pServer->updateConnParams(BLEAddress(""), 8, 8, 0, 400);
BLEDevice::setMTU(100);
Serial.println("BLE service started");
}

// Encoder ISRs
void IRAM_ATTR ai0() {
if (digitalRead(ENCODER_PIN_B) == LOW) {
counter++;
} else {
counter--;
}
}

void IRAM_ATTR ai1() {
if (digitalRead(ENCODER_PIN_A) == LOW) {
counter--;
} else {
counter++;
}
}

bool setupIMU(Adafruit_ICM20948 &imu, uint8_t address) {
if (!imu.begin_I2C(address)) {
Serial.print("IMU at address 0x");
Serial.print(address, HEX);
Serial.println(" not found!");
return false;
}
Serial.print("IMU at address 0x");
Serial.print(address, HEX);
Serial.println(" initialized!");
return true;
}

void initializeDisplay() {
tft.begin();
tft.setRotation(3);
tft.fillScreen(ILI9341_BLACK);
tft.setTextColor(ILI9341_WHITE);
tft.setTextSize(2);
tft.setCursor(0, 10);
tft.println("VR Kayaking System");
tft.drawLine(0, 30, tft.width(), 30, ILI9341_WHITE);
updateDisplay();
}

void drawWeightSettingButtons() {
// Clear the button area first
tft.fillRect(0, BUTTON_Y_POS, tft.width(), 120, ILI9341_BLACK);

// Instruction text
tft.setTextColor(ILI9341_WHITE);
tft.setCursor(10, BUTTON_Y_POS + 10);
tft.println("Tap screen to increase weight");
tft.setCursor(10, BUTTON_Y_POS + 30);
tft.print("Auto-confirms in ");
tft.print((WEIGHT_CONFIRM_TIMEOUT - (millis() - lastTouchTime)) / 1000);
tft.println("s");

// Current weight display
tft.fillRoundRect(tft.width()/2 - BUTTON_WIDTH/2, BUTTON_Y_POS + 60, BUTTON_WIDTH, BUTTON_HEIGHT, 5, ILI9341_BLUE);
tft.setCursor(tft.width()/2 - 30, BUTTON_Y_POS + 75);
tft.print(userWeight, 1);
tft.println("kg");
}

void drawSensorStatus() {
// Draw sensor status in top-right corner
int statusX = tft.width() - 100; // Right side of screen
int statusY = 200;                // Top of screen

// IMU status
tft.setCursor(statusX, statusY);
tft.print("IMU:");
tft.setTextColor(imuWorking ? ILI9341_GREEN : ILI9341_RED);
tft.println(imuWorking ? "OK" : "ERR");

// Encoder status
tft.setCursor(statusX, statusY + 20);
tft.setTextColor(ILI9341_WHITE);
tft.print("ENC:");
tft.setTextColor(encoderWorking ? ILI9341_GREEN : ILI9341_RED);
tft.println(encoderWorking ? "OK" : "ERR");

tft.setTextColor(ILI9341_WHITE); // Reset color
}

void updateDisplay() {
tft.fillRect(0, 40, tft.width(), tft.height() - 40, ILI9341_BLACK);

int x = 0;
int y = 50;
int lineHeight = 40;  // Vertical spacing between lines

// Draw sensor status first
drawSensorStatus();

tft.setCursor(x, y);
tft.setTextColor(ILI9341_WHITE);
tft.print("System: ");
tft.setTextColor(ILI9341_GREEN);
tft.println("WORKING");

y += lineHeight;
tft.setCursor(x, y);
tft.setTextColor(ILI9341_WHITE);
tft.print("Paddling Speed: ");
tft.setTextColor(encoderWorking ? ILI9341_YELLOW : ILI9341_RED);
tft.print(abs(speedRad), 2);
tft.println(" rad/s");

y += lineHeight;
tft.setCursor(x, y);
tft.setTextColor(ILI9341_WHITE);
tft.print("Calories: ");
tft.setTextColor(ILI9341_CYAN);
tft.print(caloriesBurned, 1);
tft.println(" kcal");

y += lineHeight;
tft.setCursor(x, y);
tft.setTextColor(ILI9341_WHITE);
tft.print("Weight: ");
tft.setTextColor(ILI9341_MAGENTA);
tft.print(userWeight, 1);
tft.println(" kg");

y += lineHeight;
tft.setCursor(x, y);
tft.setTextColor(ILI9341_WHITE);
tft.print("BLE: ");
tft.setTextColor(deviceConnected ? ILI9341_GREEN : ILI9341_RED);
tft.println(deviceConnected ? "Connected" : "Disconnected");

tft.setTextColor(ILI9341_WHITE);  // Reset color to default

if (settingWeight) {
drawWeightSettingButtons();
}
}

void handleTouch() {
TSPoint p = ts.getPoint();
pinMode(XM, OUTPUT);
pinMode(YP, OUTPUT);

if (p.z > ts.pressureThreshhold) {
int16_t x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
int16_t y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());

// Update last touch time
lastTouchTime = millis();

if (settingWeight) {
  // In setting mode, any touch increases weight
  userWeight = min(150.0f, userWeight + WEIGHT_INCREMENT); // Max 150kg
  drawWeightSettingButtons();
} 
else if (y > 120 && y < 170) {  // Weight display area
  // Enter weight setting mode
  settingWeight = true;
  lastTouchTime = millis(); // Reset timeout
  updateDisplay();
} 
else if (y > 60 && y < 120) {  // Calories display area
  caloriesBurned = 0;
  updateDisplay();
}


}
}

void checkSensorStatus() {
static unsigned long lastCheckTime = 0;
if (millis() - lastCheckTime > 1000) { // Check every second
// Check IMU status
sensors_event_t accel, gyro, temp;
if (imu1Active) {
imuWorking = imu1.getEvent(&accel, &gyro, &temp);
} else if (imu2Active) {
imuWorking = imu2.getEvent(&accel, &gyro, &temp);
} else {
imuWorking = false;
}


// Check encoder status by detecting changes
noInterrupts();
int currentCounter = counter;
interrupts();
encoderWorking = (currentCounter != prevCounter);

lastCheckTime = millis();


}
}

void setup() {
Serial.begin(115200);
Wire.begin();
initializeDisplay();

imu1Active = setupIMU(imu1, 0x68);
if (!imu1Active) {
imu2Active = setupIMU(imu2, 0x69);
}

pinMode(ENCODER_PIN_A, INPUT_PULLUP);
pinMode(ENCODER_PIN_B, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), ai0, RISING);
attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), ai1, RISING);

setupBLE();
Serial.println("System Initialized");
}

void readIMUData(Adafruit_ICM20948 &imu, sensors_event_t &accel, sensors_event_t &gyro, sensors_event_t &temp) {
static bool imuFailed = false;

if (!imuFailed) {
try {
if (!imu.getEvent(&accel, &gyro, &temp)) {
imuFailed = true;
}
} catch (...) {
imuFailed = true;
}


if (imuFailed) {
  accel.acceleration.x = 0;
  accel.acceleration.y = 0;
  accel.acceleration.z = 0;
  gyro.gyro.x = 0;
  gyro.gyro.y = 0;
  gyro.gyro.z = 0;
}


} else {
accel.acceleration.x = 0;
accel.acceleration.y = 0;
accel.acceleration.z = 0;
gyro.gyro.x = 0;
gyro.gyro.y = 0;
gyro.gyro.z = 0;
}
}

void calculateCalories() {
static float accumulatedRotation = 0;
accumulatedRotation += abs(speedRad) * 0.1;

if (accumulatedRotation >= 2 * PI) {
int strokes = accumulatedRotation / (2 * PI);
caloriesBurned += strokes * userWeight * CALORIES_PER_STROKE_PER_KG;
accumulatedRotation = fmod(accumulatedRotation, 2 * PI);


static unsigned long lastUpdate = 0;
if (millis() - lastUpdate > 1000) {
  updateDisplay();
  lastUpdate = millis();
}


}
}

void loop() {
unsigned long currentTime = millis();
float dt = (currentTime - prevTime) / 1000.0;
prevTime = currentTime;

handleTouch();
checkSensorStatus();

// Check for weight setting timeout
if (settingWeight && (currentTime - lastTouchTime > WEIGHT_CONFIRM_TIMEOUT)) {
settingWeight = false;
updateDisplay();
}

sensors_event_t accel, gyro, temp;
float accelAngleX = 0, accelAngleY = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

if (imu1Active) {
readIMUData(imu1, accel, gyro, temp);


if (accel.acceleration.x == 0 && accel.acceleration.y == 0 && accel.acceleration.z == 0 &&
    gyro.gyro.x == 0 && gyro.gyro.y == 0 && gyro.gyro.z == 0) {
  imu1Active = false;
  if (!imu2Active) {
    imu2Active = setupIMU(imu2, 0x69);
  }
}


}

if (!imu1Active && imu2Active) {
readIMUData(imu2, accel, gyro, temp);
}

if (imu1Active || imu2Active) {
accelAngleX = atan2(accel.acceleration.y, accel.acceleration.z) * 90 / PI;
accelAngleY = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 90 / PI;


gyroX = gyro.gyro.x;
gyroY = gyro.gyro.y;
gyroZ = gyro.gyro.z;

angleX = alpha * (angleX + gyroX * dt) + (1 - alpha) * accelAngleX;
angleY = alpha * (angleY + gyroY * dt) + (1 - alpha) * accelAngleY;


} else {
angleX = 0;
angleY = 0;
gyroZ = 0;
}

noInterrupts();
int currentCounter = counter;
interrupts();

float pulsesPerSecond = (currentCounter - prevCounter) / dt;
speedRad = (pulsesPerSecond * 2.0 * PI) / PPR;
prevCounter = currentCounter;

calculateCalories();

String dataPacket = "{" + String(angleX, 2) + "," + String(angleY, 2) + "," + String(gyroZ, 2) + "," + String(speedRad, 2) + "}";

if (deviceConnected) {
pCharacteristic->setValue(dataPacket.c_str());
pCharacteristic->notify();
}

static unsigned long lastDisplayUpdate = 0;
if (millis() - lastDisplayUpdate > 500) {
updateDisplay();
lastDisplayUpdate = millis();
}

}   

