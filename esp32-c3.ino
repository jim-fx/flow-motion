#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
//#include "MPU6050.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define SERVICE_UUID "9b9f77c6-7e68-4109-b987-b096233d9525"
#define CHARACTERISTIC_UUID "1ab2c9f4-19c0-48dd-8932-ed72558ec593"

MPU6050 mpu;
BLECharacteristic *pCharacteristic;
BLEServer *pServer;
bool deviceConnected = false;

//int16_t ax, ay, az;
//int16_t gx, gy, gz;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
int16_t ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Device connected");
    deviceConnected = true;
  }

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Device disconnected");
    deviceConnected = false;
    BLEDevice::startAdvertising();  // Restart advertising
  }
};

void setup_ble() {
  BLEDevice::init("ESP32");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);

  uint8_t gyroData[] = { 1, 2, 3, 4, 5, 6 };
  pCharacteristic->setValue(gyroData, sizeof(gyroData));
  pCharacteristic->notify();
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void setup() {
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(115200);

  setup_ble();

  // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    delay(2000);
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      // display yaw pitch roll in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

            for(int i = 0; i < 3; i++)
      {
        ypr[i] = (ypr[i] * 180/M_PI);
        if (ypr[i] > 90 && ypr[i] <= 180)
        {
          ypr[i] = 90-(ypr[i]-90);
        }
        if (ypr[i] < -90 && ypr[i] >= -180)
        {
          ypr[i] = -90-(ypr[i]+90);
        }
        ypr[i] = ypr[i]/10.0;
      }
          Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
  }

    int16_t Roll = ypr[0];
    int16_t Pitch = ypr[1];
    int16_t Yaw = ypr[2];

  if (deviceConnected) {

    uint8_t encodedData[12];

    encodedData[0] = static_cast<uint8_t>(aaWorld.x & 0xFF);
    encodedData[1] = static_cast<uint8_t>((aaWorld.x >> 8) & 0xFF);

    encodedData[2] = static_cast<uint8_t>(aaWorld.y & 0xFF);
    encodedData[3] = static_cast<uint8_t>((aaWorld.y >> 8) & 0xFF);

    encodedData[4] = static_cast<uint8_t>(aaWorld.z & 0xFF);
    encodedData[5] = static_cast<uint8_t>((aaWorld.z >> 8) & 0xFF);

    encodedData[6] = static_cast<uint8_t>(Roll & 0xFF);
    encodedData[7] = static_cast<uint8_t>((Roll >> 8) & 0xFF);

    encodedData[8] = static_cast<uint8_t>(Pitch & 0xFF);
    encodedData[9] = static_cast<uint8_t>((Pitch >> 8) & 0xFF);

    encodedData[10] = static_cast<uint8_t>(Yaw & 0xFF);
    encodedData[11] = static_cast<uint8_t>((Yaw >> 8) & 0xFF);

    pCharacteristic->setValue(encodedData, sizeof(encodedData));
    pCharacteristic->notify();

    Serial.println("KLSJLSJDLKSDKL");
  }

  delay(50);
}
