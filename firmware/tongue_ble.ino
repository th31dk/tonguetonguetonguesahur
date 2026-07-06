/*
  Robotic Tongue - Bluetooth LE Control
  Board: Seeed XIAO ESP32C3
  Servos: 3x SG90
    - Back/forth -> D0
    - Yaw        -> D1
    - Pitch      -> D2

  Control: Bluetooth LE (Web Bluetooth from a browser page - see tongue_controller.html)
  This REPLACES the WiFi web server version - WiFi and BLE both running well
  at the same time is not reliable on this chip, so this sketch only uses BLE.

  Protocol (single BLE characteristic, write-only):
    Browser sends a plain text string, comma-separated, newline-terminated:
      "<yaw>,<pitch>,<backforth>,<circle>\n"
    Each of yaw/pitch/backforth is an integer -100..100 (joystick/slider position).
    circle is 0 or 1 (1 = spin button held/toggled on).

    While circle=1, the ESP32 ignores incoming yaw/pitch and drives them itself
    in a circular sweep. backforth is still taken from input while circling.

  Calibration (from your measured min/max pulse widths, in microseconds):
    Input value +100 maps to the "max" pulse width you measured.
    Input value -100 maps to the "min" pulse width you measured.
    (It's fine that some "min" numbers are numerically larger than "max" -
     that just means that servo's mechanical range is mounted in reverse.)
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

// ---------- BLE UUIDs (must match tongue_controller.html) ----------
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-abcdefabcdef"

// ---------- Pins ----------
const int PIN_BACKFORTH = D0;
const int PIN_YAW       = D1;
const int PIN_PITCH     = D2;

// ---------- Calibration: {pulse at -100, pulse at +100} in microseconds ----------
const int BF_PULSE_MIN  = 2419, BF_PULSE_MAX  = 1010;  // back/forth
const int YAW_PULSE_MIN = 2500, YAW_PULSE_MAX = 1000;  // yaw
const int PITCH_PULSE_MIN = 2240, PITCH_PULSE_MAX = 1000; // pitch

// ---------- Circle mode settings ----------
const unsigned long CIRCLE_PERIOD_MS = 2000; // time for one full circle
const float CIRCLE_RADIUS = 90.0;            // 0-100, how wide the circle sweep is

// ---------- Globals ----------
Servo servoBackForth, servoYaw, servoPitch;
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;

int inputYaw = 0, inputPitch = 0, inputBackForth = 0;
bool circleMode = false;

String rxBuffer = "";

// ---------- Helpers ----------
int valueToPulse(int value, int pulseAtMin, int pulseAtMax) {
  value = constrain(value, -100, 100);
  return map(value, -100, 100, pulseAtMin, pulseAtMax);
}

void applyServos(int yawVal, int pitchVal, int bfVal) {
  servoYaw.writeMicroseconds(valueToPulse(yawVal, YAW_PULSE_MIN, YAW_PULSE_MAX));
  servoPitch.writeMicroseconds(valueToPulse(pitchVal, PITCH_PULSE_MIN, PITCH_PULSE_MAX));
  servoBackForth.writeMicroseconds(valueToPulse(bfVal, BF_PULSE_MIN, BF_PULSE_MAX));
}

void parseCommand(const String& line) {
  // Expected: "yaw,pitch,backforth,circle"
  int p1 = line.indexOf(',');
  int p2 = line.indexOf(',', p1 + 1);
  int p3 = line.indexOf(',', p2 + 1);
  if (p1 < 0 || p2 < 0 || p3 < 0) return; // malformed, ignore

  inputYaw       = line.substring(0, p1).toInt();
  inputPitch     = line.substring(p1 + 1, p2).toInt();
  inputBackForth = line.substring(p2 + 1, p3).toInt();
  circleMode     = line.substring(p3 + 1).toInt() != 0;
}

// ---------- BLE callbacks ----------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    deviceConnected = true;
    Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer* server) override {
    deviceConnected = false;
    Serial.println("BLE client disconnected, restarting advertising");
    BLEDevice::startAdvertising();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String value = characteristic->getValue();
    for (size_t i = 0; i < value.length(); i++) {
      char c = value[i];
      if (c == '\n') {
        parseCommand(rxBuffer);
        rxBuffer = "";
      } else {
        rxBuffer += c;
      }
    }
    // Handle commands sent without a trailing newline too
    if (rxBuffer.length() > 0 && rxBuffer.indexOf(',') >= 0) {
      parseCommand(rxBuffer);
      rxBuffer = "";
    }
  }
};

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);

  servoBackForth.setPeriodHertz(50);
  servoYaw.setPeriodHertz(50);
  servoPitch.setPeriodHertz(50);

  servoBackForth.attach(PIN_BACKFORTH, 500, 2500);
  servoYaw.attach(PIN_YAW, 500, 2500);
  servoPitch.attach(PIN_PITCH, 500, 2500);

  applyServos(0, 0, 0); // center on boot

  BLEDevice::init("TongueBot");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCharacteristic->setCallbacks(new CommandCallbacks());
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising as 'TongueBot' - connect from the controller page");
}

void loop() {
  if (circleMode) {
    unsigned long t = millis() % CIRCLE_PERIOD_MS;
    float angle = (2.0 * PI * t) / CIRCLE_PERIOD_MS;
    int circleYaw   = (int)(CIRCLE_RADIUS * cos(angle));
    int circlePitch = (int)(CIRCLE_RADIUS * sin(angle));
    applyServos(circleYaw, circlePitch, inputBackForth);
  } else {
    applyServos(inputYaw, inputPitch, inputBackForth);
  }
  delay(15); // ~66Hz update, smooth enough for servos
}
