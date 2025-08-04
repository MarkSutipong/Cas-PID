#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi settings
const char* ssid = "Mark 2.4G";
const char* password = "LuticsCas";

const char* mqttServer = "mqtt.netpie.io";
const int mqttPort = 1883;
const char* mqttClientID = "66417b01-db84-45c2-9a1c-f213d46b3d34"; 
const char* mqttUsername = "YfGSydnfzNHzPGaCdEyW29DtYcrCVdmv"; 
const char* mqttPassword = "rwftXYYY12VeaQ8uzQyPQZoSkMXQ7ixb"; 

WiFiClient espClient;
PubSubClient client(espClient);


const int servoPin = 23;
const int freq = 50;
const int channel = 0;
const int resolution = 16;

#define TRIG_PIN  5
#define ECHO_PIN  4

const float BEAM_LENGTH = 30.0;
const float setpoint = BEAM_LENGTH / 2.0; // 15 cm

// ---------- PID base params ----------
float Kp_base = 3.0;
float Kp_near = 5.0;           // เพิ่มแรงเมื่อ error เล็ก
float Ki = 0.12;
float Kd = 0.06;
const float FEEDFORWARD = 2.5; // bias เล็กๆ ข้ามแรงเสียดทาน

float integral = 0.0;
float lastError = 0.0;
unsigned long lastTime = 0;

// ---------- smoothing ----------
const int MA_SIZE = 5;
float posBuffer[MA_SIZE];
int maIndex = 0;
bool maFilled = false;

// ---------- servo constraints ----------
const float MAX_ADJUST_ANGLE = 60.0; // ขยายมุมให้แรงขึ้น
const int SERVO_CENTER = 90;
const int SERVO_MIN = 50;
const int SERVO_MAX = 130;
const float MIN_EFFECTIVE_OUTPUT = 5.0; // ให้มี actuation ขั้นต่ำ

// ---------- โซนกลางหยุด (dead zone) ที่คุณขอ ----------
const float CENTER_LOW = 12.0;   // ถ้า position อยู่ในช่วงนี้ให้หยุดผลัก
const float CENTER_HIGH = 13.0;

// ---------- กรณีไวน้ำ / รถออกไปขวาสุด (ระยะ >= 24) ----------
const float FAR_THRESHOLD = 24.0;
const int RECOVER_ANGLE = 65; // บังคับเซอร์โวไปมุมนี้เพื่อลากกลับ

// ---------- ฟังก์ชันช่วย ----------
uint32_t angleToDuty(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  float minPulse = 500.0;   // us
  float maxPulse = 2500.0;  // us
  float pulse = minPulse + ((float)angle / 180.0) * (maxPulse - minPulse);
  float period = 1000000.0 / freq;
  float dutyFraction = pulse / period;
  uint32_t maxDuty = (1 << resolution) - 1;
  return (uint32_t)(dutyFraction * maxDuty + 0.5);
}

float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1.0;
  return (duration * 0.0343) / 2.0;
}

float smoothPosition(float newPos) {
  posBuffer[maIndex] = newPos;
  maIndex = (maIndex + 1) % MA_SIZE;
  if (maIndex == 0) maFilled = true;
  int count = maFilled ? MA_SIZE : maIndex;
  float sum = 0;
  for (int i = 0; i < count; i++) sum += posBuffer[i];
  return sum / count;
}

// ปรับมุมเซอร์โวจาก output PID (รวม feedforward + minimum actuation)
int applyServoFromOutput(float output) {
  if (output > 0) output += FEEDFORWARD;
  else if (output < 0) output -= FEEDFORWARD;

  if (fabs(output) > 0 && fabs(output) < MIN_EFFECTIVE_OUTPUT) {
    output = (output > 0) ? MIN_EFFECTIVE_OUTPUT : -MIN_EFFECTIVE_OUTPUT;
  }

  float angleAdjust = constrain(output, -MAX_ADJUST_ANGLE, MAX_ADJUST_ANGLE);
  int targetAngle = SERVO_CENTER + (int)angleAdjust;
  targetAngle = constrain(targetAngle, SERVO_MIN, SERVO_MAX);
  uint32_t duty = angleToDuty(targetAngle);
  ledcWrite(channel, duty);
  return targetAngle;
}

// ฟังก์ชันพิเศษ: บังคับไปมุม RECOVER_ANGLE เมื่อเจอว่ารถอยู่ไกล (เงื่อนไขที่คุณขอ)
void applyRecoverAngle() {
  int targetAngle = RECOVER_ANGLE;
  targetAngle = constrain(targetAngle, SERVO_MIN, SERVO_MAX);
  uint32_t duty = angleToDuty(targetAngle);
  ledcWrite(channel, duty);
  Serial.printf(">> RECOVER mode: servo forced to %d°\n", targetAngle);
}

void setup() {
  Serial.begin(115200);
   WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  reconnect(); // connect to MQTT
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(servoPin, channel);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  for (int i = 0; i < MA_SIZE; i++) posBuffer[i] = setpoint;
  lastTime = millis();
  Serial.println("เริ่มระบบ PID+deadzone+recover (เพิ่มตามที่ขอ)");
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) return;

  float rawPos = readDistanceCM();
  float position;

  if (rawPos < 0 || rawPos > BEAM_LENGTH) {
    // อ่านผิดพลาด: ไม่สะสม integral มากและถือว่าใกล้เป้า
    integral *= 0.8;
    position = setpoint;
  } else {
    position = smoothPosition(rawPos);
  }

  float error = setpoint - position;

  // กรณีพิเศษ: ถ้าระยะ >= FAR_THRESHOLD (รถอยู่ไกล/ออกไปขวาสุด) ให้ recover
  if (rawPos >= FAR_THRESHOLD) {
    applyRecoverAngle();
  } else {
    // dead zone ตรงกลาง 12-13 ซม. ให้หยุดผลัก
    bool inDeadZone = (position >= CENTER_LOW && position <= CENTER_HIGH);
    if (inDeadZone) {
      integral *= 0.9; // ลดการสะสมเมื่ออยู่กลาง
    } else {
      integral += error * dt;
    }
    integral = constrain(integral, -100, 100);

    // gain scheduling: ถ้า error เล็กใช้ Kp_near
    float Kp = (fabs(error) < 2.0) ? Kp_near : Kp_base;
    float derivative = (error - lastError) / dt;

    float output = inDeadZone ? 0.0 : (Kp * error + Ki * integral + Kd * derivative);
    int servoAngle = applyServoFromOutput(output);

    // พิมพ์ข้อมูลครบชุด รวมมุมที่เซอร์โวไป
    Serial.printf(
      "Raw: %.2f | Smooth: %.2f | Err: %.2f | InZone:%d | Kp:%.2f | Int:%.2f | Der:%.2f | Out:%.2f | Servo:%d\n",
      rawPos, position, error, inDeadZone ? 1 : 0, Kp, integral, derivative, output, servoAngle
    );
  }

  lastError = error;
  lastTime = now;
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // ส่งค่าไป NETPIE
  client.publish("@shadow/data", "{\"data\":{\"temp\":25.5}}");

  delay(50);
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  Serial.print("Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect(mqttClientID, mqttUsername, mqttPassword)) {
      Serial.println("connected");
      client.subscribe("@msg/#");  // Subscribe to incoming messages
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(10);
    }
  }
}
