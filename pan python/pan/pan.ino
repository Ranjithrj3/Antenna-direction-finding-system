
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =====================================================
// CONFIGURATION
// =====================================================

#define ESPNOW_CHANNEL 1

// =====================================================
// PINS
// =====================================================

const int STEP_PIN = 18;
const int DIR_PIN  = 19;
const int EN_PIN   = 17;
const int HOME_SWITCH = 27;

// =====================================================
// MOTOR SETTINGS
// =====================================================

// TB6600: S1 ON, S2 OFF, S3 ON
// Selects 400 pulses/revolution on your driver's label.
// Assumes a direct-drive motor with no gearbox.

const long STEPS_PER_REV = 400;
const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0f;

// =====================================================
// HOMING SETTINGS
// =====================================================

const long HOMING_MAX_STEPS = 10000;

const unsigned int HOMING_DELAY_US = 5000;
const unsigned int MOVE_DELAY_US = 3000;

const unsigned int STEP_HIGH_US = 10;
const unsigned int DIR_SETUP_US = 10;

// =====================================================
// POSITION AND STATE
// =====================================================

float currentAngle = 0.0f;

volatile float receivedAngle = 0.0f;
volatile bool newAngle = false;

bool homed = false;

// =====================================================
// ESP-NOW PACKET
// Must match the ESP1 packet structure
// =====================================================

typedef struct
{
  float angle;
} PanAnglePacket;

PanAnglePacket incoming;

// =====================================================
// STEP PULSE
// =====================================================

void stepMotor(unsigned int lowDelayUs)
{
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_HIGH_US);

  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(lowDelayUs);
}

// =====================================================
// HOMING
// =====================================================

bool findHome()
{
  Serial.println();
  Serial.println("==============================");
  Serial.println("STARTING HOMING");
  Serial.println("==============================");

  // Switch is connected to GND:
  // LOW = pressed, HIGH = released

  if (digitalRead(HOME_SWITCH) == LOW)
  {
    Serial.println("HOME SWITCH ALREADY PRESSED");
    currentAngle = 0.0f;
    Serial.println("HOME DONE");
    return true;
  }

  Serial.println("HOME SWITCH RELEASED");
  Serial.println("Moving toward HOME...");

  // IMPORTANT:
  // This direction must physically move toward the switch.
  // If it moves away, change LOW to HIGH.

  digitalWrite(DIR_PIN, LOW);
  delayMicroseconds(DIR_SETUP_US);

  long steps = 0;

  while (digitalRead(HOME_SWITCH) == HIGH)
  {
    stepMotor(HOMING_DELAY_US);
    steps++;

    if (steps % 500 == 0)
    {
      Serial.print("Homing steps: ");
      Serial.println(steps);
    }

    if (steps >= HOMING_MAX_STEPS)
    {
      Serial.println("==============================");
      Serial.println("HOME ERROR");
      Serial.println("MAX HOMING STEPS REACHED");
      Serial.println("Check direction, switch and wiring");
      Serial.println("==============================");

      return false;
    }
  }

  // Debounce: verify switch remains pressed.
  delay(20);

  if (digitalRead(HOME_SWITCH) != LOW)
  {
    Serial.println("HOME ERROR: SWITCH NOT STABLE");
    return false;
  }

  currentAngle = 0.0f;

  Serial.println("==============================");
  Serial.println("HOME SWITCH DETECTED");
  Serial.println("HOME DONE");
  Serial.println("CURRENT ANGLE = 0");
  Serial.println("==============================");

  return true;
}

// =====================================================
// MOVE TO ABSOLUTE ANGLE
// =====================================================

void moveToAngle(float targetAngle)
{
  if (!homed)
  {
    Serial.println("MOVEMENT BLOCKED: NOT HOMED");
    return;
  }

  if (!isfinite(targetAngle) ||
      targetAngle < 0.0f ||
      targetAngle > 359.0f)
  {
    Serial.println("INVALID ANGLE");
    return;
  }

  float difference = targetAngle - currentAngle;

  Serial.println();
  Serial.println("==============================");
  Serial.println("PAN MOVEMENT");
  Serial.println("==============================");

  Serial.print("Current angle: ");
  Serial.println(currentAngle, 2);

  Serial.print("Target angle: ");
  Serial.println(targetAngle, 2);

  Serial.print("Difference: ");
  Serial.println(difference, 2);

  // Avoid unnecessary movements due to tiny differences.
  if (fabsf(difference) < 0.5f)
  {
    Serial.println("Already at target");
    return;
  }

  if (difference > 0.0f)
  {
    digitalWrite(DIR_PIN, HIGH);
  }
  else
  {
    digitalWrite(DIR_PIN, LOW);
    difference = -difference;
  }

  // Allow DIR signal to settle before STEP pulses.
  delayMicroseconds(DIR_SETUP_US);

  long steps = lroundf(difference * STEPS_PER_DEGREE);

  Serial.print("Steps required: ");
  Serial.println(steps);

  for (long i = 0; i < steps; i++)
  {
    stepMotor(MOVE_DELAY_US);
  }

  // This is the commanded position, not measured feedback.
  currentAngle = targetAngle;

  Serial.println("MOVE COMPLETE");

  Serial.print("CURRENT ANGLE: ");
  Serial.println(currentAngle, 2);
}

// =====================================================
// ESP-NOW RECEIVE CALLBACK
// =====================================================

void OnDataRecv(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int len)
{
  if (len != sizeof(PanAnglePacket))
  {
    return;
  }

  PanAnglePacket packet;
  memcpy(&packet, data, sizeof(packet));

  if (!isfinite(packet.angle) ||
      packet.angle < 0.0f ||
      packet.angle > 359.0f)
  {
    return;
  }

  receivedAngle = packet.angle;
  newAngle = true;
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  // Preserve your existing enable logic.
  // Verify this polarity against your TB6600 wiring.
  digitalWrite(EN_PIN, LOW);

  pinMode(HOME_SWITCH, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  esp_wifi_set_channel(
      ESPNOW_CHANNEL,
      WIFI_SECOND_CHAN_NONE
  );

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW INIT FAILED");

    while (true)
    {
      delay(1000);
    }
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP-NOW READY");

  delay(1000);

  homed = findHome();

  if (homed)
  {
    Serial.println("SYSTEM READY FOR ANGLE COMMANDS");
  }
  else
  {
    Serial.println("HOMING FAILED - MOVEMENT DISABLED");
  }
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  if (newAngle)
  {
    // Copy and clear the pending command.
    noInterrupts();
    float target = receivedAngle;
    newAngle = false;
    interrupts();

    moveToAngle(target);
  }

  delay(1);
}
