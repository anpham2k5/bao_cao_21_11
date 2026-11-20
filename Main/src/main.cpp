#include <Arduino.h>
#include <Servo.h>

// Servo tay gắp
#define PIN_SERVO         PB8

// Driver L298N (Động cơ giảm tốc vàng nâng/hạ cánh tay)
#define PIN_MOTOR_IN1     PB0
#define PIN_MOTOR_IN2     PB1
#define PIN_MOTOR_ENA     PA0   

const float TARGET_SENSOR2    = 7.0f;  // Ngưỡng kẹp vừa đủ của Cảm biến 2 (Mạch 2)
const float TARGET_SENSOR3    = 9.5f;  // Ngưỡng kẹp vừa đủ của Cảm biến 3 (Mạch 3)
const float MAX_SAFE_FORCE    = 12.0f; // Ngưỡng an toàn tối đa (chống bóp nát vật)

const int SERVO_OPEN_ANGLE    = 0;     // Góc mở ban đầu của servo (độ)
const int SERVO_MAX_CLOSE     = 80;    // Góc đóng tối đa (chặn cơ khí tránh quá tải)

const unsigned long TIME_LIFT  = 15000; 
const unsigned long TIME_HOLD  = 1000;  
const unsigned long TIME_LOWER = 15000; 
const int MOTOR_SPEED          = 255;  


HardwareSerial SerialSensor3(PA3, PA2); 

Servo gripperServo;

float forceSensor2 = 0.0f; 
float forceSensor3 = 0.0f; 
int currentServoAngle = SERVO_OPEN_ANGLE;

enum ArmState {
  STATE_GRIP,       // Kẹp servo từ từ theo phản hồi lực
  STATE_LIFT,       // Động cơ L298N nâng cánh tay lên
  STATE_HOLD,       // Giữ vật trên cao
  STATE_LOWER,      // Động cơ L298N hạ cánh tay xuống
  STATE_RELEASE,    // Mở kẹp servo thả vật
  STATE_DONE        // Hoàn thành chu trình (dừng hẳn, chờ bấm nút RESET)
};

ArmState currentState = STATE_GRIP;
unsigned long stateTimer = 0;
unsigned long gripStepTimer = 0;

String bufferSensor2 = "";
String bufferSensor3 = "";

void motorStop() {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_ENA, LOW);
}

void motorLift(int speed) {
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_ENA, HIGH);
}

void motorLower(int speed) {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, HIGH);
  digitalWrite(PIN_MOTOR_ENA, HIGH);
}

// ==================== ĐỌC DỮ LIỆU TỪ 2 CỔNG UART ====================
void readSensors() {
  // Đọc từ Cảm biến 2 (UART1 - Chân PA10)
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      bufferSensor2.trim();
      if (bufferSensor2.length() > 0) {
        forceSensor2 = bufferSensor2.toFloat();
      }
      bufferSensor2 = "";
    } else if (c != '\r') {
      if (bufferSensor2.length() < 30) {
        bufferSensor2 += c;
      }
    }
  }

  // Đọc từ Cảm biến 3 (UART2 - Chân PA3)
  while (SerialSensor3.available() > 0) {
    char c = (char)SerialSensor3.read();
    if (c == '\n') {
      bufferSensor3.trim();
      if (bufferSensor3.length() > 0) {
        forceSensor3 = bufferSensor3.toFloat();
      }
      bufferSensor3 = "";
    } else if (c != '\r') {
      if (bufferSensor3.length() < 30) {
        bufferSensor3 += c;
      }
    }
  }
}

void setup() {
  // Cấu hình 2 cổng UART ở tốc độ 115200
  Serial.begin(115200);         // UART1: RX = PA10
  SerialSensor3.begin(115200);  // UART2: RX = PA3

  // Cấu hình chân motor L298N
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_ENA, OUTPUT);

  motorStop();

  gripperServo.attach(PIN_SERVO);
  gripperServo.write(SERVO_OPEN_ANGLE);
  currentServoAngle = SERVO_OPEN_ANGLE;


  delay(2500);

  // Bắt đầu chu trình kẹp
  currentState = STATE_GRIP;
  gripStepTimer = millis();
}

void loop() {
  readSensors();

  switch (currentState) {
    
    case STATE_GRIP:
      // Cứ mỗi 100ms nhích servo thêm 1 độ
      if (millis() - gripStepTimer >= 100) {
        gripStepTimer = millis();

        // ĐIỀU KIỆN 1: Cả hai cảm biến đều đạt ngưỡng kẹp giữ của riêng nó
        bool bothGripped = (forceSensor2 >= TARGET_SENSOR2) && (forceSensor3 >= TARGET_SENSOR3);

        // ĐIỀU KIỆN 2 (CHỐT AN TOÀN): 1 bên bị dồn lực vượt ngưỡng tối đa (chống bóp nát)
        bool overForceProtect = (forceSensor2 >= MAX_SAFE_FORCE) || (forceSensor3 >= MAX_SAFE_FORCE);

        // ĐIỀU KIỆN 3: Đã chạm góc đóng tối đa (chặn cơ khí)
        bool maxAngleReached = (currentServoAngle >= SERVO_MAX_CLOSE);

        // NẾU THỎA MÃN BẤT KỲ ĐIỀU KIỆN NÀO TRÊN -> DỪNG SERVO & CHUYỂN NÂNG CÁNH TAY
        if (bothGripped || overForceProtect || maxAngleReached) {
          currentState = STATE_LIFT;
          stateTimer = millis();
          motorLift(MOTOR_SPEED); // Bắt đầu nâng cánh tay
        } else {
          currentServoAngle++;
          gripperServo.write(currentServoAngle);
        }
      }
      break;

    case STATE_LIFT:
      if (millis() - stateTimer >= TIME_LIFT) {
        motorStop(); // Dừng động cơ
        currentState = STATE_HOLD;
        stateTimer = millis();
      }
      break;

    case STATE_HOLD:
      if (millis() - stateTimer >= TIME_HOLD) {
        currentState = STATE_LOWER;
        stateTimer = millis();
        motorLower(MOTOR_SPEED); // Bắt đầu hạ cánh tay xuống
      }
      break;

    case STATE_LOWER:
      if (millis() - stateTimer >= TIME_LOWER) {
        motorStop(); // Dừng động cơ
        currentState = STATE_RELEASE;
        gripperServo.write(SERVO_OPEN_ANGLE); // Mở kẹp servo để thả vật
        stateTimer = millis();
      }
      break;

    case STATE_RELEASE:
      if (millis() - stateTimer >= 1000) { // Chờ 1 giây để servo mở hết góc
        currentServoAngle = SERVO_OPEN_ANGLE;
        currentState = STATE_DONE;
      }
      break;

    case STATE_DONE:
      motorStop();
      break;
  }
}
