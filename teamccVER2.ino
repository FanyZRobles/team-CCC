#include <Wire.h>
#include <MPU6050_tockn.h>
#include <ESP32Servo.h>
#include <NewPing.h>

// ========== CONFIGURACIÓN DE PINES ==========
// Motores
#define LEFT_IN1 25
#define LEFT_IN2 26
#define LEFT_IN3 27
#define LEFT_IN4 33
#define ENA 12

#define RIGHT_IN1 14
#define RIGHT_IN2 13
#define RIGHT_IN3 15
#define RIGHT_IN4 2
#define ENB 4

// Sensores Ultrasónicos
#define TRIG_FRONT 23
#define ECHO_FRONT 35
#define TRIG_LEFT 16
#define ECHO_LEFT 39
#define TRIG_RIGHT 17
#define ECHO_RIGHT 18
#define MAX_DISTANCE 200

// Servo
#define SERVO_PIN 19

// IMU
#define SDA_PIN 21
#define SCL_PIN 22

// ========== OBJETOS GLOBALES ==========
MPU6050 mpu6050(Wire);
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);
Servo panServo;

// ========== CONSTANTES ==========
const float TILT_THRESHOLD = 15.0;  // 15 grados
const int OBSTACLE_DISTANCE = 30;   // 30 cm
const int SPEED_NORMAL = 200;       // 0-255
const int SPEED_SLOW = 120;

// ========== VARIABLES GLOBALES ==========
int targetX = -1;
bool obstacleDetected = false;
//uint8_t imageBuffer[320*240*2]; // Buffer para imagen QVGA RGB565
size_t imageSize = 0;
size_t bytesReceived = 0;
bool receivingImage = false;
uint32_t imageHeader = 0xAA55AA55;

// ========== CONFIGURACIÓN INICIAL ==========

#define IMAGE_WIDTH 160   // QQVGA
#define IMAGE_HEIGHT 120
uint8_t* imageBuffer = NULL; // Puntero en lugar de array


void setup() {
  Serial.begin(115200);

  #if CONFIG_SPIRAM_SUPPORT
    imageBuffer = (uint8_t*)ps_malloc(IMAGE_WIDTH * IMAGE_HEIGHT * 2);
  #else
    imageBuffer = (uint8_t*)malloc(80*60*2); // Más pequeño si no hay PSRAM
  #endif
  
  if(!imageBuffer) {
    Serial.println("Error: No se pudo asignar memoria para imagen");
    while(1);
  }
  
  // Configurar pines de motores
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_IN3, OUTPUT);
  pinMode(LEFT_IN4, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Inicializar servo
  panServo.attach(SERVO_PIN);
  panServo.write(90); // Posición central

  // Inicializar IMU
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  Serial.println("Sistema 4WD inicializado!");
}

// ========== BUCLE PRINCIPAL ==========
void loop() {
  static unsigned long lastUpdate = 0;
  
  // 1. Recibir datos de imagen
  receiveImageData();
  
  if(millis() - lastUpdate >= 100) {
    lastUpdate = millis();
    
    // 2. Lectura de sensores
    mpu6050.update();
    int frontDist = sonarFront.ping_cm();
    int leftDist = sonarLeft.ping_cm();
    int rightDist = sonarRight.ping_cm();

    // 3. Verificar inclinación
//    if(abs(mpu6050.getAngleX()) > TILT_THRESHOLD) {
//      emergencyStop();
//      return;
//    }

    // 4. Evitar obstáculos
    if(frontDist > 0 && frontDist < OBSTACLE_DISTANCE) {
      avoidObstacle(leftDist, rightDist);
      obstacleDetected = true;
    } else {
      obstacleDetected = false;
    }

    // 5. Seguimiento de objeto
    if(!obstacleDetected && imageSize > 0 && bytesReceived == imageSize) {
      targetX = detectRedObject(imageBuffer, 320, 240);
      
      if(targetX != -1) {
        followTarget();
      } else {
        //searchPattern();
      }
      
      // Reset para nueva imagen
      imageSize = 0;
      bytesReceived = 0;
    }
  }
}

// ========== FUNCIONES DE RECEPCIÓN DE IMAGEN ==========
void receiveImageData() {
  static uint32_t headerPos = 0;
  static uint32_t header = 0;
  
  while(Serial.available()) {
    uint8_t byte = Serial.read();
    
    if(!receivingImage) {
    // Buscar encabezado
      header = (header << 8) | byte;
      headerPos++;
      
      if(headerPos == 4) {
        if(header == imageHeader) {
          receivingImage = true;
          headerPos = 0;
        } else {
          header = 0;
          headerPos = 0;
        }
      }
    } else {
      if(imageSize == 0) {
        // Leer tamaño de imagen
        *((uint8_t*)&imageSize + bytesReceived) = byte;
        bytesReceived++;
        
        if(bytesReceived == sizeof(imageSize)) {
          bytesReceived = 0;
          if(imageSize > sizeof(imageBuffer)) {
            receivingImage = false;
            imageSize = 0;
          }
        }
      } else {
        // Recibir datos de imagen
        if(bytesReceived < imageSize) {
          imageBuffer[bytesReceived] = byte;
          bytesReceived++;
        } else {
          receivingImage = false;
        }
      }
    }
  }
}

// ========== DETECCIÓN DE COLOR ==========
int detectRedObject(uint8_t *img, int width, int height) {
  const int STEP = 4; // Saltar píxeles para mayor velocidad
  int redCount = 0;
  int totalX = 0;
  
  for(int y = 0; y < height; y += STEP) {
    for(int x = 0; x < width; x += STEP) {
      int index = (y * width + x) * 2;
      uint16_t pixel = (img[index+1] << 8) | img[index]; // Little-endian
      
      uint8_t r = (pixel >> 11) & 0x1F;
      uint8_t g = (pixel >> 5) & 0x3F;
      uint8_t b = pixel & 0x1F;
      
      // Detección rápida de rojo
      if(r > 24 && g < 20 && b < 20) {
        redCount++;
        totalX += x;
      }
    }
  }
  return (redCount > 20) ? (totalX / redCount) : -1;
}

// ========== FUNCIONES DE MOVIMIENTO ==========
void followTarget() {
  int error = targetX - 160; // Centro de imagen QVGA (320x240)
  
  if(abs(error) > 30) {
    if(error < 0) {
      turnLeft(SPEED_NORMAL);
    } else {
      turnRight(SPEED_NORMAL);
    }
  } else {
    moveForward(SPEED_NORMAL);
  }
}

void avoidObstacle(int leftDist, int rightDist) {
  if(leftDist > rightDist && leftDist > OBSTACLE_DISTANCE) {
    turnLeft(SPEED_SLOW);
    delay(100);
  } else if(rightDist > OBSTACLE_DISTANCE) {
    turnRight(SPEED_SLOW);
    delay(100);
  } else {
    moveBackward(SPEED_SLOW);
    delay(250);
  }
}

void searchPattern() {
  static int pos = 0;
  int angles[] = {45, 90, 135};
  panServo.write(angles[pos]);
  pos = (pos + 1) % 3;
}

// ========== CONTROL DE MOTORES ==========
void moveForward(int speed) {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_IN3, HIGH);
  digitalWrite(LEFT_IN4, LOW);
  
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_IN3, HIGH);
  digitalWrite(RIGHT_IN4, LOW);
  
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void turnLeft(int speed) {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(LEFT_IN3, LOW);
  digitalWrite(LEFT_IN4, HIGH);
  
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_IN3, HIGH);
  digitalWrite(RIGHT_IN4, LOW);
  
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void turnRight(int speed) {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_IN3, HIGH);
  digitalWrite(LEFT_IN4, LOW);
  
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, HIGH);
  
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void moveBackward(int speed) {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(LEFT_IN3, LOW);
  digitalWrite(LEFT_IN4, HIGH);
  
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, HIGH);
  
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void emergencyStop() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_IN3, LOW);
  digitalWrite(LEFT_IN4, LOW);
  
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, LOW);
  
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
