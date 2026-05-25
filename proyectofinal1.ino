#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <DabbleESP32.h>

#define DEVICE_NAME "RobotArana"

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ      50
#define SERVO_MIN_PULSE 102
#define SERVO_MAX_PULSE 512

// ── Pines HC-SR04 ────────────────────────────────────────────
#define TRIG_PIN 5
#define ECHO_PIN 18

// ── Estructura de servo ───────────────────────────────────────
struct Servo_t {
  uint8_t pin;
  int     minGrados;
  int     maxGrados;
  int     pieGrados;
};

// ── Tabla de servos ──────────────────────────────────────────
Servo_t pata1[3] = {  // Delantera Derecha
  {  0,  10,  110,  65  },   // hombro
  {  1,   0,  120,  20 },   // codo
  {  2,   0,  135, 5 },   // muñeca
};
Servo_t pata2[3] = {  // Trasera Derecha
  {  4,  10,  100,  15  },   // hombro
  {  5,   5,  140,  90  },   // codo
  {  6,   0,  120,  100  },   // muñeca
};
Servo_t pata3[3] = {  // Trasera Izquierda
  {  8,   0,   80,  40  },   // hombro
  {  9,   5,  140,  30 },   // codo
  { 10,   0,  125,  0   },   // muñeca
};
Servo_t pata4[3] = {  // Delantera Izquierda
  { 12,   0,   90,  30  },   // hombro
  { 13,   0,  130,  80  },   // codo
  { 14,   0,  120,  100  },   // muñeca
};

// ── Máquina de estados ───────────────────────────────────────
enum EstadoRobot {
  INIT,
  MANUAL,
  AUTOMATICO
};
EstadoRobot estadoActual = INIT;

// ── Tiempos de mitigación de corriente ───────────────────────
const unsigned long DELAY_INTER_PATA = 180;    // Tiempo para que termine un movimiento antes de empezar otro
const unsigned long DELAY_ANTI_CHISPA = 100;   // Pausa entre servos individuales en el encendido

// ── Variables de estado ──────────────────────────────────────
bool robotDePie = false;
int  direccionActual = 0;       // 1=adelante, -1=atrás, 2=der, 3=izq

// ── Parámetros del sensor ultrasonido ────────────────────────
float distanciaActual = 999.0;
const float DISTANCIA_MINIMA = 25.0;  // cm
int girosIntentados = 0;
int ultimaDir = 1;  // 1=Girar derecha, -1=Girar izquierda

uint16_t gradosPulso(int grados) {
  grados = constrain(grados, 0, 180);
  return (uint16_t)map(grados, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void moverServo(Servo_t &s, int grados) {
  grados = constrain(grados, s.minGrados, s.maxGrados);
  pca.setPWM(s.pin, 0, gradosPulso(grados));
}

void posicionDePie() {
  Serial.println("[ENERGÍA] Posicionando servos uno a uno...");
  for (int i = 0; i < 3; i++) {
    moverServo(pata1[i], pata1[i].pieGrados); delay(DELAY_ANTI_CHISPA);
    moverServo(pata2[i], pata2[i].pieGrados); delay(DELAY_ANTI_CHISPA);
    moverServo(pata3[i], pata3[i].pieGrados); delay(DELAY_ANTI_CHISPA);
    moverServo(pata4[i], pata4[i].pieGrados); delay(DELAY_ANTI_CHISPA);
  }
  robotDePie = true;
}

float leerDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duracion = pulseIn(ECHO_PIN, HIGH, 20000); 
  if (duracion == 0) return 999.0;
  
  return (duracion * 0.0343) / 2.0;
}

// ============================================================
//  SECUENCIAS DE MOVIMIENTO
// ============================================================
void marchaSECUENCIAL_Adelante() {
  // PATA 1
  moverServo(pata1[1], 0); delay(DELAY_INTER_PATA);
  moverServo(pata1[0], 110); delay(DELAY_INTER_PATA);
  moverServo(pata1[1], 60); delay(DELAY_INTER_PATA);
  moverServo(pata1[0], 10); delay(DELAY_INTER_PATA);
  moverServo(pata1[1], 0); delay(DELAY_INTER_PATA);
  moverServo(pata1[0], 50); delay(DELAY_INTER_PATA);
  moverServo(pata1[1], pata1[1].pieGrados); delay(DELAY_INTER_PATA);

  // PATA 4
  moverServo(pata4[1], 115); delay(DELAY_INTER_PATA);
  moverServo(pata4[0], 10); delay(DELAY_INTER_PATA);
  moverServo(pata4[1], 60); delay(DELAY_INTER_PATA);
  moverServo(pata4[0], 100); delay(DELAY_INTER_PATA);
  moverServo(pata4[1], 125); delay(DELAY_INTER_PATA);
  moverServo(pata4[0], 30); delay(DELAY_INTER_PATA);
  moverServo(pata4[1], pata4[1].pieGrados); delay(DELAY_INTER_PATA);

  // PATA 3
  moverServo(pata3[1], 10); delay(DELAY_INTER_PATA);
  moverServo(pata3[0], 0); delay(DELAY_INTER_PATA);
  moverServo(pata3[1], 50); delay(DELAY_INTER_PATA);
  moverServo(pata3[0], 90); delay(DELAY_INTER_PATA);
  moverServo(pata3[1], 10); delay(DELAY_INTER_PATA);
  moverServo(pata3[0], 45); delay(DELAY_INTER_PATA);
  moverServo(pata3[1], pata3[1].pieGrados); delay(DELAY_INTER_PATA);

  // PATA 2
  moverServo(pata2[1], 125); delay(DELAY_INTER_PATA);
  moverServo(pata2[0], 80); delay(DELAY_INTER_PATA);
  moverServo(pata2[1], 60); delay(DELAY_INTER_PATA);
  moverServo(pata2[0], -10); delay(DELAY_INTER_PATA);
  moverServo(pata2[1], 125); delay(DELAY_INTER_PATA);
  moverServo(pata2[0], 25); delay(DELAY_INTER_PATA);
  moverServo(pata2[1], pata2[1].pieGrados); delay(DELAY_INTER_PATA);
}

void marchaSECUENCIAL_Atras() {
  moverServo(pata1[1], 0); delay(DELAY_INTER_PATA);
  moverServo(pata1[0], 10); delay(DELAY_INTER_PATA);
  moverServo(pata1[1], 40); delay(DELAY_INTER_PATA);
  moverServo(pata1[0], 110); delay(DELAY_INTER_PATA);
  moverServo(pata1[1], 0); delay(DELAY_INTER_PATA);
  moverServo(pata1[0], 50); delay(DELAY_INTER_PATA);
  moverServo(pata1[1], pata1[1].pieGrados); delay(DELAY_INTER_PATA);

  moverServo(pata3[1], 10); delay(DELAY_INTER_PATA);
  moverServo(pata3[0], 90); delay(DELAY_INTER_PATA);
  moverServo(pata3[1], 50); delay(DELAY_INTER_PATA);
  moverServo(pata3[0], 0); delay(DELAY_INTER_PATA);
  moverServo(pata3[1], 10); delay(DELAY_INTER_PATA);
  moverServo(pata3[0], 45); delay(DELAY_INTER_PATA);
  moverServo(pata3[1], pata3[1].pieGrados); delay(DELAY_INTER_PATA);

  moverServo(pata4[1], 115); delay(DELAY_INTER_PATA);
  moverServo(pata4[0], 100); delay(DELAY_INTER_PATA);
  moverServo(pata4[1], 60); delay(DELAY_INTER_PATA);
  moverServo(pata4[0], 10); delay(DELAY_INTER_PATA);
  moverServo(pata4[1], 125); delay(DELAY_INTER_PATA);
  moverServo(pata4[0], 30); delay(DELAY_INTER_PATA);
  moverServo(pata4[1], pata4[1].pieGrados); delay(DELAY_INTER_PATA);

  moverServo(pata2[1], 125); delay(DELAY_INTER_PATA);
  moverServo(pata2[0], 0); delay(DELAY_INTER_PATA);
  moverServo(pata2[1], 60); delay(DELAY_INTER_PATA);
  moverServo(pata2[0], 80); delay(DELAY_INTER_PATA);
  moverServo(pata2[1], 125); delay(DELAY_INTER_PATA);
  moverServo(pata2[0], 25); delay(DELAY_INTER_PATA);
  moverServo(pata2[1], pata2[1].pieGrados); delay(DELAY_INTER_PATA);
}

// NUEVA FUNCIÓN: Ahora hace el movimiento opuesto exacto para girar a la DERECHA
void girarDerecha() {
  moverServo(pata1[1], 0); delay(DELAY_INTER_PATA);
  moverServo(pata1[0], 10); delay(DELAY_INTER_PATA);
  moverServo(pata1[1], 40); delay(DELAY_INTER_PATA);
  moverServo(pata1[0], 110); delay(DELAY_INTER_PATA);
  moverServo(pata1[1], 0); delay(DELAY_INTER_PATA);
  moverServo(pata1[0], 50); delay(DELAY_INTER_PATA);
  moverServo(pata1[1], pata1[1].pieGrados); delay(DELAY_INTER_PATA);
  
  moverServo(pata4[1], 115); delay(DELAY_INTER_PATA);
  moverServo(pata4[0], 10); delay(DELAY_INTER_PATA);
  moverServo(pata4[1], 60); delay(DELAY_INTER_PATA);
  moverServo(pata4[0], 100); delay(DELAY_INTER_PATA);
  moverServo(pata4[1], 125); delay(DELAY_INTER_PATA);
  moverServo(pata4[0], 30); delay(DELAY_INTER_PATA);
  moverServo(pata4[1], pata4[1].pieGrados); delay(DELAY_INTER_PATA);

}

// CORREGIDO: Vuestra secuencia buena original ahora se llama correctamente GIRAR IZQUIERDA
void girarIzquierda() {
  moverServo(pata1[1], 0); delay(DELAY_INTER_PATA);
  moverServo(pata1[0], 110); delay(DELAY_INTER_PATA);
  moverServo(pata1[1], 60); delay(DELAY_INTER_PATA);
  moverServo(pata1[0], 10); delay(DELAY_INTER_PATA);
  moverServo(pata1[1], 0); delay(DELAY_INTER_PATA);
  moverServo(pata1[0], 50); delay(DELAY_INTER_PATA);
  moverServo(pata1[1], pata1[1].pieGrados); delay(DELAY_INTER_PATA);

  moverServo(pata4[1], 115); delay(DELAY_INTER_PATA);
  moverServo(pata4[0], 100); delay(DELAY_INTER_PATA);
  moverServo(pata4[1], 60); delay(DELAY_INTER_PATA);
  moverServo(pata4[0], 10); delay(DELAY_INTER_PATA);
  moverServo(pata4[1], 125); delay(DELAY_INTER_PATA);
  moverServo(pata4[0], 30); delay(DELAY_INTER_PATA);
  moverServo(pata4[1], pata4[1].pieGrados); delay(DELAY_INTER_PATA);
}

// ============================================================
//  SETUP Y LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  
  Dabble.begin(DEVICE_NAME);
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(SERVO_FREQ);
  delay(100);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
    estadoActual = INIT;
}

void loop() {
  Dabble.processInput();
  
  if (GamePad.isStartPressed()) {
    if (estadoActual != MANUAL) {
      estadoActual = MANUAL;
      posicionDePie();
      delay(300);
    }
  }
  
  if (GamePad.isSelectPressed()) {
    if (estadoActual != AUTOMATICO) {
      estadoActual = AUTOMATICO;
      posicionDePie();
      girosIntentados = 0;
      delay(300);
    }
  }
  
  switch (estadoActual) {
    case INIT:
      if (!robotDePie) posicionDePie();
      break;
    case MANUAL:
      modoManual();
      break;
    case AUTOMATICO:
      modoAuto();
      break;
  }
}

// ============================================================
//  MODO MANUAL
// ============================================================
void modoManual() {
  int input = 0;
  if (GamePad.isUpPressed()) input = 1;
  else if (GamePad.isDownPressed()) input = -1;
  else if (GamePad.isRightPressed()) input = 2;
  else if (GamePad.isLeftPressed()) input = 3;
  
  if (input != 0) {
    direccionActual = input;
    switch (input) {
      case 1:  marchaSECUENCIAL_Adelante(); break;
      case -1: marchaSECUENCIAL_Atras();    break;
      case 2:  girarDerecha();              break; // Llama a la nueva lógica invertida
      case 3:  girarIzquierda();            break; // Llama a la secuencia buena original
    }
  } 
  else if (direccionActual != 0) {
    posicionDePie();
    direccionActual = 0;
  }
}

// ============================================================
//  MODO AUTOMÁTICO
// ============================================================
void modoAuto() {
  distanciaActual = leerDistancia();  
  if (distanciaActual > 0 && distanciaActual < DISTANCIA_MINIMA) {
    posicionDePie();
    delay(150);
    marchaSECUENCIAL_Atras();
    
    if (ultimaDir == 1) {
      girarIzquierda();
    } else {
      girarDerecha();
    }
    delay(150);
    
    girosIntentados++;
    if (girosIntentados >= 2) {
      ultimaDir = -ultimaDir; 
      girosIntentados = 0;
    }
  } 
  else {
    marchaSECUENCIAL_Adelante();
    girosIntentados = 0; 
  }
}