#include <PinChangeInterrupt.h>

// --- PINOS MOTORES ---
#define PIN_L_DIR 4
#define PIN_L_PWM 5
#define PIN_L_BREAK 11
#define PIN_R_DIR 7
#define PIN_R_PWM 9
#define PIN_R_BREAK 13

// --- PINOS ODOMETRIA ---
#define ODO_L 10 
#define ODO_R 6  

// --- AJUSTES DE POTÊNCIA E CORREÇÃO ---
const int MAX_PWM = 40; // Aumentado para ter torque (ajuste se ficar muito rápido)         
const int MIN_PWM = 10;  // Força mínima para vencer a inércia           
const float TRIM_ESQ = 0.000;     
const float TRIM_DIR = 0.000;        

// --- VARIÁVEIS DE ODOMETRIA ---
volatile long pulsosEsq = 0;
volatile long pulsosDir = 0;
unsigned long tempoAnterior = 0;

// --- VARIÁVEIS DE CONTROLE ROS ---
float target_ros_E = 0;
float target_ros_D = 0;
unsigned long last_cmd_time = 0;
const float MAX_VEL_M_S = 1.0; 

void setup() {
  Serial.begin(115200);
  pinMode(PIN_L_DIR, OUTPUT); pinMode(PIN_R_DIR, OUTPUT);
  pinMode(PIN_L_BREAK, OUTPUT); pinMode(PIN_R_BREAK, OUTPUT);

  pinMode(ODO_L, INPUT_PULLUP);
  pinMode(ODO_R, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPCINT(ODO_L), countEsq, RISING);
  attachPinChangeInterrupt(digitalPinToPCINT(ODO_R), countDir, RISING);
}

void loop() {
  // 1. LEITURA COMANDOS ROS 2
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("CMD:")) {
      int separator = input.indexOf(';', 4);
      if (separator != -1) {
        float vE = input.substring(4, separator).toFloat();
        float vD = input.substring(separator + 1).toFloat();
        target_ros_E = (vE / MAX_VEL_M_S) * 100.0;
        target_ros_D = (vD / MAX_VEL_M_S) * 100.0;
        last_cmd_time = millis(); 
      }
    }
  }

  // 2. CONTROLE DE SEGURANÇA (Watchdog ROS)
  if (millis() - last_cmd_time < 500) {
    moveMotor(constrain(target_ros_E, -100, 100), constrain(target_ros_D, -100, 100));
  } else {
    moveMotor(0, 0); // Para imediatamente se o teleop parar de enviar
  }

  // 3. ENVIO DOS PULSOS CRUS PARA O ROS
  unsigned long tempoAtual = millis();
  unsigned long dt_ms = tempoAtual - tempoAnterior;
  
  if (dt_ms >= 20) {
    // Formato: ODO:pulsosEsq;pulsosDir;dt_ms
    Serial.print("ODO:");
    Serial.print(pulsosEsq); Serial.print(";");
    Serial.print(pulsosDir); Serial.print(";");
    Serial.println(dt_ms);
    tempoAnterior = tempoAtual;
  }
}

void moveMotor(int velE, int velD) {
  if(abs(velE) < 5) velE = 0;
  if(abs(velD) < 5) velD = 0;

  int magE = abs(velE);
  int magD = abs(velD);

  magE = constrain(magE + (magE * TRIM_ESQ), 0, 100);
  magD = constrain(magD + (magD * TRIM_DIR), 0, 100);

  if (magE != 0) {
    int pwmE = map(magE, 5, 100, MIN_PWM, MAX_PWM);
    digitalWrite(PIN_L_BREAK, LOW);
    digitalWrite(PIN_L_DIR, (velE > 0)); 
    analogWrite(PIN_L_PWM, pwmE);
  } else {
    digitalWrite(PIN_L_BREAK, HIGH);
    analogWrite(PIN_L_PWM, 0);
  }

  if (magD != 0) {
    int pwmD = map(magD, 5, 100, MIN_PWM, MAX_PWM);
    digitalWrite(PIN_R_BREAK, LOW);
    digitalWrite(PIN_R_DIR, (velD < 0)); 
    analogWrite(PIN_R_PWM, pwmD);
  } else {
    digitalWrite(PIN_R_BREAK, HIGH);
    analogWrite(PIN_R_PWM, 0);
  }
}

// --- FUNÇÕES DE INTERRUPÇÃO LIMPAS ---
void countEsq() { 
  if (digitalRead(PIN_L_DIR) == HIGH) pulsosEsq--; else pulsosEsq++; 
}
void countDir() { 
  if (digitalRead(PIN_R_DIR) == LOW) pulsosDir++; else pulsosDir--; 
}
