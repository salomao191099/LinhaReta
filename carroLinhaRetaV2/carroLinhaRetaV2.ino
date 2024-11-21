#include <PID_v1_bc.h> // Biblioteca PID

const int motorEsqFrente = 7;
const int motorEsqTras = 6;
const int motorDirFrente = 9;
const int motorDirTras = 8;
const int pwmEsq = 10;  // Pino PWM para motor esquerdo
const int pwmDir = 11;  // Pino PWM para motor direito
float percorre = 0;
int velEsq = 95;
int velDir = 95;

int ce = 0;
int cd = 0;

// Variáveis de controle do carro
int baseSpeed = 150;  // Velocidade base
int SpeedD = baseSpeed; //121
int SpeedE = baseSpeed; //78
int Esensor = 2;
int Dsensor = 3;
volatile double Epulso = 0;
volatile double Dpulso = 0;
volatile double Epulso2 = 50;
volatile double Dpulso2 = 50;
volatile double Epulso3 = 0;
volatile double Dpulso3 = 0;


// Variáveis PID
double Setpoint, Input, Output;
double Kp = 0.2, Ki = 12.0, Kd = 0.7;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Milis
double tempoInicial;
double tempoAtual;
double pausa = 10000;

void setup() {
  // Inicializa a comunicação serial para receber comandos via Bluetooth
  Serial.begin(9600);

  // Configuração de pinos do carro
  pinMode(motorEsqFrente, OUTPUT);
  pinMode(motorEsqTras, OUTPUT);
  pinMode(motorDirFrente, OUTPUT);
  pinMode(motorDirTras, OUTPUT);
  pinMode(pwmEsq, OUTPUT);
  pinMode(pwmDir, OUTPUT);
  pinMode(Esensor, INPUT);
  pinMode(Dsensor, INPUT);

  // Configurações do carro
  //forward();
  attachInterrupt(digitalPinToInterrupt(Esensor), Econta, FALLING);
  attachInterrupt(digitalPinToInterrupt(Dsensor), Dconta, FALLING);

  // Inicialização do PID
//  Setpoint = 0;  // Queremos que a diferença entre Epulso e Dpulso seja zero
//  myPID.SetMode(AUTOMATIC);
  delay(5000);
}

void loop() {
  forward();
  Serial.print("Epulso: ");
  Serial.print(Epulso);
  Serial.print(" | Dpulso: ");
  Serial.println(Dpulso);
  if(Epulso>Dpulso){
    Serial.println("Diminuir Esqueda");
    cd += 1;
    ce = 1;
    if(cd >2){
      cd = 1;
    }
    velDir += 1*cd;
    velEsq -= 1*ce;
    //delay(1000);
  }
  if(Dpulso>Epulso){
    Serial.println("Diminuir Direita");
    cd  = 1;
    ce += 1;
    if(ce > 2){
      ce= 1;
    }
    velDir -= 1*cd;
    velEsq += 1*ce;
    //delay(1000);
  }
  if((Epulso>400)||(Dpulso>400)){
    calcula((Epulso+Dpulso)/2);
    Stop();
    while(true){
      //Serial.println(percorre/100);
    }
    delay(60000);
  }
  //delay(500);
}

void balancoVelocidade() {
  Input = Epulso - Dpulso;
  myPID.Compute();
  
  SpeedD = baseSpeed + Output;
  SpeedE = baseSpeed - Output;

  // Garante que as velocidades estejam dentro dos limites
  SpeedD = constrain(SpeedD, 0, 255);
  SpeedE = constrain(SpeedE, 0, 255);

  //analogWrite(pwmEsq, SpeedD);
  //analogWrite(pwmDir, SpeedE);
}

void Econta() {
  Epulso++;
}

void Dconta() {
  Dpulso++;
}

void tempo() {
  tempoAtual = millis();
  if ((tempoAtual - tempoInicial) > pausa) {
    Stop();
    delay(5000);
    forward();
    tempoInicial = millis();
  }
}

void forward() {
  digitalWrite(motorEsqFrente, LOW);
  digitalWrite(motorEsqTras, HIGH);
  digitalWrite(motorDirFrente, HIGH);
  digitalWrite(motorDirTras, LOW);
  analogWrite(pwmEsq, velEsq); // Ajustar a velocidade
  analogWrite(pwmDir, velDir);
}

void back() {
  digitalWrite(motorEsqFrente, HIGH);
  digitalWrite(motorEsqTras, LOW);
  digitalWrite(motorDirFrente, LOW);
  digitalWrite(motorDirTras, HIGH);
  //analogWrite(pwmEsq, velEsq); // Ajustar a velocidade
  //analogWrite(pwmDir, velDir);
}

void left() {
  digitalWrite(motorEsqFrente, LOW);
  digitalWrite(motorEsqTras, HIGH);
  digitalWrite(motorDirFrente, LOW);
  digitalWrite(motorDirTras, HIGH);
  //analogWrite(pwmEsq, velEsq); // Ajustar a velocidade
  //analogWrite(pwmDir, velDir);
}

void right() {
  digitalWrite(motorEsqFrente, HIGH);
  digitalWrite(motorEsqTras, LOW);
  digitalWrite(motorDirFrente, HIGH);
  digitalWrite(motorDirTras, LOW);
  //analogWrite(pwmEsq, velEsq); // Ajustar a velocidade
  //analogWrite(pwmDir, velDir);
}

void Stop() {
  digitalWrite(motorEsqFrente, LOW);
  digitalWrite(motorEsqTras, LOW);
  digitalWrite(motorDirFrente, LOW);
  digitalWrite(motorDirTras, LOW);
  analogWrite(pwmEsq, 0);
  analogWrite(pwmDir, 0);
}

void calcula(int a){
  int b = a/20;
  float circ = 2*3.141593*3.5;
  percorre = b*circ;
}