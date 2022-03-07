#include <PID_v1.h>
#include <Encoder.h>
//Variaveis motor de passo
const int PUL = 13; //define Pulse pin D2
const int DIR = 11; //define Direction pin D3
const int ENA = 12; //define Enable Pin D4
int POS = 0; //Variável de posição POS=(relação angulo x pps)
int POS90 = 0; //Ref de eixo variavel de posição POS=(relação angulo x pps)+90graus
int i = 0;
//declaracao dos pinos utilizados para controlar o motor cc
const int PINO_IN1 = 5;
const int PINO_IN4 = 6;
const int PINO_IN2 = 9;
const int PINO_IN3 = 10;
//Variaveis do encoder
Encoder Enc(2, 3);
float   EncAtual = 0.0;
float   EncAnterior = 0.0;
double T = 0.02;//Período de amostragem (T = )
unsigned long lastSend = 0;
//Define variaveis pro pid
double Setpoint, pSetpoint, vSetpoint, Input, Output;
double ErroP, Cp;
float   P0 = 0.0;
float   DEnc = 0.0;
float   VEnc = 0.0;
double Kp2 = 0.00333333333; // Calculo 1 de Kp2=0.003333 ou Kp2= 0.003183098862
double  Kp = 0.53282;
double  Ki = 11.9894;
double  Kd = 0.059198;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//Timer 1 interrupcao
ISR(TIMER1_COMPA_vect) {
  EncAtual = -(Enc.read()); //recebe o valor da interrupção
  DEnc = EncAtual - EncAnterior;
  EncAnterior = EncAtual;
  VEnc = (60 * DEnc / (12000 * T));
}
//Configuracao timer1
void Tempo_Config_1(void)
{
  TCCR1B = 0;
  TCCR1A = 0;
  TIMSK1 = 0;
  // Timer de 16bits, conta até 65535
  TCCR1B |= (1 << WGM12) | (1 << CS12); //Timer em CTC e prescaler de 256
  //TCCR1B |= (1 << WGM12) | (1 << CS10)| (1 << CS12); //Timer em CTC e prescaler de 1024
  TIMSK1 = (1 << OCIE1A); //Interrupção no COMPA
  //Com prescaler de 8, no modo CTC f = fcpu/(Prescaler*(1+OCR1A))
  //f = 16Hz
  //T = 0.02s
  OCR1A = ((16000000 / 256) * T) - 1;
}

void setup() {

  Tempo_Config_1();//Seta o tempo de interrupção
  Serial.begin(115200);

  //Pinos motor de passso
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);

  //configuracao dos pinos como saida
  pinMode(PINO_IN1, OUTPUT);
  pinMode(PINO_IN2, OUTPUT);
  pinMode(PINO_IN3, OUTPUT);
  pinMode(PINO_IN4, OUTPUT);
  //inicia o codigo com os motores parados
  digitalWrite(ENA, LOW);
  digitalWrite(DIR, LOW);
  digitalWrite(PUL, LOW);
  digitalWrite(PINO_IN1, LOW);
  digitalWrite(PINO_IN2, LOW);
  digitalWrite(PINO_IN3, LOW);
  digitalWrite(PINO_IN4, LOW);
  //Inicia led de controle
  pinMode(LED_BUILTIN, OUTPUT);
  P0 = Enc.read();


  Input = 0;
  myPID.SetMode(AUTOMATIC);
}


byte motorP(byte bstart,  double angulo, byte dir) {//(bstart = estado do motor de passo ON ou OFF, angulo em graus, Horário = Low AntiHorario=High)
  POS = angulo * 40;
  if (dir == HIGH) {
    digitalWrite(DIR, HIGH);
  } else {
    digitalWrite(DIR, LOW);
  }

  if (bstart == HIGH) {
    for (i = 0; i < POS; i++) //Trocar 6400 por variável
    {
      Serial.println(POS);//Azul
      Serial.println(i);

      digitalWrite(PUL, HIGH);
      delayMicroseconds(500);
      digitalWrite(PUL, LOW);
      delayMicroseconds(500);
    }
    POS = 0;

  }
  bstart = LOW;
  return i;
}

byte motorCC(byte bstart, double angulo, double vel) {
  if (bstart == HIGH) {
    pSetpoint = angulo / 0.03; //Setpoint para 1 volta
    vSetpoint = vel;
    ErroP = ((pSetpoint) - (EncAtual)) * Kp2;
    Input = VEnc;
    Cp = min((ErroP), vSetpoint);
    Setpoint = Cp;
    if (ErroP > 0) {
      Cp = min((ErroP), vSetpoint);
    } else {
      pSetpoint = 0;
    }
    myPID.Compute();
    Setpoint = (P0 + pSetpoint) * Kp2;
    if (Output < 44 && Cp > 0) {
      Output = 44;
    }

    analogWrite(PINO_IN1, Output);
    analogWrite(PINO_IN4, Output);



  }

  bstart = LOW;
  return bstart;
}

void loop() {
  /*
    if (i <= 0) { // Enquanto não há retorno de posição torna-se necessário a fim o if para o motor de passo não ficar em regime permanente.
      POS = 90 ;// Coloque o angulo em graus
      i = motorP(LOW, POS, HIGH); //(bstart = estado do motor de passo ON ou OFF, angulo em graus, Horário = Low AntiHorario=High)
    }*/
  motorCC(HIGH, (360 * 5), 40); //Estado do motor HIGH = LIGADO LOW=Desligado, angulo em graus, velocidade em rpm

  if (millis() - lastSend > 80) {
    lastSend = millis();
    Serial.print(" Erro ");
    Serial.print(ErroP);
    Serial.print(" Vel ");
    Serial.print(VEnc);//Vermelho
    Serial.print(" In ");
    Serial.print(Input);//Verde
    Serial.print(" Out ");
    Serial.print(Output);
    Serial.println(" ");
    i++;
  }
  if(i==600)
  Serial.end();
}
