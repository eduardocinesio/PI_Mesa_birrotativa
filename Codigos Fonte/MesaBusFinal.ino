#include <PID_v1.h>
#include <Encoder.h>
//Versão FINAL
//Variaveis de comunicacao
unsigned long tempo_chaveamento = 1000; // Tempo a cada comunicacao
//Entradas do IRB140 são as saidas do Arduino
const int R_0 = 99; // Done(terminou)   ex 43
const int R_1 = 99; //Received(recebeu) EX 45
const int R_2 = 99; //Reserva
const int R_3 = 99; //Reserva

//Saídas do IRB140 são as entradas do Arduino
const int T_7 = 45;//35>>25;
const int T_6 = 27;//37;
const int T_5 = 47;//39>>29;
const int T_4 = 49;//41>>31;
const int T_3 = 35;//43;
const int T_2 = 37;//45;
const int T_1 = 39;//47;
const int T_0 = 43;//49;
//Variaveis do encoder
Encoder Enc(2, 3);
float   EncAtual = 0.0;
float   EncAnterior = 0.0;
//Variaveis motor de passo
const int PUL = 13; //define Pulse pin D2
const int DIR = 11; //define Direction pin D3
const int ENA = 12; //define Enable Pin D4
int POS = 0; //Variável de posição POS=(relação angulo x pps)
int POS_Atual = 0;  //Posicao atual relativa a inicializacao
int POS_Anterior = 0;  //Posicao atual relativa a inicializacao

//Variaveis Motor CC
//declaracao dos pinos utilizados para controlar o motor cc
const int PINO_IN1 = 5;
const int PINO_IN4 = 6;
const int PINO_IN2 = 9;
const int PINO_IN3 = 10;
int POS_CC = 0;//Posicao motor cc
int VEL_CC = 0;//Velocidade motor cc
double T = 0.05;//Período de amostragem (T = )
double T1 = 0.5;//Período de amostragem (T = )

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
//Auxiliares
unsigned long lastSend = 0;
int i = 0;  //Auxiliar for mensagem
int j = 0;  //Auxiliar for mensagem
byte msg = 0;//Inicia o byte de mensagem
int imsg = 0; //Inicia o int de mensagem
byte motorGO;
byte motorGOP;
byte motorDIRP;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Timer 1 interrupcao
ISR(TIMER1_COMPA_vect) {
  EncAtual = -(Enc.read()); //recebe o valor da interrupção
  DEnc = EncAtual - EncAnterior;
  EncAnterior = EncAtual;
  VEnc = (60 * DEnc / (12000 * T));
}
ISR(TIMER2_COMPA_vect) {
  /* motorGO = HIGH;
    VEL_CC = 17;
    POS_CC = 360;
  */
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

//Configuracao timer2 IGUAL Timer 1
void Tempo_Config_2(void)
{
  TCCR2B = 0;
  TCCR2A = 0;
  TIMSK2 = 0;
  // Timer de 16bits, conta até 65535
  TCCR2A |= (1 << WGM21); //Timer em CTC e prescaler de 256
  TCCR2B |= (1 << CS21) | (1 << CS22); //Timer em CTC e prescaler de 256
  //TCCR1B |= (1 << WGM12) | (1 << CS10)| (1 << CS12); //Timer em CTC e prescaler de 1024
  TIMSK2 = (1 << OCIE2A); //Interrupção no COMPA
  //Com prescaler de 8, no modo CTC f = fcpu/(Prescaler*(1+OCR1A))
  //f = 16Hz
  //T = 0.2s
  OCR2A = ((16000000 / 256) * T1) - 1;
}

void setup() {
  //Setup da comunicacao
  tempo_chaveamento = 5 * 1000; // t#5*1s
  //Pinos de comunicação IRB140/Arduino
  pinMode (T_0, INPUT);
  pinMode (T_1, INPUT);
  pinMode (T_2, INPUT);
  pinMode (T_3, INPUT);
  pinMode (T_4, INPUT);
  pinMode (T_5, INPUT);
  pinMode (T_6, INPUT);
  pinMode (T_7, INPUT);
  pinMode (R_0, OUTPUT);
  pinMode (R_1, OUTPUT);
  pinMode (R_2, OUTPUT);
  pinMode (R_3, OUTPUT);
  //Desligando as saidas e garantindo pulldown
  digitalWrite(T_0, LOW);
  digitalWrite(T_1, LOW);
  digitalWrite(T_2, LOW);
  digitalWrite(T_3, LOW);
  digitalWrite(T_4, LOW);
  digitalWrite(T_5, LOW);
  digitalWrite(T_6, LOW);
  digitalWrite(T_7, LOW);
  digitalWrite(R_0, LOW);
  digitalWrite(R_1, LOW);
  digitalWrite(R_2, LOW);
  digitalWrite(R_3, LOW);

  Tempo_Config_1();//Seta o tempo de interrupção
  Tempo_Config_2();//Seta o tempo de interrupção
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
  myPID.SetSampleTime(200);
}

byte motorP(byte bstart,  double angulo, byte dir) {//(bstart = estado do motor de passo ON ou OFF, angulo em graus, Horário = Low AntiHorario=High)
  POS = angulo * 40;
  i = 0;
  if (dir == HIGH) {
    digitalWrite(DIR, HIGH);
  } else {
    digitalWrite(DIR, LOW);
  }
  if (bstart == HIGH) {
    for (i = 0; i < POS; i++) //Trocar 6400 por variável
    {
      //Serial.println(POS);//Azul
      //Serial.println(i);
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
      P0 = 0;
      pSetpoint = 0;
      Output = 0;
    }
    //myPID.Compute();
    Setpoint = (P0 + pSetpoint) * Kp2;
    if (Output < 44 && Cp > 0) {
      Output = 44;
    }
    analogWrite(PINO_IN1, Output);
    analogWrite(PINO_IN4, Output);
  }
  if (ErroP == 0 & Output == 0) {
    bstart = LOW;
  }
}

void loop() {



  //Carrega os valores das entradas em bool
  //Serial.print(" Erro ");
  myPID.Compute();
  //  if(POS_Anterior > POS){
  //  POS_Anterior =  motorP(motorGOP, POS, motorDIRP);
  //}
  //  POS_Anterior = motorP(LOW,LOW,LOW);

  motorGO = motorCC(motorGO, POS_CC, VEL_CC);
  //Estado do motor HIGH = LIGADO LOW=Desligado, angulo em graus, velocidade em rpm
  boolean DT_0 = digitalRead(T_0);
  boolean DT_1 = digitalRead(T_1);
  boolean DT_2 = digitalRead(T_2);
  boolean DT_3 = digitalRead(T_3);
  boolean DT_4 = digitalRead(T_4);
  boolean DT_5 = digitalRead(T_5);
  boolean DT_6 = digitalRead(T_6);
  boolean DT_7 = digitalRead(T_7);
  //Cria um array de boleanos
  msg = 0;//Inicia o byte de mensagem
  imsg = 0; //Inicia o int de mensagem
  // Looping para percorrer o array
  boolean DI_A[8] = {DT_0, DT_1, DT_2, DT_3, DT_4, DT_5, DT_6, DT_7};

  for (i = 0; i < 8; i++) {
    if (DI_A[i]) {
      msg |= (1 << (7 - i));
    }
  }

  imsg = int(msg); // Faz um cast do byte para inteiro
  digitalWrite(R_1, HIGH); //Liga o bit de recebido

  //Se a mensagem for 1
  if (imsg == 0) {
    POS_CC = 0;
    VEL_CC = 23;
    motorGO = LOW;
    motorGOP = LOW;

    digitalWrite(R_0, HIGH); //Liga o bitDone
    analogWrite(PINO_IN1, 0);
    analogWrite(PINO_IN4, 0);
    Enc.write(0);
    ErroP = 0;
  }
  //Se a mensagem for 180
  else if (imsg <= 180) {
    /*if(ErroP<0 & Output ==0){
      Enc.write(0);
      ErroP=0;
      }*/
    POS = imsg;
    POS_CC = 2 * imsg;
    digitalWrite(R_0, HIGH); //Liga o bitDone
  }
  //Se a mensagem for 181
  else if (imsg == 181) {
    digitalWrite(R_0, HIGH); //Liga o bitDone
    motorGO = HIGH;
  }
  //Se a mensagem for 182
  else if (imsg == 182) {
    if ((POS_Atual + POS) <= 180) { // Enquanto não há retorno de posição torna-se necessário a fim o if para o motor de passo não ficar em regime permanente.
      POS_Atual = POS_Atual + motorP(HIGH, POS, LOW); //(bstart = estado do motor de passo ON ou OFF, angulo em graus, Horário = Low AntiHorario=High)
    } else {
    }
    digitalWrite(R_0, HIGH); //Liga o bitDone
  }
  else if (imsg == 183) {
    if ((POS_Atual - POS) < 0) { // Enquanto não há retorno de posição torna-se necessário a fim o if para o motor de passo não ficar em regime permanente.
      POS_Atual = POS_Atual - motorP(HIGH, POS, HIGH); //(bstart = estado do motor de passo ON ou OFF, angulo em graus, Horário = Low AntiHorario=High)
    } else {
    }

    digitalWrite(R_0, HIGH); //Liga o bitDone
  } else if (imsg <= 205) {
    VEL_CC = imsg - 165;
    digitalWrite(R_0, HIGH); //Liga o bitDone
  }
  else {
    //POS_CC = 0;
    //VEL_CC = 0;
    motorGO = LOW;
    POS_Atual = motorP(LOW, POS, LOW);

    analogWrite(PINO_IN1, 0);
    analogWrite(PINO_IN4, 0);
    //Se for algum overflow de codigo
    //Note que nao tem o bit de done acionado isso sera uma flag para um erro ou timeout
  }

  //Faz o conteudo da mensagem a cada tempo_chaveamento
  if (millis() - lastSend > 1000) {
    lastSend = millis();// Atualiza o tempo anterior
    Serial.println(" I ");
    Serial.print(imsg);
    Serial.print(" POS");
    Serial.println(POS);
    /*Utilizado para debug
      Serial.print(" POS Anterior");
      Serial.println(POS_Anterior);
      Serial.print(" Erro ");
      Serial.print(ErroP);
      Serial.print(" Vel ");
      Serial.print(VEnc);//Vermelho
      Serial.print(" In ");
      Serial.print(Input);//Verde
      Serial.print(" Out ");
      Serial.print(Output);
       Serial.print(" VEL CC");
      Serial.print(VEL_CC);
      Serial.print(" VEL SP");
      Serial.print(vSetpoint);
      Serial.print(" VEL CP");
      Serial.print(Cp);
    */
    if (motorGO == HIGH) {
      Serial.print(" high");
    } else {
      Serial.print(" low");
    }

    j++;
  }
  if (j == 600)
  {
    Serial.end();
  }
}
