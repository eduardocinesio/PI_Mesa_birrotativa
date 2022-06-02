unsigned long lastSend = 0; // Tempo anterior de execucao
unsigned long tempo_chaveamento = 0; // Tempo a cada comunicacao
// VARIAVEIS DAS IOs DO IRB140
//Entradas do IRB140 são as saidas do Arduino
const int I00 = 22; // Done(terminou)
const int I01 = 23; //Received(recebeu)
//Saídas do IRB140 são as entradas do Arduino
const int Q00 = 24;
const int Q01 = 25;
const int Q02 = 26;
const int Q03 = 27;
const int Q04 = 28;
const int Q05 = 29;
const int Q06 = 30;
const int Q07 = 31;

void setup() {
  Serial.begin(9600);//Inicia comunicacao serial com baud de 9600
  tempo_chaveamento = 1000;// t#1s

  //Pinos de comunicação IRB140/Arduino
  pinMode (Q00, INPUT);
  pinMode (Q01, INPUT);
  pinMode (Q02, INPUT);
  pinMode (Q03, INPUT);
  pinMode (Q04, INPUT);
  pinMode (Q05, INPUT);
  pinMode (Q06, INPUT);
  pinMode (Q07, INPUT);
  pinMode (I00, OUTPUT);
  pinMode (I01, OUTPUT);
  digitalWrite(24, LOW);
  digitalWrite(25, LOW);
  digitalWrite(26, LOW);
  digitalWrite(27, LOW);
  digitalWrite(28, LOW);
  digitalWrite(29, LOW);
  digitalWrite(30, LOW);
  digitalWrite(31, LOW);

}

void loop() {
  //Carrega os valores das entradas em bool
  boolean DQ00 = digitalRead(Q00);
  boolean DQ01 = digitalRead(Q01);
  boolean DQ02 = digitalRead(Q02);
  boolean DQ03 = digitalRead(Q03);
  boolean DQ04 = digitalRead(Q04);
  boolean DQ05 = digitalRead(Q05);
  boolean DQ06 = digitalRead(Q06);
  boolean DQ07 = digitalRead(Q07);

  //Cria um array de boleanos
  boolean DI_A[8] = {DQ00, DQ01, DQ02, DQ03, DQ04, DQ05, DQ06, DQ07};
  byte msg = 0;//Inicia o byte de mensagem
  int imsg = 0; //Inicia o int de mensagem
  // Looping para percorrer o array
  for (int i = 0; i < 8; i++) {
    if (DI_A[i]) {
      msg |= (1 << (7 - i));
    }
  }
  imsg = int(msg); // Faz um cast do byte para inteiro
  //Faz o conteudo da mensagem a cada tempo_chaveamento
  if ((millis() - lastSend) > tempo_chaveamento) {
    lastSend = millis();// Atualiza o tempo anterior
    digitalWrite(I01, HIGH); //Liga o bit de recebido
    Serial.print(" teste byte: ");//Label do binario
    Serial.println(msg, BIN);  //Binario gerado
    Serial.print(" int: ");//Label do inteiro
    Serial.println(imsg); //Inteiro gerado
    //Se a mensagem for 1
    if (imsg == 0) {
      Serial.println(" Codigo 0"); // Onde ficara a função desligar
      digitalWrite(I00, HIGH); //Liga o bitDone
    }
    //Se a mensagem for 180
    else if (imsg < 180) {
      Serial.println(" Codigo 180"); // Onde ficara o parâmetro posicao
      digitalWrite(I00, HIGH); //Liga o bitDone
    }
    //Se a mensagem for 181
    else if (imsg == 181) {
      Serial.println(" Codigo 181"); // Onde ficara o acionamento do eixo 1
      digitalWrite(I00, HIGH); //Liga o bitDone
    }
    //Se a mensagem for 182
    else if (imsg == 182) {
      Serial.println(" Codigo 182");// Onde ficara o acionamento do eixo 2
      digitalWrite(I00, HIGH); //Liga o bitDone
    }
    //Se a mensagem for 205
    else if (imsg < 205) {
      Serial.println(" Codigo 205");// Onde ficara o parâmetro velocidade
      digitalWrite(I00, HIGH); //Liga o bitDone
    }
    else {
      //Se for algum overflow de codigo
      Serial.println(" Codigo Erro");// Onde ficará a mensagem de erro, separada devidamente
      //Note que nao tem o bit de done acionado isso sera uma flag para um erro ou timeout
    }
  }
}





