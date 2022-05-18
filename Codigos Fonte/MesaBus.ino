unsigned long lastSend = 0;
unsigned long tempo_chaveamento =0;
void setup(){
Serial.begin(9600);
tempo_chaveamento = 1000;//1segundo
}

void loop(){
// boolean DI_A[8] = {true, false, true, true, false, true, false, true};//126
// boolean DI_A[8] = {false, false, false, false, false, false, false, false};//0
boolean DI_A[8] = {false, true, true, true, true, true, true, false};//126
//boolean DI_A[8] = {DI00, DI01, DI02, DI03, DI04, DI05, DI06, DI07};
byte msg = 0;
int imsg = 0;
for(int i=0; i<8; i++){
    if (DI_A[i]){
       msg |= (1 << (7-i));
}
}
imsg = int(msg);
if ((millis()-lastSend) > tempo_chaveamento){
    if(imsg==0){
        lastSend = millis();
        Serial.print(" byte: ");
        Serial.println(msg, BIN);  //  11000101
        Serial.print(" int: ");
        Serial.println(imsg);
        Serial.println(" Codigo 0");
    }
    else if(imsg<180){
        lastSend = millis();
        Serial.print(" byte: ");
        Serial.println(msg, BIN);  //  11000101
        Serial.print(" int: ");
        Serial.println(imsg);
        Serial.println(" Codigo 180");
    }
    else if(imsg==181){
        lastSend = millis();
        Serial.print(" byte: ");
        Serial.println(msg, BIN);  //  11000101
        Serial.print(" int: ");
        Serial.println(imsg);
        Serial.println(" Codigo 181");
    }
    else if(imsg==182){
        lastSend = millis();
        Serial.print(" byte: ");
        Serial.println(msg, BIN);  //  11000101
        Serial.print(" int: ");
        Serial.println(imsg);
        Serial.println(" Codigo 182");
    }

}
}