/* This skecth has been developed in order to implement the PID algorithm in the robot arm RRR.
 * The code is hosted at https://github.com/vbenhur/Robo-RRR-PID-Arduino-MATLAB
 * The libraries used were:
 *    aJson - https://github.com/interactive-matter/aJson
 *    LiquidCrystal_I2C - https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
 * Developers: 
 *    Idemar dos Santos - idemas_@hotmail.com
 *    Danilo Pinto - danilojpinto@hotmail.com
 *    Ícaro Farias - icsfar@gmail.com
 *    Victor Ben-Hur - vbenhur@gmail.com
 *    Ney Barros - junior.m_16@hotmail.com
 * This work is licensed under a Creative Commons Attribution 4.0 International License. http://creativecommons.org/licenses/by/4.0/
 * 7 Nov 2015
 */

// Biblioteca Json para comunicação serial
#include <aJSON.h>
aJsonStream serial_stream(&Serial);

//Inicialização do LCD

// LCD I2C
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>  // F Malpartida's NewLiquidCrystal library

/*-----( Declare Constants )-----*/
#define I2C_ADDR    0x27  // Define I2C Address for the PCF8574A 
//---(Following are the PCF8574 pin assignments to LCD connections )----
// This are different than earlier/different I2C LCD displays
#define BACKLIGHT_PIN  3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

#define  LED_OFF  0
#define  LED_ON  1

LiquidCrystal_I2C  lcd( I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);


/*working variables*/
unsigned long lastTime[4], lastErr[4];
double Input[4], Output[4],  SetpointG, errSum[4];
int angulos[4];
int Setpoint[4];
int Saida[4];
double ITerm, lastInput;
double kp[4] = {
  15, 5, 5, 5};  // Base, Ombro, Cotovelo, Pulso
double ki[4] = {
  0, 0, 0, 0}; // Base, Ombro, Cotovelo, Pulso
double kd[4] = {
  2, 3, 5, 1}; // Base, Ombro, Cotovelo, Pulso

int SampleTime = 1000; //1 sec
double outMin, outMax;

// Definiçoes dos elos
#define Bmin 150
#define Bmax 860
#define BmaxGraus 220

#define Omin 417
#define Omax 640
#define OmaxGraus 80

#define Cmin 170
#define Cmax 870
#define CmaxGraus 230

#define Pmin 130
#define Pmax 678
#define PmaxGraus 200

// Pinos Garra
#define btAbreGarra A14
#define btFechaGarra A15

int valAbreGarra = 0;
int valFechaGarra = 0;

// Chave Seletora Manual/Automatico
#define ChaveMan  46
#define ChaveAut  48

int val_Man = 0;
int val_Aut = 0;

// Botao Emergencia
#define btEmerg  44
int val_Emerg = 0;
#define Buzzer  53
int estado_ledEmerg = 0;


unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change :
const long interval = 1000;           // interval at which to blink (milliseconds)

// Pinos Leds
#define ledB 8
#define ledO 9
#define ledC 10
#define ledP 11
#define ledEmerg 42
// Pinos Base
#define potMB  A0
#define potB  A6
#define in1B  38
#define in2B  40
#define enB   6
// Pinos Ombro
#define potMO A1
#define potO A7
#define in1O 34
#define in2O 36
#define enO  5
// Pinos Cotovelo
#define potMC  A2
#define potC  A8
#define in1C  30
#define in2C  32
#define enC   4
// Pinos Pulso
#define potMP  A3
#define potP  A9
#define in1P  26
#define in2P  28
#define enP   3
// Pinos Garra
#define in1G  24
#define in2G  26
#define enG   2

// Vetor de pinos
byte pinosOut[17] = {
  Buzzer,ledB,ledO,ledC,ledP,ledEmerg,in1B,in2B,in1O,in2O,in1C,in2C,in1P,in2P,in1G,in2G,enG};

void setup() {

  // Inicialização do LCD I2C
  lcd.begin (16, 2); 
  // Switch on the backlight
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(LED_ON);

  // Chave Modo Manual e Modo Automático
  pinMode(ChaveMan, INPUT_PULLUP);
  pinMode(ChaveAut, INPUT_PULLUP);

  // Botao Garra
  pinMode(btAbreGarra, INPUT_PULLUP);
  pinMode(btFechaGarra, INPUT_PULLUP);

  // Chave de Emergencia
  pinMode(btEmerg, INPUT_PULLUP);


  // Valores Iniciais dos SetPoints
  Setpoint[0] = 472;  // 100 graus
  Setpoint[1] = 584;  // 60 graus
  Setpoint[2] = 413;  // 80 graus
  Setpoint[3] = 267;  // 50 graus

  // Declara os pinos como saída
  for (int x=0; x<17 ; x++ ){
    pinMode(pinosOut[x],OUTPUT);
  }

  Serial.begin(9600);

}

void loop() {

  // Chave de Emergencia
  val_Emerg = digitalRead(btEmerg);

  // Leitura_Setpoints_Potenciometros();
  
  if (val_Emerg == LOW) {
    // Emergencia Ativada
    alertaEmergencia();
  } 
  else {
    // Desativa emergencia
    digitalWrite(ledEmerg, LOW);
    digitalWrite(Buzzer, LOW);
   
    if (modoChave() == 0) { // Chave na posição REPOUSO
      // Parar o robô travando os motores
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("  MODO REPOUSO ");
    } 
    else if  (modoChave() == 1) {  // Chave na posição MANUAL
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("   MODO MANUAL  ");
      exibe_angulos_lcd();        // Exibe os angulos no display LCD
      controla_garraManual();     // Controla a garra manualmente
      Leitura_Setpoints_Potenciometros();  // Lê os valores dos potenciometros do painel de controle
    } 
    else if  (modoChave() == 2) {  // Chave na posição Automático
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(" MODO AUTOMATICO  ");

      recebeDadosSerial(); // Processa as informações advindas da Porta Serial atualizando os SETPOINTS
    }

    delay(100);

    Atualiza_Inputs(); // Atualiza valores de entrada das posições
    Compute(0);        // Processa o PID para o elo 0
    Compute(1);        // Processa o PID para o elo 1
    Compute(2);        // Processa o PID para o elo 2
    Compute(3);        // Processa o PID para o elo 3
    // Imprime_Setpoint();
  }
}

void Compute(int elo) {
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime[elo]);
  double error = Setpoint[elo] - Input[elo];
  errSum[elo] += (error * timeChange);
  double dErr = (error - lastErr[elo]) / timeChange;
  Output[elo] = kp[elo] * error + ki[elo] * errSum[elo] + kd[elo] * dErr;
  lastErr[elo] = error;
  lastTime[elo] = now;
  Saida[elo] = abs(Output[elo]);
  Saida[elo] = constrain(Saida[elo], 0, 1023);
  Saida[elo] = map(Saida[elo], 0, 1023, 0, 255);

  switch (elo) {
  case 0:
    analogWrite(enB, Saida[elo]);
    analogWrite(ledB, Saida[elo]);
    if (Output[elo] < 0 ) {
      digitalWrite(in1B, HIGH);
      digitalWrite(in2B, LOW);
    }
    if (Output[elo] > 0 ) {
      digitalWrite(in1B, LOW);
      digitalWrite(in2B, HIGH);
    }
    break;
  case 1:
    analogWrite(enO, Saida[elo]);
    analogWrite(ledO, Saida[elo]);
    if (Output[elo] < 0 ) {
      digitalWrite(in1O, HIGH);
      digitalWrite(in2O, LOW);
    }
    if (Output[elo] > 0 ) {
      digitalWrite(in1O, LOW);
      digitalWrite(in2O, HIGH);
    }
    break;
  case 2:
    analogWrite(enC, Saida[elo]);
    analogWrite(ledC, Saida[elo]);
    if (Output[elo] < 0 ) {
      digitalWrite(in1C, HIGH);
      digitalWrite(in2C, LOW);
    }
    if (Output[elo] > 0 ) {
      digitalWrite(in1C, LOW);
      digitalWrite(in2C, HIGH);
    }
    break;
  case 3:
    analogWrite(enP, Saida[elo]);
    analogWrite(ledP, Saida[elo]);
    if (Output[elo] < 0 ) {
      digitalWrite(in1P, HIGH);
      digitalWrite(in2P, LOW);
    }
    if (Output[elo] > 0 ) {
      digitalWrite(in1P, LOW);
      digitalWrite(in2P, HIGH);
    }
    break;
  }

}

void exibe_angulos_lcd() {

  lcd.setCursor(0, 1);
  angulos[0] = constrain((int)Input[0], 0, 1023);
  angulos[0] = map(angulos[0], 0, Bmax, 0, BmaxGraus );
  lcd.print(angulos[0]);

  lcd.setCursor(5, 1);
  angulos[1] = constrain((int)Input[1], 0, 1023);
  angulos[1] = map(angulos[1], 0, 1023, 0, OmaxGraus );
  lcd.print(angulos[1]);

  lcd.setCursor(9, 1);
  angulos[2] = constrain((int)Input[2], 0, 1023);
  angulos[2] = map(angulos[2], 0, 1023, 0, CmaxGraus );
  lcd.print(angulos[2]);

  lcd.setCursor(13, 1);
  angulos[3] = constrain((int)Input[3], 0, 1023);
  angulos[3] = map(angulos[3], 0, 1023, 0, PmaxGraus );
  lcd.print(angulos[3]);

}

void Leitura_Setpoints_Potenciometros() {
  Setpoint[0] = analogRead(potMB);
  Setpoint[0] = map(Setpoint[0], 0, 1023, Bmin, Bmax );

  Setpoint[1] = analogRead(potMO);
  Setpoint[1] = map(Setpoint[1], 0, 1023, Omin, Omax );

  Setpoint[2] = analogRead(potMC);
  Setpoint[2] = map(Setpoint[2], 0, 1023, Cmin, Cmax );

  Setpoint[3] = analogRead(potMP);
  Setpoint[3] = map(Setpoint[3], 0, 1023, Pmin, Pmax );

}

void Atualiza_Inputs() {

  Input[0] = analogRead(potB); //potenciometro Base
  Input[1] = analogRead(potO); //potenciometro Ombro
  Input[2] = analogRead(potC); //potenciometro Cotovelo
  Input[3] = analogRead(potP); //potenciometro Pulso

}


void Imprime(int elo) {
  
  Serial.print(Input[elo]);
  Serial.print("\t");
  Serial.print(Setpoint[elo]);
  Serial.print("\t");
  Serial.print(Output[elo]);
  Serial.print("\t");
  Serial.print(Saida[elo]);
  Serial.print("\n");

}

// Recebe dados da Serial e armazena no objeto do Json
void recebeDadosSerial() {
  //* Le dados da porta serial
  if (serial_stream.available()) {
    /* First, skip any accidental whitespace like newlines. */
    serial_stream.skip();
  }
  if (serial_stream.available()) {
    /* Something real on input, let's take a look. */
    aJsonObject *msg = aJson.parse(&serial_stream);
    processMessage(msg);
    aJson.deleteItem(msg);
  }
}

void Imprime_Setpoint() {
  Serial.print(Setpoint[0]);
  Serial.print("\t");
  Serial.print(Setpoint[1]);
  Serial.print("\t");
  Serial.print(Setpoint[2]);
  Serial.print("\t");
  Serial.print(Setpoint[3]);
  Serial.print("\n");

}

////Exemplo de recebimento --> {"Dados":{"B":100,"O":60,"C":80,"P":50,"G":2}}
void processMessage(aJsonObject *msg)
{
  aJsonObject *dados = aJson.getObjectItem(msg, "Dados");

  aJsonObject *aBase = aJson.getObjectItem(dados, "B");
  aJsonObject *aOmbro = aJson.getObjectItem(dados, "O");
  aJsonObject *aCotovelo = aJson.getObjectItem(dados, "C");
  aJsonObject *aPulso = aJson.getObjectItem(dados, "P");
  aJsonObject *aGarra = aJson.getObjectItem(dados, "G");


  if (  aBase != NULL) {  // Caso haja valor no objeto Base
    Setpoint[0] = aBase->valueint;  // Converte p/ inteiro
    Setpoint[0] = constrain(Setpoint[0], 0, BmaxGraus); // Garante que os valores não ultrapassem os limites
    Setpoint[0] = map(Setpoint[0], 0, BmaxGraus, Bmin, Bmax );  // Converte graus p/ valores predeterminados
    //  Serial.println(Setpoint[0]);
  }

  if (  aOmbro != NULL) {
    Setpoint[1] = aOmbro->valueint;
    Setpoint[1] = constrain(Setpoint[1], 0, OmaxGraus);
    Setpoint[1] = map(Setpoint[1], 0, OmaxGraus, Omin, Omax );
    //  Serial.println(Setpoint[1]);
  }

  if (  aCotovelo != NULL) {
    Setpoint[2] = aCotovelo->valueint;
    Setpoint[2] = constrain(Setpoint[2], 0, CmaxGraus);
    Setpoint[2] = map(Setpoint[2], 0, CmaxGraus, Cmin, Cmax );
    //  Serial.println(Setpoint[2]);
  }

  if (  aPulso != NULL) {
    Setpoint[3] = aPulso->valueint;
    Setpoint[3] = constrain(Setpoint[3], 0, PmaxGraus);
    Setpoint[3] = map(Setpoint[3], 0, PmaxGraus, Pmin, Pmax );
    //  Serial.println(Setpoint[3]);
  }

  if ( aGarra != NULL) {
    Setpoint[4] = aGarra->valueint;

    if (Setpoint[4] == 1) { // Fechar garra
      fecha_garra();
      Setpoint[4] = 0;
    } 
    else if (Setpoint[4] == 2) { // Abrir Garra
      abre_garra();
      Setpoint[4] = 0;
    } 
    else { // Trava motor
      trava_garra();
    }
  }
}

void abre_garra() {
  digitalWrite(enG, HIGH); //Enable do motor 1
  digitalWrite(in1G, HIGH); //Input1 do motor 1 alto
  digitalWrite(in2G, LOW); //Input2 do motor 2 baixo
}

void fecha_garra() {
  digitalWrite(enG, HIGH); //Enable do motor 1
  digitalWrite(in1G, LOW); //Input1 do motor 1 alto
  digitalWrite(in2G, HIGH); //Input2 do motor 2 baixo
}

void trava_garra() {
  digitalWrite(enG, HIGH); //Enable do motor 1
  digitalWrite(in1G, LOW); //Input1 do motor 1 alto
  digitalWrite(in2G, LOW); //Input2 do motor 2 baixo
}

int modoChave() {

  val_Man = digitalRead(ChaveMan);
  val_Aut = digitalRead(ChaveAut);

  if ((val_Man == LOW)) {
    //  Serial.println("MODO MANUAL");
    return 1;
  }
  if ((val_Aut == LOW)) {
    //  Serial.println("MODO AUTOMATICO");
    return 2;
  }

  if ((val_Man == HIGH) && (val_Aut == HIGH)) {
    //  Serial.println("DESLIGA");
    return 0;
  }
}


void controla_garraManual() {

  valAbreGarra = digitalRead(btAbreGarra);
  valFechaGarra = digitalRead(btFechaGarra);
  if (valAbreGarra == LOW) {
    abre_garra();
  } 
  else if (valFechaGarra == LOW) {
    fecha_garra();
  } 
  else {
    trava_garra();
  }
}

void alertaEmergencia() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time 
    previousMillis = currentMillis;
    lcd.clear();
    // if the LED is off turn it on and vice-versa:
    if (estado_ledEmerg == LOW) {
      estado_ledEmerg = HIGH;

      lcd.print("   EMERGENCIA  ");
    }
    else {
      estado_ledEmerg = LOW;
      lcd.print("                ");
    }
    // set the LED with the ledState of the variable:
    digitalWrite(ledEmerg, estado_ledEmerg);
    digitalWrite(Buzzer, estado_ledEmerg);
  }

}
