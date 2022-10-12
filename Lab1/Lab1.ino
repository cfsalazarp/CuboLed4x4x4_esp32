//Actaulizado 11/10/22
#define efectosSW 34
#define tiemposSW 39
int led[] = {15,2,4,16,17,5,18,19,32,33,25,26,27,14,12,13}; //Salidas de las columnas
int lvl[] = {21,3,23,22}; //Salidas de las filas
int times[] = {50, 150, 350, 750, 1250}; //tiempos establecidos
int w = 0; //Variable para cambiar de efectos
int a = 0; //Variable para cambiar de tiempos
int y; //Variable para encender las filas
int t;

//const int freq = 5000;
//const int channel[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
//const int res = 8;

volatile uint8_t flag_efectos;
volatile uint8_t flag_tiempos;
volatile uint32_t rebote1;
volatile uint32_t rebote2;

void IRAM_ATTR efectos(){
  Serial.println("Entro a isr efectos");
  w++;
  flag_efectos=1; //Activa bandera para indicar ingreso a rutina de interrupción
  rebote1=millis(); //Leer valor actual de la funcion millis()
  detachInterrupt(digitalPinToInterrupt(efectosSW));
}

void IRAM_ATTR tiempos(){
  Serial.println("Entro a isr tiempos");
  a++;
  flag_tiempos=1; //Activa bandera para indicar ingreso a rutina de interrupción
  Serial.print("timesIntr:");
  Serial.println(t);
  rebote2=millis(); //Leer valor actual de la funcion millis()
  detachInterrupt(digitalPinToInterrupt(tiemposSW));
}


void setup() { //En esta parte se establece la configuración
 Serial.begin(115200);
 for (int x = 0; x < 16; x ++) { //Declarar los pines de las columnas como salidas
 pinMode(led[x], OUTPUT);
 //ledcSetup(channel[x], freq, res);
 //ledcAttachPin(led[x], channel[x]);
 }
 for (int x = 0; x < 4; x ++) { //Declarar los pines de las filas como salidas
 pinMode(lvl[x], OUTPUT);
 }
 randomSeed(1);
 pinMode(efectosSW, INPUT_PULLUP);
 pinMode(tiemposSW, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(efectosSW),efectos,FALLING); //Habilita int. generada por el GPIO14
 attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING); //Habilita int. generada por el GPIO14
 flag_efectos=0;
 flag_tiempos=0;
 w = 1;
 a = 0;
 t = times[0];
}

void loop() { //En esta parte se repite la secuencia infinitas veces
  Serial.print("w:"); Serial.println(w);
  Serial.print("a:"); Serial.println(a);
  Serial.print("times:"); Serial.println(times[a]);
  Serial.println(flag_efectos);
  Serial.println(flag_tiempos);
  if(millis()-rebote1>300&&flag_efectos){ //debouncing de pulsador efectosSW
    flag_efectos=0;
    attachInterrupt(digitalPinToInterrupt(efectosSW),efectos,FALLING); //Habilita nuevamente int. generada por el GPIO14
  }
  if(millis()-rebote2>300&&flag_tiempos){ //debouncing de pulsador tiemposSW
    flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING); //Habilita nuevamente int. generada por el GPIO14
  }
  if( w > 11){ w = 1; }
  if( a > 4){ a = 0;}
  t = times[a];
  switch(w){
    case 1:
    Serial.println("Entra a case 1");
        extCol(t);break; //exitoso
    case 2: 
    Serial.println("Entra a case 2");
        uxu(t);break; //exitoso
    case 3: 
    Serial.println("Entra a case 3");
        pxp(t);break; //exitoso
    case 4: 
    Serial.println("Entra a case 4");
        cubito(t); break; //exitoso
    case 5: 
    Serial.println("Entra a case 5");
        caraxcara(t); break; //exitoso
    case 6: 
    Serial.println("Entra a case 6");
        culebrita(t); break; //exitoso
    case 7: 
    Serial.println("Entra a case 7");
        cuadrado(t); break; //exitoso
    case 8: 
    Serial.println("Entra a case 8");
        randLed(t); break; //exitoso
    case 9: 
    Serial.println("Entra a case 9");
        cargaCubo(t); break;
    case 10: 
    Serial.println("Entra a case 10");
        diagonal(t); break;
    case 11: 
    Serial.println("Entra a case 11");
        extCol(t);
        uxu(t);
        pxp(t);
        cubito(t);
        caraxcara(t);
        culebrita(t);
        cuadrado(t);
        randLed(t);
        cargaCubo(t);
        diagonal(t);
        break; 
  }
}

//Esta función permite establecer valores de encendido y apagado a cada columna, donde 1 es encendido y 0 es apagado
void LED (int h, int i, int j, int k, int l, int m, int n, int o, int p, int q,
int r, int s, int t, int u, int v, int w) {
 digitalWrite (led[0], h);
 digitalWrite (led[1], i);
 digitalWrite (led[2], j);
 digitalWrite (led[3], k);
 digitalWrite (led[4], l);
 digitalWrite (led[5], m);
 digitalWrite (led[6], n);
 digitalWrite (led[7], o);
 digitalWrite (led[8], p);
 digitalWrite (led[9], q);
 digitalWrite (led[10], r);
 digitalWrite (led[11], s);
 digitalWrite (led[12], t);
 digitalWrite (led[13], u);
 digitalWrite (led[14], v);
 digitalWrite (led[15], w);
}
void LVL (int h, int i, int j, int k) { //Esta función permite establecer valores de encendido y apagado a cada fila
  digitalWrite (lvl[0], h);
  digitalWrite (lvl[1], i);
  digitalWrite (lvl[2], j);
  digitalWrite (lvl[3], k);
}

void level() { //En esta función se establecen los valores para las filas donde con un 1 los leds no encienden y con 0 encienden
 switch (y) {
 case 0: LVL (1, 0, 0, 0); break; //La primera fila encendida
 case 1: LVL (0, 1, 0, 0); break; //La segunda fila encendida
 case 2: LVL (0, 0, 1, 0); break; //La tercera fila encendida
 case 3: LVL (0, 0, 0, 1); break; //La cuarta fila encendida
 case 4: LVL (1, 0, 0, 1); break; //Las filas exteriores encendidas
 case 5: LVL (0, 1, 1, 0); break; //Las filas interiores encendidas
 case 6: LVL (1, 1, 1, 1); break; //Todas las filas encendidas
 case 7: LVL (1, 1, 1, 0); break; //Todas excepto la última
 case 8: LVL (1, 1, 0, 0); break; //Las primeras dos
 case 9: LVL (0, 0, 1, 1); break; //Las últimas dos
 case 10: LVL (0, 0, 0, 0); break; //Ninguna encendida
 case 11: LVL (0, 1, 1, 1); break; //Todas excepto la primera
 }
}


//Funcion para encender columnas exteriores
void extCol(int t){
  int pins[]={0,1,2,3,7,11,15,14,13,12,8,4}; //columnas que se deben encender
  Serial.println("extCol"); //Mensaje que indica que entro al efecto
  y=6; level(); //"y" define el caso que se evaluara en la funcion level()
  for(int i = 0; i < 12; i++){ 
    if(flag_efectos){break;} //bandera que sale del for si se oprime el boton que cambia los efectos
    LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1); //apaga todos los leds
    digitalWrite(led[pins[i]],0); //enciende el led que se necesita
    delay(t); //delay para ver efecto
    if(flag_tiempos){t = times[a];flag_tiempos=0; //bandera que cambia el tiempo que se esta usando cuando se oprime el boton
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);} //activa nuevamente la interrupcion
  }
  //w++;
}

//En esta animación prende led por led
void uxu(int t) {
  Serial.println("uxu");
  for (int j = 0; j < 4; j++) { //Este for se utiliza para pasar de una fila a otra
    y = j; level();
    for (int i = 0; i < 16; i++){
      LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
      digitalWrite(led[i],0);
      delay(t);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
      if(flag_efectos){break;}
    }
    if(flag_efectos){break;}    
  }
 //w++;
}

//En esta animación prende fila por placa
void pxp(int t) {
  Serial.println("pisoxpiso");
  //int t = 500; //Esta variable determina el tiempo que espera el Arduino para realizar la siguiente acción
  for (int j = 0; j < 4; j++) { //Este for se utiliza para pasar de una fila a otra
    y = j; level();
    LED (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); delay(t);
    if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    if(flag_efectos){break;}
  }
  for (int k = 3; k >= 0; k--) { //Este for se utiliza para pasar de una fila a otra
    if(flag_efectos){break;}
    y = k; level();
    LED (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); delay(t);
    if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
  }
 //w++;
}


//En esta animación prende un cubo en el centro y después prende un cubo en el exterior
void cubito(int t) {
 Serial.println("cubito");
 for(int i = 0; i < 10; i++){
  if(flag_efectos){break;}
   y = 6; level();
   LED (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); delay(t);
   if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
   y = 5; level();
   LED (1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1); delay(t);
   if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
 }
 //w++;
}

//En esta animación prende una cara del cubo
void caraxcara(int t) {
  Serial.println("caraxcara");
  y = 6; level();
  LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  for(int i = 0; i < 16; i = i + 4){ //activa las columnas de inicio a fin
    if(flag_efectos){break;}
    digitalWrite(led[i],0);
    digitalWrite(led[i+1],0);
    digitalWrite(led[i+2],0);
    digitalWrite(led[i+3],0);
    delay(t);
    if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
  }
    for(int i = 0; i < 16; i = i + 4){ //desactiva las columnas de inicio a fin
    if(flag_efectos){break;}
    digitalWrite(led[i],1);
    digitalWrite(led[i+1],1);
    digitalWrite(led[i+2],1);
    digitalWrite(led[i+3],1);
    delay(t);
    if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
  }
    for(int i = 15; i >= 0; i = i - 4){ //activa las columnas de fin a inicio
    if(flag_efectos){break;}
    digitalWrite(led[i],0);
    digitalWrite(led[i-1],0);
    digitalWrite(led[i-2],0);
    digitalWrite(led[i-3],0);
    delay(t);
    if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
  }
    for(int i = 15; i >= 0; i = i - 4){ //desactiva las columnas de fin a inicio
    if(flag_efectos){break;}
    digitalWrite(led[i],1);
    digitalWrite(led[i-1],1);
    digitalWrite(led[i-2],1);
    digitalWrite(led[i-3],1);
    delay(t);
    if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
  }
  //w++;
}

//Esta funcion recorre desde el centro hasta la punta
void culebrita(int t){
  int pins[]={9,10,6,5,4,8,12,13,14,15,11,7,3,2,1,0}; //leds que se van a encender
  int culb[]={0,8,7,6}; //niveles que se van a encender
  for (int j = 0 ; j < 4 ; j++){ //for para recorrer los niveles a encender
    if(flag_efectos){break;}
    y=culb[j]; level();
    for (int i = 0; i < 16 ; i++){ //for para recorrer los leds que se van a encender de inicio a fin
      if(flag_efectos){break;}
      LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
      digitalWrite(led[pins[i]],0);
      delay(t);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for (int i = 15; i >= 0; i--){ //for para recorrer los leds que se van a encender de fin a inicio
      if(flag_efectos){break;}
      LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
      digitalWrite(led[pins[i]],0);
      delay(t);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
  }
  //w++;
}

void cuadrado(int t){
  int culb[]={0,8,7,6};
  int pins[]={0,1,2,3,7,11,15,14,13,12,8,4}; //leds que se van a encender
  y = 0; level();
  LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1); //inicializa los leds
  for(int j = 0; j < 12; j++){
    if(flag_efectos){break;}
    digitalWrite(led[pins[j]],0);
    delay(t);
    if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
  }
  for(int i = 0; i < 2; i++){
    if(flag_efectos){break;}
      LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1); delay(t);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
      LED (0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0); delay(t);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
  }
  for (int j = 0; j < 4; j++) { //Este for se utiliza para pasar de una fila a otra
    if(flag_efectos){break;}
    y = culb[j]; level();
    LED (0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0); delay(t);
    if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
  }
  y=6; level();
  for(int i = 0; i < 2; i++){
    if(flag_efectos){break;}
      LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1); delay(t);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
      LED (0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0); delay(t);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
  }

  //w++;
}

//Este efecto usa pwm para encender y apagar los leds con efecto fade
//void PWMled(){
//  Serial.println("vertices");
//  y=6; level();
//  LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
//  for(int duty = 0; duty <= 255; duty++){
//    Serial.println(duty);
//    for(int x = 0; x < 16; x++){
//      Serial.println(x);
//     ledcWrite(channel[x], duty); 
//    }
//    delay(20);
//  }
//  for(int duty = 255; duty >= 0; duty--){
//    Serial.println(duty);
//    for(int x = 0; x < 16; x++){
//      Serial.println(x);
//     ledcWrite(channel[x], duty); 
//    }
//    delay(20);
//  }
//}

//Efecto de leds Random
void randLed(int t){
  for (int j = 0; j < 50; j++){
    if(flag_efectos){break;}
     LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
     y = 10; level();
     digitalWrite(led[random(0,16)],0);
     digitalWrite(lvl[random(0,4)],1);
     delay(t);
    if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
  }
  //w++;
}

//Funcion para efecto de carga de cubo
void cargaCubo(int t){
  int lvls[]={3,2,1,0};
  Serial.println("cargaCubo");
  for (int i = 0; i < 4; i++){
    if(flag_efectos){break;}
    LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
    y = lvls[i]; level();
    for(int j = 15; j >= 0; j--){
      if(flag_efectos){break;}
      digitalWrite(led[j],0);
      delay(t);
    if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
  }
  //w=1;
}

//Este efecto ilumina diagonalmente el cubo
void diagonal(int t){
  Serial.println("diagonal");
    LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
    int ini=millis();
    for(int i=0;i<(t/5);i++){
      if(flag_efectos){break;}
      y=0;level();
      LED (0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1); delay(5);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for(int i=0;i<(t/4);i++){
      if(flag_efectos){break;}
      y=8;level();
      LED (0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      y=0;level();
      LED (1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for(int i=0;i<(t/6);i++){
      if(flag_efectos){break;}
      y=7;level();
      LED (0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      y=8;level();
      LED (1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      y=0;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1); delay(2);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for(int i=0;i<(t/8);i++){
      if(flag_efectos){break;}
      y=6;level();
      LED (0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      y=7;level();
      LED (1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      y=8;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1); delay(2);
      y=0;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0); delay(2);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for(int i=0;i<(t/6);i++){
      if(flag_efectos){break;}
      y=6;level();
      LED (0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      y=7;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1); delay(2);
      y=8;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0); delay(2);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for(int i=0;i<(t/4);i++){
      if(flag_efectos){break;}
      y=6;level();
      LED (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1); delay(2);
      y=7;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0); delay(2);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for(int i=0;i<(t/4);i++){
      if(flag_efectos){break;}
      y=6;level();
      LED (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); delay(2);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for(int i=0;i<(t/4);i++){
      if(flag_efectos){break;}
      y=11;level();
      LED (0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      y=6;level();
      LED (1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); delay(2);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for(int i=0;i<(t/6);i++){
      if(flag_efectos){break;}
      y=9;level();
      LED (0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      y=11;level();
      LED (1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      y=6;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0); delay(2);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for(int i=0;i<(t/8);i++){
      if(flag_efectos){break;}
      y=3;level();
      LED (0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      y=9;level();
      LED (1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      y=11;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1); delay(2);
      y=6;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0); delay(2);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for(int i=0;i<(t/6);i++){
      if(flag_efectos){break;}
      y=3;level();
      LED (1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1); delay(2);
      y=9;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1); delay(2);
      y=11;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0); delay(2);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for(int i=0;i<(t/4);i++){
      if(flag_efectos){break;}
      y=3;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1); delay(2);
      y=9;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0); delay(2);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    for(int i=0;i<(t/4);i++){
      if(flag_efectos){break;}
      y=3;level();
      LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0); delay(2);
      if(flag_tiempos){t = times[a];flag_tiempos=0;
    attachInterrupt(digitalPinToInterrupt(tiemposSW),tiempos,FALLING);}
    }
    LED (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
}
