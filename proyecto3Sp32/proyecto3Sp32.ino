//*****************************************************************************
// Universidad del Valle de Guatemala
// BE3015 - Electrónica Digital 2
// Fernando Delgado 19144
// Proyecto #3
//*****************************************************************************

//*****************************************************************************
// Librerías
//*****************************************************************************
#include <Arduino.h>      
#include <Separador.h>
//librerías de NEOPIXEL
#include <Adafruit_NeoPixel.h>

//Librerías para MAX30105
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

//*****************************************************************************
// Definición de pines
//****************************************************************************



//se nombran el pin del neopixel
#define PIN 4
#define MAX_BRIGHTNESS 255
//*****************************************************************************
// Prototipos de función
//*****************************************************************************
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);//esto indica cuantos neopixel son se puede cambiar dependiendo la cantidad 
void sensorMAX30105(void);
//*****************************************************************************
// Variables Globales
//*****************************************************************************
// Create An LCD Object. Signals: [ RS, EN, D4, D5, D6, D7 ]

uint8_t voltaje3; //comunicacion USART

int adcRaw;    
float voltaje; 

//filtro para el sensor
float adcFiltradoEMA = 0; // S(0) = Y(0)

float dutycycleled1 = 0; //Se dara uso para la señal que recibira este codigo atravez de la comunicacion UART
int dutycycleled2 = 0;
double alpha = 0.09; // Factor de suavizado (0-1) Factor necesario para el filtro aplicado en el sensor

String voltaje2 = ""; //se denota otra variale tipo texto

//Contador
String dutycycleled3 = "";

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; // dato
int32_t spo2; //SPO2
int8_t validSPO2; //valor del SPO2 es valido
int32_t heartRate; //Ritmo Cardíaco 
int8_t validHeartRate; /Ritmo cardíaco es valido

byte pulseLED = 11; 
byte readLED = 13; 

String dato ="";
String  HR = ""; 
String  SPO2="";
Separador s;

//************************************************************************
//ISR
//*************************************************************************

//*****************************************************************************
// Configuración
//*****************************************************************************
void setup()
{

  Serial.begin(115200);//se inciia los dos monitores seriales para envio y docuemntacion de datos 
  Serial2.begin(115200);
  //Configuración MAX30105
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  
  // Initialización para MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //se inicializa la velocidad del sensor 
  {
    Serial.println(F("No se encontró sensor MAX30105, mala conexión"));
    while (1);
  }

  Serial.println(F("coloque el dedo en sensor e ingrese dato al monitor"));
  while (Serial.available() == 0) ; //se ha iniciado el monitor, si o no?
  Serial.read();

  byte ledBrightness = 60; //Configuración: 0=Off a 255=50mA
  byte sampleAverage = 4; //Configuración: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Configuración: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Configuración: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Configuración: 69, 118, 215, 411
  int adcRange = 4096; //Configuración: 2048, 4096, 8192, 16384
  
  //Configuración del sensor MAX30105
   particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  //Inicialización de NEOPIXEL
  strip.begin();
  strip.setBrightness(50);
  strip.show(); // Initialize all pixels to 'off'
}

//*****************************************************************************
// Loop Principal
//*****************************************************************************
void loop()
{
  colorWipe(255,20);
  sensorMAX30105();
  if(Serial2.read()=='guardando'){
    rainbow(20);
  }
  
}

//---------------------------------------------------------------------------------------------------------------------
//Sensor MAX30105
//---------------------------------------------------------------------------------------------------------------------
void sensorMAX30105(void){
  //Tamaño del buffer de 100 para alacenamiento de datos 
   bufferLength = 100; 

   //lee 100 muestras y determina el rango de la señal
  for (byte i = 0 ; i < bufferLength ; i++){

  
  while (particleSensor.available() == false)
  
      particleSensor.check(); //variable para inicalizar datos 

  redBuffer[i] = particleSensor.getRed();
  irBuffer[i] = particleSensor.getIR();
  particleSensor.nextSample(); //siguiente muestra 

  Serial.print(F("red="));//despliegue de valores del buffer
  Serial.print(redBuffer[i], DEC);
  Serial.print(F(", ir="));
  Serial.println(irBuffer[i], DEC);

  }
  //Calcula despues de 100 muestras los valores 
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);


  while (1){
    //se colocan 25 muestras y otras 75 
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //restar 25 muestras para calculos 
    for (byte i = 75; i < 100; i++)
    {
      //ver nuevos datos 
      while (particleSensor.available() == false) 
      //repetir los datos y verificar que no haya mas datos 
        particleSensor.check(); 

      digitalWrite(readLED, !digitalRead(readLED)); //Parpadea con la led cada que ingresa nuevos datos

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //se mueve a la siguiente

      Serial2.print(heartRate, DEC);
      Serial2.print(",");
      Serial2.println(spo2,DEC);

     if (Serial2.available()>0){
      rainbowCycle(2);//ciclo para neopixel 
      
     //se incializa la lectura de datos desde la tiva 
      dato = Serial2.readStringUntil('\n');//leer el dato hasta que haya un enter 
      //Separa el primer dato del primer indicador, en este caso la ',' y el primer dato se indica con el 0
      HR = s.separa(dato, ',',0);
      //Separa el segundo dato con el indicador ',' en este caso se indica con el 1
      SPO2 = s.separa(dato,',',1);
     
      Serial.println("Heart rate= "+HR);//mostrar los datos obtenidos 
      Serial.println("Oximetría= "+SPO2);
      }
     }

      //Después de tomar 25 muestras se recalculan los datos del ritmo cardíaco y el SPO2
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    }
}
//---------------------------------------------------------------------------------------------------------------------
//Funciones para Neopixel
//---------------------------------------------------------------------------------------------------------------------
// Secuencia de leds para orden 
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
//funcion para el movimiento 
void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Arcoiris neopixel
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*2; j++) { 
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//recorrido colores leds
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) { 
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);   
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);      
      }
    }
  }
}


void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {    
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);       
      }
    }
  }
}


uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
