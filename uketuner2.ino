
#define yLED 23 
#define gLED 22
#define bLED 21
#define buttonG 36
#define buttonC 39
#define buttonE 34
#define buttonA 35
#define mic 32

#include "arduinoFFT.h"

const double freqRange = .5;

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 1200;
const uint8_t amplitude = 100;
const int lowFreq = samplingFrequency/(samples);
const int delayMicro = 1000000.0/samplingFrequency-92.33; // 92.33us ADC conversion time 
int startFreq;
int endFreq;
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

double targetFreq;

void setG(){
  targetFreq = 414
  ;
  startFreq =  targetFreq - 100;
 endFreq = targetFreq + 100;
}

void setC(){
  targetFreq = 264;
  startFreq =  targetFreq - 100;
 endFreq = targetFreq + 100;
}

void setE(){
  targetFreq = 333;
  startFreq =  targetFreq - 100;
 endFreq = targetFreq + 100;
}

void setA(){
  targetFreq = 446;
  startFreq =  targetFreq - 100;
 endFreq = targetFreq + 100;
}
void setup() {
  // put your setup code here, to run once:
  
  pinMode (yLED, OUTPUT);
  digitalWrite (yLED, LOW);
  pinMode (gLED, OUTPUT);
  digitalWrite (gLED, LOW);
  pinMode (bLED, OUTPUT);
  digitalWrite (bLED, LOW);
  pinMode (buttonG, INPUT);
  pinMode (buttonC, INPUT);
  pinMode (buttonE, INPUT);
  pinMode (buttonA, INPUT);
  pinMode (mic, INPUT);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(buttonG),setG, RISING);
  attachInterrupt(digitalPinToInterrupt(buttonC),setC, RISING);
  attachInterrupt(digitalPinToInterrupt(buttonE),setE, RISING);
  attachInterrupt(digitalPinToInterrupt(buttonA),setA, RISING);

}

void loop() {
  // put your main code here, to run repeatedly:
  double vReal[samples];
  double vImag[samples]={0};
  unsigned long start = millis();
  for(int i=0;i<samples;i++){
  vReal[i]=analogRead(mic);
  delayMicroseconds(delayMicro);
}

  unsigned long end = millis();
  Serial.println();
  Serial.println(end-start);
  Serial.println(start);
  Serial.println(end);
  Serial.println();

  // Serial.println("Data:");
  // PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  // Serial.println("Weighed data:");
  // PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  // Serial.println("Computed Real values:");
  // PrintVector(vReal, samples, SCL_INDEX);
  // Serial.println("Computed Imaginary values:");
  // PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  // Serial.println("Computed magnitudes:");
  // PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  // double x = FFT.MajorPeak(vReal, samples, samplingFrequency, startFreq/lowFreq, endFreq/lowFreq);
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  // Serial.println(x, 6);
  // while(1); /* Run Once */
  // delay(2000); /* Repeat after delay */
//  if(x<targetFreq-100)
//   return;
  Serial.print("frequency: ");
  Serial.println(x);
  Serial.print("startFreq: ");
  Serial.println(startFreq);
  Serial.print("endFreq: ");
  Serial.println(endFreq);
  Serial.print("lowFreq: ");
  Serial.println(lowFreq);


  if(x <= targetFreq + freqRange && x>= targetFreq - freqRange ){
  digitalWrite (gLED, HIGH);
  digitalWrite (bLED, LOW);
  digitalWrite (yLED, LOW);
  delay(1500);
  }
  else if(x> targetFreq){
  digitalWrite (yLED, HIGH);
  digitalWrite (gLED, LOW);
  digitalWrite (bLED, LOW);
  }
  else if(x< targetFreq){
    digitalWrite (bLED, HIGH);
  digitalWrite (yLED, LOW);
  digitalWrite (gLED, LOW);
  }

}