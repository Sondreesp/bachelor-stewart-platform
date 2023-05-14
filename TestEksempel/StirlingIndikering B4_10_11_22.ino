const int pinUt =  10;// Utgangspinn
const int pinRpm =  11;// Utgangspinn
const byte interruptPin = 2; //Inngangspinn fra sensor
unsigned long Periode [160]; //Tabell for pulslengde data 
int PeriodeI; // Peker for tabell
long  Sum = 0; // For utregning av Gjennomsnitt pulsbredde
int GPulsbredde = 10;// Gjennomsnitt pulsbredde
int PulsBredde = 200;
bool NyPuls = false;



void setup() {
  Serial.begin(19200); // open the serial port at 19200 bps: 
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(pinUt, OUTPUT);
  pinMode(pinRpm, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(interruptPin), pinInterupt, RISING);      // Interupt inngang 
  
  TCCR1A = 0;
  TCCR1B = 0; 
  TCCR1B = (0<<WGM12) |(1<<CS11);     //Prescalar=8
  TIMSK1 = (0<<OCIE1A)|(1<<TOIE1);  

  TCCR2A = 0;
  TCCR2B = 0; 
  TCCR2B = (0<<WGM22) |(1<<CS21);     //Prescalar=8 CNC
  TIMSK2 = (0<<OCIE2A)|(1<<TOIE2);

}

void loop() {
   delay(1000);
  Serial.println(GPulsbredde);
  //Serial.println(PeriodeI);
}

void pinInterupt(void){
    Periode[PeriodeI] = TCNT1;          //  Står tidlig for å redusere forsinkelse
    TCNT1 = 0;
    TCNT2 = 0;
    digitalWrite(pinUt, HIGH);          // Sampel puls høy
    OCR2B = GPulsbredde/2 ;
    TIMSK2 |= (1<<OCF2B);
    TIFR2 = (1<<OCF2B);
    if (Periode[PeriodeI] > 3000) {
        //Serial.println(PeriodeI); 
        PeriodeI = 0;
        Sum = 0;
        
        for (int i = 1; i <= 55; i++) {
            Sum += Periode[i];
            }
          GPulsbredde = Sum/56;
          
    }    
    else {
      PeriodeI++;
      if (PeriodeI > 58) {
        //Serial.println(PeriodeI);
        //PeriodeI = 0;
        }
    
    }
    if (PeriodeI == 57) {
      NyPuls = true;
      OCR2A = GPulsbredde; 
      TIMSK2 |= (1<<OCIE2A);
      TIFR2 = (1<<OCF2A);
    }
    if (PeriodeI == 57) {
        digitalWrite(pinRpm, HIGH);       // Startpuls
        delayMicroseconds(PulsBredde);
        digitalWrite(pinRpm, LOW);     
    }    
} 
    
    
        
   
ISR(TIMER2_COMPA_vect){
   TCNT2 = 0;
   digitalWrite(pinUt, HIGH);
   OCR2B = GPulsbredde/2 ;
   TIMSK2 |= (1<<OCF2B);
   TIFR2 |= (1<<OCF2B);
  
  if (NyPuls){
    OCR2A = GPulsbredde ;
    NyPuls = false;
    TIMSK2 |= (1<<OCIE2A);   // Aktiver andre puls
  }
  else{
    TIMSK2 &= ~(1<<OCIE2A);
    }
  }

ISR(TIMER2_COMPB_vect){
  digitalWrite(pinUt, LOW);
  TIMSK2 &= ~(1<<OCIE2B);
  }
