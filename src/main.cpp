#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>

#define SS PB3
#define DO PB1
#define DI PB0
#define CLK PB2
#define CS PB4

// data mapping
uint8_t datadisplay[] = {0x3F,0x6,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
struct dataBMP
{
  uint16_t T1,P1;
  signed short T2,T3,P2,P3,P4,P5,P6,P7,P8,P9;
};

void spiInit();
uint8_t spiTrans(uint8_t data);
void temperatuurConverter(float temperatuur);
void Pa(float druk);
dataBMP kalibratiedata (uint8_t data,uint8_t port);
void setBMP280Modus(uint8_t data,uint8_t port);
uint32_t adcData (uint8_t data,uint8_t port);
uint8_t converterDataToDisplay(uint8_t data);
void display(uint8_t right,uint8_t left);
long temperatuurAndPressueDisplay();



int main(int argc, char const *argv[])
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
   temperatuurAndPressueDisplay();
  }
 
  return 0;
}

// spi init functie
void spiInit(){

      DDRB |= (1<<DDB3)|(1<<DDB1)|(1<<DDB2)|(1<<DDB4);//data direction als output
      DDRB &= ~(1 << DDB0);  // data direction als in
      USICR = (1<<USIWM0); // select 3 wire mode
      PORTB |= (1<<SS)|(1<<CS); // maak ss en cs hoog voor spi communicatie
};

// spi transfer functie
uint8_t spiTrans(uint8_t data){
  
  USIDR = data;
   for (char i = 0; i < 8; i++)
  {
    //toggel clock in dus in totale 16 keer
    USICR = (1<<USIWM0) | (1<<USITC); 
    USICR = (1<<USIWM0)|(1<<USICLK)|(1<<USITC);
  }
  
  return USIDR;
}

// functie om de sensor mode te selecteren 
void setBMP280Modus(uint8_t reg,uint8_t cs){

  spiInit();
  PORTB &= ~(1<<cs);
  spiTrans(reg);
  spiTrans(0b01000011);
  PORTB |= (1<<cs);
}

// sensor calibratie
dataBMP kalibratiedata (uint8_t reg,uint8_t cs)
{

  dataBMP resulte;
  
          spiInit();
          PORTB &= ~(1<<cs);
          spiTrans(reg);
        // read temperatuur data voor calibratie
          uint8_t T1LSB = spiTrans(0);
          uint8_t T1MSB = spiTrans(0);
          signed char T2LSB = spiTrans(0);
          signed char T2MSB = spiTrans(0);
          signed char T3LSB = spiTrans(0);
          signed char T3MSB = spiTrans(0);
        // read drukte data voor calibratie
          uint8_t P1LSB = spiTrans(0);
          uint8_t P1MSB = spiTrans(0);
          signed char P2LSB = spiTrans(0);
          signed char P2MSB = spiTrans(0);
          signed char P3LSB = spiTrans(0);
          signed char P3MSB = spiTrans(0);
          signed char P4LSB = spiTrans(0);
          signed char P4MSB = spiTrans(0);
          signed char P5LSB = spiTrans(0);
          signed char P5MSB = spiTrans(0);
          signed char P6LSB = spiTrans(0);
          signed char P6MSB = spiTrans(0);
          signed char P7LSB = spiTrans(0);
          signed char P7MSB = spiTrans(0);
          signed char P8LSB = spiTrans(0);
          signed char P8MSB = spiTrans(0);
          signed char P9LSB = spiTrans(0);
          signed char P9MSB = spiTrans(0);

           PORTB |= (1<<cs);

          // samenvoegen van msb en lsb bits om een 16bits krijgen voor T1,T2,T3 (temperatuureratuur)
           resulte.T1 = (T1MSB<<8)|T1LSB;
           resulte.T2 = (T2MSB<<8)|T2LSB;
           resulte.T3 = (T3MSB<<8)|T3LSB;

          // samenvoegen van msb en lsb bits om een 16bits krijgen voor Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9 (temperatuureratuur)
           resulte.P1 = (P1MSB<<8)|P1LSB;
           resulte.P2 = (P2MSB<<8)|P2LSB;
           resulte.P3 = (P3MSB<<8)|P3LSB;
           resulte.P4 = (P4MSB<<8)|P4LSB;
           resulte.P5 = (P5MSB<<8)|P5LSB;
           resulte.P6 = (P6MSB<<8)|P6LSB;
           resulte.P7 = (P7MSB<<8)|P7LSB;
           resulte.P8 = (P8MSB<<8)|P8LSB;
           resulte.P9 = (P9MSB<<8)|P9LSB;
    return resulte;        
}


// functie om de adc data te verkrijgen
uint32_t adcData (uint8_t reg,uint8_t cs)
{

  uint32_t adc_T=0;
  
          spiInit();
          PORTB &= ~(1<<cs);
          spiTrans(reg);

          uint32_t firstBits = spiTrans(0);
          uint32_t secondBits = spiTrans(0);
          uint32_t lasttBits = spiTrans(0);
          
          adc_T = (firstBits)<<16|(secondBits)<<8|(lasttBits);
          PORTB |= (1<<cs);   

    return adc_T;        
}

//functie om de temperatuur en lucht te verkrijgen
long temperatuurAndPressueDisplay (){

  dataBMP val = kalibratiedata(0x88,CS);
  setBMP280Modus(0X74,CS);
 
  while (true)
  {
  // temperatuur berenken
    long adc_T = adcData(0xFA,CS);
    adc_T >>=4;

    long  var1 = (((( adc_T  >> 3) - ((long)val.T1 << 1))) * ((long)val.T2)) >> 11;
    long  var2 = ((((( adc_T  >> 4) - ((long)val.T1)) * ((adc_T  >> 4) - ((long)val.T1))) >> 12) * ((long)val.T3)) >> 14;
   
    float  temperatuur = (var1 + var2) / 5120.0;
    float Fahrenheit = (temperatuur*1.8)+32;

// Pa waarde bereken

    var1 = ((long)temperatuur / 2.0) - 64000.0;
    var2 = var1 * var1 * ((long)val.P6) / 32768.0;
    var2 = var2 + var1 * ((long)val.P5) * 2.0;
    var2 = (var2 / 4.0) + (((long)val.P4) * 65536.0);
    var1 = (((long) val.P3) * var1 * var1 / 524288.0 + ((long) val.P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((long)val.P1);

    if (var1 == 0.0)return 0; // voorkomen delen bij zero error;

    long p = 1048576.0 - (long)adc_T;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((long) val.P9) * p * p / 2147483648.0;
    var2 = p * ((long) val.P8) / 32768.0;
    p = p + (var1 + var2 + ((long)val.P7)) / 16.0;
    float druk = p/100;

// print getal op het display
    temperatuurConverter(temperatuur);
    temperatuurConverter(Fahrenheit); 
    Pa(druk);
}    
  return 0; 
}

// convert int to getal van de temperatuur/drukte naar hex voor de display
uint8_t converterDataToDisplay(uint8_t data){

switch (data)
{
case 0:
  data = datadisplay[0];
  break;

case 1:
  data = datadisplay[1];
  break;
case 2:
  data = datadisplay[2];
  break;
case 3:
  data = datadisplay[3];
  break;
case 4:
  data = datadisplay[4];
  break;
case 5:
  data = datadisplay[5];
  break;
case 6:
  data = datadisplay[6];
  break;
case 7:
  data = datadisplay[7];
  break;
case 8:
  data = datadisplay[8];
  break;
case 9:
  data = datadisplay[9];
  break;
  case 10:
  data = 0x80;
  break;

  case 11:
  data = 0;
  break;
}
     return data;
}

// geeft temperatuur/ drukte weer op het dissplay
void display(uint8_t right, uint8_t left){
  
  right = converterDataToDisplay(right);
  left = converterDataToDisplay(left);
  PORTB &= ~(1<<SS);
     spiTrans(left);
     spiTrans(right);
     PORTB |= (1<<SS);
     _delay_ms(100);
}

//temperatuur weergave
void temperatuurConverter(float temperatuur){
  int tientallen = (int(temperatuur/10));
    int eenheden = ((int)temperatuur)%10;
    int tienden = (int(temperatuur * 10))%10;
     display(tientallen,eenheden);
    display(10,tienden);
    display(11,11);
}

//drukte weergave
void Pa(float druk){
  int hondertallen = (int(druk/100));
  int tientallen = ((int)druk%100)/10;
  int eenheden = ((int)druk%100)%10;
    display(hondertallen,tientallen);
    display(eenheden,11);
    display(11,11);
}