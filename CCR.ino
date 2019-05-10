#include <Adafruit_ILI9341.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <SPI.h>
#include <string.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <EEPROM.h>

#define demo 0           // 0 - adc       1 - simulace hodnot
#define sensitivity 2    // 1 -citlivejsi 2 - mene citlive

#define rst 8
#define cs 10
#define dc 9

#define _RED 1
#define _GREEN 2
#define _ORANGE 3
#define _YELLOW 3
#define _OFF 0
#define _ON 1

#define displayLight  A0 //5 //9
#define displayPwr 5  //A0 // 7 //10
#define sol 4
#define sol1 A6    //port bundle with A7
#define sol2 A7    //port bundle with A6
#define hud1 6
#define hud2 7
//#define bzuk A7
#define batteryADC A1
#define waterContact A2

// Color definitions
#define	BLACK           ILI9341_BLACK
#define	RED             ILI9341_RED
#define	BLUE            ILI9341_BLUE
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          ILI9341_YELLOW  
#define WHITE           0xFFFF

#define TIMERMENU       250
#define WAKEUP_DELAY    5 // for piezo it should be something near 0

#define MAX_DIFF 12   // bylo 5   20140717 uprava pro Michala Trucku na Lestince

#define OXY 0.98
#define OFFSET -3

#define _DIFF      10
#define _ROTATION  11
#define _BCKLIGHT  12
#define _KEYLEFT   13
#define _KEYRIGHT  14
#define _CELL1_L   20     
#define _CELL1_H   21     
#define _CELL2_L   22     
#define _CELL2_H   23     
#define _CELL3_L   24     
#define _CELL3_H   25   
#define _STACK_L   26
#define _STACK_H   27
#define _STACKMAX_L   28
#define _STACKMAX_H   29
#define _HUDMODE  30



//ST7735 tft = ST7735(cs, dc, rst);  
Adafruit_ILI9341 tft = Adafruit_ILI9341(cs, dc, rst);  
//Adafruit_ILI9341 tft = Adafruit_ILI9341();  

Adafruit_ADS1115 ads(0x48);

char s0[11]=" 123456789";       // retezec pro prevod cisla 0.34 => .34   /   string for number conversion  0.34 -> .34
uint8_t button=0;
unsigned int tcnt2;
unsigned int counter=0;
unsigned int counterSolenoid=0;
volatile static unsigned int menu=0;
volatile static unsigned char backlightCnt=0;
unsigned int menuPrev=0;
unsigned int count=0;
unsigned int timerMenu=0;
uint8_t setpoint=70;
uint8_t setpoint_tmp;
uint8_t solenoid=1;
uint8_t solenoid_tmp;
int16_t adc0,adc1,adc2,adc3;
uint16_t cal1,cal2,cal3;
uint16_t avg;
double c1, c2, c3;
uint16_t ppo0,ppo1,ppo2;
double mv0;
double batteryVoltage, batteryPrev;
float diffSet, diffSet_tmp;
uint8_t screenRotation, screenRotation_tmp;
uint8_t backlightSet, backlightSet_tmp;
uint8_t keyLeft, keyRight;
uint8_t vote;
uint8_t solSwitch=0;
uint8_t solPref;
uint8_t spPref;
uint8_t test_count=0;
uint8_t dispRewrite=0;
int16_t stackTime=0, stackTimeTmp, stackTimeMax;
uint16_t timerINTprev=0;
uint8_t firstRun=1;
uint8_t resetCount;
uint16_t waterSwitch;
volatile static uint8_t HUDMode;
uint16_t background, font, fontGreen, fontRed, fontYellow;



unsigned int test[] = {
//    rozlitana
         56,90,58,  65,120,65,  69,136,70,  70,140,71,  90,153,96,  
         130,210,131,  195,252,195,  293,401,294,  162,190,162,  
         131,200,132,  92,162,94,  71,139,72,  68,126,69,  
         67,124,68,  59,115,59,  57,112,58,  55,110,57

//    linearni kolem 0.7
//        140,145,142, 141,145,143, 141,146,143, 142,146,143,
//        143,147,143, 144,148,143, 144,148,144, 144,148,144,
//        144,148,144, 145,149,144, 145,149,144, 144,148,144,
//        143,147,143, 142,147,142, 141,146,142, 140,145,142

//    linearni kolem 0.9-1.2
//        163,175,162, 174,184,173, 174,184,174, 175,187,175,
//        178,189,179, 182,192,183, 188,202,189, 191,217,192,
//        192,219,192, 193,217,193, 192,212,189, 187,208,186,
//        180,185,179, 175,181,176, 170,178,171, 165,174,165

};

typedef struct                                          // struktura, zde bitove pole 
{ 
   unsigned keyChange:   1;                                 // "1" = 1bit 
   unsigned keyUp:      1; 
   unsigned keyDown:   1; 
} tlacitka_t; 

typedef struct
{
   unsigned processing: 1;
   unsigned char length;
   unsigned char position;
   unsigned char string[100];
} blink_t;

typedef struct
{
   uint16_t ppo[3];
   double mV[3];
} cells_t;

volatile static tlacitka_t tlacitka; 
volatile static blink_t blink;
cells_t cells;
volatile static uint16_t timerINT1=0;
volatile static uint16_t timerINTsec=0;
volatile static uint16_t timerINTmin=0;
volatile static uint16_t timerHUD=0;

ISR(TIMER2_OVF_vect) 
{ 
   static unsigned char up, down, ms;   // promenne "static" zustavaji v pameti i po opusteni funkce (u globalni prom. ma slovo jiny vyznam)) 
   TCNT2 = tcnt2;
    
   if (timerINT1++ > 495)
   {
     timerINT1=0;
     if (timerINTsec++ > 59)
     {
       timerINTsec=0;
       if (timerINTmin++ >16000)
         timerINTmin=0;
     }  
   }

   
   if ((digitalRead(keyLeft)) && (tlacitka.keyChange==0)) {
	 if (up < 2) // pro reed switch 255
	   up++;
	 else {
	   tlacitka.keyUp = 1;
           tlacitka.keyChange=1;
	   up=0;
	 }
   }
   else if ((digitalRead(keyRight)) && (tlacitka.keyChange==0)) {
     	 if (down < 2) 
	   down++;
	 else {
	   tlacitka.keyDown = 1;
           tlacitka.keyChange=1;
	   down=0;
	 }
   }
   else {
	 up=1;   // pro reed bylo 254
	 down=1;
   }
   

   
   if (backlightCnt<backlightSet)
   {
     digitalWrite(displayLight,LOW); 
     backlightCnt++;
     backlightCnt++;
   }
   else
   {
     digitalWrite(displayLight,HIGH); 
     backlightCnt=0;
   }


// vyblikavani ppo2
   if ((HUDMode == 1) && (timerHUD++ > 40))
   {
     timerHUD=0;
     if ((blink.processing == 1) && (blink.position < blink.length))
     {
        switch (blink.string[++blink.position])
        {
           case 0: digitalWrite(hud1,LOW); digitalWrite(hud2,LOW); break;
           case 1: digitalWrite(hud1,HIGH); digitalWrite(hud2,LOW); break;
           case 2: digitalWrite(hud1,LOW); digitalWrite(hud2,HIGH); break;
//           case 3: digitalWrite(hud1,LOW); digitalWrite(hud2,HIGH);digitalWrite(hud1,HIGH); digitalWrite(hud2,LOW); break;
        }
        if (blink.position == blink.length)
        {
           blink.position = 0;
           blink.processing = 0;
        }
     }
   }
   
 
}



void wakeUpNow()
{
    tlacitka.keyChange=0;
}

void sleepNow()
{
  for (byte i=0; i<22; i++)
  {
    pinMode(i,OUTPUT);
    digitalWrite(i,LOW);
  }
  byte adcsra_save=ADCSRA;
  ADCSRA=0;
    
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
//  power_all_disable();  

  pinMode(displayPwr,INPUT);
//  digitalWrite(displayPwr,LOW);

  attachInterrupt(0,wakeUpNow, HIGH);
  sleep_mode();
  sleep_disable();
  detachInterrupt(0); 
  delay(WAKEUP_DELAY);

  ADCSRA=adcsra_save;
  
  setup();
  tlacitka.keyChange=0;
}



void setup(void) {

  if ( EEPROM.read(1) == 111 )         // pokud je naplnena pamet, precti hodnoty, jinak vypln defaulty
  {
    diffSet = ((double)EEPROM.read(_DIFF)) / 100;
    screenRotation = EEPROM.read(_ROTATION);
    backlightSet = EEPROM.read(_BCKLIGHT);
    keyLeft = EEPROM.read(_KEYLEFT);
    keyRight = EEPROM.read(_KEYRIGHT);
    
    cal1 = EEPROM.read(_CELL1_L) + (EEPROM.read(_CELL1_H)<<8);
    cal2 = EEPROM.read(_CELL2_L) + (EEPROM.read(_CELL2_H)<<8);
    cal3 = EEPROM.read(_CELL3_L) + (EEPROM.read(_CELL3_H)<<8);

    stackTime = EEPROM.read(_STACK_L) + (EEPROM.read(_STACK_H)<<8);
    stackTimeMax = EEPROM.read(_STACKMAX_L) + (EEPROM.read(_STACKMAX_H)<<8);

    HUDMode = EEPROM.read(_HUDMODE);
  }
  else
  {
    diffSet = 0.1;
    screenRotation = 1; // left hand vr3   - 1 & 3
    backlightSet = 2;
    keyLeft = 3;
    keyRight = 2;
    EEPROM.write(_DIFF, (uint8_t)(diffSet * 100)); 
    EEPROM.write(_ROTATION,screenRotation);
    EEPROM.write(_BCKLIGHT,backlightSet);
    EEPROM.write(_KEYLEFT,keyLeft);
    EEPROM.write(_KEYRIGHT,keyRight);
    EEPROM.write(1,111);

  }

background = BLACK;
font = WHITE;
fontGreen = GREEN;
fontRed = RED;
fontYellow = YELLOW;

  pinMode(hud1,OUTPUT);
  pinMode(hud2,OUTPUT);

  pinMode(displayLight,OUTPUT);
  pinMode(displayPwr,OUTPUT);
  pinMode(sol,OUTPUT);
//  pinMode(bzuk,OUTPUT);

  pinMode(sol1, OUTPUT);
  pinMode(sol2, OUTPUT);
  
  pinMode(keyRight,INPUT);
  pinMode(keyLeft,INPUT);
  pinMode(batteryADC,INPUT);
  pinMode(waterContact,INPUT);
  

  digitalWrite(displayPwr,HIGH); 

  tft.begin();


  tft.setRotation(screenRotation);
  
  tft.fillScreen(background);
  digitalWrite(displayLight,HIGH); 


  delay(100);
  HUD(_OFF);
  delay(100);
  HUD(_RED);
  delay(100);
  HUD(_GREEN);
  delay(100);
  HUD(_OFF);
  delay(100);


  digitalWrite(sol,LOW); 

  digitalWrite(sol,HIGH);
           tft.fillRect(200, 215, 24, 24, BLUE);
  delay(50); 
  digitalWrite(sol,LOW); 
           tft.fillRect(200, 215, 24, 24, background);
  delay(50); 
  digitalWrite(sol,HIGH);
           tft.fillRect(200, 215, 24, 24, BLUE);
  delay(50); 
  digitalWrite(sol,LOW); 
           tft.fillRect(200, 215, 24, 24, background);
  delay(50); 
  digitalWrite(sol,HIGH);
           tft.fillRect(200, 215, 24, 24, BLUE);
  delay(50); 
  digitalWrite(sol,LOW); 
           tft.fillRect(200, 215, 24, 24, background);
  delay(50); 
  digitalWrite(sol,HIGH);
           tft.fillRect(200, 215, 24, 24, BLUE);
  delay(50); 
  digitalWrite(sol,LOW); 
           tft.fillRect(200, 215, 24, 24, background);

  tft.fillScreen(background);

  if (demo==0)
  {
    ads.setGain(GAIN_TWOTHIRDS);
    ads.begin();                 // initialize a ADS1115 chip
  }
  
  TIMSK2 &= ~(1<<TOIE2);
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
  TCCR2B &= ~(1<<WGM22);
  ASSR &= ~(1<<AS2);
  TIMSK2 &= ~(1<<OCIE2A);
  TCCR2B |= (1<<CS22) | (1<<CS20); // Set bits
  TCCR2B &= ~(1<<CS21);             // Clear bit
  tcnt2 = 131; 
  TCNT2 = tcnt2;
  TIMSK2 |= (1<<TOIE2);

  tlacitka.keyUp = 0;
  tlacitka.keyDown = 0;
  tlacitka.keyChange = 0;
  button = 0;
  dispRewrite = 1;
  solenoid=1;

  analogReference(DEFAULT);
  adc0 = analogRead(batteryADC);
  // 2.56 / 1023 * 2
  batteryVoltage = ((double)adc0 * (3.3 / 1023)) * 2;

//  batteryVoltage = ((double)adc0 * 0.0001875);

  blink.processing=0; 
  drawStatic();
  firstRun =  0;
  resetCount = 4;
}




void loop() {

  if (tlacitka.keyUp){
     button=1;
     tlacitka.keyUp=0;
     count=0;
     timerMenu=TIMERMENU;
   }
   else if (tlacitka.keyDown){    
     button=2;
     tlacitka.keyDown=0;
     count=0;
     timerMenu=TIMERMENU;
   }
   else {
     button=0;
   }

  if (count < sensitivity) 
    count++ ;
  else
  {
    count=0;
    tlacitka.keyChange=0;
  }

  counterSolenoid++;
  if (counterSolenoid > 120)
    counterSolenoid=0;

       
  counter++;

    
  //tft.drawString2(0, 85, font, 1, "counter=%d", counter);

    if (timerMenu > 1)
      timerMenu--;
    else if (timerMenu == 1)
    {
      tft.fillRect(0,158,320,50,background);
      timerMenu--;
      menu=0;
      dispRewrite = 1;
    }      
    else
    {
      menu=0;
    } 

      if (demo==0)
      {
        adc0 = ads.readADC_SingleEnded(0);  // battery
        adc1 = ads.readADC_SingleEnded(1);  // cell 1
        adc2 = ads.readADC_SingleEnded(2);  // cell 2
        adc3 = ads.readADC_SingleEnded(3);  // cell 3
      }
      else
      {
        adc1 = test[test_count]+20;
        adc2 = test[test_count+1]+20;
        adc3 = test[test_count+2]+20;
      }

     
      batteryVoltage = ((double)adc0 * 0.0001875);

      batteryVoltage = (batteryVoltage + batteryPrev)/2;
      batteryPrev=batteryVoltage;
      waterSwitch = analogRead(waterContact);

    
    if (adc1 < 1) adc1=1;
    if (adc2 < 1) adc2=1;
    if (adc3 < 1) adc3=1;

    c1 = (((double)(adc1)-OFFSET) / ((double)(cal1)));
    c2 = (((double)(adc2)-OFFSET) / ((double)(cal2)));
    c3 = (((double)(adc3)-OFFSET) / ((double)(cal3)));

    ppo0 = c1*100;
    ppo1 = c2*100;
    ppo2 = c3*100;
    
    if (ppo0 < 1) ppo0=1;
    if (ppo1 < 1) ppo1=1;
    if (ppo2 < 1) ppo2=1;

    cells.ppo[1]=ppo0;
    cells.ppo[2]=ppo1;
    cells.ppo[3]=ppo2;


    if (HUDMode == 0)
    {

        if (((counter > 0) && (counter <= 8)) || ((counter > 20) && (counter <= 28)) || ((counter > 40) && (counter <= 48)) || ((counter > 60) && (counter <= 68)))
        {
          if ((vote == 000) || (abs_XY (avg,setpoint) >= (diffSet*100)))
            HUD(_RED);
        }

        if (((counter > 8) && (counter <= 20)) || ((counter > 28) && (counter <= 40)) || ((counter > 48) && (counter <= 60)) || ((counter > 68) && (counter <= 80)))
          HUD(_OFF);
        
        
        if ((counter > 80) && (counter <=88))  //rozsvit 80
        {
          if ((vote == 111) && (abs_XY (avg,setpoint) < (diffSet*100)))  HUD(_GREEN);  // misto diffSet bylo puvodne 18
          else if (((vote == 110) || (vote == 101) || (vote == 011)) ) HUD(_YELLOW);
          else HUD(_RED);
        } 
        if (counter > 88)  //shasni a nuluj 90
        {
          HUD(_OFF);
        }

        if (counter == 100)
          counter=0;
          
    }


// fill-up string for hud / naplneni retezce vyblikavani ppo2
    if ( blink.processing == 0)
    {
       blink.string[1]=0;
       blink.string[2]=0;
       blink.string[3]=0;
       blink.string[4]=0;
       blink.string[5]=0;
       blink.string[6]=0;
       blink.string[7]=0;
       blink.string[8]=0;
       blink.string[9]=0;
       blink.string[10]=0;
       blink.string[11]=0;
       blink.string[12]=0;
       blink.position=12;
       
       uint8_t mezera,svetlo,diffblink;

       for (uint8_t cidlo = 1; cidlo < 4; cidlo++)
       {
          blink.string[++blink.position] = 0;
          blink.string[++blink.position] = 0;
          blink.string[++blink.position] = 0;
          if (cells.ppo[cidlo] < 100)
          {
             if (cells.ppo[cidlo] < 50)
             {
                diffblink = 59 - cells.ppo[cidlo];
                mezera = 1;
                svetlo = 2;
             }
             else
             {
                diffblink = 109 - cells.ppo[cidlo];
                mezera = 0;
                svetlo = 2;
             }
          }
          else
          {
             if (cells.ppo[cidlo] < 110)
               svetlo = 3;
             else
               svetlo = 1;
             diffblink = cells.ppo[cidlo] - 100;
             mezera = 0;

          }
          if (diffblink < 10)
            diffblink = 10;
          diffblink = diffblink / 10;

          for (uint8_t bl = 1; bl <= diffblink; bl++)
          {
             blink.string[++blink.position] = mezera;
             blink.string[++blink.position] = mezera;
             blink.string[++blink.position] = mezera;
             if (svetlo==3)
             {
               blink.string[++blink.position] = 1;
               blink.string[++blink.position] = 2;
             }
             else             
               blink.string[++blink.position] = svetlo;
          }
       }
       blink.length = blink.position;
       blink.position = 0;
       blink.processing = 1;
    }
    

    vote = vote_ABC(ppo0, ppo1, ppo2);


    if (vote == 111)
      avg = (ppo0+ppo1+ppo2)/3;
    else if (vote == 110)
      avg = (ppo0+ppo1)/2;
    else if (vote == 101)
      avg = (ppo0+ppo2)/2;
    else if (vote == 11)
      avg = (ppo1+ppo2)/2;
    else 
      avg = 0;

    if ((avg == 1) || (avg == 0))
      vote=0;


// solenoid time adjustment / pripocteme empiricky zjistenou hdonotu  ... 
          switch (setpoint)
          {
             case 40:               spPref = 50;                 break;
             case 70:               spPref = 80;                 break;
             case 100:              spPref = 110;                break;
             case 120:              spPref = 150;                break; // bylo 150  (petak 130)
             case 140:              spPref = 170;                break; // bylo 170  (petak 150)
             case 160:              spPref = 185;                break; // bylo 190, ale Freedom rve prekroceni 1.6 (petak 165)
             default:                       break;
          }



// main condition for solenoid / hlavni podminka pro spusteni solenoidu
       if ((avg < setpoint) && (avg > 1))
       {
         solPref=((uint8_t)((((float)(abs_XY(spPref,avg)))/160)*9.9)); 
         if (solPref == 0) // pokud je priznak spusteni solenoidu mensi jak 1, ale avg < sp, nastavime rucne priznak na 1, aby sel dostrik pri avg tesne pod sp ...
           solPref = 1;
         if (solPref > 9)
           solPref = 9;

         if ((counterSolenoid < (solPref * 12)) && (solenoid > 0) && (avg != 0))
         {
           digitalWrite(sol,HIGH); 

           solSwitch=1;
           tft.fillRect(200, 215, 24, 24, BLUE);
         }
         else
         {
           digitalWrite(sol,LOW); 
           solSwitch=0;
           tft.fillRect(200, 215, 24, 24, background);
         }

         if ((solenoid == 0) && (avg < 20) && (avg != 0))     // zapne solenoid pri poklesu ppO2 pod 0.2
         {
           if (counterSolenoid < 5)
           {
             digitalWrite(sol,HIGH); 
             solSwitch=1;
             tft.fillRect(200, 215, 24, 24, BLUE);
           }
           else
           {
             digitalWrite(sol,LOW); 
             solSwitch=0;
             tft.fillRect(200, 215, 24, 24, background);
           }
         }
       }
       else
       {
         digitalWrite(sol,LOW); 
         solSwitch=0;
         tft.fillRect(200, 215, 24, 24, background);
       }


       if (avg > setpoint)                             // switch-off solenoid when avg => sp / vypne solenoid, kdyz je avg => sp
       {
         digitalWrite(sol,LOW); 
         solSwitch=0;
         tft.fillRect(200, 215, 24, 24, background);

         
       }

// udaje zobrazujeme i v menu
       if ((counter%50) == 0)
       {
         drawCells(ppo0, ppo1, ppo2, vote);
//         if (((uint16_t)(batteryVoltage*10)) != ((uint16_t)(batteryPrev*10)))
           drawBAT(batteryVoltage);

// water contact - currently disabled / zrusime hvezdicku pro nezapojene kontakty       
//           if (waterSwitch < 400)  // symbol ponoru 
//             tft.drawString(150, 40, "*", tft.color565(112,148,255),1);
//           else
//             tft.drawString(150, 40, "*", background,1);
             
         if (menu == 1)
           drawMVolts(adc1, adc2, adc3);




         if ((timerINTprev != timerINTmin) || (firstRun == 0))
         {
           if ((waterSwitch < 400) && (stackTime>0))  // odecita je pri spojenych vodnich kontaktech 
             stackTime--;
           firstRun = 1;
           EEPROM.write(_STACK_L,stackTime);
           EEPROM.write(_STACK_H,stackTime>>8);

           timerINTprev = timerINTmin;
           if (stackTime >60)
             tft.drawString2H(112, 12, GREEN, 3, " %3d",stackTime);
           else
             tft.drawString2H(112, 12, RED, 3, " %3d",stackTime);
         }
       }

          if ((counter == 0))
          {
            if (demo==1) 
            {
              if (test_count<45)
                test_count=test_count + 3;
              else
                test_count=0;
            }
          }


    switch (menu) 
    {
      case 0:
          
          if (dispRewrite == 1)
          {
            tft.drawString(0, 112, "MENU", font,1);
            tft.drawString(141, 112, " mV", font,1);
            drawSP(setpoint);
            dispRewrite = 0;
          }
          
        if (button == 1)
        {
          menu = 100;
          dispRewrite = 1;
        }
        if (button == 2)
        {
          menu = 1;
          dispRewrite = 1;
          resetCount = 20;
          
        }
        break;
        
      case 1:

        if (button == 1)
        {
          menu = 0;
          timerMenu = 1;
          cleanMVolts();
          dispRewrite = 1;
        }
        if (button == 2)
          {
            resetCount--;
            if (resetCount == 0)
            {
              menu = 0;
              tlacitka.keyChange=0;
              digitalWrite(displayLight,LOW);
              digitalWrite(displayPwr,LOW); 
              digitalWrite(hud1,LOW); 
              digitalWrite(hud2,LOW); 
              digitalWrite(sol,LOW); 
              digitalWrite(cs,LOW);
              digitalWrite(dc,LOW);
              digitalWrite(rst,LOW);  
              sleepNow();  
            }
          }


        if (dispRewrite == 1)
        {
          tft.drawString(0, 112, "EXIT", font,1);
          tft.drawString(141, 112, "   ", font,1);
          dispRewrite = 0;
        }

        
        break;
      case 100:
        if (dispRewrite == 1)
        {
          tft.drawString(0, 90, "SETPOINT     ", font,2);
          tft.drawString(0, 112, "NEXT", font,1);
          tft.drawString(141, 112, "SEL", font,1);
          dispRewrite = 0;
        }

        if (button == 1)
        {
          menu = 200;

          dispRewrite = 1;
        }
        else if (button == 2)
        {
          setpoint_tmp = setpoint;
          menu = 110;

          dispRewrite = 1;
        }
        break;

      case 110:
        if (dispRewrite == 1)
        {
          tft.drawString2(0, 90, font,2, " -> setpoint");         //
          tft.drawString(0, 112, "SAVE", font,1);
          tft.drawString(141, 112, "<+>", font,1);
          dispRewrite = 0;
        }

        if (button == 1)
        {
          menu = 999;
          setpoint = setpoint_tmp;
          drawSP(setpoint);
          timerMenu=1;
          dispRewrite = 1;
        }
        else if (button == 2)
        {
          switch (setpoint_tmp)
          {
             case 40:
               setpoint_tmp = 70;
               break;
             case 70:
               setpoint_tmp = 100;
               break;
             case 100:
               setpoint_tmp = 120;
               break;
             case 120:
               setpoint_tmp = 140;
               break;
             case 140:
               setpoint_tmp = 160;
               break;
             case 160:
               setpoint_tmp = 40;
               break;
             default:
               break;
          }
          drawSP(setpoint_tmp);
          menu = 110;
          dispRewrite = 1;

          
        }
        break;

      case 200:
        solenoid_tmp=solenoid;
        if (dispRewrite == 1)
        {
          tft.drawString(0, 90, "SOLENOID     ", font,2);
          tft.drawString(0, 112, "NEXT", font,1);
          tft.drawString(141, 112, "SEL", font,1);
          dispRewrite = 0;
        }

        if (button == 1)
        {
          menu = 300;

          dispRewrite = 1;
        }
        else if (button == 2)
        {
          menu = 210;

          dispRewrite = 1;
        }
        break;

      case 210:
        if (dispRewrite == 1)
        {
          switch (solenoid_tmp)
          {
            case 0:
              tft.drawString(0, 90, "->SOL     OFF", font,2);
              break;
            case 1:
              tft.drawString(0, 90, "->SOL    AUTO", font,2);
              break;
            default: break;
          }
          tft.drawString(0, 112, "SAVE", font,1);
          tft.drawString(141, 112, "<+>", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 0;
          timerMenu=1;
          solenoid=solenoid_tmp;
          dispRewrite = 1;
          if (solenoid == 0)
            tft.drawString2H(41, 15, RED, 2, "SOLENOID");
          else
            tft.drawString2H(41, 15, RED, 2, "        ");
        }
        else if (button == 2)
        {
          menu = 210;

          switch (solenoid_tmp)
          {
             case 0 :
               solenoid_tmp=1;
               break;
             case 1 :
               solenoid_tmp=0;
               break;
             default: break;
          }
          dispRewrite = 1;
        }
        break;
        
      case 300:
        if (dispRewrite == 1)
        {
          tft.drawString(0, 90, "SETUP         ", font,2);
          tft.drawString(0, 112, "NEXT", font,1);
          tft.drawString(141, 112, "SEL", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 400;
          dispRewrite = 1;
        }
        else if (button == 2)
        {
          menu = 310;

          dispRewrite = 1;
        }
        break;


      case 310:                                //
        if (dispRewrite == 1)
        {
          tft.drawString(0, 90, "- BACKLIGHT  ", font,2);
          tft.drawString(0, 112, "NEXT", font,1);
          tft.drawString(141, 112, "SEL", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 330;

          dispRewrite = 1;
        }
        else if (button == 2)
        {
          backlightSet_tmp = backlightSet;
          menu = 311;

          dispRewrite = 1;
        }
        break;

      case 311:
        if (dispRewrite == 1)
        {
          tft.drawString2(0, 90, font, 2, " - BckLight %1d", 4-(backlightSet/4));       //
          tft.drawString(0, 112, "SAVE", font,1);
          tft.drawString(141, 112, "(-)", font,1);
          dispRewrite = 0;
        }

        if (button == 1)
        {
          menu = 0;
          timerMenu=1;
          EEPROM.write(_BCKLIGHT,backlightSet);
          dispRewrite = 1;
        }
        else if (button == 2)
        {
          backlightSet = backlightSet + 4;
          if (backlightSet > 16)
            backlightSet = 0;
          menu = 311;

          dispRewrite = 1;
        }
        break;




      case 320:                                //
        if (dispRewrite == 1)
        {
          tft.drawString(0, 90, "- HUD DIFF   ", font,2);
          tft.drawString(0, 112, "NEXT", font,1);
          tft.drawString(141, 112, "SEL", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 340;

          dispRewrite = 1;
        }
        else if (button == 2)
        {
          diffSet_tmp = diffSet;
          menu = 321;

          dispRewrite = 1;
        }
        break;

      case 321:
        if (dispRewrite == 1)
        {
          tft.drawString2(0, 90, font, 2, " - Diff 0.%02d ", (uint8_t)(diffSet_tmp * 100));       //
          tft.drawString(0, 112, "SAVE", font,1);
          tft.drawString(141, 112, "(+)", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 0;
          diffSet = diffSet_tmp;
          timerMenu=1;
          EEPROM.write(_DIFF, (uint8_t)(diffSet * 100)); 
          dispRewrite = 1;
        }
        else if (button == 2)
        {
          diffSet_tmp = diffSet_tmp + 0.05;
          if (diffSet_tmp > 0.3)
            diffSet_tmp = 0.05;
          menu = 321;

          dispRewrite = 1;
        }
        break;


      case 330:                              //
        if (dispRewrite == 1)
        {
          tft.drawString(0, 90, "- ROTATION   ", font,2);
          tft.drawString(0, 112, "NEXT", font,1);
          tft.drawString(141, 112, "SEL", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 320;

          dispRewrite = 1;
        }

        else if (button == 2)
        {
          menu = 331;
          screenRotation_tmp = screenRotation;

          dispRewrite = 1;
        }
        break;

      case 331:
        if (dispRewrite == 1)
        {
          if (screenRotation_tmp == 1)
            tft.drawString2(0, 90, font, 2, " - Rotation L");       //
          else 
            tft.drawString2(0, 90, font, 2, " - Rotation R");       //
          tft.drawString(0, 112, "SAVE", font,1);
          tft.drawString(141, 112, "(+)", font,1);
          dispRewrite = 0;
        }

        if (button == 1)
        {
          menu = 999;
          screenRotation = screenRotation_tmp;

          tft.setRotation(screenRotation);
          tft.fillScreen(background);
          drawStatic();
          
          if (screenRotation == 1)
          {
            keyLeft = 3;
            keyRight = 2;
//            HUD(RED);
          }
          else
          {
            keyLeft = 2;
            keyRight = 3;
//            HUD(GREEN);
          }
          
          EEPROM.write(_ROTATION,screenRotation);
          EEPROM.write(_KEYLEFT,keyLeft);
          EEPROM.write(_KEYRIGHT,keyRight);
         
          timerMenu=1;
          dispRewrite = 1;
        }
        else if (button == 2)
        {
       
          if (screenRotation_tmp == 1)
            screenRotation_tmp = 3;
          else
            screenRotation_tmp = 1;
          menu = 331;

          dispRewrite = 1;
        }
        break;


      case 340:
        if (dispRewrite == 1)
        {
          tft.drawString(0, 90, "- CALIBRATION", font,2);
          tft.drawString(0, 112, "NEXT", font,1);
          tft.drawString(141, 112, "SEL", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 350;
          dispRewrite = 1;
        }
        else if (button == 2)
        {
          menu = 341;

          dispRewrite = 1;
        }
        break;


      case 341:
        if (dispRewrite == 1)
        {
          drawCells(ppo0, ppo1, ppo2, vote);
          drawMVolts(adc1, adc2, adc3);
          tft.drawString(0, 90, " - o2 Calibration", font,2);       //
          tft.drawString(0, 112, "SAVE", font,1);
          tft.drawString(141, 112, "ESC", font,1);

          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 0;
          timerMenu=1;
          cal1 = (uint16_t)(((double)(adc1 - OFFSET))/OXY);
          cal2 = (uint16_t)(((double)(adc2 - OFFSET))/OXY);
          cal3 = (uint16_t)(((double)(adc3 - OFFSET))/OXY);

// puvode se ukladala hodnota z adcX
          EEPROM.write(_CELL1_L,cal1);
          EEPROM.write(_CELL1_H,cal1>>8);
          EEPROM.write(_CELL2_L,cal2);
          EEPROM.write(_CELL2_H,cal2>>8);
          EEPROM.write(_CELL3_L,cal3);
          EEPROM.write(_CELL3_H,cal3>>8);
          dispRewrite = 1;
        }
        else if (button == 2)
        {
          menu = 340;

          dispRewrite = 1;
        }
        break;



      case 350:
        if (dispRewrite == 1)
        {
          tft.drawString(0, 90, "- STACKTIME     ", font,2);
          tft.drawString(0, 112, "NEXT", font,1);
          tft.drawString(141, 112, "SEL", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 360;
          dispRewrite = 1;
        }
        else if (button == 2)
        {
          menu = 351;
          dispRewrite = 1;
        }
        break;


      case 351:
        if (dispRewrite == 1)
        {
          tft.drawString(0, 90, " - stack RESET  ", font,2);       //
          tft.drawString(0, 112, "NEXT", font,1);
          tft.drawString(141, 112, "SEL", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 352;

          dispRewrite = 1;
        }
        else if (button == 2)
        {
          menu = 353;
          dispRewrite = 1;
        }
        break;

      case 352:
        if (dispRewrite == 1)
        {
          tft.drawString(0, 90, " - stack SET  ", font,2);       //
          tft.drawString(0, 112, "EXIT", font,1);
          tft.drawString(141, 112, "SEL", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 350;

          dispRewrite = 1;
        }
        else if (button == 2)
        {
          menu = 354;
          dispRewrite = 1;
        }
        break;

//    EEPROM.write(_STACKMAX_L,stackTimeMax);
//    EEPROM.write(_STACKMAX_H,stackTimeMax>>8);

      case 353:
        if (dispRewrite == 1)
        {
          tft.drawString2(0, 90,font,2, " - reset to %d?", stackTimeMax );       //
          tft.drawString(0, 112, "YES ", font,1);
          tft.drawString(141, 112, "ESC", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 0;
          timerMenu=1;
          EEPROM.write(_STACK_L,stackTimeMax);
          EEPROM.write(_STACK_H,stackTimeMax>>8);
          stackTime = stackTimeMax;
          dispRewrite = 1;
          firstRun = 0;
        }
        else if (button == 2)
        {
          menu = 350;
          dispRewrite = 1;
        }
        break;

      case 354:
        if (dispRewrite == 1)
        {
          tft.drawString2(0, 90, font,2, " - stacktime %d ", stackTimeMax);       //
          tft.drawString(0, 112, "SAVE", font,1);
          tft.drawString(141, 112, "<+>", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 351;
          EEPROM.write(_STACKMAX_L,stackTimeMax);
          EEPROM.write(_STACKMAX_H,stackTimeMax>>8);
          dispRewrite = 1;
        }
        else if (button == 2)
        {
          menu = 354;
          stackTimeMax = stackTimeMax + 60;
          if (stackTimeMax > 720)
            stackTimeMax = 60;
          dispRewrite = 1;
        }
        break;

      case 360:
        if (dispRewrite == 1)
        {
          tft.drawString(0, 90, "- HUD MODE      ", font,2);
          tft.drawString(0, 112, "EXIT", font,1);
          tft.drawString(141, 112, "SEL", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 300;
          dispRewrite = 1;
        }
        else if (button == 2)
        {
          menu = 361;
          dispRewrite = 1;
        }
        break;


      case 361:
        if (dispRewrite == 1)
        {
          if (HUDMode == 1)
            tft.drawString(0, 90, " - mode PPO2    ", font,2);       //
          else
            tft.drawString(0, 90, " - mode CLASSIC ", font,2);       //
          
          tft.drawString(0, 112, "SAVE", font,1);
          tft.drawString(141, 112, "<+>", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 0;
          timerMenu=1;
          EEPROM.write(_HUDMODE,HUDMode);
          dispRewrite = 1;
        }
        else if (button == 2)
        {
          menu = 361;
          if (HUDMode == 1)
            HUDMode = 0;
          else
            HUDMode = 1;
          dispRewrite = 1;
        }
        break;


        
      case 400:
        if (dispRewrite == 1)
        {
          tft.drawString(0, 90, "SHUTDOWN     ", font,2);
          tft.drawString(0, 112, "EXIT", font,1);
          tft.drawString(141, 112, "SEL", font,1);
          dispRewrite = 0;
        }
        
        if (button == 1)
        {
          menu = 0;
          tft.fillRect(0,168,320,33,background);

          dispRewrite = 1;
        }
        else if (button == 2)
        {
          menu = 410;
        }
        break;

      case 410:
        menu = 0;
        tlacitka.keyChange=0;
        tft.fillScreen(background);
        tft.drawString(0, 60, " -> sleep ..", font, 2);
        delay(2000);
        digitalWrite(displayLight,LOW);
        digitalWrite(displayPwr,LOW); 
        digitalWrite(hud1,LOW); 
        digitalWrite(hud2,LOW); 
        digitalWrite(sol,LOW); 
        digitalWrite(cs,LOW);
        digitalWrite(dc,LOW);
        digitalWrite(rst,LOW);  
        sleepNow();  

        break;
      case 999:
        drawSP(setpoint);
        drawStatic();
        menu=0;
        break;
        
        default :
        break;
    }
    button = 0;
}


void drawStatic() {
  drawSP(setpoint);
  tft.drawString(1, 0, "SETPOINT", WHITE,1);
  tft.drawString(128, 0, "STACK", WHITE,1);
}


void drawSP(uint8_t sp) {
  if ((sp == 40) || (sp == 160))
    {
    if ((sp/100) >= 1)
      tft.drawString2H(2, 12, YELLOW, 3, "%1d", sp/100);
    else
      tft.fillRect(4, 18, 20, 48, background);
    tft.fillRect(28, 60, 6, 6, YELLOW);
    tft.drawString2H(20, 12, YELLOW, 3, "%1d", (sp/10)%10);
  }
  else {
    if ((sp/100) >= 1)
      tft.drawString2H(2, 12, font, 3, "%1d", sp/100);
    else
      tft.fillRect(4, 18, 20, 48, background);
    tft.fillRect(28, 60, 6, 6, font);
    tft.drawString2H(20, 12, font, 3, "%1d", (sp/10)%10);
  }
}

void drawBAT(double battery) {
  
  if (battery > 3.7) 
    tft.drawString2H(70, 105, font, 1, "%1d.%1dV", (uint16_t)battery, (uint16_t)(battery*10)-((uint16_t)battery)*10);
  else 
    tft.drawString2H(70, 105, RED, 1, "%1d.%1dV", (uint16_t)battery, (uint16_t)(battery*10)-((uint16_t)battery)*10);
}
 

void drawCells(uint8_t a, uint8_t b, uint8_t c, uint8_t vote) {
#define t1x 32
#define t1y 134
#define t2x 140
#define t2y 134
#define t3x 258
#define t3y 134
    
  if ((a<20) || (a>160)) {
    if (a < 100) {
      tft.fillRect(8, 98, 20, 45, background);
      tft.fillRect(t1x, t1y, 6, 6, (vote/100)?YELLOW:RED);
      tft.drawString2H(22, 50, (vote/100)?YELLOW:RED,3,"%02d",a%100);
    }
    else {
      tft.drawString2HH(3, 50, (vote/100)?YELLOW:RED,3,"%1c",s0[a/100]);
      tft.fillRect(t1x, t1y, 6, 6, (vote/100)?YELLOW:RED);
      tft.drawString2H(22, 50, (vote/100)?YELLOW:RED,3,"%02d",a%100);
    }
  }
  else {
    if (a < 100) {
      tft.fillRect(8, 98, 20, 45, background);
      tft.fillRect(t1x, t1y, 6, 6, (vote/100)?GREEN:RED);
      tft.drawString2H(22, 50, (vote/100)?GREEN:RED,3,"%02d",a%100);
    }
    else {
      tft.drawString2HH(3, 50, (vote/100)?GREEN:RED,3,"%1c",s0[a/100]);
      tft.fillRect(t1x, t1y, 6, 6, (vote/100)?GREEN:RED);
      tft.drawString2H(22, 50, (vote/100)?GREEN:RED,3,"%02d",a%100);
    }

  }


  if ((b<20) || (b>160)) {
    if (b < 100) {
      tft.fillRect(114, 98, 20, 45, background);
      tft.fillRect(t2x, t2y, 6, 6, ((vote%100)/10)?YELLOW:RED);
      tft.drawString2H(76, 50, ((vote%100)/10)?YELLOW:RED,3,"%02d",b%100);
    }
    else {
      tft.drawString2H(55, 50, ((vote%100)/10)?YELLOW:RED,3,"%1c",s0[b/100]);
      tft.fillRect(t2x, t2y, 6, 6, ((vote%100)/10)?YELLOW:RED);
      tft.drawString2H(76, 50, ((vote%100)/10)?YELLOW:RED,3,"%02d",b%100);
    }
  }
  else {
    if (b < 100) {
      tft.fillRect(114, 98, 20, 45, background);
      tft.fillRect(t2x, t2y, 6, 6, ((vote%100)/10)?GREEN:RED);
      tft.drawString2H(76, 50, ((vote%100)/10)?GREEN:RED,3,"%02d",b%100);
    }
    else {
      tft.drawString2H(55, 50, ((vote%100)/10)?GREEN:RED,3,"%1c",s0[b/100]);
      tft.fillRect(t2x, t2y, 6, 6, ((vote%100)/10)?GREEN:RED);
      tft.drawString2H(76, 50, ((vote%100)/10)?GREEN:RED,3,"%02d",b%100);
    }
  }

  if ((c<20) || (c>160)) {
    if (c < 100) {
      tft.fillRect(234, 98, 20, 45, background);
      tft.fillRect(t3x, t3y, 6, 6, (vote%10)?YELLOW:RED);
      tft.drawString2H(135, 50, (vote%10)?YELLOW:RED,3,"%02d",c%100);
    }
    else {
      tft.drawString2H(117, 50, (vote%10)?YELLOW:RED,3,"%1c",s0[c/100]);
      tft.fillRect(t3x, t3y, 6, 6, (vote%10)?YELLOW:RED);
      tft.drawString2H(135, 50, (vote%10)?YELLOW:RED,3,"%02d",c%100);
    }
  }
  else {
    if (c < 100) {
      tft.fillRect(234, 98, 20, 45, background);
      tft.fillRect(t3x, t3y, 6, 6, (vote%10)?GREEN:RED);
      tft.drawString2H(135, 50, (vote%10)?GREEN:RED,3,"%02d",c%100);
    }
    else {
      tft.drawString2H(117, 50, (vote%10)?GREEN:RED,3,"%1c",s0[c/100]);
      tft.fillRect(t3x, t3y, 6, 6, (vote%10)?GREEN:RED);
      tft.drawString2H(135, 50, (vote%10)?GREEN:RED,3,"%02d",c%100);
    }
  }
}


void drawMVolts(uint16_t c1, uint16_t c2, uint16_t c3) {
  double mv1, mv2, mv3;
  mv1 = (double)adc1 * 0.1875; 
  mv2 = (double)adc2 * 0.1875; 
  mv3 = (double)adc3 * 0.1875; 
  tft.drawString2(0, 80, font,1,"% 3d.%01dmV",(uint16_t)mv1, (uint16_t)((mv1-(uint16_t)mv1)*10));
  tft.drawString2(55, 80, font,1,"% 3d.%01dmV",(uint16_t)mv2, (uint16_t)((mv2-(uint16_t)mv2)*10));
  tft.drawString2(110, 80, font,1,"% 3d.%01dmV",(uint16_t)mv3, (uint16_t)((mv3-(uint16_t)mv3)*10));
}

void cleanMVolts() {
  tft.fillRect(0, 160, 320, 20, background);
 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ridi HUD display a HUD stav na display  /   HUD display & HUD state driving
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HUD(uint8_t led)
{
  uint16_t yellow;
  yellow = 0xFFE0;

  if (led == _OFF)
  {
	digitalWrite(hud1,LOW);
	digitalWrite(hud2,LOW);
//        tft.drawString(150, 48, "*", background, 1);
        tft.fillRect(100, 215, 24, 24, background);
        
  }
  else 
  {
	if (led == _RED)
	{
          digitalWrite(hud1,LOW);
          digitalWrite(hud2,HIGH);
//          tft.drawString(150, 48, "*", RED, 1);
          tft.fillRect(100, 215, 24, 24, RED);

	}
        else if (led == _ORANGE)
	{
          digitalWrite(hud1,LOW);
          digitalWrite(hud2,HIGH);
//          tft.drawString(150, 48, "*", yellow, 1);
          tft.fillRect(100, 215, 24, 24, yellow);
	}
	else
	{
          digitalWrite(hud1,HIGH);
          digitalWrite(hud2,LOW);
//          tft.drawString(150, 48, "*", GREEN, 1);
          tft.fillRect(100, 215, 24, 24, GREEN);
	}
  }
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// vraci prumer z hodnot x a y   /   return avg. value from X,Y
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t avg_XY (uint16_t x, uint16_t y)
{
  return ((x+y)/2);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// vraci abs. hodnotu rozdilu dvou cisel  /  return abs. value from X,Y
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t abs_XY(uint16_t x, uint16_t y)
{
  if (x >= y)
    return x-y;
  else
    return y-x;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// vraci 3 cifry 'xyz', kde napr. 101 znamena 1. a 3. cidlo v poradku, atd. hodnoty a,b,c, jsou ppO2 hodnoty cidel
//   voting logic - return 3 cifers 'xyz', example: 101 means that 1.st cell and 3.rd cell are OK
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t vote_ABC(uint16_t a, uint16_t b, uint16_t c)
{
  uint16_t avgABC;   //bylo double
  uint16_t difAB, difBC, difAC;   //bylo double

 
  avgABC = avg_XY(a,b);
  avgABC = avg_XY(avgABC,c);

  if (a >= b)
    difAB = a - b;
  else
    difAB = b - a;

  if (a >= c)
    difAC = a - c;
  else     
    difAC = c - a;

  if (b >= c)
    difBC = b - c;
  else
    difBC = c - b;

//  tft.drawString2(1, 100, font, 1, "%d %d %d    ", (uint16_t)difAB,(uint16_t)difBC,(uint16_t)difAC);  

  if (difAB <= difAC)
  {
    if (difAB <= difBC)
	{
      // cidla A a B jsou nejbliz  /  A nad B are closer
      if (difAB <= MAX_DIFF)
	  {
        if (abs_XY(avg_XY(a,b),c) <= MAX_DIFF)
          return 111;
        else
          return 110;
	  }
      else
        return 0;  
    }    
    else
	{
      // cidla B a C jsou nejbliz  /  B nad C are closer
      if (difBC <= MAX_DIFF)
	  {
        if (abs_XY(avg_XY(b,c),a) <= MAX_DIFF)
          return 111;
        else
          return 11;
	  }
      else
        return 0;  
    }
  }
  else
  {
    if (difAC <= difBC)
	{
      // cidla A a C jsou nejbliz  /  A nad C are closer
      if (difAC <= MAX_DIFF)
	  {
        if (abs_XY(avg_XY(a,c),b) <= MAX_DIFF)
          return 111;
        else
          return 101;
	  }
      else
        return 0;  
	}
	else
	{
	// cidla B a C jsou nejbliz  /  B nad C are closer
      if (difBC <= MAX_DIFF)
	  {
        if (abs_XY(avg_XY(b,c),a) <= MAX_DIFF)
          return 111;
        else
          return 11;
	  }
	}
  }
  return 0;
}

