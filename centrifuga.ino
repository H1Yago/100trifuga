//**************************************************************************//
//Programa : Centrífuga - Eng Unificada AX -  UFABC							//
//Autor : Gabriel Sousa - 11070016											//
//Data inicio: 		10/11/2019												//
//Ultima Versão: 	10/11/2019												//	
//**************************************************************************//


//Carrega a bibliotecas
#include <RotaryEncoder.h>
#include <LiquidCrystal.h>
#include "data_types.h"


//define constantes
#define shortClick  10   //ms
#define longClick   200  //ms
#define MAX_TIME    999   //second
#define MAX_RPM     12500       //
#define STEP_RPM    MAX_RPM/(200+1) //

//Define dos pinos
RotaryEncoder encoder(2, 3);
#define swEnc 	4
#define lcdRS 	7
#define lcdEN 	6
#define lcdD4	8
#define lcdD5 	9
#define lcdD6 	10
#define lcdD7 	11
//                RS  EN  D4 D5 D6 D7
LiquidCrystal lcd(lcdRS, lcdEN, lcdD4, lcdD5, lcdD6, lcdD7);
#define buzz	24
#define	oMotor	13		//1
#define iTach	20


//Variavel globais
int valor = 0,newPos = 0,i=0,rpmAvg=0;
static int pos = 0;
unsigned int  timer1_counter = 0,
			  swCount=0,revCount=0;
int timeOut=0,tBeep=0;
unsigned int setRPM=0,setTime=0,tOper;
String lcd1,lcd2;
char data[16];
unsigned char maskPWM=0;

long currtime=0,idletime=0,prevtime=0,
	realRPM=0,calcRPM=0,showRPM=0,oPWM=0;

#define init 1
#define main 2
#define continuo 3
#define timer 4
#define on 5
#define test 6

signed char state,selPos=0;


//variables pid
//float kp=40,ki=5,kd=20;
//PID constants
double kp = 0.255;
double ki = 0.01;
double kd = 0.01;
 
unsigned long currentTime, previousTime, elapsedTime;
double error;
double lastError;
int input, output, setPoint;
double cumError, rateError;
 
//define flags
uint8_flags_t fgl1,fgl2; 

#define fswSClick    	fgl1.b0 
#define fswLClick    	fgl1.b1 
#define fencUp       	fgl1.b2 
#define fencDw       	fgl1.b3 
#define ftick        	fgl1.b4 
#define fCont_Timer  	fgl1.b5 	//0 - continuos 1-timer
#define frefreshLCD  	fgl1.b6
#define fEdit        	fgl1.b7

#define f100ms      	fgl2.b0		// atualizar o rpm no momento do modo on



void setup()
{
	//define modo dos pinos
	pinMode(swEnc, INPUT_PULLUP);
	pinMode(buzz,OUTPUT);
	pinMode(oMotor,OUTPUT);

	// initialize timer1 
	noInterrupts();           // disable all interrupts
	TCCR1A = 0;
	TCCR1B = 0;

	timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz

	TCNT1 = timer1_counter;   // preload timer
	TCCR1B |= (1 << CS12);    // 256 prescaler 
	TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

	
	pinMode(iTach, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(iTach), RPMCount, RISING);
	
	lcd.begin(16, 2);

	Serial.begin(115200);
	Serial.println("Gire o encoder....");


	lcd1="  UFABC - 2019  ";
	lcd2=" Eng. Unif. AX  ";
	timeOut = 350;
	tBeep = 20;
	frefreshLCD=1;
	ftick = 0;
	f100ms=0;
	state = init;//test;
	maskPWM=oPWM=30;
	
	
	interrupts();             // enable all interrupts
}


void loop() //superloop
{
	if(ftick) //10ms
	{
		ftick = 0;

		if(frefreshLCD)
		{
			frefreshLCD=0;
			lcd.setCursor(0, 0);
			lcd.print(lcd1);
			lcd.setCursor(0, 1);
			lcd.print(lcd2);

			if(state>1 && state <5) //estados de edição
			{
				if(selPos<2)lcd.setCursor(0, selPos); 
				else        lcd.setCursor(13, 1);  //posicao ON

				if(fEdit) lcd.write(0x3E);
				else      lcd.write(0x7E);
			}
		}


		switch(state)//maquina de estados
		{
			case init:
			{
				fswLClick=fswSClick=fencUp=fencDw=0;  //desabilitadas
				if(timeOut--==0)
				{
					state=main;
					lcd1=" Modo Continuo  ";
					lcd2=" Modo Timer     ";
					frefreshLCD=1;

					fEdit=0;
				}
				break;
			}

			case main:
			{
			fswLClick=0;  //desabilitadas
			if(fswSClick)//click curto na chave
			{
			  fswSClick=0;
			  if(selPos)  //timer
			  {
				state=timer;
				fCont_Timer=1;
				setTime=100;
				setRPM=1000;
				lcd1=" t=   s          ";
				lcd1.setCharAt(3,(setTime/100)%10 + 0x30);
				lcd1.setCharAt(4,(setTime/10)%10 + 0x30);
				lcd1.setCharAt(5,(setTime)%10 + 0x30);

				lcd2=" RPM=         ON";
				lcd2.setCharAt(5,(setRPM/10000)%10 + 0x30);
				lcd2.setCharAt(6,(setRPM/1000)%10 + 0x30);
				lcd2.setCharAt(7,(setRPM/100)%10 + 0x30);
				lcd2.setCharAt(8,(setRPM/10)%10 + 0x30);
				lcd2.setCharAt(9,(setRPM)%10 + 0x30);
				frefreshLCD=1;
				fEdit=0;
			  }
			  else
			  {                
				state=continuo;
				selPos=1;
				fCont_Timer=0;
				setRPM=1000;
				lcd1="                ";
				lcd2=" RPM=         ON";
				lcd2.setCharAt(5,(setRPM/10000)%10 + 0x30);
				lcd2.setCharAt(6,(setRPM/1000)%10 + 0x30);
				lcd2.setCharAt(7,(setRPM/100)%10 + 0x30);
				lcd2.setCharAt(8,(setRPM/10)%10 + 0x30);
				lcd2.setCharAt(9,(setRPM)%10 + 0x30);
				frefreshLCD=1;
				fEdit=0;
			  }
			}
			if(fencUp)
			{
				selPos=1;
				frefreshLCD=1;
				fencUp=0;
			}
			if(fencDw)
			{
				selPos=0;              
				frefreshLCD=1;
				fencDw=0;
			}
			break;
			}
			case continuo:
			{
			if(fswLClick)//click longo na chave
			{
				fswLClick=0;
				state=main;
				lcd1=" Modo Continuo  ";
				lcd2=" Modo Timer     ";
				frefreshLCD=1;
				fEdit=0;
			}           
			if(fswSClick)//click curto na chave
			{
				fswSClick=0;
				if(selPos<2)fEdit=!fEdit;
				else
				{
					state=on;
					
					oPWM = ((unsigned long)setRPM<<8)/MAX_RPM;
					
					Serial.println(setRPM<<8);
					Serial.println(MAX_RPM);
					Serial.println((setRPM<<8)/MAX_RPM);
					Serial.println(oPWM);
					if(!fCont_Timer)
					{
						lcd1="Rset =          ";						
						lcd2="Rreal=          ";
						lcd1.setCharAt(6,(setRPM/10000)%10 + 0x30);
						lcd1.setCharAt(7,(setRPM/1000)%10 + 0x30);
						lcd1.setCharAt(8,(setRPM/100)%10 + 0x30);
						lcd1.setCharAt(9,(setRPM/10)%10 + 0x30);
						lcd1.setCharAt(10,(setRPM)%10 + 0x30);
						
						lcd2.setCharAt(6,(realRPM/10000)%10 + 0x30);
						lcd2.setCharAt(7,(realRPM/1000)%10 + 0x30);
						lcd2.setCharAt(8,(realRPM/100)%10 + 0x30);
						lcd2.setCharAt(9,(realRPM/10)%10 + 0x30);
						lcd2.setCharAt(10,(realRPM)%10 + 0x30);
					}
					else
					{
						tOper=setTime*10;	//f100ms base
						
						lcd1="Rset=XXXXX t=XXX";
						lcd1.setCharAt(5,(setRPM/10000)%10 + 0x30);
						lcd1.setCharAt(6,(setRPM/1000)%10 + 0x30);
						lcd1.setCharAt(7,(setRPM/100)%10 + 0x30);
						lcd1.setCharAt(8,(setRPM/10)%10 + 0x30);
						lcd1.setCharAt(9,(setRPM)%10 + 0x30);
						lcd1.setCharAt(13,(tOper/1000)%10 + 0x30);
						lcd1.setCharAt(14,(tOper/100)%10 + 0x30);
						lcd1.setCharAt(15,(tOper/10)%10 + 0x30);
            /*
						lcd2="Rreal=XXXXX     ";
						lcd2.setCharAt(6,(realRPM/10000)%10 + 0x30);
						lcd2.setCharAt(7,(realRPM/1000)%10 + 0x30);
						lcd2.setCharAt(8,(realRPM/100)%10 + 0x30);
						lcd2.setCharAt(9,(realRPM/10)%10 + 0x30);
						lcd2.setCharAt(10,(realRPM)%10 + 0x30);*/
					
					}
				}
				frefreshLCD=1;
				break;
			}
			if(fencUp)
			{
				if(!fEdit)
				{
					if(selPos++>2)selPos=2;
				}
				else
				{
					switch(selPos)
					{
					  case 0: 
						break;
					  case 1:
						if(setRPM<MAX_RPM)setRPM+=100;
						break;
					}
				}

				lcd1="                ";
				lcd2=" RPM=         ON";
				lcd2.setCharAt(5,(setRPM/10000)%10 + 0x30);
				lcd2.setCharAt(6,(setRPM/1000)%10 + 0x30);
				lcd2.setCharAt(7,(setRPM/100)%10 + 0x30);
				lcd2.setCharAt(8,(setRPM/10)%10 + 0x30);
				lcd2.setCharAt(9,(setRPM)%10 + 0x30);
				frefreshLCD=1;
				fencUp=0;
			}
			if(fencDw)
			{
				if(!fEdit)
				{
					if(selPos>1)selPos--;
					else		selPos=1;              
				}
				else
				{
					switch(selPos)
					{
					  case 0: 
						break;
					  case 1:
						if(setRPM>100)setRPM-=100;
						break;
					}
				}

				lcd1="                ";
				lcd2=" RPM=         ON";
				lcd2.setCharAt(5,(setRPM/10000)%10 + 0x30);
				lcd2.setCharAt(6,(setRPM/1000)%10 + 0x30);
				lcd2.setCharAt(7,(setRPM/100)%10 + 0x30);
				lcd2.setCharAt(8,(setRPM/10)%10 + 0x30);
				lcd2.setCharAt(9,(setRPM)%10 + 0x30);
				fencDw=0;
				frefreshLCD=1;
			}
			break;
			}
			case timer:
			{
			if(fswLClick)//click longo na chave
			{
				fswLClick=0;
				state=main;
				lcd1=" Modo Continuo  ";
				lcd2=" Modo Timer     ";
				frefreshLCD=1;
				fEdit=0;
			}           
			if(fswSClick)//click curto na chave
			{
				fswSClick=0;
				if(selPos<2)fEdit=!fEdit;
				else
				{
					state=on;
					
					oPWM = ((unsigned long)setRPM<<8)/MAX_RPM;
					
					Serial.println(setRPM<<8);
					Serial.println(MAX_RPM);
					Serial.println((setRPM<<8)/MAX_RPM);
					Serial.println(oPWM);
					if(!fCont_Timer)
					{
						lcd1="Rset =        ";
						lcd2="Rreal=        ";
					}
					else
					{
						tOper=setTime*10;	//f100ms base
						
						lcd1="Rset=XXXXX t=XXX";
						lcd1.setCharAt(5,(setRPM/10000)%10 + 0x30);
						lcd1.setCharAt(6,(setRPM/1000)%10 + 0x30);
						lcd1.setCharAt(7,(setRPM/100)%10 + 0x30);
						lcd1.setCharAt(8,(setRPM/10)%10 + 0x30);
						lcd1.setCharAt(9,(setRPM)%10 + 0x30);
						lcd1.setCharAt(13,(tOper/1000)%10 + 0x30);
						lcd1.setCharAt(14,(tOper/100)%10 + 0x30);
						lcd1.setCharAt(15,(tOper/10)%10 + 0x30);
            /*
						lcd2="Rreal=XXXXX     ";
						lcd2.setCharAt(6,(realRPM/10000)%10 + 0x30);
						lcd2.setCharAt(7,(realRPM/1000)%10 + 0x30);
						lcd2.setCharAt(8,(realRPM/100)%10 + 0x30);
						lcd2.setCharAt(9,(realRPM/10)%10 + 0x30);
						lcd2.setCharAt(10,(realRPM)%10 + 0x30);*/
					
					}
				}
				frefreshLCD=1;
			}
			if(fencUp)
			{
				if(!fEdit)
				{
					if(selPos++>2)selPos=2;
				}
				else
				{
					switch(selPos)
					{
					  case 0: 
						if(setTime<MAX_TIME)setTime+=10;
						break;
					  case 1:
						if(setRPM<MAX_RPM)setRPM+=100;
						break;
					}
				}

				lcd1=" t=   s          ";
				lcd1.setCharAt(3,(setTime/100)%10 + 0x30);
				lcd1.setCharAt(4,(setTime/10)%10 + 0x30);
				lcd1.setCharAt(5,(setTime)%10 + 0x30);

				lcd2=" RPM=         ON";
				lcd2.setCharAt(5,(setRPM/10000)%10 + 0x30);
				lcd2.setCharAt(6,(setRPM/1000)%10 + 0x30);
				lcd2.setCharAt(7,(setRPM/100)%10 + 0x30);
				lcd2.setCharAt(8,(setRPM/10)%10 + 0x30);
				lcd2.setCharAt(9,(setRPM)%10 + 0x30);
				frefreshLCD=1;
				fencUp=0;
			}
			if(fencDw)
			{
				if(!fEdit)
				{
					if(selPos>0)selPos--;
					else		selPos=0;              
				}
				else
				{
					switch(selPos)
					{
					  case 0: 
						if(setTime>10)setTime-=10;
						break;
					  case 1:
						if(setRPM>100)setRPM-=100;
						break;
					}
				}

				lcd1=" t=   s          ";
				lcd1.setCharAt(3,(setTime/100)%10 + 0x30);
				lcd1.setCharAt(4,(setTime/10)%10 + 0x30);
				lcd1.setCharAt(5,(setTime)%10 + 0x30);

				lcd2=" RPM=         ON";
				lcd2.setCharAt(5,(setRPM/10000)%10 + 0x30);
				lcd2.setCharAt(6,(setRPM/1000)%10 + 0x30);
				lcd2.setCharAt(7,(setRPM/100)%10 + 0x30);
				lcd2.setCharAt(8,(setRPM/10)%10 + 0x30);
				lcd2.setCharAt(9,(setRPM)%10 + 0x30);
				fencDw=0;
				frefreshLCD=1;
			}
			break;
			}
			case on:	//centrifuga operando
			{
				fswSClick=fencUp=fencDw=0;  //desabilitadas
				if(fswLClick||(tOper==0&&fCont_Timer))//volta ao menu anterior
				{
					fswLClick=0;
					state=(fCont_Timer?timer:continuo);
					
					if(state==timer)
					{
						lcd1=" t=   s          ";
						lcd1.setCharAt(3,(setTime/100)%10 + 0x30);
						lcd1.setCharAt(4,(setTime/10)%10 + 0x30);
						lcd1.setCharAt(5,(setTime)%10 + 0x30);

						lcd2=" RPM=         ON";
						lcd2.setCharAt(5,(setRPM/10000)%10 + 0x30);
						lcd2.setCharAt(6,(setRPM/1000)%10 + 0x30);
						lcd2.setCharAt(7,(setRPM/100)%10 + 0x30);
						lcd2.setCharAt(8,(setRPM/10)%10 + 0x30);
						lcd2.setCharAt(9,(setRPM)%10 + 0x30);
					}
					else
					{
						lcd1="                ";
						lcd2=" RPM=         ON";
						lcd2.setCharAt(5,(setRPM/10000)%10 + 0x30);
						lcd2.setCharAt(6,(setRPM/1000)%10 + 0x30);
						lcd2.setCharAt(7,(setRPM/100)%10 + 0x30);
						lcd2.setCharAt(8,(setRPM/10)%10 + 0x30);
						lcd2.setCharAt(9,(setRPM)%10 + 0x30);
					}
					oPWM=0;
					analogWrite(oMotor,0);
					selPos=(fCont_Timer?0:1);
					tBeep=100;
					frefreshLCD=1;
				}

				
				
				if(f100ms)
				{
					if(tOper>0)
					{
						tOper--;
					}
					f100ms=0;
					if(fCont_Timer)
					{
						lcd1.setCharAt(13,(tOper/1000)%10 + 0x30);
						lcd1.setCharAt(14,(tOper/100)%10 + 0x30);
						lcd1.setCharAt(15,(tOper/10)%10 + 0x30);
					}
					/**
					lcd2.setCharAt(6,(showRPM/10000)%10 + 0x30);
					lcd2.setCharAt(7,(showRPM/1000)%10 + 0x30);
					lcd2.setCharAt(8,(showRPM/100)%10 + 0x30);
					lcd2.setCharAt(9,(showRPM/10)%10 + 0x30);
					lcd2.setCharAt(10,(showRPM)%10 + 0x30);
					frefreshLCD = 1;
          */
					//Serial.println((255*setRPM)/MAX_RPM);
          
				}
				
				
				//CONTROLE PID
				
				
				

				
				Serial.print(showRPM);
				Serial.print("\t");
				Serial.print(setRPM);
				Serial.print("\t");
				Serial.print(oPWM);
				Serial.println("\t");
			}
			break;
			case test:
			{
				
				
				fswLClick=0;  //desabilitadas
				
				
				
				if(fswSClick)//click curto na chave
				{
					fswSClick=0;
					
				}
				if(fencUp)
				{
					fencUp=0;
					if(oPWM<255)oPWM++;
					Serial.println(oPWM);
				}
				if(fencDw)
				{         
					fencDw=0;
					
					if(oPWM>0)oPWM--;
					Serial.println(oPWM);
					
				}
				
				
				if(f100ms)
				{
					
					lcd2.setCharAt(6,(showRPM/10000)%10 + 0x30);
					lcd2.setCharAt(7,(showRPM/1000)%10 + 0x30);
					lcd2.setCharAt(8,(showRPM/100)%10 + 0x30);
					lcd2.setCharAt(9,(showRPM/10)%10 + 0x30);
					lcd2.setCharAt(10,(showRPM)%10 + 0x30);
					frefreshLCD = 1;
				}
				
				analogWrite(oMotor,oPWM);
				//analogWrite(buzz,oPWM);
			}
			break;

		}
		
		if(tBeep)
		{
			tBeep--;
			digitalWrite(buzz, 1);
		}
		else
		{
			digitalWrite(buzz, 0);
		}

		
		if(revCount)
		{
			
			currtime=millis();
			idletime=currtime-prevtime;
			
			

			realRPM=(((unsigned long)60000*revCount)/(18*idletime));
			calcRPM += realRPM;
			/*
			Serial.print(idletime);
			Serial.print(" ");
			Serial.print(currtime);
			Serial.print(" ");
			Serial.print(prevtime);
			Serial.print(" ");
			Serial.print(revCount);
			Serial.print(" ");
			Serial.print(showRPM);
			Serial.print(" ");
			Serial.println(realRPM);
			*/
						
			rpmAvg++;
			if(rpmAvg>64)
			{
				rpmAvg=0;
				showRPM=calcRPM>>6;	//div by 32
				
				if((showRPM%100)<50)
				{
					showRPM=(int)(showRPM/100);	//trunca na centena
					showRPM=showRPM*100;
				}
				else
				{
					showRPM=(int)(showRPM/100);	//trunca na centena
					showRPM=(showRPM+1)*100;
				}

				calcRPM=0;
			}
			
			revCount=0;
			prevtime=currtime;
		}
		else
		{
			//showRPM=0;
		}
		
	}
	refreshEnc();
	
	if(state==on)
	{
		//oPWM = pid(setRPM,showRPM);
		oPWM = ((unsigned long)setRPM<<8)/MAX_RPM;
		
	}
	else
	{
		if(state!=test)
		{
				oPWM = 0;
		}
	}
	
	analogWrite(oMotor,oPWM);
}


int pid(int Setpoint, int inp)
{
	currentTime = millis();               				 //get current time
	elapsedTime = (currentTime - previousTime);        //compute time elapsed from previous computation
	
	error = Setpoint - inp;                                	// determine error
	cumError += error * elapsedTime;                		// compute integral
	rateError = (error - lastError)/elapsedTime;   			// compute derivative

	int out = ((kp*error + ki*cumError + kd*rateError)<0)?0:(kp*error + ki*cumError + kd*rateError);                //PID output               

	if(out>255)out=255;
	
	lastError = error;                                //remember current error
	previousTime = currentTime;                        //remember current time

	return out;                                        //have function return the PID output
}

void RPMCount()                                 
{
	revCount++;   
	digitalWrite(buzz, digitalRead(buzz)^1);	
}

ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = timer1_counter;   // preload timer
  //digitalWrite(13, digitalRead(13) ^ 1);  //ledRun
  
  //Le as informacoes do encoder
  ftick = 1;

  if(digitalRead(swEnc)==0)
	{
	  swCount++;
	  if(swCount==longClick)fswLClick=1;
	}
	else
	{ 
	  if((swCount>=shortClick)&&(swCount<longClick))  
	  {
		fswSClick=1;
		swCount=0;
	  }
	  else  swCount=0;
	}
	if(i<10)i++;
	else
	{
		i=0;
		f100ms=1;
	}
}

void refreshEnc()
{
  if(newPos>pos)
  {
	fencUp=1;
	pos=newPos;
  }
  if(newPos<pos)
  {
	fencDw=1;
	pos=newPos;
  }
  encoder.tick();
  newPos = encoder.getPosition();
}
