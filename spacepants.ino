#include "application.h"
#include "neopixel/neopixel.h"

SYSTEM_MODE(AUTOMATIC);

// IMPORTANT: Set pixel COUNT, PIN and TYPE
#define PIXEL_PIN D2
#define PIXEL_COUNT 10
#define PIXEL_TYPE WS2812B

/* Function prototypes -------------------------------------------------------*/

int tinkerDigitalRead(String pin);
int tinkerDigitalWrite(String command);
int tinkerAnalogRead(String pin);
int tinkerAnalogWrite(String command);

int analogPinInMic = A2;
int analogPinOutLED = A4;

const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

int mymax = 0;

void setup()
{
    //Serial.begin(9600);
    
    Spark.function("digitalread", tinkerDigitalRead);
    Spark.function("digitalwrite", tinkerDigitalWrite);
    Spark.function("analogread", tinkerAnalogRead);
    Spark.function("analogwrite", tinkerAnalogWrite);

    
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    
    //INPUT
    pinMode(analogPinInMic, INPUT);
  
    //OUTPUT
    pinMode(analogPinOutLED, OUTPUT);
}
void loop()
{
  unsigned long startMillis= millis();  // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 13841200;
  //unsigned int signalMin = 1024;

  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(analogPinInMic);

    if (sample < signalMin)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
         signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
         signalMin = sample;  // save just the min levels
      }
    }

  }

  peakToPeak = signalMax - signalMin;

  double volts = (peakToPeak * 3.3) / signalMin;  // convert to volts

  //int potentMultiplier = map(potentValue, 35, 4100, 0, 10);

  //mymax = volts*potentMultiplier;
  mymax = volts*10000;

  if ( mymax > 255 ) mymax = 255;

  analogWrite(analogPinOutLED, mymax);
  
  //Serial.println("mymax:"+String(mymax));
  //Serial.println("volts:"+String(volts));
}

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

void setAll(uint16_t r, uint16_t g, uint16_t b) {
  uint16_t i;

    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, r,g,b);
    }
    
    strip.show();
}


void turnOff() {
  uint16_t i;

    setAll(0,0,0);
    
    strip.show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


int tinkerDigitalRead(String pin)
{
	//convert ascii to integer
	int pinNumber = pin.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	if(pin.startsWith("D"))
	{
		pinMode(pinNumber, INPUT_PULLDOWN);
		return digitalRead(pinNumber);
	}
	else if (pin.startsWith("A"))
	{
		pinMode(pinNumber+10, INPUT_PULLDOWN);
		return digitalRead(pinNumber+10);
	}
	return -2;
}

/*
Source: Tweaking4All.com - Arduino - LEDStrip effects for NeoPixel and FastLED (http://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/)
*/
void rgbLoop()
{   
    for(int j = 0; j < 3; j++ ) 
    {     
        // Fade IN     
        for(int k = 0; k < 256; k++) 
        {       
            switch(j) 
            {         
                case 0: 
                    setAll(k,0,0); 
                    break;         
                
                case 1: 
                    setAll(0,k,0); 
                    break;         
                
                case 2: 
                    setAll(0,0,k); 
                    break;       
            }       
                strip.show();       
                delay(3);     
                
        }     
            
        // Fade OUT     
        for(int k = 255; k >= 0; k--) 
        {       
            switch(j) 
            {         
                case 0: 
                    setAll(k,0,0); 
                    break;         
                    
                case 1: 
                    setAll(0,k,0); 
                    break;         
                    
                case 2: 
                    setAll(0,0,k); 
                    break;       
            }       
                
                strip.show();       
                delay(3);     
        }   
    } 
}

/*
Source: Tweaking4All.com - Arduino - LEDStrip effects for NeoPixel and FastLED (http://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/)
*/
void FadeInOut(byte red, byte green, byte blue){   float r, g, b;         for(int k = 0; k < 256; k=k+1) {     r = (k/256.0)*red;     g = (k/256.0)*green;     b = (k/256.0)*blue;     setAll(r,g,b);     strip.show();   }         for(int k = 255; k >= 0; k=k-2) {     r = (k/256.0)*red;     g = (k/256.0)*green;     b = (k/256.0)*blue;     setAll(r,g,b);     strip.show();   } }

void FadeInOutOffMic(byte red, byte green, byte blue)
{   
    float r, g, b;         
    
    for(int k = 0; k < 256; k=k+1) 
    {     
        r = (mymax/256.0)*red;     
        g = (mymax/256.0)*green;     
        b = (mymax/256.0)*blue;     
        setAll(r,g,b);     
        strip.show();   
    }         
}


/*******************************************************************************
 * Function Name  : tinkerDigitalWrite
 * Description    : Sets the specified pin HIGH or LOW
 * Input          : Pin and value
 * Output         : None.
 * Return         : 1 on success and a negative number on failure
 *******************************************************************************/
int tinkerDigitalWrite(String command)
{
	bool value = 0;
	//convert ascii to integer
	int pinNumber = command.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	if(command.substring(3,7) == "HIGH") value = 1;
	else if(command.substring(3,6) == "LOW") value = 0;
	else return -2;

	if(command.startsWith("D"))
	{
	    if (pinNumber == 1)
	    {
	        //turn on rgb fade
	        if ( value == 1 )
	        {
    	        pinMode(pinNumber, OUTPUT);
    	        rgbLoop();
	        }
	        //turn off
	        else
	        {
	            pinMode(pinNumber, OUTPUT);
	            turnOff();
	        }
	    }
	    else if (pinNumber == 2)
	    {
	        //turn on rainbow
	        if ( value == 1 )
	        {
    	        pinMode(pinNumber, OUTPUT);
    	        rainbow(20);
	        }
	        //turn off
	        else
	        {
	            pinMode(pinNumber, OUTPUT);
	            turnOff();
	        }
	    }
	    else if (pinNumber == 3)
	    {
	        //turn on rainbow
	        if ( value == 1 )
	        {
    	        pinMode(pinNumber, OUTPUT);
    	        FadeInOut(0x00, 0x00, 0xff);
	        }
	        //turn off
	        else
	        {
	            pinMode(pinNumber, OUTPUT);
	            turnOff();
	        }
	    }
	    else if (pinNumber == 4)
	    {
	        //turn on rainbow
	        if ( value == 1 )
	        {
    	        pinMode(pinNumber, OUTPUT);
    	        FadeInOutOffMic(0xff, 0x00, 0x00);
	        }
	        //turn off
	        else
	        {
	            pinMode(pinNumber, OUTPUT);
	            turnOff();
	        }
	    }
	    else
	    {
		    pinMode(pinNumber, OUTPUT);
		    digitalWrite(pinNumber, value);
	    }
	    
		return 1;
	}
	else if(command.startsWith("A"))
	{
		pinMode(pinNumber+10, OUTPUT);
		digitalWrite(pinNumber+10, value);
		return 1;
	}
	else return -3;
}

/*******************************************************************************
 * Function Name  : tinkerAnalogRead
 * Description    : Reads the analog value of a pin
 * Input          : Pin
 * Output         : None.
 * Return         : Returns the analog value in INT type (0 to 4095)
                    Returns a negative number on failure
 *******************************************************************************/
int tinkerAnalogRead(String pin)
{
	//convert ascii to integer
	int pinNumber = pin.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	if(pin.startsWith("D"))
	{
		return -3;
	}
	else if (pin.startsWith("A"))
	{
		return analogRead(pinNumber+10);
	}
	return -2;
}
