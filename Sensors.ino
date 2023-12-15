#include <Adafruit_BMP085.h>
#include "BH1750FVI.h"
#include <Bonezegei_DHT11.h>

unsigned char turnsCount = 0;
bool lastHallState = false;

int sensorValue = 0;
float sensorVoltage = 0;
float windSpeed = 0;

float voltageConversionConstant = .004882814;
int sensorDelay = 1000;

float voltageMin = .004;
float windSpeedMin = 0;

float voltageMax = 0.05;
float windSpeedMax = 32;

int event;
int eventLastState;

 volatile byte rpmcount;   // count signals
 volatile unsigned long last_micros;
 unsigned long timeold;
 unsigned long timemeasure = 25.00; // seconds
 unsigned long timeNow;
 int countThing = 0;
 int GPIO_pulse = 12;  // NodeMCU = D6 
 float rpm, rps;         // frequencies
 float radius = 0.060; // meters - measure of the lenght of each the anemometer wing
 float linear_velocity = 0; // m/seg
 float linear_velocity_kmh; // km/h
 float omega = 0; // rad/seg

BH1750FVI myLux(0x23);

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

Adafruit_BMP085 bmp;
Bonezegei_DHT11 dht(15);
const int RPMPin = 7;
int RPMState;
int RPMon = 0;

bool CheckTurnsCount(int pin, bool& lastState, unsigned char& count, unsigned char maxCount)
{
    bool curState = digitalRead(pin); 

    count += (lastState && !curState);  // tests and comparisons return either 0 or 1
    lastState = curState;

    if (count < maxCount)
        return false;

    count = 0;
    return true;
}


void setup() {
  Serial.begin(9600);
  Wire.begin();
  myLux.powerOn();
  myLux.setContHighRes();
  dht.begin();
  pinMode(RPMPin, INPUT_PULLUP);
  eventLastState = digitalRead(RPMPin);
  attachInterrupt(digitalPinToInterrupt(RPMPin), count, RISING);
  rpmcount = 0;
  rpm = 0;
  timeold = 0;
  timeNow = 0;
  
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
}
  
void loop() {
    if ((millis() - timeold) >= timemeasure*1000){ 
      countThing ++;
      detachInterrupt(digitalPinToInterrupt(RPMPin));        // Disable interrupt when calculating
      rps = float(rpmcount)/float(timemeasure);       // rotations per second
      rpm = 60*rps;                         // rotations per minute
      omega = 2*PI*rps;                     // rad/seg
      linear_velocity = omega*radius;       // m/seg
      linear_velocity_kmh = linear_velocity * 3.6;  // km/h
      attachInterrupt(digitalPinToInterrupt(RPMPin), count, RISING);
    }
    float RPM;
    float wind;
    int hum;
    int RPMon;

    int sensorValue = analogRead(A0);
    float voltage = sensorValue*5/1023.0;
    int direction = map(sensorValue, 0, 1023, 0, 360);

    Serial.print(linear_velocity_kmh);
    Serial.print(" km/h ");

    Serial.print(direction);
    Serial.print(" direction ");
  
    Serial.print(myLux.getLux());
    Serial.print(" Lux ");

    Serial.print(bmp.readTemperature());
    Serial.print(" *C ");
    
    Serial.print(bmp.readPressure());
    Serial.print(" Pa ");

    if (dht.getData()) {                         // get All data from DHT11
    hum = dht.getHumidity();  // return humidity
     Serial.print(hum);
    Serial.print("% Humidity ");
  }
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print(bmp.readAltitude());
    Serial.print(" meters ");

    Serial.print(bmp.readSealevelPressure());
    Serial.print(" Pa ");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print(bmp.readAltitude(101500));
    Serial.print(" meters");
    
    Serial.println();
    delay(5000);
}

void count() {
  event = event + 1;
}

void rpm_anemometer(){
    if(long(micros() - last_micros) >= 5000){      // time to debounce measures
        rpmcount++;
        last_micros = micros();
    }    
//   Serial.println("***** detect *****");
 }
