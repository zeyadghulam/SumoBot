#include <Servo.h>

Servo myservo;

//int ps = 95;


// Standard Setup COde

/*
*************************************************
Ask terry about intializing s_ServoStartTime
*************************************************
*/
 

unsigned long s_ServoStartTime;

static const int s_searchSpeed = 255 ;
static const int s_AttackSpeed = 255;

static const int s_nMot1DirPin = 5;    // Atmega Pin 11
static const int s_nMot2DirPin = 7;    // Atmega Pin 13
static const int s_nMot1EnPin = 8;     // Atmega Pin 14
static const int s_nMot2EnPin = 6;     // Atmega Pin 12
static const int s_nMot1PwmPin = 10;   // Atmega Pin 16
static const int s_nMot2PwmPin = 9;    // Atmega Pin 15

static const int s_nLed1Pin = 2;      // Atmega Pin 4
static const int s_nLed2Pin = 3;      // Atmega Pin 5
static const int s_nLed3Pin = 4;      // Atmega Pin 6

static const int s_n38KHzEnPin = A1;  // Atmega Pin 24
//static const int s_nRange1Pin = A2;   // Atmega Pin 25
//static const int s_nRange2Pin = A3;   // Atmega Pin 26
static const int s_nRangeTriggerPin = A2;
static const int s_nRangeEchoPin = A3;
 
static const int s_nLine1Pin = A5;    // Atmega Pin 28
static const int s_nLine2Pin = A4;    // Atmega Pin 27

static int const s_nBatVoltPin = A0;

static volatile unsigned long s_ulRange1LowTime;
static volatile unsigned long s_ulRange1HighTime;
static volatile bool s_bRange1Valid = false;

void Drive( int nSpeed1, int nSpeed2 );

//#include <MsTimer2.h>
class CRange{ //bot detection case, by default 'CRANGE' is private
  enum { RANGE_INIT, RANGE_START, RANGE_HIGH, RANGE_TRIGGER, RANGE_MEASURE, RANGE_RESET } state;//defines these as seperate values
  unsigned long starttime;
  unsigned long endtime;
  bool ready;
  long distance;

public:
  CRange (){
    state = RANGE_INIT;
    ready = false;
  }

 void loop (){ //bot detect state machine
  switch (state){
    case RANGE_INIT:
      state = RANGE_START;
      starttime = micros();
      break;
    case RANGE_START:
      digitalWrite(s_nRangeTriggerPin, LOW);
      if (micros() >= starttime + 50){
        state = RANGE_HIGH;
        starttime = micros();
        //Serial.println( "s->HIGH" );
      }
      break;
    case RANGE_HIGH:
     digitalWrite(s_nRangeTriggerPin, HIGH);
      if (micros() >= starttime + 20){
        state = RANGE_TRIGGER;
        starttime = micros();
        //Serial.println( "s->TRIGGER" );
      }
      break;
    case RANGE_TRIGGER:
      digitalWrite(s_nRangeTriggerPin, LOW);
      // wait for echo pin to go high so we can measure it.
      if (digitalRead(s_nRangeEchoPin) == HIGH ) {
        state = RANGE_MEASURE;
        starttime = micros();  
        //Serial.println( "s->MEASURE" );
      }
      else if( micros() - starttime > 1000000 ) {
        state = RANGE_RESET;
        starttime = micros();
        //Serial.println( "s->RESET" );
      }
      break;
    case RANGE_MEASURE:
      if (digitalRead(s_nRangeEchoPin) == LOW || (micros() - starttime) > 1000000){
        endtime = micros();
        distance = (endtime - starttime)/58.2;
        //Serial.print( "Distance = " );
        //Serial.println( distance );
        ready = true;
        state = RANGE_RESET;
        starttime = micros();  
        //Serial.println( "s->RESET" );
      }
      break;
    case RANGE_RESET:
 //     digitalWrite( s_nRangeTriggerPin, HIGH );
      if( micros() - starttime > 100000 ) {
        state = RANGE_START;
        starttime = micros();
        //Serial.println( "s->START" );
      }
  }
 }
 long getrange(){ //what the main code looks at to get the range value
  if (ready)
    return distance;
  else
    return -1;
 }
};
CRange range;


void setup()
{
  myservo.attach(13);
// set up IR interrupts 
//  PCMSK1 |= (1 << PCINT10);
//  PCICR |= (1 << PCIE1);

  pinMode( s_nMot1DirPin, OUTPUT );
  pinMode( s_nMot2DirPin, OUTPUT );
  pinMode( s_nMot1EnPin, OUTPUT );
  pinMode( s_nMot2EnPin, OUTPUT );
  pinMode( s_nMot1PwmPin, OUTPUT );
  pinMode( s_nMot2PwmPin, OUTPUT );
  
  pinMode( s_nLed1Pin, OUTPUT );
  pinMode( s_nLed2Pin, OUTPUT );
  pinMode( s_nLed3Pin, OUTPUT );
  digitalWrite( s_nLed1Pin, HIGH );
  digitalWrite( s_nLed2Pin, HIGH );
  digitalWrite( s_nLed3Pin, HIGH );

  pinMode( s_nRangeTriggerPin, OUTPUT );
  pinMode( s_nRangeEchoPin, INPUT ); //reading state of pin
  
  pinMode( s_nLine1Pin, INPUT_PULLUP );
  pinMode( s_nLine2Pin, INPUT_PULLUP );
  
  analogReference( DEFAULT );
  Serial.begin( 9600 );
}

void SetDrive( int nSpeed1, int nSpeed2 )
{
  if( nSpeed1 > 0 )
  {
    digitalWrite( s_nMot1DirPin, LOW );
    analogWrite( s_nMot1PwmPin, nSpeed1 );
    digitalWrite( s_nMot1EnPin, HIGH );
  }
  else if( nSpeed1 < 0 )
  {
    digitalWrite( s_nMot1DirPin, HIGH );
    analogWrite( s_nMot1PwmPin, 255 + nSpeed1 );
    digitalWrite( s_nMot1EnPin, HIGH );
  }
  else
  {  
    digitalWrite( s_nMot1EnPin, LOW );
  }
  
  if( nSpeed2 > 0 )
  {
    digitalWrite( s_nMot2DirPin, LOW );
    analogWrite( s_nMot2PwmPin, nSpeed2 );
    digitalWrite( s_nMot2EnPin, HIGH );
  }
  else if( nSpeed2 < 0 )
  {
    digitalWrite( s_nMot2DirPin, HIGH );
    analogWrite( s_nMot2PwmPin, 255 + nSpeed2 );
    digitalWrite( s_nMot2EnPin, HIGH );
  }
  else
  {
    digitalWrite( s_nMot2EnPin, LOW );
  }
}

boolean bDetectLine1()
{
  if( digitalRead( s_nLine1Pin ) ) //line detect code here
    return false; //line detected
  else
    return true; //no line
}

boolean bDetectLine2()
{
  if( digitalRead( s_nLine2Pin ) )
    return false;
  else
    return true;
}

void StartDrive( int nSpeed1, int nSpeed2, int nMillis )
{
  unsigned long ulDriveEndMillis;

  ulDriveEndMillis = millis() + nMillis;
  SetDrive( nSpeed1, nSpeed2 );
  while( millis() < ulDriveEndMillis );
}
//may stuck in the loop and keep on doing whatever the machine is doing

void forward()
{
  SetDrive( 100, 100 );

}

void turnRight()
{
  SetDrive( 100, -100 );
  delay( 500 );
}

void turnLeft()
{
  SetDrive( -100, 100 );
  delay( 500 );
}
//
void stopBot()
{
  SetDrive( 0, 0 );
  delay( 1000 );
}

void StateMachine( boolean bInRange )
{
  static enum { sInit, sSearch, sAvoidLine, sAttack } s_state = sInit;
//initial moving, after pressing the button
  switch( s_state )
  {
    case sInit: //start of match, driving forward
      delay( 2000 );
      s_state = sSearch;
      SetDrive( s_searchSpeed, s_searchSpeed );
      Serial.println( "Search" );
      break;
    case sSearch:
      if( bDetectLine1() )
      {
        SetDrive( -s_searchSpeed, -s_searchSpeed );
        Serial.println( "AvoidLine" );
        s_state = sAvoidLine;
      }
  #if 1
      if( bInRange )
      {
        SetDrive( s_AttackSpeed, s_AttackSpeed );
        Serial.println( "Attack" );
       s_ServoStartTime= millis() + 2500; 
        s_state = sAttack;
      }
 #endif
     break;
   case sAvoidLine:
     if( !bDetectLine1() )
     {
       delay( 500 );
       SetDrive( s_searchSpeed, -s_searchSpeed );
       delay( 750 );
       SetDrive( s_searchSpeed, s_searchSpeed );
       s_state = sSearch;
       Serial.println( "Search"  );
     }
     break;
   case sAttack:
      if( bDetectLine1() )
      {
        SetDrive( -s_searchSpeed, -s_searchSpeed );
        s_state = sAvoidLine;
        Serial.println( "Attack" );
        myservo.write(92);
      }
      if( !bInRange )
      {
        SetDrive( s_searchSpeed, s_searchSpeed );
        s_state = sSearch;
        Serial.println( "Search" );
        myservo.write(92);
      }
      if(millis() >= s_ServoStartTime  )
      {
        if(myservo.read() < 180 )
        {
           myservo.write(180);
           Serial.println("servo 180");
        }
      }
      else {
        if( myservo.read() > 92 )
        {
          myservo.write(92);
          Serial.println("servo 92");
        }
      }
     break;
  }
}
  
void loop()
{
  static boolean bLastInRange = true;
  boolean bInRange = bLastInRange;

  range.loop();
  
#if 0  
  val = analogRead( s_nBatVolt );
  
  // Input voltage goes through a voltage divider network of 10K & 75K, so V = 10 * Vbat / (10 + 75)
  // The analog reference is set to 5V, val = 0 -> 0V, val = 1023 = 5V, but we are measuring after the divider
  // so the real voltage range is 5V * 85 / 10 = 42.5.
  // We will map the voltage to 0 -> 4250 hundredths of a volt or 42.50volts
  // Something should be done when the voltage drops below 11V
  val2 = map( val, 0, 1023, 0, 4250 );
  Serial.println( val2 );
#endif

#if 0
  if( s_bRange1Valid )
  {
    unsigned long ulLen = s_ulRange1HighTime - s_ulRange1LowTime;
    if( ulLen > 2 )
    {
      digitalWrite( s_nLed3Pin, ulLen > 45 ? LOW : HIGH );
      bInRange = ulLen > 45 ? true : false;
    }
    s_bRange1Valid = false;
  }
#endif
  long distance = range.getrange();
  if( distance < 0 || distance > 100)
    bInRange = false;
   else
      bInRange = true;
  
  StateMachine( bInRange );
  bLastInRange = bInRange;

  
}


