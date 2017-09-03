const int c_line1 = A5;
const int c_line2 = A4;

void setup() {
// put your setup code here, to run once:
Serial.begin( 9600 );
}

void loop() {
// put your main code here, to run repeatedly:
float line1Volts = analogRead( c_line1 ) * 5.F / 1023.F;
float line2Volts = analogRead( c_line2 ) * 5.F / 1023.F;

Serial.print( "Line 1 Volts: " );
Serial.print( line1Volts );
Serial.print( ", Line 2 Volts: " );
Serial.println( line2Volts );
}
