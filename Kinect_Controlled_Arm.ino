#include <Servo.h>
Servo shoulderR;
Servo shoulderFR;
Servo elbowR;
Servo gribR;
Servo stoppos;
 
void setup() 
{ 
// attach servos to their pins 
shoulderFR.attach(6);
shoulderR.attach(5);
elbowR.attach(8);
gribR.attach(9);
stoppos.attach(2);

stoppos.write(90);
shoulderR.write(90); 
shoulderFR.write(180); 
elbowR.write(90);                
gribR.write(80); 
    Serial.begin(9600); 
} 
void loop()
{ 
   while(!(Serial.available()>=4));
   byte shoulderAngler_br = Serial.read();
   byte elbowAngler_br  = Serial.read() ; 
   byte shoulderAnglezr_br  = Serial.read() ;
   byte gribr_br =Serial.read();
     
 shoulderR.write(abs(180-shoulderAngler_br)); 
 shoulderFR.write(abs(180-shoulderAnglezr_br)); 
 elbowR.write(abs(elbowAngler_br));                
 gribR.write(abs(gribr_br));          
}


