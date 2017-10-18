/*
  Title: Hall Effect Speed Calculation for ATtiny84
  Author: Matthew Shepherd ('19)
  Description: This code calculated the speed of the car using the signal generated
    by the Hall effect sensor. Magnetic teeth on the car's wheels pass by the
    sensor, which generates a digital pulse signal. By calculating the difference
    between edges of this pulse, we can calculate the angular speed of the wheel, and
    therefore the car's velocity.
  Resources: http://ww1.microchip.com/downloads/en/DeviceDoc/doc8006.pdf
*/

const int sensorPin = 0; // This is the same as chip pin 13; see Resources

const float milesPerTooth = 0; // Get this measurement
const float millisPerHour = 3600000;

volatile unsigned long lastEdge = 0;

volatile float carSpeed = 0;

void setup() {
  pinMode(sensorPin, INPUT_PULLUP);
}

void loop() {
  // Write carSpeed over CAN or SPI
}

//external interrupt subroutine (for interrupt pin INT0/chip pin 13/data pin PA0)
ISR(INT0_vect)
{
  cli();
  unsigned long currentEdge = millis();
  float teethPerMillis = 1.0/(float) (currentEdge - lastEdge);
  carSpeed = milesPerTooth * teethPerMillis * millisPerHour;
  lastEdge = currentEdge;
  sei();
}

