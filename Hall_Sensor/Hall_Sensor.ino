/*
  Title: Hall Effect Speed Calculation for ATmega328P
  Author: Matthew Shepherd ('19), Connor Monahan ('21)
  Description: This code calculated the speed of the car using the signal generated
    by the Hall effect sensor. Magnetic teeth on the car's wheels pass by the
    sensor, which generates a digital pulse signal. By calculating the difference
    between edges of this pulse, we can calculate the angular speed of the wheel, and
    therefore the car's velocity.
   This code calculates the velocity recorded by two sensors.
  Resources: http://ww1.microchip.com/downloads/en/DeviceDoc/doc8006.pdf
*/


const float milesPerTooth = 0.0000401624;
const float millisPerHour = 3600000;

volatile uint32_t lastEdge1 = 0, lastEdge2 = 0;
volatile float carSpeed1 = 0, carSpeed2 = 0;

/**
 * Interrupt reference:
 * attachInterrupt 2,3 (INT0, INT1) - hardware interrupts
 * PCICR / PCMSKn - pin change interrupts
 *  ISR(PCINT0_vect){} // for pins PCINT0-PCINT7   (PB0-PB7)
 *  ISR(PCINT1_vect){} // for pins PCINT8-PCINT14  (PC0-PC6)
 *  ISR(PCINT2_vect){} // for pins PCINT16-PCINT23 (PD0-PD7)
 */
void ISR_INT1();
void setup_hall_sensors();
void write_hall_sensors();

void setup() {
    Serial.begin(115200);
    setup_hall_sensors();
}

void loop() {
    write_hall_sensors();
}

void setup_hall_sensors() {
    Serial.println("Begin hall effect speed calculation");
    
    // enable hardware interrupt
    attachInterrupt(digitalPinToInterrupt(3), ISR_INT1, RISING);
    
    // enable pin change interrupt
    PCICR |= (1 << PCIE1);
    //choose pins for interrupt
    PCMSK1 = (1<<PCINT8) /* | (1<<PCINT9) */;
}

void write_hall_sensors() {
    // Write carSpeed over CAN or SPI
    Serial.print("Speed1 (mph): ");
    Serial.print(carSpeed1);
    Serial.print(" Speed2 (mph): ");
    Serial.print(carSpeed2);
    Serial.println();
}

// hardware interrupt subroutine (for interrupt pin INT1/3)
void ISR_INT1() {
    unsigned long currentEdge = millis();
    float teethPerMillis = 1.0/(float) (currentEdge - lastEdge1);
    carSpeed1 = milesPerTooth * teethPerMillis * millisPerHour;
    lastEdge1 = currentEdge;
}

//external interrupt subroutine (for interrupt pin A0/PCINT8)
ISR(PCINT1_vect)
{
    cli();
    if (digitalRead(A0) == HIGH) {
        unsigned long currentEdge = millis();
        float teethPerMillis = 1.0/(float) (currentEdge - lastEdge2);
        carSpeed2 = milesPerTooth * teethPerMillis * millisPerHour;
        lastEdge2 = currentEdge;
    }
    sei();
}


