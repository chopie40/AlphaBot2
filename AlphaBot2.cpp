#include "AlphaBot2.h"


/*
  ** ================================================================================= **
	** ============================= Variables globales ================================ **
  ** ================================================================================= **
*/
volatile uint8_t rData;


/*
  ** ================================================================================= **
	** ========================== Définition des fonctions ============================= **
  ** ================================================================================= **
*/

/*
  ** ================================== PCF8574 ====================================== **
*/
/*
  "I2C communication with this device is initiated by a master sending a start condition,
  a high-to-low transition on the SDA I/O while the SCL input is high. After the start
  condition, the device address byte is sent, MSB first, including the data direction bit.
  This device does not respond to the general call address. After receiving the valid
  address byte, this device responds with an acknowledge, a low on the SDA I/O during the
  high of the acknowledge-related clock pulse. The address inputs (A0–A2) of the slave
  device must not be changed between the start and the stop conditions. The data byte
  follows the address acknowledge. If the R/W bit is high, the data from this device are
  the values read from the P port. If the R/W bit is low, the data are from the master,
  to be output to the P port. The data byte is followed by an acknowledge sent from this
  device. If other data bytes are sent from the master, following the acknowledge, they
  are ignored by this device. Data are output only if complete bytes are received and
  acknowledged. The output data will be valid at time, tpv, after the low-to-high
  transition of SCL and during the clock cycle for the acknowledge. A stop condition,
  which is a low-to-high transition on the SDA I/O while the SCL input is high, is sent
  by the master."

  ** Selon la datasheet SCPS068J-July 2001-Revised march 2015 par Texas Instruments. ** 
*/
void twiWrite(uint8_t data) {
  Wire.beginTransmission(0x20);
  Wire.write(data);
  Wire.endTransmission();
}

/*
  Lecture du port P de PCF8574 pour lire l'état des capteurs infrarouges ST188. Si les
  broches P6-P7 tombent à 0V, ça veux dire qu'un capteur a détecté un obstacle.
*/
uint8_t twiRead() {
  rData = -1;
  Wire.requestFrom((uint8_t)0x20, (uint8_t)0x1);
  if (Wire.available()) {
    rData = Wire.read();
  }
  return rData;
}

/*
  ** ================================= TB6612FNG ===================================== **
*/
/* 
  *                     H-Bridge Control function
  *  __________________________________________________________________
  *  |           Input            |               Output              |
  *  ==================================================================
  *  | IN1 | IN2 |  PWM  |  STBY  |  OUT1  |  OUT2  |       Mode      |
  *  ==================================================================
  *  |  H  |  H  |  H/L  |    H   |    L   |    L   |   Short brake   |
  *  ==================================================================
  *  |  L  |  H  |   H   |    H   |    L   |    H   |       CCW       | moteur avance.
  *  |     |     |   L   |    H   |    L   |    L   |   Short brake   |
  *  ==================================================================
  *  |  H  |  L  |   H   |    H   |    H   |    L   |        CW       | moteur recule.
  *  |     |     |   L   |    H   |    L   |    L   |   Short brake   |
  *  ==================================================================
  *  |  L  |  L  |   H   |    H   |  OFF(Tri-state) |      Stop       |
  *  ==================================================================
  *  | H/L | H/L |  H/L  |    L   |  OFF(Tri-state) |     Standby     |
  *  ==================================================================
  *
*/
void forward() {
  PORTC &= 0b11111001;    // AIN1 0, BIN1 0.
  PORTC |= 0b00001001;    // AIN2 1, BIN2 1.
}

void backward() {
  PORTC |= 0b00000110;    // AIN1 1, BIN1 1.
  PORTC &= 0b11110110;    // AIN2 0, BIN2 0.
}

void right() {
  // A = Moteur droit.
  OCR0A |= 0b01000110;    // 25% Duty Cycle.
  OCR0B |= 0b01000110;
  PORTC &= 0b11110101;    // AIN1 0, BIN2 0.
  PORTC |= 0b00000101;    // AIN2 1, BIN1 1.
}

void left() {
  // B = Moteur gauche.
  OCR0A |= 0b01000110;    // 25% Duty Cycle.
  OCR0B |= 0b01000110;
  PORTC |= 0b00001010;    // AIN1 1, BIN2 1.
  PORTC &= 0b11111010;    // AIN2 0, BIN1 0.
}

void stop() {
  OCR0A |= 0b01011010;    // 35% Duty Cycle.
  OCR0B |= 0b01011010;
  PORTC |= 0b00001111;    // AIN2 1, AIN1 1, BIN1 1, BIN2 1.
}