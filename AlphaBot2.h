#pragma once

#include <Arduino.h>
#include <Wire.h>

/*
  ** ================================================================================= **
	** ========================== Déclaration des fonctions ============================ **
  ** ================================================================================= **
*/
void twiWrite(uint8_t data);
uint8_t twiRead();
void forward();
void backward();
void right();
void left();
void stop();
