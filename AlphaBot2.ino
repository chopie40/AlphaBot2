/* 
  WaveShare ARPICAR Run Forward/Backward/Left/Right Test
   
  ARPICAR run forward,backward,left right and so on..
   
  Created 25th June 2016 by Xinwu Lin
           
  CN: http://www.waveshare.net/
  EN: http://www.waveshare.com/
*/

/*
  ***************************************************************************************
  ********** Modifié par Dany Champagne et Mathis Grenier en mars-avril 2024. ***********
  ***************************************************************************************
  * - 1.0 -> Dany C.
  * - Amélioré la précision de la fonction de lecture de la distance.
  * - Amélioré le loop() en remplacant les "si-sinonsi-sinon" par des "selonque" et un
  *   tantque.
  * - Amélioré la lecture des capteurs en doublant les test de lecture.
  * - Le robot peux éviter un obstacle soit en tournant à gauche, à droite ou même en
  *   reculant dépendamment de l'état des capteurs.
  * - Effectué quelques optimisations sur les types.
  * - Éliminé le serial print car ralenti le programme.
  * - Séparé le code en plusieurs modules, ce qui s'appelle programmation modulaire.
  * - Remplacé par millis() tous les delay() car peuvent bloquées les autres procédures.
  * - Corrigé quelques erreurs présente dans le code original.
  * - 1.1 -> Dany C.
  * - Intégré mon pseudo-code au code.
  * - Optimisé l'initialisation en écrivant directement sur les ports E/S du registre
  *   TIMER0.
  * - 1.2
  * - Optimisé les fonctions de contrôle des moteurs avec le registre TIMER0 pour le 
  *   PWM.
  * - Optimisé la lecture de la distance avec le TIMER2 pour prendre connaissance de la
  *   valeur du capteur ultrason en temps réel avec une interruption toutes les 130 ms. 
  * - Envoyer la version finale du code sur le site internet 
  *   github.com/chopie40/AlphaBot2.
*/

/*
  * Optimisation:
  *              Original -> 5852 bytes (18%), 472 bytes (23%) dynamic memory.
  *              1.0      -> 4314 bytes (13%), 236 bytes (11%)    "      "   .
  *              1.1      -> 4126 bytes (12%), 241 bytes (11%)    "      "   .
  *              1.2      -> 3774 bytes (11%), 241 bytes (11%)    "      "   .
  *              1.3      -> 3740 bytes (11%), 237 bytes (11%)    "      "   .
  *              1.4      -> 3738 bytes (11%), 238 bytes (11%)    "      "   .
*/

/*
  *                            AlphaBot2.ino
  *
  *                    Ceci est le sketch principal.
*/
/*
  ********************************** Pseudo-code ****************************************
  * - INITIALISER
  *
  * - RÉPÉTER JUSQU'A ce que l'on coupe l'alimentation
  * -   SI (temps écoulé > 100 ms) ALORS 
  * -       LIRE la distance avec ultrason
  * -   FIN_SI
  * -   LIRE infrarouge
  * -   TANT QUE (la distance < 15 cm OU obstacle détecté)
  * -       SI (temps écoulé > 100 ms) ALORS 
  * -           LIRE la distance avec ultrason
  * -       FIN_SI
  * -       LIRE infrarouge
  * -       SELON (obstacle détecté possible)
  * -           CAS obstacle a gauche:
  * -               TOURNE a droite
  * -           CAS obstacle a droite:
  * -               TOURNE a gauche
  * -           CAS chemin bloqué:
  * -               RECULE
  * -           SINON
  * -               AVANCE
  * -       FIN_SELON
  * -   FIN_TANT_QUE
  * -   AVANCE
  * - FIN_RÉPÉTER
  *
*/
#include <Wire.h>
#include "AlphaBot2.h"

/*
  ** ================================================================================= **
	** ============================= Variables globales ================================ **
  ** ================================================================================= **
*/
uint8_t uiValue;
unsigned long i = 0ul;
volatile uint8_t j = 0;
volatile unsigned long ulDistance = 0ul;
volatile uint16_t uiDist = 0;


int main() {
  /*
  ** ================================================================================= **
	** ========================= Routine d'initialisation ============================== **
  ** ================================================================================= **
*/
  init();
  //Serial.begin(115200);    // pour debug.
  Wire.begin();
  // Moteur reglé pour avancer à l'initialisation.
  // Port C.
  DDRC |= 0b00001111;   // A0-A3 output.
  PORTC &= 0b11111001;  // AIN1 0, BIN1 0.
  PORTC |= 0b00001001;  // AIN2 1, BIN2 1.
  // PWM, Trig et Echo.
  // Port D.
  DDRD |= 0b01101000;  // D3(Trig), D5(PWMB), D6(PWMA) output.
  DDRD &= 0b11111011;  // D2(Echo) input.
  // TIMER0 8-bit
  // Timer/counter control register doivent être initialisé.
  TCCR0A |= 0b10100011;  // Mode Fast PWM et prescaler 64. Horloge
  TCCR0B |= 0b00000011;  // 16000000 / (64 * 256) = 977 Hz.
  // Initialise le compteur à zéro.
  TCNT0 = 0;
  // Output compare register utilisé pour générer le PWM au broches D5-D6(OC0A-OC0B).
  OCR0A |= 0b01011010;  // PWM initialisé à 35% (90/256).
  OCR0B |= 0b01011010;
  // TIMER2 8-bit
  // Init ISR pour la lecture du capteur ultrason en temps réel.
  TCCR2A &= 0b00000000;  // Mode normal et prescaler 32.
  TCCR2B |= 0b00000011;  // 16000000 / (32 * 256) = 1953 Hz
  TCNT2 = 0;
  /* Le bit TOIE2 active le vecteur d'interruption TIMER2_OVF, qui déclenche une
    interruption à chaque débordement du compteur TCNT2. */
  TIMSK2 |= 0b00000001;

  /*
  ** ================================================================================= **
	** ============================ Routine principale ================================= **
  ** ================================================================================= **
*/
  while (true) {
    //Serial.println(uiDist);
    // Premier test des capteurs infrarouges détecteurs d'obstacles.
    twiWrite(0xC0 | twiRead());
    uiValue = twiRead() | 0x3F;
    while (uiDist < 15 || uiValue != 0xFF) {
      //Serial.println(uiDist);
      // Deuxième test.
      twiWrite(0xC0 | twiRead());
      uiValue = twiRead() | 0x3F;
      switch (uiValue) {
        case 0x7F:  // capteur gauche, tourne a droite.
          right();
          for (i = 0; i < 175000ul; i++) {
            asm("");
          }
          stop();
          break;
        case 0xBF:  // capteur droite, tourne a gauche.
          left();
          for (i = 0; i < 175000ul; i++) {
            asm("");
          }
          stop();
          break;
        case 0x3F:  // Les deux capteurs ont détectés, recule.
          backward();
          for (i = 0; i < 175000ul; i++) {
            asm("");
          }
          stop();
          break;
        default:
          forward();
          break;
      }
    }
    forward();
  }
}


/*
  ** ================================================================================= **
	** =========================== Routine d'interruption ============================== **
  ** ================================================================================= **
*/
/*
  Évaluation de la distance avec un objet dépendamment du temps écoulé en us durant
  l'aller-retour du signal du capteur ultrason. 
  Calcul = ((340 m / sec) x (100 cm / m) x (1 sec / 1 000 000 us)) / 2 = 0.017 cm / us
  (facteur de conversion) ou (1 / 0.017) = 58.8235294. Donc, distance en cm = temps
  echo en us / 58.82.
*/

ISR(TIMER2_OVF_vect) {
  PORTD &= 0b11110111;  // Clear TRIG pour 2us.
  for (j = 0; j < 50; j++) {
    asm("");
  }
  PORTD |= 0b00001000;  // Set TRIG pour 10us.
  for (j = 0; j < 175; j++) {
    asm("");
  }
  PORTD &= 0b11110111;  // Clear TRIG et lire ECHO.
  ulDistance = pulseIn(2, 0x1);
  uiDist = (uint16_t)(ulDistance / 58);  // Donne la distance en cm.
}
