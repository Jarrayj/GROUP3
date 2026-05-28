// ════════════════════════════════════════════════════════════════
// M1_Rampe_v15.ino - Suivi de ligne propre après demi-tour
// Corrections v15 :
//   1. Suppression de sensMarche (le demi-tour physique suffit)
//   2. Désactivation de la détection ultrason après le demi-tour
//      → le robot suit la ligne jusqu'à la fin sans nouvelle séquence rampe
// ════════════════════════════════════════════════════════════════


#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>


// ── Adresses I2C ─────────────────────────────────────────────
#define MOTEUR_A            0x66
#define MOTEUR_B            0x68
#define LINE_FOLLOWER_ADDR  0x20


// ── Directions ───────────────────────────────────────────────
#define ARRET    0x00
#define AVANT    0x01
#define ARRIERE  0x02


// ── Pins ─────────────────────────────────────────────────────
#define PIN_US   7
#define PIN_LED  4
#define NB_LEDS  30


// ── Paramètres conduite ───────────────────────────────────────
#define VITESSE_NORMALE     40
#define VITESSE_VIRAGE      27
#define VITESSE_MIN          5
#define VITESSE_PIVOT       20
#define Kp  6.0f
#define Kd  15.0f


int SEUIL_NOIR = 50;
float adj_A = 1.0f, adj_B = 0.91f;


// ── Paramètres perte de ligne ─────────────────────────────────
#define TIMEOUT_RECHERCHE   2500
#define DELAI_VIRAGE         400


// ── Paramètres rampe ──────────────────────────────────────────
#define DIST_RAMPE_DETECTE   80
#define DIST_RAMPE_STOP      20
#define DUREE_ATTENTE_MUR   3000
#define FREQ_CLIGNOT_MS      500


// ── Demi-tour ─────────────────────────────────────────────────
#define VITESSE_DEMI_TOUR    20
#define DUREE_PIVOT_MS      2500
#define TIMEOUT_AFFINAGE    1500


// ── Seuils couleur ────────────────────────────────────────────
#define SEUIL_TOTAL_COULEUR   5
#define SEUIL_DOMINANCE    0.03f
#define DIST_LECTURE_MAX     25


// ── Variables PD ──────────────────────────────────────────────
float         erreurPrecedente        = 0.0f;
float         derniereDirVirage       = 0.0f;
unsigned long tempsPerduLigne         = 0;
bool          lignePrecedemmentPerdue = false;


// ── Flag : rampe déjà effectuée → plus de détection ultrason ──
bool rampeEffectuee = false;   // ← NOUVEAU


// ── États ────────────────────────────────────────────────────
enum Etat { ATTENTE_LIGNE, SUIVI_LIGNE, SEQUENCE_RAMPE };
Etat etatCourant = ATTENTE_LIGNE;


// ── Périphériques ─────────────────────────────────────────────
Adafruit_NeoPixel leds(NB_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_16X);
bool tcsDisponible = false;
uint8_t couleurR = 0, couleurG = 0, couleurB = 0;


// ════════════════════════════════════════════════════════════════
//  MOTEURS
// ════════════════════════════════════════════════════════════════
void piloterMoteur(byte adresse, byte direction, byte vitesse, float adj = 1.0f) {
 if (adresse == MOTEUR_B) {
   if      (direction == AVANT)   direction = ARRIERE;
   else if (direction == ARRIERE) direction = AVANT;
 }
 int v = constrain((int)(vitesse * adj), 0, 63);
 Wire.beginTransmission(adresse);
 Wire.write(0x00);
 Wire.write(((byte)v << 2) | direction);
 Wire.endTransmission();
}


void avancer(int vG, int vD) {
 piloterMoteur(MOTEUR_A, ARRIERE, (byte)constrain(vG, VITESSE_MIN, 63), adj_A);
 piloterMoteur(MOTEUR_B, ARRIERE, (byte)constrain(vD, VITESSE_MIN, 63), adj_B);
}
void avancerDroit(byte vitesse) { avancer(vitesse, vitesse); }


void pivoterGauche(byte vitesse) {
 piloterMoteur(MOTEUR_A, AVANT,   vitesse, adj_A);
 piloterMoteur(MOTEUR_B, ARRIERE, vitesse, adj_B);
}
void pivoterDroite(byte vitesse) {
 piloterMoteur(MOTEUR_A, ARRIERE, vitesse, adj_A);
 piloterMoteur(MOTEUR_B, AVANT,   vitesse, adj_B);
}


void stopper() { piloterMoteur(MOTEUR_A, ARRET, 0); piloterMoteur(MOTEUR_B, ARRET, 0); }


void pivotBrutDirect(byte v) {
 Wire.beginTransmission(MOTEUR_A);
 Wire.write(0x00);
 Wire.write(((byte)v << 2) | AVANT);
 Wire.endTransmission();
 Wire.beginTransmission(MOTEUR_B);
 Wire.write(0x00);
 Wire.write(((byte)v << 2) | AVANT);
 Wire.endTransmission();
}


// ════════════════════════════════════════════════════════════════
//  ULTRASON
// ════════════════════════════════════════════════════════════════
float mesureUS() {
 pinMode(PIN_US, OUTPUT);
 digitalWrite(PIN_US, LOW);  delayMicroseconds(2);
 digitalWrite(PIN_US, HIGH); delayMicroseconds(10);
 digitalWrite(PIN_US, LOW);
 pinMode(PIN_US, INPUT);
 long dur = pulseIn(PIN_US, HIGH, 30000UL);
 return dur == 0 ? 999.0f : dur * 0.0343f / 2.0f;
}


// ════════════════════════════════════════════════════════════════
//  CAPTEUR LIGNE
// ════════════════════════════════════════════════════════════════
uint8_t lireRegistre(byte reg) {
 Wire.beginTransmission(LINE_FOLLOWER_ADDR);
 Wire.write(reg);
 Wire.endTransmission(false);
 Wire.requestFrom(LINE_FOLLOWER_ADDR, 1);
 return Wire.available() ? Wire.read() : 0xFF;
}


bool ligneDetectee() {
 return (lireRegistre(0x01) < SEUIL_NOIR || lireRegistre(0x02) < SEUIL_NOIR ||
         lireRegistre(0x03) < SEUIL_NOIR || lireRegistre(0x04) < SEUIL_NOIR);
}


void resetSuivi() {
 erreurPrecedente        = 0.0f;
 derniereDirVirage       = 0.0f;
 tempsPerduLigne         = 0;
 lignePrecedemmentPerdue = false;
}


// ════════════════════════════════════════════════════════════════
//  LEDs
// ════════════════════════════════════════════════════════════════
void allumerLEDs(uint8_t r, uint8_t g, uint8_t b) {
 for (int i = 0; i < NB_LEDS; i++) leds.setPixelColor(i, leds.Color(r, g, b));
 leds.show();
}
void eteindreLEDs() { leds.clear(); leds.show(); }


// ════════════════════════════════════════════════════════════════
//  ANALYSE COULEUR
// ════════════════════════════════════════════════════════════════
void analyserCouleurMur() {
 couleurR = couleurG = couleurB = 128;


 float dist = mesureUS();
 Serial.print(F("[TCS] Distance mur : ")); Serial.print(dist, 1); Serial.println(F(" cm"));
 if (dist > DIST_LECTURE_MAX) {
   Serial.println(F("[TCS] Trop loin du mur — lecture annulee"));
   return;
 }


 tcsDisponible = tcs.begin();
 if (!tcsDisponible) {
   Serial.println(F("[TCS] Non disponible"));
   return;
 }


 allumerLEDs(255, 255, 255);
 delay(150);


 uint32_t sumR = 0, sumG = 0, sumB = 0, sumC = 0;
 uint16_t r, g, b, c;
 const int NB_MESURES = 5;
 for (int i = 0; i < NB_MESURES; i++) {
   tcs.getRawData(&r, &g, &b, &c);
   sumR += r; sumG += g; sumB += b; sumC += c;
   delay(110);
 }
 r = sumR / NB_MESURES;
 g = sumG / NB_MESURES;
 b = sumB / NB_MESURES;
 c = sumC / NB_MESURES;


 eteindreLEDs();


 Serial.print(F("[TCS] R=")); Serial.print(r);
 Serial.print(F(" G="));     Serial.print(g);
 Serial.print(F(" B="));     Serial.print(b);
 Serial.print(F(" C="));     Serial.println(c);


 float total = r + g + b;


 if (total < SEUIL_TOTAL_COULEUR) {
   Serial.println(F("[TCS] Signal tres faible — fallback brut"));
   if      (b > r && b > g) { couleurR=0;   couleurG=0;   couleurB=255; Serial.println(F("BLEU (brut)")); }
   else if (g > r && g > b) { couleurR=0;   couleurG=255; couleurB=0;   Serial.println(F("VERT (brut)")); }
   else                      { couleurR=255; couleurG=0;   couleurB=0;   Serial.println(F("ROUGE (brut)")); }
   return;
 }


 float rn = r / total, gn = g / total, bn = b / total;
 float maxc   = max(rn, max(gn, bn));
 float minc   = min(rn, min(gn, bn));
 float second = rn + gn + bn - maxc - minc;


 if (maxc - second < SEUIL_DOMINANCE) {
   if      (b > r && b > g) { couleurR=0;   couleurG=0;   couleurB=255; Serial.println(F("BLEU (ambig)")); }
   else if (g > r && g > b) { couleurR=0;   couleurG=255; couleurB=0;   Serial.println(F("VERT (ambig)")); }
   else                      { couleurR=255; couleurG=0;   couleurB=0;   Serial.println(F("ROUGE (ambig)")); }
   return;
 }


 if      (rn >= gn && rn >= bn) { couleurR=255; couleurG=0;   couleurB=0;   Serial.println(F("ROUGE")); }
 else if (gn >= rn && gn >= bn) { couleurR=0;   couleurG=255; couleurB=0;   Serial.println(F("VERT")); }
 else                            { couleurR=0;   couleurG=0;   couleurB=255; Serial.println(F("BLEU")); }
}


// ════════════════════════════════════════════════════════════════
//  DEMI-TOUR
// ════════════════════════════════════════════════════════════════
void demiTourSuiveur() {
 Serial.println(F("[DEMI-TOUR] Phase 1 : pivot fixe brut"));


 pivotBrutDirect(VITESSE_DEMI_TOUR);
 delay(DUREE_PIVOT_MS);
 stopper();
 delay(150);


 Serial.println(F("[DEMI-TOUR] Phase 2 : affinage ligne"));


 unsigned long tDebut = millis();
 pivotBrutDirect(VITESSE_DEMI_TOUR);
 while (millis() - tDebut < TIMEOUT_AFFINAGE) {
   if (ligneDetectee()) {
     delay(8);
     if (ligneDetectee()) {
       stopper();
       Serial.println(F("[DEMI-TOUR] Ligne confirmee — arret"));
       delay(100);
       return;
     }
   }
   delay(8);
 }


 stopper();
 Serial.println(F("[DEMI-TOUR] Termine (timeout affinage)"));
 delay(100);
}


// ════════════════════════════════════════════════════════════════
//  SÉQUENCE RAMPE
// ════════════════════════════════════════════════════════════════
void sequenceRampe() {
 Serial.println(F("[RAMPE] Declenchement sequence"));


 // Avancer vers le mur
 while (true) {
   float dist = mesureUS();
   if (dist <= DIST_RAMPE_STOP || dist >= 990.0f) break;
   avancerDroit(VITESSE_NORMALE);
   delay(20);
 }
 stopper();
 delay(300);


 // Analyse couleur
 analyserCouleurMur();


 // Clignotement 3s
 Serial.println(F("[RAMPE] Clignotement 3s..."));
 unsigned long tDebut = millis();
 while (millis() - tDebut < DUREE_ATTENTE_MUR) {
   allumerLEDs(couleurR, couleurG, couleurB);
   delay(FREQ_CLIGNOT_MS);
   eteindreLEDs();
   delay(FREQ_CLIGNOT_MS);
 }


 // Nettoyage
 eteindreLEDs();
 tcsDisponible = false;
 couleurR = couleurG = couleurB = 0;


 // Demi-tour
 demiTourSuiveur();


 // ✅ Marquer la rampe comme effectuée → plus de détection ultrason
 rampeEffectuee = true;
 // ✅ Pas d'inversion de sensMarche : le demi-tour physique suffit,
 //    la correction PD reste identique.


 // Reset des variables PD
 resetSuivi();


 Serial.println(F("[RAMPE] Reprise du suivi de ligne normale"));
}


// ════════════════════════════════════════════════════════════════
//  SUIVI DE LIGNE — correction PD identique aller/retour
// ════════════════════════════════════════════════════════════════
void suivreLigne() {
 uint8_t eg = lireRegistre(0x01);
 uint8_t cg = lireRegistre(0x02);
 uint8_t cd = lireRegistre(0x03);
 uint8_t ed = lireRegistre(0x04);


 bool n_EG = (eg < SEUIL_NOIR);
 bool n_CG = (cg < SEUIL_NOIR);
 bool n_CD = (cd < SEUIL_NOIR);
 bool n_ED = (ed < SEUIL_NOIR);


 bool ligneVue = n_EG || n_CG || n_CD || n_ED;
 bool toutNoir = n_EG && n_CG && n_CD && n_ED;


 if (toutNoir) {
   stopper();
   Serial.println(F("Ligne complete detectee : ARRET !"));
   return;
 }


 if (ligneVue) {
   lignePrecedemmentPerdue = false;
   tempsPerduLigne = 0;


   if      (n_ED || n_CD) derniereDirVirage =  1.0f;
   else if (n_EG || n_CG) derniereDirVirage = -1.0f;


   float erreur = 0;
   int total = (n_EG?1:0) + (n_CG?1:0) + (n_CD?1:0) + (n_ED?1:0);
   if (total > 0) {
     erreur = ((n_EG ? -2.0f : 0) + (n_CG ? -1.0f : 0) +
               (n_CD ?  1.0f : 0) + (n_ED ?  2.0f : 0)) / total;
   }


   float derivee    = erreur - erreurPrecedente;
   float correction = (Kp * erreur) + (Kd * derivee);
   erreurPrecedente = erreur;


   int vitesseBase;
   if (n_CG || n_CD) {
     vitesseBase = VITESSE_NORMALE;
   } else {
     vitesseBase = (abs(correction) > 8) ? VITESSE_VIRAGE : VITESSE_NORMALE;
   }


   int vG = constrain((int)(vitesseBase + correction), VITESSE_MIN, 63);
   int vD = constrain((int)(vitesseBase - correction), VITESSE_MIN, 63);


   avancer(vG, vD);


 } else {
   if (!lignePrecedemmentPerdue) {
     tempsPerduLigne = millis();
     lignePrecedemmentPerdue = true;
   }


   unsigned long depuisPerte = millis() - tempsPerduLigne;


   if (depuisPerte < DELAI_VIRAGE) {
     if (derniereDirVirage > 0)      avancer(VITESSE_VIRAGE, VITESSE_MIN);
     else if (derniereDirVirage < 0) avancer(VITESSE_MIN, VITESSE_VIRAGE);
     else                            avancer(VITESSE_VIRAGE, VITESSE_VIRAGE);
   }
   else if (depuisPerte < TIMEOUT_RECHERCHE) {
     if (derniereDirVirage >= 0) pivoterDroite(VITESSE_PIVOT);
     else                        pivoterGauche(VITESSE_PIVOT);
   }
   else {
     stopper();
   }
 }
}


// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
 Wire.begin(); Serial.begin(9600);
 leds.begin(); leds.setBrightness(80); eteindreLEDs();
 stopper(); delay(300);


 Serial.println(F("Valeurs capteurs ligne :"));
 Serial.print(F("  EG=")); Serial.println(lireRegistre(0x01));
 Serial.print(F("  CG=")); Serial.println(lireRegistre(0x02));
 Serial.print(F("  CD=")); Serial.println(lireRegistre(0x03));
 Serial.print(F("  ED=")); Serial.println(lireRegistre(0x04));


 Serial.println(F("En attente de la ligne..."));
 rampeEffectuee = false;
}


// ════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════
void loop() {
 switch (etatCourant) {


   case ATTENTE_LIGNE:
     if (ligneDetectee()) {
       Serial.println(F("Ligne detectee — depart !"));
       resetSuivi(); delay(200);
       etatCourant = SUIVI_LIGNE;
     }
     break;


   case SUIVI_LIGNE: {
     // ✅ Ultrason désactivé une fois la rampe effectuée
     if (!rampeEffectuee) {
       static unsigned long tUS = 0;
       if (millis() - tUS > 50) {
         tUS = millis();
         float dist = mesureUS();
         if (dist > 0 && dist < DIST_RAMPE_DETECTE) {
           Serial.print(F("[US] Mur a ")); Serial.print(dist, 1); Serial.println(F(" cm"));
           etatCourant = SEQUENCE_RAMPE;
           break;
         }
       }
     }
     suivreLigne();
     break;
   }


   case SEQUENCE_RAMPE:
     sequenceRampe();
     etatCourant = SUIVI_LIGNE;
     break;
 }
 delay(5);
}



