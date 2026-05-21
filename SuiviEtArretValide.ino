// ═══════════════════════════════════════════════════════════════
// MoteurEtSuiveur_v9.ino - Suivi de ligne + Arrêt immédiat full noir
// ═══════════════════════════════════════════════════════════════


#include <Wire.h>


// ── Adresses I2C ─────────────────────────────────────────────
#define MOTEUR_A            0x66
#define MOTEUR_B            0x68
#define LINE_FOLLOWER_ADDR  0x20


// ── Directions ───────────────────────────────────────────────
#define ARRET    0x00
#define AVANT    0x01
#define ARRIERE  0x02


// ── Paramètres conduite ───────────────────────────────────────
#define VITESSE_NORMALE     39
#define VITESSE_VIRAGE      26
#define VITESSE_MIN          5


int SEUIL_NOIR = 15;


// ── Gains PD ─────────────────────────────────────────────────
#define Kp  5.0f
#define Kd  10.0f


// ── Coefficients d'équilibrage ────────────────────────────────
float adj_A = 1.0f;
float adj_B = 0.91f;


float erreurPrecedente = 0.0f;


// ── Flag arrêt définitif ──────────────────────────────────────
bool arretDefinitif = false;


// ═══════════════════════════════════════════════════════════════
//  PILOTAGE MOTEURS
// ═══════════════════════════════════════════════════════════════
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




void stopper() {
 piloterMoteur(MOTEUR_A, ARRET, 0);
 piloterMoteur(MOTEUR_B, ARRET, 0);
}


// ═══════════════════════════════════════════════════════════════
//  LECTURE CAPTEUR LIGNE
// ═══════════════════════════════════════════════════════════════
uint8_t lireRegistre(byte reg) {
 Wire.beginTransmission(LINE_FOLLOWER_ADDR);
 Wire.write(reg);
 Wire.endTransmission(false);
 Wire.requestFrom(LINE_FOLLOWER_ADDR, 1);
 return Wire.available() ? Wire.read() : 0xFF;
}


// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
 Wire.begin();
 Serial.begin(9600);
 delay(500);


 Serial.println(F("=== MoteurEtSuiveur v9 (Suivi + Arret full noir) ==="));
 delay(5000);


 stopper();
 Serial.print(F("SEUIL actif = ")); Serial.println(SEUIL_NOIR);
 Serial.println(F("Pret !"));
 delay(500);
}


// ═══════════════════════════════════════════════════════════════
//  LOOP PRINCIPALE
// ═══════════════════════════════════════════════════════════════
void loop() {


 // ── Arrêt définitif : attente commande 'R' pour reprendre ────
 if (arretDefinitif) {
   stopper();
   if (Serial.available() > 0) {
     char cmd = Serial.read();
     if (cmd == 'R' || cmd == 'r') {
       arretDefinitif = false;
       erreurPrecedente = 0.0f;
       Serial.println(F("Commande R reçue : reprise !"));
       delay(500);
     }
   }
   return;
 }


 // ── Lecture capteurs ─────────────────────────────────────────
 uint8_t eg = lireRegistre(0x01);
 uint8_t cg = lireRegistre(0x02);
 uint8_t cd = lireRegistre(0x03);
 uint8_t ed = lireRegistre(0x04);


 bool n_EG = (eg < SEUIL_NOIR);
 bool n_CG = (cg < SEUIL_NOIR);
 bool n_CD = (cd < SEUIL_NOIR);
 bool n_ED = (ed < SEUIL_NOIR);


 bool ligneVue = n_EG || n_CG || n_CD || n_ED;
 bool toutNoir  = n_EG && n_CG && n_CD && n_ED;


 // ── CAS 0 : Full noir → ARRÊT DÉFINITIF ─────────────────────
 if (toutNoir) {
   stopper();
   arretDefinitif = true;
   Serial.println(F("Full noir detecte : ARRET DEFINITIF !"));
   return;
 }


 // ── CAS 1 : Ligne visible ────────────────────────────────────
 if (ligneVue) {
   float erreur = 0;
   int total = (n_EG?1:0) + (n_CG?1:0) + (n_CD?1:0) + (n_ED?1:0);
   if (total > 0) {
     erreur = ((n_EG ? -2.0f : 0) + (n_CG ? -0.5f : 0) + (n_CD ? 0.5f : 0) + (n_ED ? 2.0f : 0)) / total;
   }


   float derivee    = erreur - erreurPrecedente;
   float correction = (Kp * erreur) + (Kd * derivee);
   erreurPrecedente = erreur;


   int vitesseBase = (n_CG || n_CD) ? VITESSE_NORMALE
                   : (abs(correction) > 8) ? VITESSE_VIRAGE
                   : VITESSE_NORMALE;


   int vG = constrain((int)(vitesseBase + correction), VITESSE_MIN, 63);
   int vD = constrain((int)(vitesseBase - correction), VITESSE_MIN, 63);


   avancer(vG, vD);


 // ── CAS 2 : Ligne perdue → stopper ──────────────────────────
 } else {
   stopper();
 }


 delay(5);
}



