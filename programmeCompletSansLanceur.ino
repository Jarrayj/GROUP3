//═══════════════════════════════════════════════════════════════════════
// parcours_complet_v3.ino
// Sections : départ (4IR) → tunnel → évitement O1+O2 → couleur/rampe → arrêt final
// ═══════════════════════════════════════════════════════════════════════


#include <Wire.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>


// ── Adresses I2C ────────────────────────────────────────────────────────
#define MOTEUR_A           0x66
#define MOTEUR_B           0x68
#define LINE_FOLLOWER_ADDR 0x20


// ── Directions ──────────────────────────────────────────────────────────
#define ARRET   0x00
#define AVANT   0x01
#define ARRIERE 0x02


// ── Pins ────────────────────────────────────────────────────────────────
#define PIN_US    7
#define PIN_SERVO 6
#define PIN_LED   4
#define NB_LEDS  30


// ── Coefficients moteurs ─────────────────────────────────────────────────
float adj_A = 1.0f;
float adj_B = 0.91f;


// ── Seuil noir (mis à jour à chaque transition) ──────────────────────────
int SEUIL_NOIR = 35;


// ── Gains PD communs ─────────────────────────────────────────────────────
#define Kp_COMMUN  5.0f
#define Kd_COMMUN 10.0f
#define Kp_COULEUR  6.0f
#define Kd_COULEUR 15.0f


// ── Vitesses ─────────────────────────────────────────────────────────────
#define VITESSE_NORMALE     46
#define VITESSE_RAMPE       58   // ← vitesse élevée pour monter la rampe avec la coque
#define VITESSE_VIRAGE      26
#define VITESSE_MIN          5
#define VITESSE_PIVOT       20
#define VITESSE_TUNNEL      37


// ── Paramètres perte de ligne ────────────────────────────────────────────
#define TIMEOUT_RECHERCHE 2500
#define DELAI_VIRAGE       400


// ── Variables PD partagées ───────────────────────────────────────────────
float         erreurPrecedente        = 0.0f;
float         derniereDirVirage       = 0.0f;
unsigned long tempsPerduLigne         = 0;
bool          lignePrecedemmentPerdue = false;


// ═══════════════════════════════════════════════════════════════════════
//  MACHINE À ÉTATS PRINCIPALE (M1)
// ═══════════════════════════════════════════════════════════════════════
enum PhaseGlobale {
PHASE_ATTENTE_4IR,
PHASE_TUNNEL,
PHASE_EVITEMENT,
PHASE_COULEUR,
PHASE_ARRET_FINAL
};
PhaseGlobale phaseGlobale = PHASE_ATTENTE_4IR;


// ── Sous-états tunnel ─────────────────────────────────────────────────
enum ModeTunnel { TUNNEL_SUIVI, TUNNEL_NAVIGATION };
ModeTunnel modeTunnel = TUNNEL_SUIVI;


// ── Sous-états couleur ────────────────────────────────────────────────
enum EtatCouleur { ATTENTE_LIGNE, SUIVI_LIGNE_C, SEQUENCE_RAMPE };
EtatCouleur etatCouleur = ATTENTE_LIGNE;


// ═══════════════════════════════════════════════════════════════════════
//  VARIABLES — TUNNEL
// ═══════════════════════════════════════════════════════════════════════
#define ANGLE_CENTRE      90
#define ANGLE_MUR_DROIT   45
#define DIST_CIBLE_MUR     3.0f
#define Kp_MUR             2.0f


Servo monServo;
unsigned long tempsSortieTunnel = 0;
bool          lignePerdueAvantTunnel = false;
unsigned long tempsLignePerdueT      = 0;


// ═══════════════════════════════════════════════════════════════════════
//  VARIABLES — ÉVITEMENT
// ═══════════════════════════════════════════════════════════════════════
#define DIST_DETECT         28
#define DIST_FREIN          32
#define VITESSE_EVIT_AVANCE 38
#define VITESSE_PIVOT_EVIT  18
#define NB_CONFIRM_OBSTACLE  3
#define TEMPS_STOP_MS       2000
#define TEMPS_PIVOT_90      1600
#define TEMPS_PIVOT_D       2000
#define TEMPS_PIVOT_900     1700
#define TEMPS_PIVOT_100     1990
#define TEMPS_AVANCE_400CM  1500
#define TEMPS_AVANCE_4000CM 3400
#define TEMPS_AVANCE_500CM  3900
#define TEMPS_AVANCE_50CM   3654
#define PAUSE_ETAPE          50
#define MAX_RECHERCHE_CM      60
#define MS_PER_CM_ESTIMATE   110
#define EVICTION_COOLDOWN_MS 3000


bool          evitementActif   = false;
int           compteurObstacle = 0;
int           obstacleIndex    = 0;
unsigned long lastEvasionMs    = 0;


// ═══════════════════════════════════════════════════════════════════════
//  VARIABLES — COULEUR
// ═══════════════════════════════════════════════════════════════════════
#define DIST_RAMPE_DETECTE   80
#define DIST_RAMPE_STOP      20
#define DUREE_ATTENTE_MUR   3000
#define FREQ_CLIGNOT_MS      500
#define VITESSE_DEMI_TOUR    20
#define DUREE_PIVOT_MS      2500
#define TIMEOUT_AFFINAGE    1500
#define SEUIL_TOTAL_COULEUR   5
#define SEUIL_DOMINANCE    0.03f
#define DIST_LECTURE_MAX     25
#define DUREE_CONFIRM_FIN_MS 80


bool          rampeEffectuee  = false;
unsigned long tDebut_toutNoir = 0;
bool          toutNoirEnCours = false;


Adafruit_NeoPixel leds(NB_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_16X);
bool    tcsDisponible = false;
uint8_t couleurR = 0, couleurG = 0, couleurB = 0;


// ═══════════════════════════════════════════════════════════════════════
//  FONCTIONS COMMUNES — MOTEURS (M11)
// ═══════════════════════════════════════════════════════════════════════
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


void stopper() {
piloterMoteur(MOTEUR_A, ARRET, 0);
piloterMoteur(MOTEUR_B, ARRET, 0);
}


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


void decelerationProgressive(unsigned long dureeMs) {
unsigned long tDebut = millis();
while (millis() - tDebut < dureeMs) {
  float ratio = 1.0f - (float)(millis() - tDebut) / dureeMs;
  int v = max((int)(VITESSE_NORMALE * ratio), VITESSE_MIN);
  avancer(v, v);
  delay(20);
}
stopper();
}


// ═══════════════════════════════════════════════════════════════════════
//  FONCTIONS COMMUNES — CAPTEURS (M11)
// ═══════════════════════════════════════════════════════════════════════
uint8_t lireRegistre(byte reg) {
Wire.beginTransmission(LINE_FOLLOWER_ADDR);
Wire.write(reg);
Wire.endTransmission(false);
Wire.requestFrom(LINE_FOLLOWER_ADDR, 1);
return Wire.available() ? Wire.read() : 0xFF;
}


float mesureUS() {
pinMode(PIN_US, OUTPUT);
digitalWrite(PIN_US, LOW);  delayMicroseconds(2);
digitalWrite(PIN_US, HIGH); delayMicroseconds(10);
digitalWrite(PIN_US, LOW);
pinMode(PIN_US, INPUT);
long dur = pulseIn(PIN_US, HIGH, 30000UL);
return (dur == 0) ? 999.0f : dur * 0.0343f / 2.0f;
}


bool ligneDetectee() {
return (lireRegistre(0x01) < SEUIL_NOIR ||
        lireRegistre(0x02) < SEUIL_NOIR ||
        lireRegistre(0x03) < SEUIL_NOIR ||
        lireRegistre(0x04) < SEUIL_NOIR);
}


bool toutNoirDetecte() {
return (lireRegistre(0x01) < SEUIL_NOIR &&
        lireRegistre(0x02) < SEUIL_NOIR &&
        lireRegistre(0x03) < SEUIL_NOIR &&
        lireRegistre(0x04) < SEUIL_NOIR);
}


void resetSuivi() {
erreurPrecedente        = 0.0f;
derniereDirVirage       = 0.0f;
tempsPerduLigne         = 0;
lignePrecedemmentPerdue = false;
}


// ═══════════════════════════════════════════════════════════════════════
//  FONCTIONS COMMUNES — LEDs (M11)
// ═══════════════════════════════════════════════════════════════════════
void allumerLEDs(uint8_t r, uint8_t g, uint8_t b) {
for (int i = 0; i < NB_LEDS; i++) leds.setPixelColor(i, leds.Color(r, g, b));
leds.show();
}
void eteindreLEDs() { leds.clear(); leds.show(); }


// ═══════════════════════════════════════════════════════════════════════
//  SUIVI DE LIGNE PD — paramétrable (M11)
//  ignorerToutNoir : true = toutNoir n'arrête pas (phases tunnel/évitement)
//  arretSurPerte   : true = stopper si ligne perdue (phase arrêt final)
// ═══════════════════════════════════════════════════════════════════════
void suivreLignePD(float kp, float kd, bool ignorerToutNoir, bool arretSurPerte) {
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


if (toutNoir && !ignorerToutNoir) {
  stopper();
  return;
}


if (ligneVue && !toutNoir) {
  lignePrecedemmentPerdue = false;
  tempsPerduLigne         = 0;
  if      (n_ED || n_CD) derniereDirVirage =  1.0f;
  else if (n_EG || n_CG) derniereDirVirage = -1.0f;


  int   total  = (n_EG?1:0)+(n_CG?1:0)+(n_CD?1:0)+(n_ED?1:0);
  float erreur = 0;
  if (total > 0)
    erreur = ((n_EG?-2.0f:0)+(n_CG?-0.5f:0)+(n_CD?0.5f:0)+(n_ED?2.0f:0)) / total;


  float correction = (kp * erreur) + (kd * (erreur - erreurPrecedente));
  erreurPrecedente = erreur;


  int vitesseBase = (n_CG || n_CD) ? VITESSE_NORMALE
                  : (abs(correction) > 8) ? VITESSE_VIRAGE
                  : VITESSE_NORMALE;


  avancer(constrain((int)(vitesseBase + correction), VITESSE_MIN, 63),
          constrain((int)(vitesseBase - correction), VITESSE_MIN, 63));


} else if (!ligneVue) {
  if (!lignePrecedemmentPerdue) {
    tempsPerduLigne         = millis();
    lignePrecedemmentPerdue = true;
  }
  if (arretSurPerte) {
    stopper();
    return;
  }
  unsigned long dep = millis() - tempsPerduLigne;
  if (dep < DELAI_VIRAGE) {
    if      (derniereDirVirage > 0) avancer(VITESSE_VIRAGE, VITESSE_MIN);
    else if (derniereDirVirage < 0) avancer(VITESSE_MIN,    VITESSE_VIRAGE);
    else                            avancer(VITESSE_VIRAGE,  VITESSE_VIRAGE);
  } else if (dep < TIMEOUT_RECHERCHE) {
    if (derniereDirVirage >= 0) pivoterDroite(VITESSE_PIVOT);
    else                        pivoterGauche(VITESSE_PIVOT);
  } else {
    stopper();
  }
}
}


// ═══════════════════════════════════════════════════════════════════════
//  TUNNEL — navigation mur droit
// ═══════════════════════════════════════════════════════════════════════
void naviguerTunnel() {
float distMur = mesureUS();
if (distMur > 30.0f) distMur = DIST_CIBLE_MUR;
int correction = constrain((int)((distMur - DIST_CIBLE_MUR) * Kp_MUR), -25, 25);
avancer(constrain(VITESSE_TUNNEL - correction, VITESSE_MIN, 63),
        constrain(VITESSE_TUNNEL + correction, VITESSE_MIN, 63));
}


// ═══════════════════════════════════════════════════════════════════════
//  ÉVITEMENT — séquence contournement
// ═══════════════════════════════════════════════════════════════════════
bool obstacleDevant() {
float d = mesureUS();
if (d > 0 && d < DIST_DETECT) {
  compteurObstacle++;
  if (compteurObstacle >= NB_CONFIRM_OBSTACLE) { compteurObstacle = NB_CONFIRM_OBSTACLE; return true; }
} else { compteurObstacle = 0; }
return false;
}


void avancerJusquARetrouverLigne(unsigned long maxMs) {
unsigned long t0 = millis();
avancerDroit(VITESSE_EVIT_AVANCE);
while (millis() - t0 < maxMs) {
  if (lireRegistre(0x01) < SEUIL_NOIR || lireRegistre(0x02) < SEUIL_NOIR ||
      lireRegistre(0x03) < SEUIL_NOIR || lireRegistre(0x04) < SEUIL_NOIR) {
    stopper(); return;
  }
  delay(20);
}
stopper();
}


void evasionSequence(bool contourGauche) {
Serial.print(F("[EVIT] Obstacle ")); Serial.print(obstacleIndex + 1);
Serial.println(contourGauche ? F(" — GAUCHE (O1)") : F(" — DROITE (O2)"));


decelerationProgressive(400);
delay(TEMPS_STOP_MS);


if (contourGauche) {
  // O1 : contournement par la gauche (PDF section 3)
  pivoterGauche(VITESSE_PIVOT_EVIT); delay(TEMPS_PIVOT_90);  stopper(); delay(PAUSE_ETAPE);
  avancerDroit(VITESSE_EVIT_AVANCE); delay(TEMPS_AVANCE_400CM); stopper(); delay(PAUSE_ETAPE);
  pivoterDroite(VITESSE_PIVOT_EVIT); delay(TEMPS_PIVOT_D);  stopper(); delay(PAUSE_ETAPE);
  avancerDroit(VITESSE_EVIT_AVANCE); delay(TEMPS_AVANCE_4000CM); stopper(); delay(PAUSE_ETAPE);
  pivoterDroite(VITESSE_PIVOT_EVIT); delay(TEMPS_PIVOT_900);  stopper(); delay(PAUSE_ETAPE);
} else {
  // O2 : contournement par la droite (PDF section 4)
  pivoterDroite(VITESSE_PIVOT_EVIT); delay(TEMPS_PIVOT_D);  stopper(); delay(PAUSE_ETAPE);
  avancerDroit(VITESSE_EVIT_AVANCE); delay(TEMPS_AVANCE_500CM); stopper(); delay(PAUSE_ETAPE);
  pivoterGauche(VITESSE_PIVOT_EVIT); delay(TEMPS_PIVOT_100); stopper(); delay(PAUSE_ETAPE);
  avancerDroit(VITESSE_EVIT_AVANCE); delay(TEMPS_AVANCE_50CM);  stopper(); delay(PAUSE_ETAPE);
}


avancerJusquARetrouverLigne((unsigned long)(MAX_RECHERCHE_CM * MS_PER_CM_ESTIMATE));
resetSuivi();
compteurObstacle = 0;
Serial.println(F("[EVIT] Fin sequence, reprise suivi"));
}


// ═══════════════════════════════════════════════════════════════════════
//  COULEUR — analyse TCS34725
// ═══════════════════════════════════════════════════════════════════════
void analyserCouleurMur() {
couleurR = couleurG = couleurB = 128;
float dist = mesureUS();
if (dist > DIST_LECTURE_MAX) { Serial.println(F("[TCS] Trop loin")); return; }
tcsDisponible = tcs.begin();
if (!tcsDisponible) { Serial.println(F("[TCS] Non dispo")); return; }


allumerLEDs(255, 255, 255); delay(150);
uint32_t sumR=0, sumG=0, sumB=0, sumC=0;
uint16_t r, g, b, c;
for (int i = 0; i < 5; i++) { tcs.getRawData(&r,&g,&b,&c); sumR+=r; sumG+=g; sumB+=b; sumC+=c; delay(110); }
r=sumR/5; g=sumG/5; b=sumB/5; c=sumC/5;
eteindreLEDs();


float total = r + g + b;
if (total < SEUIL_TOTAL_COULEUR) {
  if      (b>r&&b>g) { couleurR=0;   couleurG=0;   couleurB=255; Serial.println(F("BLEU(brut)")); }
  else if (g>r&&g>b) { couleurR=0;   couleurG=255; couleurB=0;   Serial.println(F("VERT(brut)")); }
  else               { couleurR=255; couleurG=0;   couleurB=0;   Serial.println(F("ROUGE(brut)")); }
  return;
}
float rn=r/total, gn=g/total, bn=b/total;
float maxc=max(rn,max(gn,bn)), minc=min(rn,min(gn,bn)), second=rn+gn+bn-maxc-minc;
if (maxc-second < SEUIL_DOMINANCE) {
  if      (b>r&&b>g) { couleurR=0;   couleurG=0;   couleurB=255; Serial.println(F("BLEU(ambig)")); }
  else if (g>r&&g>b) { couleurR=0;   couleurG=255; couleurB=0;   Serial.println(F("VERT(ambig)")); }
  else               { couleurR=255; couleurG=0;   couleurB=0;   Serial.println(F("ROUGE(ambig)")); }
  return;
}
if      (rn>=gn&&rn>=bn) { couleurR=255; couleurG=0;   couleurB=0;   Serial.println(F("ROUGE")); }
else if (gn>=rn&&gn>=bn) { couleurR=0;   couleurG=255; couleurB=0;   Serial.println(F("VERT")); }
else                     { couleurR=0;   couleurG=0;   couleurB=255; Serial.println(F("BLEU")); }
}


// ═══════════════════════════════════════════════════════════════════════
//  COULEUR — demi-tour
// ═══════════════════════════════════════════════════════════════════════
void demiTourSuiveur() {
pivotBrutDirect(VITESSE_DEMI_TOUR); delay(DUREE_PIVOT_MS); stopper(); delay(150);
unsigned long tD = millis();
pivotBrutDirect(VITESSE_DEMI_TOUR);
while (millis() - tD < TIMEOUT_AFFINAGE) {
  if (ligneDetectee()) { delay(8); if (ligneDetectee()) { stopper(); delay(100); return; } }
  delay(8);
}
stopper(); delay(100);
}


// ═══════════════════════════════════════════════════════════════════════
//  COULEUR — séquence rampe
// ═══════════════════════════════════════════════════════════════════════
void sequenceRampe() {
Serial.println(F("[RAMPE] Debut"));
while (true) {
  float dist = mesureUS();
  if (dist <= DIST_RAMPE_STOP || dist >= 990.0f) break;
  avancerDroit(VITESSE_RAMPE); delay(20);   // ← VITESSE_RAMPE (58) au lieu de VITESSE_NORMALE (46)
}
stopper(); delay(300);
analyserCouleurMur();


unsigned long tD = millis();
while (millis() - tD < DUREE_ATTENTE_MUR) {
  allumerLEDs(couleurR, couleurG, couleurB); delay(FREQ_CLIGNOT_MS);
  eteindreLEDs(); delay(FREQ_CLIGNOT_MS);
}
eteindreLEDs();
tcsDisponible = false;
couleurR = couleurG = couleurB = 0;


demiTourSuiveur();
rampeEffectuee = true;
resetSuivi();
Serial.println(F("[RAMPE] Reprise suivi"));
}


// ═══════════════════════════════════════════════════════════════════════
//  TRANSITIONS
// ═══════════════════════════════════════════════════════════════════════
void entrerPhaseEvitement() {                          // M3
Serial.println(F("[PHASE] TUNNEL → EVITEMENT"));
monServo.write(ANGLE_CENTRE); delay(400);
SEUIL_NOIR     = 35;
evitementActif = true;
obstacleIndex  = 0;
lastEvasionMs  = 0;
resetSuivi();
phaseGlobale = PHASE_EVITEMENT;
}


void entrerPhaseCouleur() {                            // M5
Serial.println(F("[PHASE] EVITEMENT → COULEUR"));
evitementActif = false;
SEUIL_NOIR     = 50;
rampeEffectuee = false;
toutNoirEnCours = false;
etatCouleur    = ATTENTE_LIGNE;
resetSuivi();
phaseGlobale = PHASE_COULEUR;
}


void entrerPhaseArretFinal() {                         // M7
Serial.println(F("[PHASE] COULEUR → ARRET_FINAL"));
SEUIL_NOIR = 15;
resetSuivi();
phaseGlobale = PHASE_ARRET_FINAL;
}


// ═══════════════════════════════════════════════════════════════════════
//  SETUP (M10)
// ═══════════════════════════════════════════════════════════════════════
void setup() {
Wire.begin();
Serial.begin(9600);


monServo.attach(PIN_SERVO);
monServo.write(ANGLE_CENTRE);


leds.begin(); leds.setBrightness(80); eteindreLEDs();


stopper();
Serial.println(F("=== Parcours complet v3 ==="));
Serial.println(F("4IR → TUNNEL → EVITEMENT → COULEUR → ARRET_FINAL"));
delay(3000);
Serial.println(F("Pret — en attente 4IR..."));
}


// ═══════════════════════════════════════════════════════════════════════
//  LOOP PRINCIPALE
// ═══════════════════════════════════════════════════════════════════════
void loop() {

switch (phaseGlobale) {

  // ──────────────────────────────────────────────────────────────────
  case PHASE_ATTENTE_4IR: {
  // M1 — Départ : attend que les 4 capteurs IR voient la ligne de départ
  // ──────────────────────────────────────────────────────────────────
    if (toutNoirDetecte()) {
      Serial.println(F("[DEPART] 4IR detectes — GO !"));
      stopper();
      delay(200);
      SEUIL_NOIR   = 35;
      modeTunnel   = TUNNEL_SUIVI;
      lignePerdueAvantTunnel = false;
      tempsSortieTunnel      = 0;
      resetSuivi();
      phaseGlobale = PHASE_TUNNEL;
    }
    break;
  }

  // ──────────────────────────────────────────────────────────────────
  case PHASE_TUNNEL: {
  // M2 — Section 2 : suivi de ligne → détection perte → traversée tunnel
  // ──────────────────────────────────────────────────────────────────

    switch (modeTunnel) {

      case TUNNEL_SUIVI: {
        // Suivi de ligne normal — toutNoir ignoré (croisements)
        bool ligneVue = ligneDetectee();

        if (ligneVue) {
          lignePerdueAvantTunnel = false;
          tempsLignePerdueT      = 0;
          suivreLignePD(Kp_COMMUN, Kd_COMMUN, /*ignorerToutNoir=*/true, /*arretSurPerte=*/false);

        } else {
          // Ligne perdue → stopper et chronomètre
          stopper();
          if (!lignePerdueAvantTunnel) {
            lignePerdueAvantTunnel = true;
            tempsLignePerdueT      = millis();
            Serial.println(F("[TUNNEL] Ligne perdue — attente..."));
          }

          // Protection post-sortie tunnel (1500 ms)
          bool protectionActive = (millis() - tempsSortieTunnel < 1500);

          if (!protectionActive && (millis() - tempsLignePerdueT > 700)) {
            // Entrée tunnel confirmée
            Serial.println(F("[TUNNEL] Entree confirmee"));
            avancer(70, 2); delay(590); stopper();
            monServo.write(ANGLE_MUR_DROIT); delay(500);
            avancerDroit(VITESSE_TUNNEL); delay(8500); stopper();
            modeTunnel = TUNNEL_NAVIGATION;
          }
        }
        break;
      }

      case TUNNEL_NAVIGATION: {
        // Dans le tunnel — immunité 1500 ms après l'entrée pour éviter
        // fausse sortie sur la ligne d'entrée
        static unsigned long tEntree = 0;
        if (tEntree == 0) tEntree = millis();

        bool immunite = (millis() - tEntree < 1500);

        if (!immunite && ligneDetectee()) {
          // Ligne L2 détectée → sortie tunnel
          Serial.println(F("[TUNNEL] Ligne L2 — sortie"));
          stopper();
          avancer(30, 30); delay(150); stopper();
          tEntree = 0;
          tempsSortieTunnel = millis();
          entrerPhaseEvitement();   // M3
        } else {
          naviguerTunnel();
        }
        break;
      }
    }
    delay(5);
    break;
  }

  // ──────────────────────────────────────────────────────────────────
  case PHASE_EVITEMENT: {
  // M4 — Sections 3 & 4 : suivi + évitement O1 (gauche) et O2 (droite)
  // toutNoir ignoré — ne doit pas arrêter sur croisements L3/L4/L5
  // ──────────────────────────────────────────────────────────────────

    // Transition si les 2 obstacles traités (M5)
    if (obstacleIndex >= 2) {
      entrerPhaseCouleur();
      break;
    }

    bool peutDetecter = (millis() - lastEvasionMs >= EVICTION_COOLDOWN_MS);

    if (evitementActif && peutDetecter && obstacleDevant()) {
      bool contourGauche = (obstacleIndex % 2 == 0); // 0→gauche(O1), 1→droite(O2)
      evasionSequence(contourGauche);
      obstacleIndex++;
      lastEvasionMs = millis();
      break;
    }

    // Suivi PD — toutNoir ignoré, récupération active
    suivreLignePD(Kp_COMMUN, Kd_COMMUN, /*ignorerToutNoir=*/true, /*arretSurPerte=*/false);
    delay(5);
    break;
  }

  // ──────────────────────────────────────────────────────────────────
  case PHASE_COULEUR: {
  // M6 — Sections 5-7 : rampe, couleur, LEDs, demi-tour
  // ──────────────────────────────────────────────────────────────────

    switch (etatCouleur) {

      case ATTENTE_LIGNE:
        if (ligneDetectee()) {
          Serial.println(F("[COULEUR] Ligne — depart"));
          resetSuivi(); delay(200);
          etatCouleur = SUIVI_LIGNE_C;
        }
        break;

      case SUIVI_LIGNE_C: {
        // Détection ultrason désactivée après rampe (flag rampeEffectuee)
        if (!rampeEffectuee) {
          static unsigned long tUS = 0;
          if (millis() - tUS > 30) {
            tUS = millis();
            float dist = mesureUS();
            if (dist > 0 && dist < DIST_RAMPE_DETECTE) {
              Serial.print(F("[US] Mur a ")); Serial.print(dist,1); Serial.println(F(" cm"));
              etatCouleur = SEQUENCE_RAMPE;
              break;
            }
          }
        }

        // Après rampe : surveiller toutNoir 80 ms → transition ARRET_FINAL (M7)
        if (rampeEffectuee) {
          if (toutNoirDetecte()) {
            if (!toutNoirEnCours) {
              toutNoirEnCours = true;
              tDebut_toutNoir = millis();
            } else if (millis() - tDebut_toutNoir >= DUREE_CONFIRM_FIN_MS) {
              stopper();
              entrerPhaseArretFinal();
              break;
            }
          } else {
            toutNoirEnCours = false;
          }
        }

        // Suivi PD couleur (Kp=6, Kd=15) — toutNoir ignoré avant rampe
        suivreLignePD(Kp_COULEUR, Kd_COULEUR, /*ignorerToutNoir=*/true, /*arretSurPerte=*/false);
        break;
      }

      case SEQUENCE_RAMPE:
        sequenceRampe();
        etatCouleur = SUIVI_LIGNE_C;
        break;
    }
    delay(5);
    break;
  }

  // ──────────────────────────────────────────────────────────────────
  case PHASE_ARRET_FINAL: {
  // M8 — Sections 8-10 : suivi strict, arrêt définitif sur toutNoir
  // Ligne perdue → stopper immédiat (comportement original SuiviEtArretValide)
  // toutNoir → while(true) sans 'R'
  // ──────────────────────────────────────────────────────────────────

    if (toutNoirDetecte()) {
      stopper();
      Serial.println(F("[ARRET] ARRET DEFINITIF"));
      while (true) { stopper(); delay(500); }
    }

    // Suivi PD strict — stopper si ligne perdue
    suivreLignePD(Kp_COMMUN, Kd_COMMUN, /*ignorerToutNoir=*/false, /*arretSurPerte=*/true);
    delay(5);
    break;
  }

} // fin switch phaseGlobale
}
