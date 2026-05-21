// ═══════════════════════════════════════════════════════════════
// MoteurEtSuiveur_v8.ino - Evitement + Anti-effet requin (glisse parfaite)
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

// ── PIN ULTRASON ─────────────────────────────────────────────
#define PIN_US   7

// ── Paramètres conduite ───────────────────────────────────────
#define VITESSE_NORMALE     40    
#define VITESSE_VIRAGE      26    
#define VITESSE_MIN          5
#define VITESSE_PIVOT       20    

int SEUIL_NOIR = 35;

// ── Gains PD (Réglages ANTI-TREMBLEMENT V8) ───────────────────
#define Kp  5.0f    // ↓ Adouci pour arrêter les gros coups de volant
#define Kd  10.0f   // ↑ Augmenté pour freiner les oscillations

// ── Coefficients d'équilibrage ────────────────────────────────
float adj_A = 1.0f;
float adj_B = 0.91f;

float derniereDirVirage = 0.0f;
#define TIMEOUT_RECHERCHE  2500
#define DELAI_VIRAGE        400

unsigned long tempsPerduLigne = 0;
bool lignePrecedemmentPerdue = false;
float erreurPrecedente = 0.0f;

// ═══════════════════════════════════════════════════════════════
//  PARAMÈTRES ÉVITEMENT
// ═══════════════════════════════════════════════════════════════
#define DIST_DETECT          28      // seuil de détection (cm)
#define DIST_FREIN           32      // début décélération (cm)

#define VITESSE_EVIT_AVANCE  38
#define VITESSE_PIVOT_EVIT   18

#define NB_CONFIRM_OBSTACLE   3

#define TEMPS_STOP_MS        2000
#define TEMPS_PIVOT_90       1600    // à ajuster selon ton robot
#define TEMPS_PIVOT_100      2200
#define TEMPS_AVANCE_40CM    4010
#define TEMPS_AVANCE_400CM   3000
#define TEMPS_AVANCE_4000CM  4500
#define TEMPS_AVANCE_50CM    4200
#define TEMPS_AVANCE_500CM   3900
#define TEMPS_AVANCE_30CM    3700
#define PAUSE_ETAPE          100

// Paramètre pour recherche de la ligne après contournement
#define MAX_RECHERCHE_CM     60      // distance max en cm pour retrouver la ligne
#define MS_PER_CM_ESTIMATE   110     // estimation du temps (ms) pour parcourir 1 cm
#define EVICTION_COOLDOWN_MS 3000    // délai après évitement avant re-détection (ms)

bool evitementActif = true;
int compteurObstacle = 0;
bool prochainContourAGauche = true;  // par défaut contourner à gauche
bool modeContourAuto = true;         // true = alterner automatiquement
int obstacleIndex = 0;               // nombre d'obstacles rencontrés
unsigned long lastEvasionMs = 0;

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

void pivoterGauche(byte vitesse) {
  piloterMoteur(MOTEUR_A, AVANT,   vitesse, adj_A);
  piloterMoteur(MOTEUR_B, ARRIERE, vitesse, adj_B);
}
void pivoterDroite(byte vitesse) {
  piloterMoteur(MOTEUR_A, ARRIERE, vitesse, adj_A);
  piloterMoteur(MOTEUR_B, AVANT,   vitesse, adj_B);
}

void avancerDroit(byte vitesse) {
  avancer(vitesse, vitesse);
}

void stopper() {
  piloterMoteur(MOTEUR_A, ARRET, 0);
  piloterMoteur(MOTEUR_B, ARRET, 0);
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

// ═══════════════════════════════════════════════════════════════
//  ULTRASON
// ═══════════════════════════════════════════════════════════════
float mesureUS() {
  pinMode(PIN_US, OUTPUT);
  digitalWrite(PIN_US, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_US, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_US, LOW);
  pinMode(PIN_US, INPUT);
  long dur = pulseIn(PIN_US, HIGH, 30000UL);
  if (dur == 0) return 999.0f;
  return dur * 0.0343f / 2.0f;
}

bool obstacleDevant() {
  float d = mesureUS();
  if (d > 0 && d < DIST_DETECT) {
    compteurObstacle++;
    if (compteurObstacle >= NB_CONFIRM_OBSTACLE) {
      compteurObstacle = NB_CONFIRM_OBSTACLE;
      return true;
    }
  } else {
    compteurObstacle = 0;
  }
  return false;
}

float distanceObstacle() {
  return mesureUS();
}

// ═══════════════════════════════════════════════════════════════
//  SÉQUENCE D'ÉVITEMENT
// ═══════════════════════════════════════════════════════════════
void avancerJusquARetrouverLigne(unsigned long maxMs) {
  unsigned long t0 = millis();
  avancerDroit(VITESSE_EVIT_AVANCE);
  while (millis() - t0 < maxMs) {
    uint8_t eg = lireRegistre(0x01);
    uint8_t cg = lireRegistre(0x02);
    uint8_t cd = lireRegistre(0x03);
    uint8_t ed = lireRegistre(0x04);
    bool n_EG = (eg < SEUIL_NOIR);
    bool n_CG = (cg < SEUIL_NOIR);
    bool n_CD = (cd < SEUIL_NOIR);
    bool n_ED = (ed < SEUIL_NOIR);
    if (n_EG || n_CG || n_CD || n_ED) {
      stopper();
      return;
    }
    delay(20);
  }
  stopper();
}

void evasionSequence(bool contourGauche=true) {
  Serial.println(F("[EVIT] Obstacle ! Debut sequence"));

  decelerationProgressive(400);
  delay(TEMPS_STOP_MS);

  if (contourGauche) {
    pivoterGauche(VITESSE_PIVOT_EVIT);
    delay(TEMPS_PIVOT_90);
    stopper();
    delay(PAUSE_ETAPE);

    avancerDroit(VITESSE_EVIT_AVANCE);
    delay(TEMPS_AVANCE_400CM);
    stopper();
    delay(PAUSE_ETAPE);

    pivoterDroite(VITESSE_PIVOT_EVIT);
    delay(TEMPS_PIVOT_90);
    stopper();
    delay(PAUSE_ETAPE);

    avancerDroit(VITESSE_EVIT_AVANCE);
    delay(TEMPS_AVANCE_4000CM);
    stopper();
    delay(PAUSE_ETAPE);

    pivoterDroite(VITESSE_PIVOT_EVIT);
    delay(TEMPS_PIVOT_90);
    stopper();
    delay(PAUSE_ETAPE);

    
  } else {
    // Contournement miroir (par la droite)
    pivoterDroite(VITESSE_PIVOT_EVIT);
    delay(TEMPS_PIVOT_90);
    stopper();
    delay(PAUSE_ETAPE);

    avancerDroit(VITESSE_EVIT_AVANCE);
    delay(TEMPS_AVANCE_500CM);
    stopper();
    delay(PAUSE_ETAPE);

    pivoterGauche(VITESSE_PIVOT_EVIT);
    delay(TEMPS_PIVOT_100);
    stopper();
    delay(PAUSE_ETAPE);

    avancerDroit(VITESSE_EVIT_AVANCE);
    delay(TEMPS_AVANCE_50CM);
    stopper();
    delay(PAUSE_ETAPE);

    
  }

  // Avancer jusqu'a retrouver la ligne avec limite de distance
  unsigned long maxMs = (unsigned long)(MAX_RECHERCHE_CM * MS_PER_CM_ESTIMATE);
  avancerJusquARetrouverLigne(maxMs);

  // Reset des variables pour reprendre le suivi de ligne proprement
  erreurPrecedente = 0.0f;
  lignePrecedemmentPerdue = false;
  tempsPerduLigne = 0;
  compteurObstacle = 0;
  Serial.println(F("[EVIT] Fin sequence, reprise suivi"));
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

  Serial.println(F("=== MoteurEtSuiveur v8 (Evitement + Glisse) ==="));
  Serial.println(F("Envoie 'e' pendant la course pour on/off evitement"));
  Serial.println(F("Envoie autre chose (ou attends 5s) pour demarrer"));
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
  // ── ON/OFF Evitement par le Moniteur Série ───────────────────
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'e') {
      evitementActif = !evitementActif;
      Serial.print(F("Evitement : "));
      Serial.println(evitementActif ? F("ON") : F("OFF"));
    } else if (ch == '1') {
      modeContourAuto = false;
      prochainContourAGauche = true;
      Serial.println(F("Mode contournement: FORCÉ GAUCHE"));
    } else if (ch == '2') {
      modeContourAuto = false;
      prochainContourAGauche = false;
      Serial.println(F("Mode contournement: FORCÉ DROITE"));
    } else if (ch == 'a') {
      modeContourAuto = true;
      Serial.println(F("Mode contournement: AUTOMATIQUE (alterne)"));
    }
  }

  // ── VÉRIFICATION OBSTACLE ────────────────────────────────────
  bool peutDetecterObstacle = true;
  if (millis() - lastEvasionMs < EVICTION_COOLDOWN_MS) peutDetecterObstacle = false;

  if (evitementActif && peutDetecterObstacle && obstacleDevant()) {
    // Détermination du côté de contournement
    bool contour;
    if (modeContourAuto) {
      contour = (obstacleIndex % 2 == 0); // 0 -> gauche, 1 -> droite, etc.
    } else {
      contour = prochainContourAGauche;
    }
    // Détection confirmée : lancer la séquence de contournement
    evasionSequence(contour);
    obstacleIndex++;
    lastEvasionMs = millis();
    return; // On zappe le suivi de ligne le temps de l'esquive
  }

  // ── SUIVI DE LIGNE (Anti-requin) ─────────────────────────────
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

  // CAS 0 : Croisement détecté
  if (toutNoir) {
    stopper();
    Serial.println(F("Ligne complete detectee : ARRET !"));
    return;
  }

  // CAS 1 : Ligne visible
  if (ligneVue) {
    lignePrecedemmentPerdue = false;
    tempsPerduLigne = 0;

    if      (n_ED || n_CD) derniereDirVirage =  1.0f; 
    else if (n_EG || n_CG) derniereDirVirage = -1.0f; 

    float erreur = 0;
    int total = (n_EG?1:0) + (n_CG?1:0) + (n_CD?1:0) + (n_ED?1:0);
    if (total > 0) {
      // ↓ ICI : Les poids de 0.5 pour les capteurs centraux tuent l'effet requin !
      erreur = ((n_EG ? -2.0f : 0) + (n_CG ? -0.5f : 0) + (n_CD ? 0.5f : 0) + (n_ED ? 2.0f : 0)) / total;
    }

    float derivee    = erreur - erreurPrecedente;
    float correction = (Kp * erreur) + (Kd * derivee);
    erreurPrecedente = erreur;

    // Décélération douce si un obstacle est vu "au loin"
    int vitesseMax = VITESSE_NORMALE;
    if (evitementActif) {
      float dist = distanceObstacle();
      if (dist < DIST_FREIN && dist > DIST_DETECT) {
        float ratio = (dist - DIST_DETECT) / (float)(DIST_FREIN - DIST_DETECT);
        vitesseMax = VITESSE_MIN + (int)(ratio * (VITESSE_NORMALE - VITESSE_MIN));
      }
    }

    int vitesseBase;
    if (n_CG || n_CD) {
      vitesseBase = vitesseMax; 
    } else {
      vitesseBase = (abs(correction) > 8) ? VITESSE_VIRAGE : vitesseMax;
    }

    int vG = constrain((int)(vitesseBase + correction), VITESSE_MIN, 63);
    int vD = constrain((int)(vitesseBase - correction), VITESSE_MIN, 63);
    
    avancer(vG, vD);

  // CAS 2 : Ligne perdue
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

  // ↓ ICI : 5ms au lieu de 15ms pour des réflexes chirurgicaux !
  delay(5); 
}
