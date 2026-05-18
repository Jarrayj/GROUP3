// ═══════════════════════════════════════════════════════════════
// MoteurEtSuiveur_v2.ino
// Suiveur de ligne PD — avec récupération de virage & calibration
// Serial Monitor : 9600 baud
// ═══════════════════════════════════════════════════════════════

#include <Wire.h>

// ── Adresses I2C — NE PAS MODIFIER ───────────────────────────
#define MOTEUR_A            0x66
#define MOTEUR_B            0x68
#define LINE_FOLLOWER_ADDR  0x20

// ── Directions ───────────────────────────────────────────────
#define ARRET    0x00
#define AVANT    0x01
#define ARRIERE  0x02

// ── Paramètres conduite ───────────────────────────────────────
#define VITESSE_NORMALE     38    // ↓ Réduit pour mieux tenir les virages
#define VITESSE_VIRAGE      28    // Vitesse en mode récupération de virage
#define VITESSE_MIN          5
#define VITESSE_PIVOT       22    // Vitesse de pivot sur place (recherche ligne)

// ── Seuil de détection ────────────────────────────────────────
// Sera écrasé par la calibration si elle est effectuée
int SEUIL_NOIR = 15;

// ── Gains PD ─────────────────────────────────────────────────
#define Kp  16.0f
#define Kd  10.0f

// ── Coefficients d'équilibrage ────────────────────────────────
float adj_A = 1.0f;
float adj_B = 0.80f;

// ── Mémoire de direction (récupération virage) ────────────────
// +1 = virage à droite, -1 = virage à gauche, 0 = tout droit
float derniereDirVirage = 0.0f;

// Durée max de récupération avant arrêt définitif (ms)
#define TIMEOUT_RECHERCHE  2500

// Durée de "mémoire" : si la ligne est perdue depuis moins de
// DELAI_VIRAGE ms, on continue dans la dernière direction connue
#define DELAI_VIRAGE        400

unsigned long tempsPerduLigne = 0;   // timestamp perte ligne
bool lignePrecedemmentPerdue = false;
float erreurPrecedente = 0.0f;

// ═══════════════════════════════════════════════════════════════
//  PILOTAGE MOTEURS
// ═══════════════════════════════════════════════════════════════
void piloterMoteur(byte adresse, byte direction,
                   byte vitesse, float adj = 1.0f) {
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
  piloterMoteur(MOTEUR_A, ARRIERE,
                (byte)constrain(vG, VITESSE_MIN, 63), adj_A);
  piloterMoteur(MOTEUR_B, ARRIERE,
                (byte)constrain(vD, VITESSE_MIN, 63), adj_B);
}

// Pivot sur place : un moteur avant, l'autre arrière
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

// ═══════════════════════════════════════════════════════════════
//  LECTURE CAPTEUR
// ═══════════════════════════════════════════════════════════════
uint8_t lireRegistre(byte reg) {
  Wire.beginTransmission(LINE_FOLLOWER_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LINE_FOLLOWER_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION AUTOMATIQUE
//  Pose le robot SUR la ligne noire, puis sur le fond blanc
//  pour calculer un seuil adapté à l'éclairage ambiant
// ═══════════════════════════════════════════════════════════════
void calibrer() {
  Serial.println(F("\n=== CALIBRATION ==="));
  Serial.println(F("Etape 1 : Pose le robot SUR la ligne NOIRE"));
  Serial.println(F("Puis envoie 'n' dans le moniteur serie..."));

  // Attente commande 'n'
  while (true) {
    if (Serial.available() && Serial.read() == 'n') break;
  }
  uint8_t g1 = lireRegistre(0x01);
  uint8_t m1 = lireRegistre(0x02);
  uint8_t d1 = lireRegistre(0x03);
  int valNoir = (g1 + m1 + d1) / 3;
  Serial.print(F("Valeurs NOIR  G=")); Serial.print(g1);
  Serial.print(F(" M=")); Serial.print(m1);
  Serial.print(F(" D=")); Serial.println(d1);

  Serial.println(F("\nEtape 2 : Pose le robot SUR le fond BLANC/CLAIR"));
  Serial.println(F("Puis envoie 'b' dans le moniteur serie..."));
  while (true) {
    if (Serial.available() && Serial.read() == 'b') break;
  }
  uint8_t g2 = lireRegistre(0x01);
  uint8_t m2 = lireRegistre(0x02);
  uint8_t d2 = lireRegistre(0x03);
  int valBlanc = (g2 + m2 + d2) / 3;
  Serial.print(F("Valeurs BLANC G=")); Serial.print(g2);
  Serial.print(F(" M=")); Serial.print(m2);
  Serial.print(F(" D=")); Serial.println(d2);

  // Seuil = milieu entre noir et blanc
  SEUIL_NOIR = (valNoir + valBlanc) / 2;
  Serial.print(F("\n>>> Seuil calcule : ")); Serial.println(SEUIL_NOIR);
  Serial.println(F("Calibration OK — Depart dans 2 secondes...\n"));
  delay(2000);
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(500);

  Serial.println(F("=== MoteurEtSuiveur v2 ==="));
  Serial.println(F("Envoie 'c' pour calibrer les capteurs"));
  Serial.println(F("Envoie autre chose (ou attends 5s) pour demarrer"));

  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    if (Serial.available()) {
      char ch = Serial.read();
      if (ch == 'c') { calibrer(); break; }
      else            { break; }
    }
  }

  stopper();
  Serial.print(F("SEUIL actif = ")); Serial.println(SEUIL_NOIR);
  Serial.println(F("Pret !"));
  delay(500);
}

// ═══════════════════════════════════════════════════════════════
//  LOOP PRINCIPALE
// ═══════════════════════════════════════════════════════════════
void loop() {
  uint8_t g = lireRegistre(0x01);
  uint8_t m = lireRegistre(0x02);
  uint8_t d = lireRegistre(0x03);

  bool noir_G = (g < SEUIL_NOIR);
  bool noir_M = (m < SEUIL_NOIR);
  bool noir_D = (d < SEUIL_NOIR);
  bool ligneVue = noir_G || noir_M || noir_D;

  // ── CAS 1 : ligne visible → suivi PD normal ──────────────────
  if (ligneVue) {
    lignePrecedemmentPerdue = false;
    tempsPerduLigne = 0;

    // Mise à jour de la mémoire de direction :
    // on mémorise vers quel côté on était en train de tourner
    if      (noir_D && !noir_G) derniereDirVirage =  1.0f;  // droite
    else if (noir_G && !noir_D) derniereDirVirage = -1.0f;  // gauche
    // si seulement milieu → on conserve la dernière dir connue

    // Calcul erreur
    float erreur = 0;
    int total = (noir_G?1:0) + (noir_M?1:0) + (noir_D?1:0);
    if (total > 0)
      erreur = ((noir_G ? -1.0f : 0) + (noir_D ? 1.0f : 0)) / total;

    float derivee    = erreur - erreurPrecedente;
    float correction = (Kp * erreur) + (Kd * derivee);
    erreurPrecedente = erreur;

    // Ralentit si virage (correction forte)
    int vitesseBase = (abs(correction) > 8) ? VITESSE_VIRAGE : VITESSE_NORMALE;

    int vG = constrain((int)(vitesseBase - correction), VITESSE_MIN, 63);
    int vD = constrain((int)(vitesseBase + correction), VITESSE_MIN, 63);
    avancer(vG, vD);

  // ── CAS 2 : ligne perdue ──────────────────────────────────────
  } else {

    if (!lignePrecedemmentPerdue) {
      // Première boucle sans ligne
      tempsPerduLigne = millis();
      lignePrecedemmentPerdue = true;
    }

    unsigned long depuisPerte = millis() - tempsPerduLigne;

    // Phase 1 (0 → DELAI_VIRAGE ms) :
    // On continue dans la dernière direction connue (virage)
    if (depuisPerte < DELAI_VIRAGE) {
      if (derniereDirVirage > 0) {
        // Tourner à droite : roue gauche plus rapide
        avancer(VITESSE_VIRAGE, VITESSE_MIN);
      } else if (derniereDirVirage < 0) {
        // Tourner à gauche : roue droite plus rapide
        avancer(VITESSE_MIN, VITESSE_VIRAGE);
      } else {
        // Aucune info : avancer doucement
        avancer(VITESSE_VIRAGE, VITESSE_VIRAGE);
      }

    // Phase 2 (DELAI_VIRAGE → TIMEOUT ms) :
    // Pivot sur place dans la dernière direction pour retrouver la ligne
    } else if (depuisPerte < TIMEOUT_RECHERCHE) {
      if (derniereDirVirage >= 0) {
        pivoterDroite(VITESSE_PIVOT);
      } else {
        pivoterGauche(VITESSE_PIVOT);
      }

    // Phase 3 : timeout → arrêt définitif
    } else {
      stopper();
      static unsigned long tStop = 0;
      if (millis() - tStop > 500) {
        tStop = millis();
        Serial.println(F("!!! LIGNE PERDUE — TIMEOUT ATTEINT. Recale le robot."));
      }
    }

    // Debug perte ligne
    static unsigned long tDbgP = 0;
    if (millis() - tDbgP > 200) {
      tDbgP = millis();
      Serial.print(F("[PERDU] depuis ")); Serial.print(depuisPerte);
      Serial.print(F("ms  derDir=")); Serial.print(derniereDirVirage, 0);
      Serial.print(F("  G=")); Serial.print(g);
      Serial.print(F(" M=")); Serial.print(m);
      Serial.print(F(" D=")); Serial.println(d);
    }
  }

  // ── Debug normal (toutes les 200ms) ──────────────────────────
  if (ligneVue) {
    static unsigned long tDbg = 0;
    if (millis() - tDbg > 200) {
      tDbg = millis();
      Serial.print(F("G=")); Serial.print(g);
      Serial.print(F(" M=")); Serial.print(m);
      Serial.print(F(" D=")); Serial.print(d);
      Serial.print(F("  seuil=")); Serial.print(SEUIL_NOIR);
      Serial.print(F("  nG=")); Serial.print(noir_G);
      Serial.print(F(" nM=")); Serial.print(noir_M);
      Serial.print(F(" nD=")); Serial.println(noir_D);
    }
  }

  delay(15);  // Légèrement réduit pour réactivité
}
