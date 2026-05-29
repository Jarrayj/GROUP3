// ═══════════════════════════════════════════════════════════════════════
// Code : Suiveur de ligne PD + Tunnel
// ═══════════════════════════════════════════════════════════════════════

#include <Wire.h>
#include <Servo.h>

// ── Adresses I2C — robot ────────────────────────────────────────────────
#define MOTEUR_A           0x66
#define MOTEUR_B           0x68
#define LINE_FOLLOWER_ADDR 0x20

// ── Directions robot ─────────────────────────────────────────────────────
#define ARRET   0x00
#define AVANT   0x01
#define ARRIERE 0x02

// ── Pins ─────────────────────────────────────────────────────────────────
#define PIN_US    7
#define PIN_SERVO 6

// ── Coefficients moteurs ──────────────────────────────────────────────────
float adj_A = 1.0f;
float adj_B = 0.91f;

// ── Seuil noir ────────────────────────────────────────────────────────────
int SEUIL_NOIR = 35;

// ── Gains PD ──────────────────────────────────────────────────────────────
#define Kp_COMMUN  5.0f
#define Kd_COMMUN 10.0f

// ── Vitesses robot ────────────────────────────────────────────────────────
#define VITESSE_NORMALE     46
#define VITESSE_VIRAGE      26
#define VITESSE_MIN          5
#define VITESSE_PIVOT       20
#define VITESSE_TUNNEL      37

// ── Paramètres perte de ligne ─────────────────────────────────────────────
#define TIMEOUT_RECHERCHE 2500
#define DELAI_VIRAGE       400

// ── Variables PD partagées ────────────────────────────────────────────────
float         erreurPrecedente        = 0.0f;
float         derniereDirVirage       = 0.0f;
unsigned long tempsPerduLigne         = 0;
bool          lignePrecedemmentPerdue = false;

// ═══════════════════════════════════════════════════════════════════════
//  VARIABLES — TUNNEL
// ═══════════════════════════════════════════════════════════════════════
enum ModeTunnel { TUNNEL_SUIVI, TUNNEL_NAVIGATION, TUNNEL_TERMINE };
ModeTunnel modeTunnel = TUNNEL_SUIVI;

#define ANGLE_CENTRE      90
#define ANGLE_MUR_DROIT   45
#define DIST_CIBLE_MUR     3.0f
#define Kp_MUR             2.0f

Servo monServo;
unsigned long tempsSortieTunnel      = 0;
bool          lignePerdueAvantTunnel = false;
unsigned long tempsLignePerdueT      = 0;

// ═══════════════════════════════════════════════════════════════════════
//  FONCTIONS COMMUNES — MOTEURS ROBOT
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

// ═══════════════════════════════════════════════════════════════════════
//  FONCTIONS COMMUNES — CAPTEURS
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

void resetSuivi() {
  erreurPrecedente        = 0.0f;
  derniereDirVirage       = 0.0f;
  tempsPerduLigne         = 0;
  lignePrecedemmentPerdue = false;
}

// ═══════════════════════════════════════════════════════════════════════
//  SUIVI DE LIGNE PD
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

  if (toutNoir && !ignorerToutNoir) { stopper(); return; }

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
    if (arretSurPerte) { stopper(); return; }
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
//  SETUP
// ═══════════════════════════════════════════════════════════════════════
void setup() {
  Wire.begin();
  Serial.begin(9600);

  monServo.attach(PIN_SERVO);
  monServo.write(ANGLE_CENTRE);

  stopper();
  Serial.println(F("=== Suiveur + Tunnel ==="));
  Serial.println(F("Demarrage du suivi de ligne..."));
  
  delay(1000); // Petit délai de sécurité avant de partir
  resetSuivi();
}

// ═══════════════════════════════════════════════════════════════════════
//  LOOP PRINCIPALE
// ═══════════════════════════════════════════════════════════════════════
void loop() {

  switch (modeTunnel) {

    // ────────────────────────────────────────────────────────────────
    // Phase 1 : Suivi de ligne avant le tunnel et entrée
    // ────────────────────────────────────────────────────────────────
    case TUNNEL_SUIVI: {
      bool ligneVue = ligneDetectee();

      if (ligneVue) {
        lignePerdueAvantTunnel = false;
        tempsLignePerdueT      = 0;
        suivreLignePD(Kp_COMMUN, Kd_COMMUN, true, false);

      } else {
        if (!lignePerdueAvantTunnel) {
          lignePerdueAvantTunnel = true;
          tempsLignePerdueT      = millis();
          Serial.println(F("[TUNNEL] Ligne perdue — chrono tunnel demarre"));
        }

        bool protectionActive = (millis() - tempsSortieTunnel < 1500);

        if (protectionActive) {
          suivreLignePD(Kp_COMMUN, Kd_COMMUN, true, false);

        } else if (millis() - tempsLignePerdueT > 700) {
          // Entrée dans le tunnel confirmée (ligne perdue depuis plus de 700ms)
          Serial.println(F("[TUNNEL] Entree confirmee"));
          avancer(70, 2); delay(570); stopper(); // Petit virage d'insertion (à ajuster selon ta piste)
          monServo.write(ANGLE_MUR_DROIT); delay(500);
          avancerDroit(VITESSE_TUNNEL); delay(8500); stopper(); // Avancée arbitraire dans le tunnel
          modeTunnel = TUNNEL_NAVIGATION;

        } else {
          suivreLignePD(Kp_COMMUN, Kd_COMMUN, true, false);
        }
      }
      break;
    }

    // ────────────────────────────────────────────────────────────────
    // Phase 2 : Navigation aux ultrasons dans le tunnel
    // ────────────────────────────────────────────────────────────────
    case TUNNEL_NAVIGATION: {
      static unsigned long tEntree = 0;
      if (tEntree == 0) tEntree = millis();

      bool immunite = (millis() - tEntree < 1500);

      // Si on n'est plus en période d'immunité et qu'on retrouve la ligne (sortie du tunnel)
      if (!immunite && ligneDetectee()) {
        Serial.println(F("[TUNNEL] Ligne retrouvee — sortie"));
        stopper();
        avancer(30, 30); delay(150); stopper(); // Se recaler légèrement sur la ligne
        tEntree = 0;
        tempsSortieTunnel = millis();
        monServo.write(ANGLE_CENTRE); 
        
        modeTunnel = TUNNEL_TERMINE; // Fin de la logique extraite
      } else {
        naviguerTunnel();
      }
      break;
    }

    // ────────────────────────────────────────────────────────────────
    // Phase 3 : Arrêt post-tunnel (ajouté pour tester l'extraction proprement)
    // ────────────────────────────────────────────────────────────────
    case TUNNEL_TERMINE: {
      stopper();
      // On peut reprendre un suivi de ligne normal ici si besoin.
      // suivreLignePD(Kp_COMMUN, Kd_COMMUN, true, false);
      break;
    }
  }

  delay(5);
}