Projet - Groupe 3

Membres :

* JARRAY Abdelmalek
* MEZIMECHE Hichem 
* NIANG Seydou
* BERKI Yanis
* FOREST Mae
* SOW Malick

  # Robot de parcours — Arduino UNO R3

Programme Arduino pilotant un robot autonome sur un parcours de compétition comportant suivi de ligne, tunnel, évitement d'obstacles, détection de couleur et lancement de balle.

---

## Matériel requis

| Composant | Référence |
|---|---|
| Carte microcontrôleur | Arduino UNO R3 |
| Shield | Grove Base Shield V2.0 |
| Suiveur de ligne | Me RGB Line Follower (I2C, adresse 0x20) |
| Pilote moteurs | Mini I2C Motor Driver (0x66 / 0x68) |
| Capteur couleur | Grove TCS34725 I2C Color Sensor V2.0 |
| Télémètre | Grove Ultrasonic Ranger V2.0 (pin 7) |
| Ruban LED | Grove 30 LEDs RGB NeoPixel (pin 4) |
| Servo | Grove Analog Servo (pin 6) |
| Afficheur | Grove LCD RGB Backlight 2×16 (I2C) |
| Alimentation | Piles AA rechargeables |

---

## Librairies Arduino nécessaires

```
Wire.h          (incluse avec l'IDE)
Servo.h         (incluse avec l'IDE)
Adafruit_TCS34725
Adafruit_NeoPixel
```

---

## Architecture du programme

Le programme est structuré autour d'une **machine à états principale** à 5 phases, chacune correspondant à une ou plusieurs sections du parcours.

```
PHASE_ATTENTE_4IR → PHASE_TUNNEL → PHASE_EVITEMENT → PHASE_COULEUR → PHASE_ARRET_FINAL
```

### Phase 1 — Attente départ (`PHASE_ATTENTE_4IR`)
Le robot est immobile. Il attend que les **4 capteurs IR voient la ligne noire de départ simultanément**, confirmé sur 5 lectures consécutives (~25 ms) pour éviter tout faux départ. Une fois détecté, il patiente 500 ms puis démarre.

### Phase 2 — Tunnel (`PHASE_TUNNEL`)
Deux sous-états :
- **`TUNNEL_SUIVI`** : suivi de ligne PD normal jusqu'à l'entrée du tunnel. Quand la ligne est perdue de façon confirmée (>700 ms), et qu'un délai minimum depuis le départ est écoulé (protection anti-déclenchement prématuré sur le virage V1), le robot entre dans le tunnel en avance à l'aveugle.
- **`TUNNEL_NAVIGATION`** : navigation par suivi du mur droit au télémètre ultrason (régulateur proportionnel). La sortie est détectée par la réapparition de la ligne L2.

### Phase 3 — Évitement (`PHASE_EVITEMENT`)
Suivi de ligne PD avec détection d'obstacles par ultrason. Deux obstacles traités séquentiellement :
- **O1** : contournement par la **gauche** (section 3 du parcours)
- **O2** : contournement par la **droite** (section 4)

Chaque évitement suit une séquence de pivots et d'avances chronométrés, avec cooldown de 3 s entre deux détections pour éviter les faux positifs.

### Phase 4 — Couleur (`PHASE_COULEUR`)
Trois sous-états :
- **`ATTENTE_LIGNE`** : attend la ligne après la fin de l'évitement
- **`SUIVI_LIGNE_C`** : suivi PD jusqu'à détection du mur (ultrason ≤ 20 cm)
- **`SEQUENCE_RAMPE`** : arrêt, lecture couleur (TCS34725, moyenne sur 5 mesures), clignotement du ruban LED à 0,5 Hz pendant 3 s, puis demi-tour et reprise du suivi

### Phase 5 — Arrêt final (`PHASE_ARRET_FINAL`)
Suivi PD strict. Arrêt définitif sur détection `toutNoir` (les 4 IR simultanément), correspondant à la ligne d'arrivée sur la plateforme de lancement.

---

## Régulateur PD de suivi de ligne

La fonction `suivreLignePD(kp, kd, ignorerToutNoir, arretSurPerte)` est utilisée dans toutes les phases. L'erreur est calculée à partir des 4 capteurs IR pondérés (`-2, -0.5, +0.5, +2`). Les gains varient selon la phase :

| Phase | Kp | Kd |
|---|---|---|
| Tunnel / Évitement | 5.0 | 10.0 |
| Couleur / Rampe | 6.0 | 15.0 |

En cas de perte de ligne, le robot pivote dans la dernière direction connue pendant `DELAI_VIRAGE` (400 ms), puis effectue un pivot de recherche jusqu'à `TIMEOUT_RECHERCHE` (2500 ms).

---

## Paramètres ajustables

```cpp
// Vitesses
#define VITESSE_NORMALE     46   // suivi ligne standard
#define VITESSE_TUNNEL      37   // traversée tunnel
#define VITESSE_VIRAGE      26   // virage serré
#define VITESSE_PIVOT       20   // recherche ligne perdue

// Évitement
#define DIST_DETECT         28   // distance détection obstacle (cm)
#define EVICTION_COOLDOWN_MS 3000 // délai entre deux évitements

// Tunnel
#define DELAI_MIN_AVANT_TUNNEL 4000 // ms min après départ avant d'autoriser l'entrée tunnel

// Couleur
#define DIST_RAMPE_STOP     20   // distance au mur pour s'arrêter (cm)
#define DUREE_ATTENTE_MUR   3000 // durée clignotement LEDs (ms)
#define FREQ_CLIGNOT_MS     500  // fréquence clignotement (ms)
```

---

## Branchements principaux

| Signal | Pin Arduino |
|---|---|
| Ultrason (trigger + echo) | D7 |
| Servo | D6 |
| Ruban LED NeoPixel | D4 |
| Moteurs + suiveur de ligne | I2C (A4/A5) |
| Capteur couleur TCS34725 | I2C (A4/A5) |

---

## Comportement au démarrage

1. Initialisation I2C, servo centré (90°), LEDs éteintes
2. Affichage `Pret — posez le robot sur la ligne de depart` sur le moniteur série
3. Le robot reste immobile jusqu'à détection des 4 IR sur la ligne de départ
4. Départ automatique après confirmation

Le port série (9600 baud) affiche les transitions d'état et les valeurs capteurs pour le débogage.
