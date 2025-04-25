#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <HCSR04.h>
#include <U8g2lib.h>

#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN  32  // Chip Select

// Pour un module unique 8×8
//  - Si la documentation indique 8×8, U8g2 utilise l'appellation 8 de haut × 8 multiples de large
//  - Parfois, on choisit U8G2_MAX7219_8X8_F_4W_SW_SPI ou un modèle équivalent
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(
  U8G2_R0,       // rotation
  /* clock=*/ CLK_PIN, // pin Arduino reliée à CLK (horloge)
  /* data=*/ DIN_PIN,  // pin Arduino reliée à DIN (données)
  /* cs=*/ CS_PIN,    // pin Arduino reliée à CS (chip select)
  /* dc=*/ U8X8_PIN_NONE,
  /* reset=*/ U8X8_PIN_NONE
);

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define MOTOR_INTERFACE_TYPE 4
#define IN_1 31
#define IN_2 33
#define IN_3 35
#define IN_4 37

AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);

#define TRIG_PIN 6
#define ECHO_PIN 7
#define BUZZER_PIN 4
#define LED_RED_PIN 2
#define LED_BLUE_PIN 3

int DISTANCE_ALARME = 15;
const unsigned long DUREE_SANS_DETECTION = 3000;
const unsigned long INTERVAL_CLIGNOTEMENT = 200;
const long INTERVAL_DIST = 50;
const long INTERVAL_SERIAL = 100;
const long START_DELAY = 2000;
const int DIST_MIN_ANGLE = 30;
const int DIST_MAX_ANGLE = 60;
const int ANGLE_MIN = 10;
const int ANGLE_MAX = 170;

unsigned long dernierClignotement = 0;
unsigned long alarmeDerniereDetection = 0;
unsigned long previousMillisDist = 0;
unsigned long previousMillisSerial = 0;
unsigned long previousMillisStart = 0;

int distanceCM = 0;
int targetAngle = 90;
bool initDone = false;
bool ledEtat = false;
bool alarmeActive = false;

int minStep = (ANGLE_MIN * 2038.0) / 360;
const int maxStep = 962;

HCSR04 hc(TRIG_PIN, ECHO_PIN);

enum Etat { INIT, MESURE, AFFICHAGE, TRANSMISSION, ALARME };
Etat etatActuel = INIT;

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);

  previousMillisStart = millis();
  afficherIntro();

  u8g2.begin(); // Initialisation de l'écran MAX7219
}

int LIMITE_INF = 30;
int LIMITE_SUP = 60;


void loop() {
  unsigned long currentMillis = millis();

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    //Serial.println("Reçu: " + command);
    if (command == "gDist") {
      // Afficher la distance
      Serial.print("gDist : ");
      Serial.println(distanceCM);
      afficherSymbole("✔");
    } 
    
    else if (command.startsWith("cfg;alm;")) {
      int limit = command.substring(8).toInt();
      DISTANCE_ALARME = limit;
      Serial.print("Alarme configurée à ");
      Serial.println(limit);
      afficherSymbole("✔");
    } 
    
    else if (command.startsWith("cfg;lim_inf;")) {
      int limit = command.substring(12).toInt();
      if (limit >= LIMITE_SUP) {
        Serial.println("Erreur – Limite inférieure plus grande que limite supérieure");
        afficherSymbole("🚫");
      } else {
        LIMITE_INF = limit;
        Serial.print("Limite inférieure configurée à ");
        Serial.println(limit);
        afficherSymbole("✔");
      }
    } 
    
    else if (command.startsWith("cfg;lim_sup;")) {
      int limit = command.substring(12).toInt();
      if (limit <= LIMITE_INF) {
        Serial.println("Erreur – Limite supérieure plus petite que limite inférieure");
        afficherSymbole("🚫");
      } else {
        LIMITE_SUP = limit;
        Serial.print("Limite supérieure configurée à ");
        Serial.println(limit);
        afficherSymbole("✔");
      }
    } 
    
    else {
      // Commande inconnue
      Serial.println("Commande inconnue.");
      afficherSymbole("X");
    }
  }
  

  switch (etatActuel) {
    case INIT:
      if (currentMillis - previousMillisStart >= START_DELAY) {
        initDone = true;
        lcd.clear();
        etatActuel = MESURE;
      }
      break;

    case MESURE:
      if (currentMillis - previousMillisDist >= INTERVAL_DIST) {
        previousMillisDist = currentMillis;
        distanceCM = mesurerDistance();
        targetAngle = calculerAngle(distanceCM);
        deplacerMoteur(targetAngle);
        etatActuel = AFFICHAGE;
      }
      break;

    case AFFICHAGE:
      afficherLCD(distanceCM, targetAngle);
      etatActuel = TRANSMISSION;
      break;

    case TRANSMISSION:
      if (currentMillis - previousMillisSerial >= INTERVAL_SERIAL) {
        previousMillisSerial = currentMillis;
       // Serial.print("etd:2413645,dist:");
        //Serial.print(distanceCM);
        //Serial.print(",deg:");
        //Serial.println(targetAngle);
      }
      etatActuel = ALARME;
      break;

    case ALARME:
      if (distanceCM <= DISTANCE_ALARME) {
        if (!alarmeActive) {
          alarmeActive = true;
        }
        alarmeDerniereDetection = currentMillis;
        digitalWrite(BUZZER_PIN, HIGH);
      }

      if (alarmeActive) {
        if (currentMillis - dernierClignotement >= INTERVAL_CLIGNOTEMENT) {
          dernierClignotement = currentMillis;
          ledEtat = !ledEtat;
          digitalWrite(LED_RED_PIN, ledEtat ? HIGH : LOW);
          digitalWrite(LED_BLUE_PIN, ledEtat ? LOW : HIGH);
        }

        if (alarmeActive && currentMillis - alarmeDerniereDetection >= DUREE_SANS_DETECTION) {
          alarmeActive = false;
          digitalWrite(BUZZER_PIN, LOW);
          digitalWrite(LED_RED_PIN, LOW);
          digitalWrite(LED_BLUE_PIN, LOW);
        }
      }

      myStepper.run();
      etatActuel = MESURE;
      break;
  }
}

void afficherIntro() {
  lcd.setCursor(0, 0);
  lcd.print("2413645");
  lcd.setCursor(0, 1);
  lcd.print("Labo 4B");
}

int mesurerDistance() {
  return hc.dist();
}

int calculerAngle(int distance) {
  if (distance < DIST_MIN_ANGLE) return -1;
  if (distance > DIST_MAX_ANGLE) return -2;
  return map(distance, DIST_MIN_ANGLE, DIST_MAX_ANGLE, ANGLE_MIN, ANGLE_MAX);
}

void afficherLCD(int distance, int angle) {
  lcd.setCursor(0, 0);
  lcd.print("Dist : ");
  lcd.print(distance);
  lcd.print(" cm    ");

  lcd.setCursor(0, 1);
  if (angle == -1) {
    lcd.print("Obj : Trop pret");
  } else if (angle == -2) {
    lcd.print("Obj : Trop loin");
  } else {
    lcd.print("Obj : ");
    lcd.print(angle);
    lcd.print(" deg     ");
  }
}

void deplacerMoteur(int angle) {
  if (angle >= ANGLE_MIN && angle <= ANGLE_MAX) {
    int steps = map(angle, ANGLE_MIN, ANGLE_MAX, minStep, maxStep);
    if (myStepper.distanceToGo() == 0) {
      myStepper.moveTo(steps);
    }
  }
}

// Fonction pour afficher un symbole sur la matrice LED
void afficherSymbole(String symbole) {
  u8g2.clearBuffer();

  if (symbole == "✔") {
    // Coche penchée vers la droite
    u8g2.drawPixel(0, 4);
    u8g2.drawPixel(1, 5);
    u8g2.drawPixel(2, 6);
    u8g2.drawPixel(3, 5);
    u8g2.drawPixel(4, 4);
    u8g2.drawPixel(5, 3);
    u8g2.drawPixel(6, 2);
  } 
  else if (symbole == "X") {
    for (int i = 0; i < 8; i++) {
      u8g2.drawPixel(i, i);
      u8g2.drawPixel(7 - i, i);
    }
  } 
  else if (symbole == "🚫") {
    u8g2.drawCircle(3, 3, 3);       // petit cercle
    u8g2.drawLine(0, 0, 6, 6);      // ligne diagonale
  }

  u8g2.sendBuffer(); // Affiche
  delay(3000);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

