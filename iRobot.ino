#include <avr/interrupt.h> // bibliothèque pour les capteurs Infrarouges
#include <Servo.h> //bibliothèque pour les servomoteurs

#define capteur1 9 // capteur infrarouge gauche sur la PIN 9 de l'Arduino (Port B bit 1)
#define emetteur1 8 // Emetteur LED infrarouge gauche sur la PIN 8 de l'Arduino (Port B bit 0)

#define capteur2 7 // capteur infrarouge droite sur la PIN 7 de l'Arduino (Port D bit 7)
#define emetteur2 6 // Emetteur LED infrarouge droite sur la PIN 6 de l'Arduino (Port D bit 6)

#define echoPin 3 // capteur ultrason sur la PIN 3 de l'Arduino (Port D bit 3)
#define trigPin 2 // émetteur ultrason sur la PIN 3 de l'Arduino (Port D bit 2)


// defines variables
long duration; // variable for the duration of sound wave travel
float distance; // variable for the distance measurement

Servo servoLeft; //variables pour les servomoteurs
Servo servoRight; //variables pour les servomoteurs

unsigned int frequence = 38000; // fréquence d'impulsion pour les capteurs infrarouges


void setup() {
  
  
  servoLeft.attach(12); // le servomoteur gauche est relié à la PIN 12
  servoRight.attach(13); // le servomoteur est relié à la PIN 13
  DDRD = setbit(DDRD,trigPin); //émetteur ultrason configuré en SORTIE
  DDRD = clearbit(DDRD,echoPin); //capteur ultrason configuré en ENTREE
  DDRD = setbit(DDRD,emetteur2); // Emetteur LED infrarouge droite configuré en SORTIE (Port D bit 6)
  DDRD = clearbit(DDRD,capteur2); // Capteur LED infrarouge droite configuré en SORTIE (Port D bit 7)
  DDRB = setbit(DDRB, 0); // Emetteur LED infrarouge gauche configuré en SORTIE (Port B bit 0)
  DDRB = clearbit(DDRB,1); //Capteur LED infrarouge gauche configuré en ENTREE (Port B bit 1)
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Super Cool Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
}

void loop() {
  
  int result; // variable indiquant si l'ultrason a détecté un obstacle à moins de 25 cm
  int resultIRgauche, resultIRdroite; // variables indiquant si les IR gauche et droite ont détecté des obstacles
  
  result = ultrasoundfront(); // voir définition de la fonction à la suite
  resultIRgauche = leftObstacle(); //voir définition de la fonction à la suite
  resultIRdroite = rightObstacle(); //voir définition de la fonction à la suite
  
 if (result == 1){ //si l'ultrason détecte un obstacle à moins de 25 cm (il ne peut plus avancer)
       if((resultIRgauche == 2) && (resultIRdroite ==128)){ //cas où les capteurs infrarouge ne détectent pas d'obstacle à gauche ou à droite
        Serial.println("pas d'obstacle à droite ou à gauche");
        stayStill(); //arrêter l'avancée du robot
        delay(500);
        reverse(); // faire reculer le robot pour qu'il ait la place pour tourner
        delay(500);
        left(); // aller à gauche
        delay(500);
     
          
      }else if(resultIRgauche == 2 && resultIRdroite == 0){ //cas où les capteurs IR ne détectent qu'un obstacle à droite mais pas à gauche
        Serial.println("obstacle à droite");
        stayStill(); //arrêter l'avancée du robot
        delay(500);
        left(); // aller à gauche
        delay(500);
        forward(); // recommencer l'avancée du robot
        delay(500);
       
      }else if(resultIRgauche == 0 && resultIRdroite == 128){ //cas où les capteurs IR ne détectent qu'un obstacle à gauche mais pas à droite
         Serial.println("obstacle à gauche");
        stayStill(); //arrêter l'avancée du robot
        delay(500);
        right(); // aller à droite
        delay(500);
        forward(); // recommencer l'avancée du robot
        delay(500);
    
      }else{
        Serial.println("obstacle partout"); // cas où les détecteurs IR détectent des obstacles à droite et à gauche (le robot est dans un cul-de-sac)
        reverse(); //aller en arrière
        delay(500);
  
      }
  }else{ // cas où le robot ne détecte pas d'obstacle devant lui via l'ultrason
    Serial.println("pas d'obstacle");
   forward(); // continuer vers l'avant
   delay(500);

    }
}


void forward (){ // fonction qui fait avancer le robot en ligne droite
  servoLeft.writeMicroseconds(1300); 
  servoRight.writeMicroseconds(1700);
}

void stayStill(){ // fonction qui fait s'arrêter le robot
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
}

void left(){ // fonction qui fait tourner le robot à gauche
  servoLeft.writeMicroseconds(1300);
  servoRight.writeMicroseconds(1300);
}
  
void right(){ // fonction qui fait tourner le robot à droite
  servoLeft.writeMicroseconds(1700);
  servoRight.writeMicroseconds(1700);
}
  
void reverse(){ // fonction qui fait aller le robot en arrière
  servoLeft.writeMicroseconds(1700);
  servoRight.writeMicroseconds(1300);
}
  
int ultrasoundfront(){ // fonction qui indique si le robot détecte un obstacle imminent devant lui

  // Clears the trigPin condition
  delay(500);
  PORTD = clearbit(PORTD,trigPin); 
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  PORTD = setbit(PORTD,trigPin); 
  delayMicroseconds(10);
  PORTD = clearbit(PORTD,trigPin); 
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  //Si le capteur décroche et affiche 0
  if (duration == 0){
   pinMode(echoPin,OUTPUT);
   digitalWrite(echoPin,LOW);
   delayMicroseconds(100);
   pinMode(echoPin,INPUT);
   }else {
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
   }
  if (distance>25){ //si la distance détectée est supérieure à 25 cm
    return 0;
  }else{ // si un obstacle est détecté à moins de 25 cm
    return 1;
  }
}

  
int rightObstacle(){ // fonction qui indique si un obstacle est détecté à droite
   tone(emetteur2, frequence, 30); // fonction envoyant une impulsion de 30 mS de fréquence 38kHz (voir variables globales) sur l'émetteur
   int v = PIND&=(1<<capteur2); //on vient lire la valeur du capteur gauche
   //Serial.println(v); // valeur si pas d'obstacle = 128 (bit 7 du port D et entrée en état haut par défaut) valeur si obstacle = 0
   return v;
  }
  
int leftObstacle(){ // fonction qui indique si un obstacle est détecté à gauche
  tone(emetteur1, frequence,30); // fonction envoyant une impulsion de 30 mS fréquence 38kHz (voir variables globales) sur l'émetteur
   int v = PINB&=(1<<1); //on vient lire la valeur du capteur droite
   Serial.println(v); // valeur si pas d'obstacle = 2 (bit 1 du port B et entrée en état haut par défaut) valeur si obstacle = 0
   return v;
  }



//Register Functions
uint8_t setbit (uint8_t reg, uint8_t p) { //fonction pour mettre à 1 un bit p d'un registre reg
  reg |= (1 << p);
  return reg;
}
uint8_t clearbit (uint8_t reg, uint8_t p) { //fonction pour mettre à 0 un bit p d'un registre reg
  reg &= ~(1 << p);
  return reg;
}
uint8_t togglebit (uint8_t reg, uint8_t p) { //fonction pour changer l'état d'un bit p d'un registre p
  reg ^= (1 << p); // XOR
  return reg;
}

bool checkbit (uint8_t reg, int p){ //fonction pour vérifier l'état du bit p d'un registre reg
    bool x = reg & (1<<p); 
    return  x;
}
