#include <Servo.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

#include "DFRobotDFPlayerMini.h"

SoftwareSerial softSerial(12,13);
DFRobotDFPlayerMini myDFPlayer;

// Configuración del teclado 4x4
const byte FILAS = 4;    // Número de filas
const byte COLUMNAS = 4; // Número de columnas

char teclas[FILAS][COLUMNAS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte pinesFilas[FILAS] = {10, 8, 7, 6};     // Pines de las filas
byte pinesColumnas[COLUMNAS] = {5, 4, 3, 2}; // Pines de las columnas

Keypad teclado = Keypad(makeKeymap(teclas), pinesFilas, pinesColumnas, FILAS, COLUMNAS);

// Configuración del servo de rotación continua
Servo servoMotor;            // Servo principal (ascensor)
const int servoPin = 9;      // Pin del servo principal

// Configuración del servo para la puerta
Servo doorServo;             // Servo para la puerta
const int doorServoPin = 11; // Pin del servo de la puerta
const int doorOpenAngle = 40;  // Ángulo para abrir la puerta
const int doorCloseAngle = 0;  // Ángulo para cerrar la puerta

// Configuración del LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Dirección I2C 0x27, LCD de 16 columnas y 2 filas

// Configuración del sensor infrarrojo y LEDs
const int irSensorPin = A3;  // Pin del sensor infrarrojo
const int ledGreenPin = A1;  // Pin del LED verde
const int ledRedPin = A0;    // Pin del LED rojo

// Tiempo estimado para moverse entre pisos (ajustar según pruebas)
const int timeBetweenFloors = 425; // Tiempo en milisegundos para moverse entre pisos

// Piso actual
int currentFloor = 1;  // Empezamos en el piso 1

void setup() {
  //musica
  softSerial.begin(9600);
  // Inicializar el LCD
  lcd.init();
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Sistema Iniciado");
  delay(2000);
  lcd.clear();

  // Configurar los servos
  servoMotor.attach(servoPin);
  servoMotor.write(90);  // Detener el servo inicialmente
  doorServo.attach(doorServoPin);
  doorServo.write(doorCloseAngle);  // Inicializar la puerta cerrada

  // Configurar pines del sensor y LEDs
  pinMode(irSensorPin, INPUT);
  pinMode(ledGreenPin, OUTPUT);
  pinMode(ledRedPin, OUTPUT);

  // Apagar LEDs al inicio
  digitalWrite(ledGreenPin, LOW);
  digitalWrite(ledRedPin, LOW);

  // Configurar el monitor serial para depuración
  Serial.begin(9600);

  //%%%%%%%%%%%%%% Musica %%%%%%%%%%%%

 if (!myDFPlayer.begin(softSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    delay(1000);
  }

  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms

  //----Set volume----
  myDFPlayer.volume(30);  //Set volume value (0~30).
  myDFPlayer.volumeUp(); //Volume Up
  myDFPlayer.volumeDown(); //Volume Down

  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);

  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  myDFPlayer.playMp3Folder(1); //play specific mp3 in SD:/MP3/0001.mp3; File Name(0~65535)
  
}

void loop() {
  updateSensorStatus();  // Verifica el estado del sensor infrarrojo

  char key = teclado.getKey();  // Leer la tecla presionada

  if (key) {  // Si hay una tecla presionada
    Serial.print("Tecla presionada: ");
    Serial.println(key);

    // Mostrar la tecla presionada en el LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Tecla: ");
    lcd.print(key);

    // Verificar qué tecla fue presionada y realizar acciones
    switch (key) {
      case '1':
        moveElevatorToFloor(1);  // Mover al piso 1
        break;
      case '2':
        moveElevatorToFloor(2);  // Mover al piso 2
        break;
      case '3':
        moveElevatorToFloor(3);  // Mover al piso 3
        break;
      case '4':
        moveElevatorToFloor(4);  // Mover al piso 4
        break;
      case '5':
        moveElevatorToFloor(5);  // Mover al piso 5
        break;
      case '6':
        moveElevatorToFloor(6);  // Mover al piso 6
        break;
      default:
        lcd.setCursor(0, 1);
        lcd.print("Tecla no valida");
        break;
    }
  }
}

// Función para mover el ascensor a un piso específico
void moveElevatorToFloor(int floor) {
  if (floor == currentFloor) {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Ya en piso ");
    lcd.print(floor);
    return;
  }

  // Verificar si hay un obstáculo
  if (digitalRead(irSensorPin) == LOW) { // Cambiado: Ahora detecta obstáculos en LOW
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Obstaculo! No");
    lcd.setCursor(0, 1);
    lcd.print("puede moverse");
    Serial.println("Movimiento bloqueado por obstáculo");
    delay(2000);
    return; // Cancela el movimiento
  }

  int direction = (floor > currentFloor) ? 180 : 0; // Determinar dirección
  int floorsToMove = abs(floor - currentFloor);     // Calcular pisos a mover

  // Cerrar la puerta antes de moverse
  closeDoor();

  // Mostrar movimiento en LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Moviendo a piso ");
  lcd.print(floor);

  // Mover el ascensor
  servoMotor.write(direction); // Iniciar el giro
  delay(timeBetweenFloors * floorsToMove); // Tiempo proporcional a los pisos
  servoMotor.write(90); // Detener el giro

  // Actualizar piso actual
  currentFloor = floor;

  // Abrir la puerta al llegar
  openDoor();
}

// Función para cerrar la puerta
void closeDoor() {
  if (digitalRead(irSensorPin) == LOW) { // Cambiado: Detecta obstáculos en LOW
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Obstaculo! No");
    lcd.setCursor(0, 1);
    lcd.print("puede cerrar");
    Serial.println("Puerta bloqueada por obstáculo");
    delay(2000);
    return; // Cancela el cierre de la puerta
  }

  doorServo.write(doorCloseAngle);  // Cierra la puerta
  Serial.println("Puerta cerrada");
  lcd.setCursor(0, 1);
  lcd.print("Puerta cerrada");
  delay(1000);
}

// Función para abrir la puerta
void openDoor() {
  doorServo.write(doorOpenAngle);  // Mover el servo de la puerta a posición abierta
  Serial.println("Puerta abierta");
  lcd.setCursor(0, 1);
  lcd.print("Puerta abierta ");
  delay(1000);
}

// Función para gestionar el sensor infrarrojo y LEDs
void updateSensorStatus() {
  int sensorState = digitalRead(irSensorPin);  // Leer estado del sensor

  if (sensorState == LOW) {  // Cambiado: Obstáculo detectado en LOW
    digitalWrite(ledRedPin, HIGH);   // Enciende LED rojo
    digitalWrite(ledGreenPin, LOW); // Apaga LED verde
    lcd.setCursor(0, 1);
    lcd.print("Obstaculo detect.");
    Serial.println("Obstáculo detectado");
  } else {  // No hay obstáculo
    digitalWrite(ledRedPin, LOW);    // Apaga LED rojo
    digitalWrite(ledGreenPin, HIGH); // Enciende LED verde
    lcd.setCursor(0, 1);
    lcd.print("Libre           ");
    Serial.println("Libre de obstaculos");
  }
}





