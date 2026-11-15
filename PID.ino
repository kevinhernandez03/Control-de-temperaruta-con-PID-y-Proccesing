#include <PID_v1.h>
#include <RBDdimmer.h>

// PINES
const byte PIN_LM35 = A0;
const byte PIN_DIMMER = 3;

// LIMITES
const double TEMP_MIN_SP = 20.0;
const double TEMP_MAX_SP = 70.0;
const double TEMP_CORTE = 75.0;

const int POTENCIA_MAXIMA = 95;

// TIEMPOS
const unsigned long SAMPLE_MS = 500;
const unsigned long TX_MS = 500;
const unsigned long COMM_TIMEOUT_MS = 3000;

// VARIABLES PID
double Setpoint = 40.0;
double Input = 0.0;
double Output = 0.0;

double Kp = 8.0;
double Ki = 0.15;
double Kd = 1.0;

PID pid(&Input, &Output, &Setpoint,
        Kp, Ki, Kd, DIRECT);

dimmerLamp dimmer(PIN_DIMMER);

// ESTADOS
bool runRequest = false;
bool sensorOK = false;
bool alarmTemp = false;
bool pidAutomatic = false;

int potenciaAplicada = 0;

unsigned long lastSample = 0;
unsigned long lastTx = 0;
unsigned long lastCommand = 0;

String rxLine;

// LECTURA DEL LM35
double leerTemperatura() {
  const byte N = 16;
  unsigned long suma = 0;

  for (byte i = 0; i < N; i++) {
    suma += analogRead(PIN_LM35);
    delay(2);
  }

  double adc = (double)suma / N;
  double voltaje = adc * (5.0 / 1023.0);

  return voltaje * 100.0;
}

// CONTROL DEL DIMMER
void aplicarPotencia(int potencia) {
  potencia = constrain(
    potencia, 0, POTENCIA_MAXIMA
  );

  if (potencia <= 0) {
    dimmer.setPower(0);
    dimmer.setState(OFF);
    potenciaAplicada = 0;
    return;
  }

  dimmer.setState(ON);
  dimmer.setPower(potencia);

  potenciaAplicada = potencia;
}

// DETENER PID
void detenerControl() {
  runRequest = false;
  Output = 0;

  aplicarPotencia(0);

  if (pidAutomatic) {
    pid.SetMode(MANUAL);
    pidAutomatic = false;
  }
}

// INICIAR PID
void iniciarControl() {
  if (!sensorOK || alarmTemp) {
    detenerControl();
    return;
  }

  runRequest = true;
  Output = 0;

  aplicarPotencia(0);

  pid.SetMode(MANUAL);
  pid.SetMode(AUTOMATIC);

  pidAutomatic = true;
}

// PROCESAR COMANDOS DE PROCESSING
// Formato: C,SP,KP,KI,KD,RUN
void procesarComando(String linea) {
  linea.trim();

  if (!linea.startsWith("C,")) return;

  double valores[5];
  int inicio = 2;

  for (byte i = 0; i < 5; i++) {
    int coma = linea.indexOf(',', inicio);
    String parte;

    if (i < 4) {
      if (coma < 0) return;

      parte = linea.substring(inicio, coma);
      inicio = coma + 1;
    } else {
      parte = linea.substring(inicio);
    }

    valores[i] = parte.toFloat();
  }

  Setpoint = constrain(
    valores[0], TEMP_MIN_SP, TEMP_MAX_SP
  );

  Kp = max(0.0, valores[1]);
  Ki = max(0.0, valores[2]);
  Kd = max(0.0, valores[3]);

  pid.SetTunings(Kp, Ki, Kd);

  lastCommand = millis();

  if (valores[4] >= 0.5) {
    iniciarControl();
  } else {
    detenerControl();
  }
}

// RECEPCION SERIAL
void recibirSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n') {
      procesarComando(rxLine);
      rxLine = "";
    }
    else if (c != '\r') {
      if (rxLine.length() < 100) {
        rxLine += c;
      } else {
        rxLine = "";
      }
    }
  }
}

// ENVIO DE DATOS A PROCESSING
void enviarDatos() {
  Serial.print(Input, 2);
  Serial.print(',');

  Serial.print(Setpoint, 2);
  Serial.print(',');

  Serial.print(potenciaAplicada);
  Serial.print(',');

  Serial.println(
    (runRequest && sensorOK && !alarmTemp) ? 1 : 0
  );
}

// CONFIGURACION INICIAL
void setup() {
  Serial.begin(9600);
  analogReference(DEFAULT);

  // Iniciar dimmer apagado
  dimmer.begin(NORMAL_MODE, OFF);
  dimmer.setPower(0);
  dimmer.setState(OFF);

  Input = leerTemperatura();

  sensorOK = (Input >= -5.0 && Input <= 125.0);

  pid.SetOutputLimits(0, POTENCIA_MAXIMA);
  pid.SetSampleTime(SAMPLE_MS);
  pid.SetMode(MANUAL);

  lastCommand = millis();
  lastSample = millis();
  lastTx = millis();
}

// PROGRAMA PRINCIPAL
void loop() {
  recibirSerial();

  unsigned long ahora = millis();

  if (ahora - lastSample >= SAMPLE_MS) {
    lastSample = ahora;

    Input = leerTemperatura();

    sensorOK = (Input >= -5.0 && Input <= 125.0);

    if (!sensorOK || Input >= TEMP_CORTE) {
      alarmTemp = true;
      detenerControl();
    }

    if (runRequest && sensorOK && !alarmTemp) {

      if (!pidAutomatic) {
        pid.SetMode(AUTOMATIC);
        pidAutomatic = true;
      }

      pid.Compute();

      aplicarPotencia((int)round(Output));

    } else {
      aplicarPotencia(0);
    }
  }

  // APAGADO SI SE PIERDE LA COMUNICACION
  if (runRequest &&
      ahora - lastCommand > COMM_TIMEOUT_MS) {
    detenerControl();
  }

  // ENVIO PERIODICO
  if (ahora - lastTx >= TX_MS) {
    lastTx = ahora;
    enviarDatos();
  }
}



