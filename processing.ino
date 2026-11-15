import processing.serial.*;
import controlP5.*;

Serial arduino;
ControlP5 cp5;

String SERIAL_PORT = "COM3";
int BAUD_RATE = 9600;

// VARIABLES
float temperatura = 0;
float setpoint = 40;
float potencia = 0;

int estado = 0;

float kp = 8.0;
float ki = 0.15;
float kd = 1.0;

boolean conectado = false;

String mensaje = "Conectando...";

ArrayList<Float> tempHist = new ArrayList<Float>();
ArrayList<Float> spHist = new ArrayList<Float>();

int maxPuntos = 240;

// CONFIGURACION
void setup() {
  size(1000, 650);

  surface.setTitle("Control PID de Temperatura");

  cp5 = new ControlP5(this);

  cp5.addTextfield("SP")
    .setPosition(30, 90)
    .setSize(90, 28)
    .setText("40")
    .setAutoClear(false);

  cp5.addTextfield("KP")
    .setPosition(140, 90)
    .setSize(90, 28)
    .setText("8")
    .setAutoClear(false);

  cp5.addTextfield("KI")
    .setPosition(250, 90)
    .setSize(90, 28)
    .setText("0.15")
    .setAutoClear(false);

  cp5.addTextfield("KD")
    .setPosition(360, 90)
    .setSize(90, 28)
    .setText("1")
    .setAutoClear(false);

  cp5.addButton("APLICAR")
    .setPosition(470, 90)
    .setSize(100, 28);

  cp5.addButton("START")
    .setPosition(600, 90)
    .setSize(100, 35);

  cp5.addButton("STOP")
    .setPosition(720, 90)
    .setSize(100, 35);

  try {
    arduino = new Serial(
      this, SERIAL_PORT, BAUD_RATE
    );

    arduino.bufferUntil('\n');

    conectado = true;
    mensaje = "Conectado a " + SERIAL_PORT;

  } catch (Exception e) {
    conectado = false;
    mensaje = "No se pudo abrir " + SERIAL_PORT;
  }
}

// INTERFAZ
void draw() {
  background(245);

  fill(25);
  textSize(24);

  text(
    "CONTROL DE TEMPERATURA - PID",
    30, 40
  );

  fill(60);
  textSize(13);

  text("Setpoint / Kp / Ki / Kd", 30, 75);

  fill(30);
  textSize(16);

  text(
    "Temperatura: " +
    nf(temperatura, 0, 2) + " °C",
    30, 165
  );

  text(
    "Setpoint: " +
    nf(setpoint, 0, 2) + " °C",
    30, 195
  );

  text(
    "Salida aplicada: " +
    nf(potencia, 0, 1) + " %",
    30, 225
  );

  text(
    "Estado: " +
    (estado == 1 ? "ACTIVO" : "DETENIDO"),
    30, 255
  );

  fill(
    conectado ?
    color(0, 130, 60) :
    color(180, 30, 30)
  );

  text(mensaje, 30, 285);

  dibujarGrafica(30, 330, 920, 275);
}

// RECEPCION SERIAL
void serialEvent(Serial p) {
  String linea = p.readStringUntil('\n');

  if (linea == null) return;

  String[] datos = split(trim(linea), ',');

  if (datos.length >= 4) {
    try {
      temperatura = float(datos[0]);
      setpoint = float(datos[1]);
      potencia = float(datos[2]);
      estado = int(datos[3]);

      tempHist.add(temperatura);
      spHist.add(setpoint);

      if (tempHist.size() > maxPuntos) {
        tempHist.remove(0);
        spHist.remove(0);
      }

    } catch (Exception e) {
      // Ignorar datos incompletos
    }
  }
}

// ENVIAR COMANDO A ARDUINO
void enviarComando(boolean run) {

  if (!conectado || arduino == null) {
    mensaje = "Arduino no conectado";
    return;
  }

  String comando =
    "C," +
    nf(setpoint, 0, 2) + "," +
    nf(kp, 0, 3) + "," +
    nf(ki, 0, 3) + "," +
    nf(kd, 0, 3) + "," +
    (run ? "1" : "0") + "\n";

  arduino.write(comando);
}

// BOTON APLICAR
public void APLICAR() {
  try {
    setpoint = constrain(
      float(cp5.get(Textfield.class, "SP").getText()),
      20, 70
    );

    kp = max(
      0,
      float(cp5.get(Textfield.class, "KP").getText())
    );

    ki = max(
      0,
      float(cp5.get(Textfield.class, "KI").getText())
    );

    kd = max(
      0,
      float(cp5.get(Textfield.class, "KD").getText())
    );

    enviarComando(estado == 1);

    mensaje = "Parametros enviados";

  } catch (Exception e) {
    mensaje = "Verifica los valores ingresados";
  }
}

// BOTON START
public void START() {
  enviarComando(true);
  mensaje = "Orden START enviada";
}

// BOTON STOP
public void STOP() {
  enviarComando(false);

  estado = 0;
  potencia = 0;

  mensaje = "Orden STOP enviada";
}

// GRAFICA EN TIEMPO REAL
void dibujarGrafica(
  int x, int y, int w, int h
) {
  stroke(190);
  fill(255);

  rect(x, y, w, h);

  for (int i = 0; i <= 5; i++) {
    float yy = map(i, 0, 5, y + h, y);

    stroke(225);
    line(x, yy, x + w, yy);

    fill(80);
    textSize(10);

    text(
      str(i * 20),
      x - 22, yy + 4
    );
  }

  if (tempHist.size() < 2) {
    fill(90);
    textSize(14);

    text(
      "Esperando datos del Arduino...",
      x + 20, y + 30
    );

    return;
  }

  // TEMPERATURA
  noFill();
  stroke(30, 110, 210);
  strokeWeight(2);

  beginShape();

  for (int i = 0; i < tempHist.size(); i++) {
    float xx = map(
      i, 0, maxPuntos - 1,
      x, x + w
    );

    float yy = map(
      constrain(tempHist.get(i), 0, 100),
      0, 100, y + h, y
    );

    vertex(xx, yy);
  }

  endShape();

  // SETPOINT
  stroke(220, 80, 50);
  strokeWeight(2);

  beginShape();

  for (int i = 0; i < spHist.size(); i++) {
    float xx = map(
      i, 0, maxPuntos - 1,
      x, x + w
    );

    float yy = map(
      constrain(spHist.get(i), 0, 100),
      0, 100, y + h, y
    );

    vertex(xx, yy);
  }

  endShape();

  strokeWeight(1);

  fill(30, 110, 210);
  text("Temperatura", x + 15, y + 20);

  fill(220, 80, 50);
  text("Setpoint", x + 115, y + 20);
}

