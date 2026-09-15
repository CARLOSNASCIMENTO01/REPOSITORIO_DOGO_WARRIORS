# 1 "/home/pi/ROBO_DOGO_2026/codigo_arduino/codigo_arduino.ino"
# 1 "/home/pi/ROBO_DOGO_2026/codigo_arduino/codigo_arduino.ino"
String buffer = "";

bool sonarVez = false;
float distE = 0;
float distF = 0;

unsigned long ultimoSonar = 0;
unsigned long ultimoEnvio = 0;






float lerSonar(int trig, int echo) {
  digitalWrite(trig, 0x0);
  delayMicroseconds(2);
  digitalWrite(trig, 0x1);
  delayMicroseconds(10);
  digitalWrite(trig, 0x0);
  long duracao = pulseIn(echo, 0x1, 30000);
  if (duracao == 0) return 255;
  float dist = duracao * 0.034 / 2.0;
  if (dist < 1 || dist > 255) return 255;
  return dist;
}

void atualizarSonares() {
  if (millis() - ultimoSonar < 60) return;
  ultimoSonar = millis();
  if (sonarVez)
    distF = lerSonar(6, 5);
  else
    distE = lerSonar(3, 2);
  sonarVez = !sonarVez;
}

void enviarDados() {
  Serial.print("F:");
  Serial.print(distF, 1);
  Serial.print(",E:");
  Serial.println(distE, 1);
}

void piscarLED() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(13, 0x1);
    delay(100);
    digitalWrite(13, 0x0);
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(13, 0x1);
  pinMode(3, 0x1);
  pinMode(2, 0x0);
  pinMode(6, 0x1);
  pinMode(5, 0x0);
}

void loop() {
  atualizarSonares();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer.trim();
      if (buffer == "START") piscarLED();
      buffer = "";
    } else {
      buffer += c;
    }
  }

  if (millis() - ultimoEnvio >= 10) {
    enviarDados();
    ultimoEnvio = millis();
  }
}
