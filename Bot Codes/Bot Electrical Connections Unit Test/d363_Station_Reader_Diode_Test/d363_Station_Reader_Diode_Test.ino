const String CODE_ID = "d363-StationReaderDiodeStandaloneTest";

int SR_D1 = 36;
int SR_D2 = 39;
int SR_D3 = 34;

void setup() {
  Serial.begin(115200);
  Serial.println("Bot: BB | Code ID: " + CODE_ID);
  Serial.println(__FILE__);
}

void loop() {
  int x1 = map(analogRead(SR_D1), 0, 4096, 0, 1024);
  int x2 = map(analogRead(SR_D2), 0, 4096, 0, 1024);
  int x3 = map(analogRead(SR_D3), 0, 4096, 0, 1024);
  Serial.println(String(x1) + "-" + String(x2) + "-" + String(x3));
}