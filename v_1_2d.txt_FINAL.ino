// =====================================================
//  TDOA entopismos hxou - arduino version tou matlab kwdika
//  3 mikrofwna me digital eksodo (an akousei -> to pin paei HIGH)
// =====================================================
//FINAL
//baud 115200 min ksexaseis na to valeis k sto serial monitor!!
// ---- 1) pins k theseis mikrofwnwn (se metra) ----
const int MIC1_PIN = 2;   // pins tou arduino gia ta 3 mic
const int MIC2_PIN = 3;
const int MIC3_PIN = 4;

const float m1x = 0.0, m1y = 0.0;   // m1 = [0; 0]
const float m2x = 0.2, m2y = 0.0;   // m2 = [0.2; 0]  (20cm)
const float m3x = 0.0, m3y = 0.2;   // m3 = [0; 0.2]
const float c   = 343.0;            // taxitita hxou m/s

// max xronos p perimenoume ta alla mic afou akousei to prwto
// (4m * 1.414 / 343 ~ 16.5ms ara 20ms ftanei k perisevei)
const unsigned long TIMEOUT_US = 20000;

// -----------------------------------------------------
// edw metrame tis kathisteriseis apo ta mic (anti gia to s_true tou matlab)
// epistrefei tau21 k tau31 se sec
// -----------------------------------------------------
bool measureDelays(float &tau21, float &tau31) {
  unsigned long t1 = 0, t2 = 0, t3 = 0;

  // a) perimenei mexri na akousei OPOIODIPOTE mic
  while (t1 == 0 && t2 == 0 && t3 == 0) {
    unsigned long now = micros();
    if (digitalRead(MIC1_PIN) == HIGH) t1 = now;
    if (digitalRead(MIC2_PIN) == HIGH) t2 = now;
    if (digitalRead(MIC3_PIN) == HIGH) t3 = now;
  }

  // b) perimenei k ta ipolipa (me timeout gia na min kollisei)
  unsigned long start = micros();
  while ((t1 == 0 || t2 == 0 || t3 == 0) && (micros() - start < TIMEOUT_US)) {
    unsigned long now = micros();
    if (t1 == 0 && digitalRead(MIC1_PIN) == HIGH) t1 = now;
    if (t2 == 0 && digitalRead(MIC2_PIN) == HIGH) t2 = now;
    if (t3 == 0 && digitalRead(MIC3_PIN) == HIGH) t3 = now;
  }

  if (t1 == 0 || t2 == 0 || t3 == 0) return false;   // kapoio mic den to akouse

  // c) diafores xronou (us -> sec)
  tau21 = (long)(t2 - t1) * 1e-6;   // mic2 - mic1
  tau31 = (long)(t3 - t1) * 1e-6;   // mic3 - mic1
  return true;
}

// -----------------------------------------------------
// solver - ta idia maths me ta vimata 3-7 tou matlab
// -----------------------------------------------------
bool solveTDOA(float tau21, float tau31, float &x, float &y, float &r1) {
  // ---- 3) b0 k b1 ----
  float d21 = c * tau21;
  float d31 = c * tau31;

  float K1 = m1x * m1x + m1y * m1y;   // K(m) = m.' * m
  float K2 = m2x * m2x + m2y * m2y;
  float K3 = m3x * m3x + m3y * m3y;

  float b0_1 = 0.5 * (K2 - K1 - d21 * d21);
  float b0_2 = 0.5 * (K3 - K1 - d31 * d31);
  float b1_1 = d21;
  float b1_2 = d31;

  // ---- 4) s0 = A\b0 k s1 = A\b1 ----
  // i c++ den exei to "\" opws to matlab opote lunoume to 2x2 me to xeri (cramer)
  float a11 = m2x - m1x, a12 = m2y - m1y;
  float a21 = m3x - m1x, a22 = m3y - m1y;
  float det = a11 * a22 - a12 * a21;
  if (fabs(det) < 1e-9) return false;

  float s0x = (b0_1 * a22 - a12 * b0_2) / det;
  float s0y = (a11 * b0_2 - b0_1 * a21) / det;
  float s1x = (b1_1 * a22 - a12 * b1_2) / det;
  float s1y = (a11 * b1_2 - b1_1 * a21) / det;

  // ---- 5) alpha, beta, gamma ----
  float alpha = (s1x * s1x + s1y * s1y) - 1.0;
  float beta  = -2.0 * (s0x * s1x + s0y * s1y);
  float gamma = s0x * s0x + s0y * s0y;

  // ---- 6) r1 apo tin deuterovathmia ----
  // oute roots() exei, pame me diakrinousa
  if (fabs(alpha) < 1e-6) {            // sxedon grammiki periptwsi
    r1 = -gamma / beta;
  } else {
    float disc = beta * beta - 4.0 * alpha * gamma;
    if (disc < 0) return false;        // den exei pragmatiki lisi
    float rA = (-beta + sqrt(disc)) / (2.0 * alpha);
    float rB = (-beta - sqrt(disc)) / (2.0 * alpha);
    if (rA > 0)      r1 = rA;          // kratame tin thetiki riza
    else if (rB > 0) r1 = rB;
    else return false;
  }
  if (r1 <= 0) return false;

  // ---- 7) s = s0 - s1 * r1 ----
  x = s0x - s1x * r1;
  y = s0y - s1y * r1;
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(MIC1_PIN, INPUT);
  pinMode(MIC2_PIN, INPUT);
  pinMode(MIC3_PIN, INPUT);
  Serial.println("TDOA locator ready...");
}

// ola trexoun sto loop sinexeia
void loop() {
  float tau21, tau31;

  if (!measureDelays(tau21, tau31)) {
    Serial.println("Timeout - not all mics heard the sound");
    delay(200);
    return;
  }

  float x, y, r1;
  if (solveTDOA(tau21, tau31, x, y, r1)) {
    // ---- 8) tipwnoume ta apotelesmata ----
    Serial.println("--- TDOA SOLVER RESULTS ---");
    Serial.print("tau21 (us): "); Serial.println(tau21 * 1e6);
    Serial.print("tau31 (us): "); Serial.println(tau31 * 1e6);
    Serial.print("Position: [");  Serial.print(x, 4);
    Serial.print("; ");           Serial.print(y, 4); Serial.println("]");
    Serial.print("r1 (m): ");     Serial.println(r1, 4);
  } else {
    Serial.println("No valid solution");
  }

  delay(200);   // perimenoume na svisoun t echo prin tin epomeni metrisi
}
