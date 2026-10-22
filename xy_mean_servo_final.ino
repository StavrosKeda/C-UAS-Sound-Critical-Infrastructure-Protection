// 3 HC-SR04 aisthitires vriskoun to X k Y tou target (cm)
// kratame tis teleutees 5 metriseis, petame osa einai makria apo tin mesaia timi
// k me ton meso oro twn upoloipwn girizei to servo
//
// aisthitires:  1 = TRIG 4 / ECHO 5    2 = TRIG 6 / ECHO 7    3 = TRIG 8 / ECHO 9
// servo:    to portokali kalwdio sto pin 3
// eksodos:   X, Y, mesos X/Y, poses kratise, gwnia k gwnia servo (9600 baud!!)

#include <Servo.h>
#include <math.h>

const int TRIG1 = 4, ECHO1 = 5;
const int TRIG2 = 6, ECHO2 = 7;
const int TRIG3 = 8, ECHO3 = 9;
const int SERVO_PIN = 3;

Servo servo;

// simeio i dianusma 3D
struct Vec3 {
  float x, y, z;
};

// theseis aisthitirwn se cm (metrimenes panw sti vasi)
const Vec3 S[3] = {
  { 0.0,  -3.0, 5.0},
  { 5.0,   0.0, 0.0},
  {-5.0,   0.0, 0.0}
};

// to servo einai sto 0,0
const Vec3 PIVOT = {0.0, 0.0, 0.0};

// poses metriseis kratame k poso makria apo tin mesaia epitrepetai (cm)
const int N = 5;
const float MAX_ERR = 5.0;

// oi teleutees N metriseis
float bufX[N], bufY[N];
int bufCount = 0;
int bufIndex = 0;

// voithitika gia dianusmata
Vec3 sub(Vec3 a, Vec3 b)   { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
float dot(Vec3 a, Vec3 b)  { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vec3 cross(Vec3 a, Vec3 b) {
  return { a.y*b.z - a.z*b.y,
           a.z*b.x - a.x*b.z,
           a.x*b.y - a.y*b.x };
}

// apostasi se cm apo ena sensor, -1 an den girisei echo
float readCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long t = pulseIn(echoPin, HIGH, 30000);
  if (t == 0) return -1.0;
  return t / 58.0;
}

// vriskei to target apo tis 3 apostaseis (trilateration)
bool locate(float r1, float r2, float r3, Vec3 &out) {
  Vec3 u = sub(S[1], S[0]);
  Vec3 v = sub(S[2], S[0]);

  float uu = dot(u, u);
  float vv = dot(v, v);
  float uv = dot(u, v);

  float A = (r1*r1 - r2*r2 + uu) / 2.0;
  float B = (r1*r1 - r3*r3 + vv) / 2.0;

  float det = uu*vv - uv*uv;
  if (fabs(det) < 1e-3) return false;

  float al = (vv*A - uv*B) / det;
  float be = (uu*B - uv*A) / det;

  Vec3 q0 = { al*u.x + be*v.x, al*u.y + be*v.y, al*u.z + be*v.z };
  Vec3 n  = cross(u, v);

  float t2 = (r1*r1 - dot(q0, q0)) / dot(n, n);
  if (t2 < 0) t2 = 0;
  float t = sqrt(t2);

  Vec3 pa = { S[0].x + q0.x + t*n.x, S[0].y + q0.y + t*n.y, S[0].z + q0.z + t*n.z };
  Vec3 pb = { S[0].x + q0.x - t*n.x, S[0].y + q0.y - t*n.y, S[0].z + q0.z - t*n.z };

  out = (pa.z > pb.z) ? pa : pb;   // kratame tin lusi pou einai panw
  return true;
}

// taksinomisi mikrou pinaka (insertion sort, ok gia 5 times)
void sortFloats(float *a, int n) {
  for (int i = 1; i < n; i++) {
    float key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = key;
  }
}

// mesos oros mono apo tis metriseis p einai konta stin mesaia timi
// epistrefei poses kratise
int meanOfGood(float &mx, float &my) {
  float sx[N], sy[N];
  for (int i = 0; i < bufCount; i++) {
    sx[i] = bufX[i];
    sy[i] = bufY[i];
  }
  sortFloats(sx, bufCount);
  sortFloats(sy, bufCount);
  float medX = sx[bufCount / 2];   // i mesaia timi
  float medY = sy[bufCount / 2];

  float sumX = 0, sumY = 0;
  int kept = 0;
  for (int i = 0; i < bufCount; i++) {
    if (fabs(bufX[i] - medX) <= MAX_ERR && fabs(bufY[i] - medY) <= MAX_ERR) {
      sumX += bufX[i];
      sumY += bufY[i];
      kept++;
    }
  }

  if (kept == 0) {          // den prepei na ginei pote alla just in case
    mx = medX;
    my = medY;
    return 0;
  }
  mx = sumX / kept;
  my = sumY / kept;
  return kept;
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT); pinMode(ECHO3, INPUT);

  servo.attach(SERVO_PIN);
  servo.write(90);           // ksekinaei sto kentro
}

void loop() {
  float r1 = readCm(TRIG1, ECHO1);
  delay(50);
  float r2 = readCm(TRIG2, ECHO2);
  delay(60);
  float r3 = readCm(TRIG3, ECHO3);   // delay anamesa gia na min akouei o enas to echo tou allou

  if (r1 < 0 || r2 < 0 || r3 < 0) {
    Serial.println("No echo on a sensor");
    delay(200);
    return;
  }

  Vec3 target;
  if (!locate(r1, r2, r3, target)) {
    Serial.println("Sensor layout is invalid");
    return;
  }

  // vazoume tin metrisi sto buffer twn 5
  bufX[bufIndex] = target.x;
  bufY[bufIndex] = target.y;
  bufIndex = (bufIndex + 1) % N;
  if (bufCount < N) bufCount++;

  // mesos oros twn kalwn
  float mx, my;
  int kept = meanOfGood(mx, my);

  // dianusma apo to servo pros ton meso, epipedo (mono X k Y)
  Vec3 mean = {mx, my, 0.0};
  Vec3 d = sub(mean, PIVOT);

  // gwnia metaksi dianusmatos k aksona X me eswteriko ginomeno
  float turn = 0.0;
  float len = sqrt(dot(d, d));
  if (len > 0.001) {
    Vec3 xAxis = {1.0, 0.0, 0.0};
    float cosA = dot(d, xAxis) / len;
    if (cosA > 1.0)  cosA = 1.0;
    if (cosA < -1.0) cosA = -1.0;
    turn = acos(cosA) * 180.0 / PI;
    if (d.y < 0) turn = -turn;
  }

  // periorismos 50-120 gia na min xtipaei to servo
  float angle = constrain( turn, 50, 120.0);
  servo.write(round(angle));

  Serial.print("X: ");  Serial.print(target.x, 1);
  Serial.print(" cm   Y: "); Serial.print(target.y, 1);
  Serial.print(" cm   mean X: "); Serial.print(mx, 1);
  Serial.print(" Y: "); Serial.print(my, 1);
  Serial.print("   kept: "); Serial.print(kept);
  Serial.print("   turn: "); Serial.print(turn, 1);
  Serial.print(" deg   servo: "); Serial.print(round(angle));
  Serial.println(" deg");

  delay(50);
}
