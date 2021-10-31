#include "Arduino.h"
#include "Oscillator.h"
#include <Servo.h>
#include "NeoSWSerial.h"
#include "MsTimer2.h"
#include <EEPROM.h>

#define MOONWALK_LEFT 'a'
#define MOONWALK_RIGHT 'b'
#define CRUSAITO 'c'
#define SWING 'd'
#define UPDOWN 'v'
#define FLAPPING 'w'
#define RUN 'x'
#define BACKYARD 'y'
#define BACKYARDSLOW 'z'
#define GOINGUP 'j'
#define DRUNK 'k'
#define NOGRAVITY 'l'
#define KICKLEFT 'm'
#define KICKRIGHT 'n'
#define LEGRAISE 'o'
#define LEGRAISE1 'p'
#define LEGRAISE2 'q'
#define LEGRAISE3 'r'
#define LEGRAISE4 's'
#define SITDOWN 't'
#define LATERAL 'u'

/*
         ---------------
        |     O   O     |
        |---------------|
  YR 2<== |               | <== YL 3
         ---------------
            ||     ||
            ||     ||
  RR 0==^   -----   ------  v== RL 1
         |-----   ------|
*/

/* fine-tuning temporary storage variables*/
int trim_rr;
int trim_rl;
int trim_yr;
int trim_yl;

int addr_trim_rr = 0;
int addr_trim_rl = 1;
int addr_trim_yr = 2;
int addr_trim_yl = 3;

/* Hardware interface mapping*/

#define SOFTWARE_RXD A2 //Software implementation of serial interface (audio module driver interface)
#define SOFTWARE_TXD A3

#define YL_PIN 10
#define YR_PIN 9
#define RL_PIN 12
#define RR_PIN 6


#define RECV_PIN 3

#define ECHO_PIN 4//Ultrasound interface
#define TRIG_PIN 5

#define ST188_R_PIN A1//Infrared Controller Interface
#define ST188_L_PIN A0

#define VOLTAGE_MEASURE_PIN A4 //Voltage Detection Interface

#define INDICATOR_LED_PIN A5 //LED Indicator Interface

#define MY1690_PIN 8
#define HT6871_PIN 7

#define N_SERVOS 4
#define INTERVALTIME 10.0
#define CENTRE 90
#define AMPLITUDE 30
#define ULTRA_HIGH_RATE 0.3
#define HIGH_RATE 0.5
#define MID_RATE 0.7
#define LOW_RATE 1.0
#define ULTRA_LOW_RATE 1.5

NeoSWSerial mp3Serial(SOFTWARE_RXD, SOFTWARE_TXD);

void Test_voltageMeasure(void);

unsigned long moveTime;
unsigned long ledBlinkTime;
unsigned long voltageMeasureTime;
unsigned long infraredMeasureTime;
int LED_value = 255;

char moveNum = 0;
double distance_value = 0;
int st188Val_L;
int st188Val_R;
long int ST188Threshold;
long int ST188RightDataMin;
long int ST188LeftDataMin;
int UltraThresholdMin = 7;
int UltraThresholdMax = 20;
Oscillator servo[N_SERVOS];

enum MODE
{
  IDLE,
  BLUETOOTH,
  OBSTACLE,
  FOLLOW,
  MUSIC,
  DANCE,
  VOLUME
} mode = IDLE; // Right Domain Key of Hand-Tour APP Control Interface

unsigned long preMp3Millis;

unsigned long preMp3MillisStop_OBSTACLE;
unsigned long preMp3MillisStop_FOLLOW;
int t = 1000;
double pause = 0;
char irValue = '\0';
bool serial_flag = false;
bool danceFlag = false;

bool delays(unsigned long ms)
{
  for (unsigned long i = 0; i < ms; i++)
  {
    if (serial_flag)
    {
      return true;
    }
    delay(1);
  }
  return false;
}
/*
   Implementation of MP3 Driver
*/
class MY1690_16S
{
  public:
    int volume;
    String playStatus[5] = {"0", "1", "2", "3", "4"}; // STOP PLAYING PAUSE FF FR

    /* Music Playing Choice*/
    void playSong(unsigned char num, unsigned char vol)
    {
      setVolume(vol);
      setPlayMode(4);
      CMD_SongSelet[4] = num;
      checkCode(CMD_SongSelet);
      mp3Serial.write(CMD_SongSelet, 7);
      delay(50);
    }
    /* Get playback status*/
    String getPlayStatus()
    {
      mp3Serial.write(CMD_getPlayStatus, 5);
      delay(50);
      return getStatus();
    }
    /* Get status*/
    String getStatus()
    {
      String statusMp3 = "";
      while (mp3Serial.available())
      {
        statusMp3 += (char)mp3Serial.read();
      }
      return statusMp3;
    }
    /* Stop broadcasting*/
    void stopPlay()
    {
      setPlayMode(4);
      mp3Serial.write(CMD_MusicStop, 5);
      delay(50);
    }
    /* Volume setting*/
    void setVolume(unsigned char vol)
    {
      CMD_VolumeSet[3] = vol;
      checkCode(CMD_VolumeSet);
      mp3Serial.write(CMD_VolumeSet, 6);
      delay(50);
    }
    /* Voice Enhancement*/
    void volumePlus()
    {
      mp3Serial.write(CMD_VolumePlus, 5);
      delay(50);
    }
    /* Lower volume*/
    void volumeDown()
    {
      mp3Serial.write(CMD_VolumeDown, 5);
      delay(50);
    }

    void setPlayMode(unsigned char mode)
    {
      CMD_PlayMode[3] = mode;
      checkCode(CMD_PlayMode);
      mp3Serial.write(CMD_PlayMode, 6);
      delay(50);
    }

    void checkCode(unsigned char *vs)
    {
      int val = vs[1];
      int i;
      for (i = 2; i < vs[1]; i++)
      {
        val = val ^ vs[i];
      }
      vs[i] = val;
    }

    void ampMode(int p, bool m)
    {
      pinMode(p, OUTPUT);
      if (m)
      {
        digitalWrite(p, HIGH);
      }
      else
      {
        digitalWrite(p, LOW);
      }
    }

    void init()
    {
      ampMode(HT6871_PIN, HIGH);
      stopPlay();
      volume = 15;
    }

  private:
    byte CMD_MusicPlay[5] = {0x7E, 0x03, 0x11, 0x12, 0xEF};
    byte CMD_MusicStop[5] = {0x7E, 0x03, 0x1E, 0x1D, 0xEF};
    byte CMD_MusicNext[5] = {0x7E, 0x03, 0x13, 0x10, 0xEF};
    byte CMD_MusicPrev[5] = {0x7E, 0x03, 0x14, 0x17, 0xEF};
    byte CMD_VolumePlus[5] = {0x7E, 0x03, 0x15, 0x16, 0xEF};
    byte CMD_VolumeDown[5] = {0x7E, 0x03, 0x16, 0x15, 0xEF};
    byte CMD_VolumeSet[6] = {0x7E, 0x04, 0x31, 0x00, 0x00, 0xEF};
    byte CMD_PlayMode[6] = {0x7E, 0x04, 0x33, 0x00, 0x00, 0xEF};
    byte CMD_SongSelet[7] = {0x7E, 0x05, 0x41, 0x00, 0x00, 0x00, 0xEF};
    byte CMD_getPlayStatus[5] = {0x7E, 0x03, 0x20, 0x23, 0xEF};
} mp3;

bool oscillate(int A[N_SERVOS], int O[N_SERVOS], int T, double phase_diff[N_SERVOS])
{
  for (int i = 0; i < 4; i++)
  {
    servo[i].SetO(O[i]);
    servo[i].SetA(A[i]);
    servo[i].SetT(T);
    servo[i].SetPh(phase_diff[i]);
  }
  double ref = millis();
  for (double x = ref; x < T + ref; x = millis())
  {
    for (int i = 0; i < 4; i++)
    {
      servo[i].refresh();
    }
    if (irValue)
      return true;
  }
  return false;
}

unsigned long final_time;
unsigned long interval_time;
int oneTime;
int iteration;
float increment[N_SERVOS];
int oldPosition[] = {CENTRE, CENTRE, CENTRE, CENTRE};

bool home()
{
  int move1[] = {90, 90, 90, 90};
  if (moveNServos(t, move1) || delays(t))
    return true;
  return false;
}

/*
    Setting the 90-degree position of the steering gear to make the penguin stand on its feet
*/
void homes(int millis_t)
{
  servo[0].SetPosition(90);
  servo[1].SetPosition(90);
  servo[2].SetPosition(90);
  servo[3].SetPosition(90);
  delay(millis_t);
}

bool moveNServos(int time, int newPosition[])
{
  for (int i = 0; i < N_SERVOS; i++)
  {
    increment[i] = ((newPosition[i]) - oldPosition[i]) / (time / INTERVALTIME);
  }
  final_time = millis() + time;
  iteration = 1;
  while (millis() < final_time)
  {
    interval_time = millis() + INTERVALTIME;
    oneTime = 0;
    while (millis() < interval_time)
    {
      if (oneTime < 1)
      {
        for (int i = 0; i < N_SERVOS; i++)
        {
          servo[i].SetPosition(oldPosition[i] + (iteration * increment[i]));
        }
        iteration++;
        oneTime++;
      }
      if (serial_flag)
        return true;
    }
  }

  for (int i = 0; i < N_SERVOS; i++)
  {
    oldPosition[i] = newPosition[i];
  }
  return false;
}

/*
  Walking control realization:
*/
bool walk(int steps, int T, int dir)
{

  int AMP = 20;

  int move1[] = {90, 90 + 35, 90 + AMP, 90 + AMP};
  int move2[] = {90 + 25, 90 + 30, 90 + AMP, 90 + AMP};
  int move3[] = {90 + 20, 90 + 20, 90 - AMP, 90 - AMP};
  int move4[] = {90 - 35, 90, 90 - AMP, 90 - AMP};
  int move5[] = {90 - 40, 90 - 30, 90 - AMP, 90 - AMP};
  int move6[] = {90 - 20, 90 - 20, 90 + AMP, 90 + AMP};

  int move21[] = {90, 90 + 35, 90 - AMP, 90 - AMP};
  int move22[] = {90 + 25, 90 + 30, 90 - AMP, 90 - AMP};
  int move23[] = {90 + 20, 90 + 20, 90 + AMP, 90 + AMP};
  int move24[] = {90 - 35, 90, 90 + AMP, 90 + AMP};
  int move25[] = {90 - 40, 90 - 30, 90 + AMP, 90 + AMP};
  int move26[] = {90 - 20, 90 - 20, 90 - AMP, 90 - AMP};

  if (dir == 1) //Walking forward
  {
    for (int i = 0; i < steps; i++)
      if (
        moveNServos(T * 0.2, move1) ||
        delays(t / 10) ||
        moveNServos(T * 0.2, move2) ||
        delays(t / 10) ||
        moveNServos(T * 0.2, move3) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move4) ||
        delays(t / 2) ||
        moveNServos(T * 0.2, move5) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move6) ||
        delays(t / 5))
        return true;
  }
  else //Walking backward
  {
    for (int i = 0; i < steps; i++)
      if (
        moveNServos(T * 0.2, move21) ||
        delays(t / 10) ||
        moveNServos(T * 0.2, move22) ||
        delays(t / 10) ||
        moveNServos(T * 0.2, move23) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move24) ||
        delays(t / 2) ||
        moveNServos(T * 0.2, move25) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move26))
        return true;
  }

  return false;
}
/*
    Realization of Turn Control
*/
bool turn(int steps, int T, int dir)
{
  int move1[] = {90 - 55, 90 - 20, 90 + 20, 90 + 20};
  int move2[] = {90 - 20, 90 - 20, 90 + 20, 90 - 20};
  int move3[] = {90 + 20, 90 + 55, 90 + 20, 90 - 20};
  int move4[] = {90 + 20, 90 + 20, 90 - 20, 90 + 20};
  int move5[] = {90 - 55, 90 - 20, 90 - 20, 90 + 20};
  int move6[] = {90 - 20, 90 - 20, 90 + 20, 90 - 20};
  int move7[] = {90 + 20, 90 + 55, 90 + 20, 90 - 20};
  int move8[] = {90 + 20, 90 + 20, 90 - 20, 90 + 20};

  int move21[] = {90 + 20, 90 + 55, 90 + 20, 90 + 20};
  int move22[] = {90 + 20, 90 + 20, 90 + 20, 90 - 20};
  int move23[] = {90 - 55, 90 - 20, 90 + 20, 90 - 20};
  int move24[] = {90 - 20, 90 - 20, 90 - 20, 90 - 20};
  int move25[] = {90 + 20, 90 + 55, 90 - 20, 90 + 20};
  int move26[] = {90 + 20, 90 + 20, 90 + 20, 90 - 20};
  int move27[] = {90 - 55, 90 - 20, 90 + 20, 90 - 20};
  int move28[] = {90 - 20, 90 - 20, 90 - 20, 90 - 20};

  if (dir == 1)
  {
    for (int i = 0; i < steps; i++)
      if (
        moveNServos(T * 0.2, move1) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move2) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move3) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move4) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move5) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move6) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move7) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move8) ||
        delays(t / 5))
        return true;
  }
  else
  {
    for (int i = 0; i < steps; i++)
      if (
        moveNServos(T * 0.2, move21) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move22) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move23) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move24) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move25) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move26) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move27) ||
        delays(t / 5) ||
        moveNServos(T * 0.2, move28) ||
        delays(t / 5))
        return true;
  }

  return false;
}
/* Turn right*/
bool moonWalkRight(int steps, int T)
{
  int A[4] = {25, 25, 0, 0};
  int O[4] = { -15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  return false;
}
/* Turn left*/
bool moonWalkLeft(int steps, int T)
{
  int A[4] = {25, 25, 0, 0};
  int O[4] = { -15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 - 120), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  return false;
}

bool stamp(int steps, int tempo)
{
  int move1[] = {60, 90, 90, 90};
  int move2[] = {90, 90, 90, 90};
  for (int i = 0; i < steps; i++)
    if (moveNServos(tempo * ULTRA_HIGH_RATE, move1) ||
        moveNServos(tempo * ULTRA_HIGH_RATE, move2))
      return true;
  return false;
}

bool crusaito(int steps, int T)
{
  int A[4] = {25, 25, 30, 30};
  int O[4] = { -15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90), DEG2RAD(90)};
  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  if (home())
    return true;
  return false;
}
bool swing(int steps, int T)
{
  int A[4] = {25, 25, 0, 0};
  int O[4] = { -15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  return false;
}

bool upDown(int steps, int T)
{
  int A[4] = {25, 25, 0, 0};
  int O[4] = { -15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(180), DEG2RAD(0), DEG2RAD(270), DEG2RAD(270)};
  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  if (home())
    return true;
  return false;
}

bool flapping(int steps, int T)
{
  int A[4] = {15, 15, 8, 8};
  int O[4] = { -A[0], A[1], 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180), DEG2RAD(-90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  return false;
}

bool reverseFlapping(int steps, int T)
{
  int A[4] = {15, 15, 8, 8};
  int O[4] = { -A[0], A[1], 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180), DEG2RAD(90), DEG2RAD(-90)};

  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  return false;
}

bool run(int steps, int T)
{
  int A[4] = {10, 10, 10, 10};
  int O[4] = {0, 0, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  return false;
}

bool backyard(int steps, int T)
{
  int A[4] = {15, 15, 30, 30};
  int O[4] = {0, 0, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(-90), DEG2RAD(-90)};

  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  return false;
}

bool foryard(int steps, int T)
{
  int A[4] = {15, 15, 30, 30};
  int O[4] = {0, 0, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  return false;
}

bool backyardSlow(int steps, int T)
{
  int A[4] = {15, 15, 30, 30};
  int O[4] = {0, 0, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(-90), DEG2RAD(-90)};

  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  return false;
}

bool goingUp(int tempo)
{
  int move1[] = {50, 130, 90, 90};
  if (moveNServos(tempo * HIGH_RATE, move1) ||
      delays(tempo * 3) ||
      home())
    return true;
  return false;
}

bool waves(int steps, int T)
{
  int A[4] = {25, 25, 0, 0};
  int O[4] = { -15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  return false;
}

bool reverseWave(int steps, int T)
{
  int A[4] = {25, 25, 0, 0};
  int O[4] = { -15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(180 + 120), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    if (oscillate(A, O, T, phase_diff))
      return true;
  return false;
}

bool goingDown(int tempo)
{
  int move1[] = {130, 50, 90, 90};
  if (moveNServos(tempo * HIGH_RATE, move1) ||
      delays(tempo * 3) ||
      home())
    return true;
  return false;
}

bool drunk(int tempo)
{
  int move1[] = {70, 70, 90, 90};
  int move2[] = {110, 110, 90, 90};
  int move3[] = {70, 70, 90, 90};
  int move4[] = {110, 110, 90, 90};
  if (moveNServos(tempo * MID_RATE, move1) ||
      moveNServos(tempo * MID_RATE, move2) ||
      moveNServos(tempo * MID_RATE, move3) ||
      moveNServos(tempo * MID_RATE, move4) ||
      home())
    return true;
  return false;
}

bool noGravity(int tempo)
{
  int move1[] = {120, 140, 90, 90};
  int move2[] = {120, 30, 90, 90};
  int move3[] = {120, 120, 90, 90};
  int move4[] = {120, 30, 120, 120};
  int move5[] = {120, 30, 60, 60};
  if (moveNServos(tempo * MID_RATE, move1) ||
      moveNServos(tempo * MID_RATE, move2) ||
      moveNServos(tempo * LOW_RATE, move4) ||
      delays(2.3 * tempo) ||
      moveNServos(tempo * LOW_RATE, move5) ||
      delays(tempo / 4) ||
      moveNServos(tempo * LOW_RATE, move4) ||
      delays(tempo / 2) ||
      home())
    return true;
  return false;
}

bool kickLeft(int tempo)
{
  int move1[] = {120, 140, 90, 90};
  int move2[] = {120, 90, 90, 90};
  int move3[] = {120, 120, 90, 90};
  int move4[] = {120, 90, 120, 120};
  int move5[] = {120, 120, 60, 60};
  if (moveNServos(tempo * MID_RATE, move1) ||
      delays(tempo) ||
      moveNServos(tempo * MID_RATE, move2) ||
      delays(tempo / 4) ||
      moveNServos(tempo * MID_RATE, move3) ||
      delays(tempo / 4) ||
      moveNServos(tempo * LOW_RATE, move4) ||
      delays(tempo / 4) ||
      moveNServos(tempo * LOW_RATE, move5) ||
      delays(tempo / 4) ||
      home())
    return true;
  return false;
}

bool kickRight(int tempo)
{
  int move1[] = {40, 60, 90, 90};
  int move2[] = {90, 60, 90, 90};
  int move3[] = {60, 60, 90, 90};
  int move4[] = {90, 60, 120, 120};
  int move5[] = {60, 60, 60, 60};
  if (moveNServos(tempo * MID_RATE, move1) ||
      delays(tempo) ||
      moveNServos(tempo * MID_RATE, move2) ||
      delays(tempo / 4) ||
      moveNServos(tempo * MID_RATE, move3) ||
      delays(tempo / 4) ||
      moveNServos(tempo * LOW_RATE, move4) ||
      delays(tempo / 4) ||
      moveNServos(tempo * LOW_RATE, move5) ||
      delays(tempo / 4) ||
      home())
    return true;
  return false;
}

bool legRaise(int tempo, int dir)
{
  if (dir)
  {
    int move1[] = {70, 70, 60, 60};
    if (moveNServos(tempo * MID_RATE, move1) || delays(tempo))
      return true;
  }
  else
  {
    int move1[] = {110, 110, 120, 120};
    if (moveNServos(tempo * MID_RATE, move1) || delays(tempo))
      return true;
  }
  if (home())
    return true;
  return false;
}

bool legRaise1(int tempo, int dir)
{
  if (dir)
  {
    int move1[] = {50, 60, 90, 90};
    int move2[] = {60, 60, 120, 90};
    int move3[] = {60, 60, 60, 90};
    if (moveNServos(tempo * MID_RATE, move1) ||
        delays(tempo) ||
        moveNServos(tempo * LOW_RATE, move2) ||
        delays(tempo / 4) ||
        moveNServos(tempo * LOW_RATE, move3) ||
        delays(tempo / 4) ||
        moveNServos(tempo * LOW_RATE, move2) ||
        delays(tempo / 4) ||
        moveNServos(tempo * LOW_RATE, move3) ||
        delays(tempo / 4))
      return true;
  }
  else
  {
    int move1[] = {120, 130, 90, 90};
    int move2[] = {120, 120, 90, 60};
    int move3[] = {120, 120, 90, 120};
    if (moveNServos(tempo, move1) ||
        delays(tempo) ||
        moveNServos(tempo * MID_RATE, move2) ||
        delays(tempo / 4) ||
        moveNServos(tempo * MID_RATE, move3) ||
        delays(tempo / 4) ||
        moveNServos(tempo * MID_RATE, move2) ||
        delays(tempo / 4) ||
        moveNServos(tempo * MID_RATE, move3) ||
        delays(tempo / 4))
      return true;
  }
  if (home())
    return true;
  return false;
}

bool legRaise2(int steps, int tempo, int dir)
{
  if (dir)
  {
    int move1[] = {20, 60, 90, 90};
    int move2[] = {20, 90, 120, 90};
    for (int i = 0; i < steps; i++)
    {
      if (moveNServos(tempo * 0.7, move1) ||
          delays(tempo / 4) ||
          moveNServos(tempo * 0.7, move2) ||
          delays(tempo / 4))
        return true;
    }
  }
  else
  {
    int move1[] = {120, 160, 90, 90};
    int move2[] = {90, 160, 90, 60};
    for (int i = 0; i < steps; i++)
    {
      if (moveNServos(tempo * 0.7, move1) ||
          delays(tempo / 4) ||
          moveNServos(tempo * 0.7, move2) ||
          delays(tempo / 4))
        return true;
    }
  }
  if (home())
    return true;
  return false;
}

bool legRaise3(int steps, int tempo, int dir)
{
  if (dir)
  {
    int move1[] = {20, 60, 90, 90};
    int move2[] = {20, 90, 90, 90};
    for (int i = 0; i < steps; i++)
    {
      if (moveNServos(tempo * 0.5, move1) ||
          delays(tempo / 4) ||
          moveNServos(tempo * 0.5, move2) ||
          delays(tempo / 4))
        return true;
    }
  }
  else
  {
    int move1[] = {120, 160, 90, 90};
    int move2[] = {90, 160, 90, 90};
    for (int i = 0; i < steps; i++)
    {
      if (moveNServos(tempo * 0.5, move1) ||
          delays(tempo / 4) ||
          moveNServos(tempo * 0.5, move2) ||
          delays(tempo / 4))
        return true;
    }
  }
  if (home())
    return true;
  return false;
}

bool legRaise4(int tempo, int dir)
{
  if (dir)
  {
    int move1[] = {20, 60, 90, 90};
    int move2[] = {20, 90, 90, 90};

    if (moveNServos(tempo * MID_RATE, move1) ||
        delays(tempo / 4) ||
        moveNServos(tempo * MID_RATE, move2) ||
        delays(tempo / 4))
      return true;
  }
  else
  {
    int move1[] = {120, 160, 90, 90};
    int move2[] = {90, 160, 90, 90};
    if (moveNServos(tempo * MID_RATE, move1) ||
        delays(tempo / 4) ||
        moveNServos(tempo * MID_RATE, move2) ||
        delays(tempo / 4))
      return true;
  }
  if (home())
    return true;
  return false;
}

bool sitdown(int t)
{
  int move1[] = {150, 90, 90, 90};
  int move2[] = {150, 30, 90, 90};
  if (moveNServos(t * ULTRA_LOW_RATE, move1) ||
      delays(t / 2) ||
      moveNServos(t * ULTRA_LOW_RATE, move2) ||
      delays(t / 2) ||
      home())
    return true;
  return false;
}

bool lateral_fuerte(boolean dir, int tempo)
{
  if (dir)
  {
    int move1[] = {CENTRE - 2 * AMPLITUDE, CENTRE - AMPLITUDE, CENTRE, CENTRE};
    int move2[] = {CENTRE + AMPLITUDE, CENTRE - AMPLITUDE, CENTRE, CENTRE};
    int move3[] = {CENTRE - 2 * AMPLITUDE, CENTRE - AMPLITUDE, CENTRE, CENTRE};
    if (moveNServos(tempo * LOW_RATE, move1) || delays(tempo * 1.5) ||
        moveNServos(tempo * ULTRA_HIGH_RATE, move2) || delays(tempo / 2) ||
        moveNServos(tempo * ULTRA_HIGH_RATE, move3) || delays(tempo))
      return true;
  }
  else
  {
    int move1[] = {CENTRE + AMPLITUDE, CENTRE + 2 * AMPLITUDE, CENTRE, CENTRE};
    int move2[] = {CENTRE + AMPLITUDE, CENTRE - AMPLITUDE, CENTRE, CENTRE};
    int move3[] = {CENTRE + AMPLITUDE, CENTRE + 2 * AMPLITUDE, CENTRE, CENTRE};
    if (moveNServos(tempo * LOW_RATE, move1) || delays(tempo * 1.5) ||
        moveNServos(tempo * ULTRA_HIGH_RATE, move2) || delays(tempo / 2) ||
        moveNServos(tempo * ULTRA_HIGH_RATE, move3) || delays(tempo))
      return true;
  }
  if (home())
    return true;
  return false;
}

void start()
{
  waitForStart();
}

void waitForStart()
{
  while (objectIsClose());
  startDance();
  servoAttach();
}

bool objectIsClose() {
  int distance1 = getDistance();
  int distance2 = getDistance();
  int distance3 = getDistance();
  return distance1 < 10 && distance2 < 10 && distance3 < 10;
}

void startDance()
{
  mp3.stopPlay();
  delays(10);
  mp3.playSong(6, 25); //mp3.volume);
  servoAttach();
  delay(600);
    stamp(4, t);
    lateral_fuerte(1, t);
    lateral_fuerte(0, t);
    goingDown(t);
    goingUp(t);
    delay(2000);
    swing(2, 3 * t / 4);
    kickRight(t/4);
    swing(1, 3 * t / 4);
    kickLeft(t/4);
    delay(400);
    backyard(3, 4 * t / 5);
    delay(2300);
    foryard(3, 4 * t / 5);
    delay(2300);
    flapping(3, 4 * t /5);
    delay(2300);
    reverseFlapping(3, 4 * t/5);
    delay(2300);
    noGravity(t);
    legRaise1(t/2, 1);
    legRaise1(t/2, 0);
    swing(2, 3 * t / 4);
    kickRight(t/4);
    swing(1, 3 * t / 4);
    kickLeft(t/4);
    delay(1000);
    waves(2, 2 * t);
    delay(500);
    reverseWave(2, 2 * t);
    delay(500);
    waves(2, 2 * t);
    delay(500);
    reverseWave(2, 2 * t);
    delay(500);
    waves(2, 2 * t);
    delay(500);
    reverseWave(2, 2 * t);
    delay(500);
    waves(2, 2 * t);
    delay(500);
    reverseWave(2, 2 * t);
    servoDetach();
    }

    void st188Adjust(int dis)
    {
    if (millis() - infraredMeasureTime > 1000 && dis > 20 && dis < 200 && analogRead(ST188_L_PIN) < 300 && analogRead(ST188_R_PIN) < 300)
    {
      unsigned long st188RightData = 0;
      unsigned long st188LeftData = 0;
      for (int n = 0; n < 10; n++)
      {
          st188LeftData += analogRead(ST188_L_PIN);
          st188RightData += analogRead(ST188_R_PIN);
      }
      ST188LeftDataMin = st188LeftData / 10;
      ST188RightDataMin = st188RightData / 10;
      ST188Threshold = ST188LeftDataMin - ST188RightDataMin;
      infraredMeasureTime = millis();
    }
    }


    void servoAttach()
    {
    //
    servo[0].SetTrim(trim_rr);
    servo[1].SetTrim(trim_rl);
    servo[2].SetTrim(trim_yr);
    servo[3].SetTrim(trim_yl);

    servo[0].attach(RR_PIN);
    servo[1].attach(RL_PIN);
    servo[2].attach(YR_PIN);
    servo[3].attach(YL_PIN);
    }

    void servoDetach()
    {
    servo[0].detach();
    servo[1].detach();
    servo[2].detach();
    servo[3].detach();
    }

    /* Realization of Ultrasound Ranging*/
  int getDistance()
  {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    return (int)pulseIn(ECHO_PIN, HIGH) / 58;
  }

  /* Control command acquisition implementation*/
  void getCommand()
  {
    if (Serial.available())
    {
      irValue = Serial.read();
      if (irValue && irValue != '\0')
      {
        // Serial.print("new data: ");
        // Serial.println(irValue);
        serial_flag = true;
      }
      else
      {
        // Serial.print("error data: ");
        // Serial.println(irValue);
        irValue = '\0';
      }
    }
    Test_voltageMeasure();// Realization of Voltage Detection
  }

  void servoInit()
  {
    if (EEPROM.read(addr_trim_rr) != 255)
    {
      trim_rr = EEPROM.read(addr_trim_rr) - 90;
    }
    else
    {
      trim_rr = 0;
    }

    if (EEPROM.read(addr_trim_rl) != 255)
    {
      trim_rl = EEPROM.read(addr_trim_rl) - 90;
    }
    else
    {
      trim_rl = 0;
    }

    if (EEPROM.read(addr_trim_yr) != 255)
    {
      trim_yr = EEPROM.read(addr_trim_yr) - 90;
    }
    else
    {
      trim_yr = 0;
    }

    if (EEPROM.read(addr_trim_yl) != 255)
    {
      trim_yl = EEPROM.read(addr_trim_yl) - 90;
    }
    else
    {
      trim_yl = 0;
    }
  }

  void setup()
  {
    Serial.begin(9600);
    mp3Serial.begin(9600);
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(INDICATOR_LED_PIN, OUTPUT);
    pinMode(VOLTAGE_MEASURE_PIN, INPUT);

    analogWrite(INDICATOR_LED_PIN, LED_value);
    mp3.init();
    MsTimer2::set(50, getCommand);
    MsTimer2::start();
    servoInit();
    servoAttach();
    homes(200);
    servoDetach();
    delays(2000);
    start();
  }

  /*
    Voltage detection implementation function:

    Acquisition of battery voltage, preset abnormal voltage threshold, control LED flashing to remind users of charging
  */
  void Test_voltageMeasure(void) //Realization of Voltage Detection
  {

    static boolean voltageMeasure_flag = true;
    static boolean is_flag = true;
    if (millis() - voltageMeasureTime > 10000)
    {
      double volMeasure = analogRead(VOLTAGE_MEASURE_PIN) * 4.97 / 1023;
      //Serial.print("Battery voltage: ");
      //Serial.println(volMeasure)

      //if (volMeasure < 3.70 || volMeasure >= 4.97)//Detection of power supply voltage below or above the set value is regarded as an abnormal phenomenon
      if (volMeasure < 3.70 )//Detection of power supply voltage below or above the set value is regarded as an abnormal phenomenon
      {
        voltageMeasure_flag = false;
      }
      else
      {
        voltageMeasure_flag = true;
      }
      voltageMeasureTime = millis();
    }

    if (voltageMeasure_flag)
    {
      digitalWrite(INDICATOR_LED_PIN, HIGH);
    }
    else
    {
      if (is_flag)
      {
        is_flag = false;
        digitalWrite(INDICATOR_LED_PIN, HIGH);
      }
      else
      {
        is_flag = true;
        digitalWrite(INDICATOR_LED_PIN, LOW);
      }
    }
  }

  void loop()
  {
    if (irValue != '\0')// Bluetooth serial port data stream on app side (character acquisition is completed in timer 2)
    {
      serial_flag = false;
      switch (irValue) {
        case MOONWALK_RIGHT:
          mode = DANCE;
          servoAttach();
          moonWalkRight(3, t);
          servoDetach();
          break;
        case MOONWALK_LEFT:
          mode = DANCE;
          servoAttach();
          moonWalkLeft(3, t);
          servoDetach();
          break;
        case CRUSAITO:
          mode = DANCE;
          servoAttach();
          crusaito(3, t);
          servoDetach();
          break;
        case SWING:
          mode = DANCE;
          servoAttach();
          swing(3, t);
          servoDetach();
          break;
        case UPDOWN:
          mode = DANCE;
          servoAttach();
          upDown(3, t);
          servoDetach();
          break;
        case FLAPPING:
          mode = DANCE;
          servoAttach();
          flapping(3, t);
          servoDetach();
          break;
        case RUN:
          mode = DANCE;
          servoAttach();
          run(3, t);
          servoDetach();
          break;
        case BACKYARD:
          mode = DANCE;
          servoAttach();
          backyard(3, t);
          servoDetach();
          break;
        case BACKYARDSLOW:
          mode = DANCE;
          servoAttach();
          backyardSlow(3, t);
          servoDetach();
          break;
        case GOINGUP:
          mode = DANCE;
          servoAttach();
          goingUp(t);
          servoDetach();
          break;
        case DRUNK:
          mode = DANCE;
          servoAttach();
          drunk(t);
          servoDetach();
          break;
        case NOGRAVITY:
          mode = DANCE;
          servoAttach();
          noGravity(t);
          servoDetach();
          break;
        case KICKLEFT:
          mode = DANCE;
          servoAttach();
          kickLeft(t);
          servoDetach();
          break;
        case KICKRIGHT:
          mode = DANCE;
          servoAttach();
          kickRight(t);
          servoDetach();
          break;
        case LEGRAISE:
          mode = DANCE;
          servoAttach();
          legRaise(t, 1);
          servoDetach();
          break;
        case LEGRAISE1:
          mode = DANCE;
          servoAttach();
          legRaise1(t, 1);
          servoDetach();
          break;
        case LEGRAISE2:
          mode = DANCE;
          servoAttach();
          legRaise2(3, t, 1);
          servoDetach();
          break;
        case LEGRAISE3:
          mode = DANCE;
          servoAttach();
          legRaise3(3, t, 1);
          servoDetach();
          break;
        case LEGRAISE4:
          mode = DANCE;
          servoAttach();
          legRaise4(t, 1);
          servoDetach();
          break;
        case SITDOWN:
          mode = DANCE;
          servoAttach();
          sitdown(t);
          servoDetach();
          break;
        case LATERAL:
          mode = DANCE;
          servoAttach();
          lateral_fuerte(1, t);
          servoDetach();
          break;
        default:
          break;
      }
      if (serial_flag == false)
      {
        irValue = '\0'; // Data Command Clearing Serial Cache
      }
    }
    switch (mode)
    {
      case IDLE:
        mp3.stopPlay();
        servoDetach();
        break;
      default:
        break;

      case DANCE:
        break;
    }
  }
