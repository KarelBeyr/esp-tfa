#include "Arduino.h"
#include "WiFi.h"
#include "thingspeak.h"
#include "watchdog.h"
#include "debugUtils.h"

#include "pass.h"

const byte interruptPin = 25;

volatile byte counter = 0;

struct packetData {
  byte hum;
  int temp;
  byte channel;
  byte head1;
  byte head2;
  byte tail;
};

typedef struct packetData PacketData;

struct SensorData {
  byte hum;
  int temp;
  bool isValid;
  unsigned long timestamp;
};

volatile PacketData currentPacket;
volatile PacketData readPackets[2];
volatile SensorData sensorsToSend[3];

volatile bool wasMark = false;
volatile unsigned long lastMicros = 0;

WiFiClient client;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
RTC_DATA_ATTR unsigned long bootCount = 0;

void processBit(byte val)
{
  if (counter >= 0 && (counter <= 7))
    currentPacket.head1 = currentPacket.head1 * 2 + val;
  if (counter >= 8 && (counter <= 13))
    currentPacket.head2 = currentPacket.head2 * 2 + val;
  if (counter >= 14 && (counter <= 15))
    currentPacket.channel = currentPacket.channel * 2 + val;
  if (counter >= 16 && (counter <= 27))
    currentPacket.temp = currentPacket.temp * 2 + val;
  if (counter >= 28 && (counter <= 35))
    currentPacket.hum = currentPacket.hum * 2 + val;
  if (counter >= 36 && (counter <= 43))
    currentPacket.tail = currentPacket.tail * 2 + val;
  counter++;
  wasMark = false;
}

void resetPacket()
{
  counter = 0;
  currentPacket.hum = 0;
  currentPacket.temp = 0;
  currentPacket.channel = 0;
  currentPacket.head1 = 0;
  currentPacket.head2 = 0;
  currentPacket.tail = 0;
  wasMark = false;
}

bool isPacketValid2() {
  if (currentPacket.temp > 800) return false;
  if (currentPacket.hum > 100) return false;
  if (currentPacket.channel < 0 || currentPacket.channel > 2) return false;
  if (currentPacket.head1 != 144) return false;   //HACK, nevim presne co je head1, mozna obsahuje info o urovni akumulatoru?
  if (currentPacket.tail != 0) return false;
  return true;
}

bool lastThreeReadPacketsSame() {
  if (currentPacket.hum != readPackets[0].hum) return false;
  if (currentPacket.temp != readPackets[0].temp) return false;
  if (currentPacket.channel != readPackets[0].channel) return false;
  //zakomentovano, protoze jsem uz dlouho (s kontrolou na 3 packety) nedostal zadny sum. Naopak jsem ale mel problem ze jsem obcas hrozne dlouho zadny validni packet nedostal (radove jednotky hodin)
  // if (currentPacket.hum != readPackets[1].hum) return false;
  // if (currentPacket.temp != readPackets[1].temp) return false;
  // if (currentPacket.channel != readPackets[1].channel) return false;
  return true;
}

/*
TFA packet consists of
PREAMBLE 35x[MARK DATA]
PREAMBLE lasts more than 5000 us
MARK lasts 450 - 550 us
DATA0 lasts 1500 - 2500 us
DATA1 lasts 3500 - 4500 us

each packet is sent 6 times, then silence during which other sensor might transmit. Each sensor has different length of radio silence. 50s/53s/56s
*/


void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  unsigned long delta = micros() - lastMicros;
  lastMicros = micros();
  if ((delta > 450) && (delta < 550) && (wasMark == true)) resetPacket();
  if ((delta > 450) && (delta < 550) && (wasMark == false)) wasMark = true;
  if ((delta > 1500) && (delta < 2900)) processBit(0);
  if ((delta > 3100) && (delta < 4900)) processBit(1);
  if ((delta > 5000)) resetPacket();

  if (counter >= 36)
  {
    int ch = currentPacket.channel;
    if (isPacketValid2())
    {
      if (lastThreeReadPacketsSame())
      {
        DEBUG_PRINT(ch + 1);
        DEBUG_PRINT(") Temperature: ");
        DEBUG_PRINTF("%4.1f", currentPacket.temp / 10.0);
        DEBUG_PRINT("°C Humidity: ");
        DEBUG_PRINT(currentPacket.hum);
        DEBUG_PRINT("% head1: ");
        DEBUG_PRINT(currentPacket.head1);
        DEBUG_PRINT(" head2: ");
        DEBUG_PRINT(currentPacket.head2);
        DEBUG_PRINT(" tail: ");
        DEBUG_PRINT(currentPacket.tail);
        DEBUG_PRINT(" micros: ");
        DEBUG_PRINTLN(lastMicros);

        sensorsToSend[ch].hum = currentPacket.hum;
        sensorsToSend[ch].temp = currentPacket.temp;
        sensorsToSend[ch].isValid = true;
        sensorsToSend[ch].timestamp = lastMicros;
      }

      readPackets[1].hum = readPackets[0].hum;
      readPackets[1].temp = readPackets[0].temp;
      readPackets[1].channel = readPackets[0].channel;
      readPackets[0].hum = currentPacket.hum;
      readPackets[0].temp = currentPacket.temp;
      readPackets[0].channel = currentPacket.channel;
    }
    resetPacket();
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  setupWatchdog(60);
  DEBUG_SERIAL_START(115200);
  DEBUG_PRINTLN("Monitoring interrupts: ");

  bootCount = bootCount+1;
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, CHANGE);
  for (int i = 0; i < 3; i++) {
    sensorsToSend[i].isValid = false;
  }
}

bool needMoreTime() {
  for (int i = 0; i < 3; i++) 
    if (sensorsToSend[i].isValid == false) return true;
  return false;
}

bool hasTimeLeft() {
  return millis() < 3 * 60 * 1000;
}

byte indexOfSmallestElement(unsigned long a, unsigned long b, unsigned long c) {
  if (a < b && a < c) return 0;
  if (b < a && b < c) return 1;
  return 2;
}

unsigned long max(unsigned long a, unsigned long b) {return a > b ? a : b;}
unsigned long min(unsigned long a, unsigned long b) {return a < b ? a : b;}

unsigned long spanOfElements(unsigned long a, unsigned long b, unsigned long c) {
  unsigned long biggest = max(a, max(b, c));
  unsigned long smallest = min(a, min(b, c));
  return biggest - smallest;
}

unsigned long smallestDifferenceBetweenElements(unsigned long a, unsigned long b, unsigned long c) {
  return min(abs(a-b), min(abs(a-c), abs(b-c)));
}

unsigned long computeSleepDuration() {
  unsigned long times[3];
  unsigned long now = micros();
  unsigned long deltas[3] = {50000000, 53000000, 56000000 };
  for (int i = 0; i < 3; i++) times[i] = sensorsToSend[i].timestamp;
  bool done = false;
  while(!done) {
    byte idx = indexOfSmallestElement(times[0], times[1], times[2]);
    times[idx] = times[idx] + deltas[idx];
    if (times[idx] - now > 360000000) {  // 6 minutes, WOHAK: ale potencialne ne k nejmensiu
      if (spanOfElements(times[0], times[1], times[2]) < 30000000) {
        if (smallestDifferenceBetweenElements(times[0], times[1], times[2]) > 2000000)
          done = true;
      }
    }
  }
  byte idx = indexOfSmallestElement(times[0], times[1], times[2]);
  return times[idx] - now;
}

void loop() {
  while(needMoreTime() && hasTimeLeft() ) { delay(100); feedWatchdog(); }  //cekam max 3 minuty
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  DEBUG_PRINT("Got all data, starting wifi at ");
  unsigned long gotAllDataAt = millis();
  DEBUG_PRINTLN(gotAllDataAt);

  unsigned long sleepDuration = computeSleepDuration();

  WiFi.begin(WifiSsid, WifiPassword);

  while (WiFi.status() != WL_CONNECTED) {   //TODO zblbuvzdornit pripojovani na wifi. Kdyz se nepodari po x vterinach, zrestartovat
    delay(500);
    DEBUG_PRINT(".");
  }
  feedWatchdog();
  unsigned long wifiStartedAt = millis();
  unsigned long delta = wifiStartedAt - gotAllDataAt;
  DEBUG_PRINTLN("");
  DEBUG_PRINT("WiFi connected after ");
  DEBUG_PRINT(delta);
  DEBUG_PRINTLN("ms");
  DEBUG_PRINTLN("IP address: ");
  DEBUG_PRINTLN(WiFi.localIP());

  maybeLogTelemetryToThingspeak(WriteAPIKeyTFA, 0, 
      sensorsToSend[0].isValid ? String((float)sensorsToSend[0].temp / 10.0) : "", 
      sensorsToSend[0].isValid ? String(sensorsToSend[0].hum) : "", 
      sensorsToSend[1].isValid ? String((float)sensorsToSend[1].temp / 10.0) : "", 
      sensorsToSend[1].isValid ? String(sensorsToSend[1].hum) : "", 
      sensorsToSend[2].isValid ? String((float)sensorsToSend[2].temp / 10.0) : "", 
      sensorsToSend[2].isValid ? String(sensorsToSend[2].hum) : "",
      String(gotAllDataAt / 1000),
      String(bootCount)
  );

  DEBUG_PRINT("Sending to thingspeak took ");
  DEBUG_PRINTLN(millis() - wifiStartedAt);
  DEBUG_PRINTLN("Going to sleep");

  unsigned long sleepTime = 10 * 60 * 1000 * 1000 - micros(); //jak dlouho chci spat: 10 minut - doba jak dlouho jsem cetl senzory, minimalne 1s
  if (sleepTime < 1000000)
    sleepTime = 1000000;
  esp_sleep_enable_timer_wakeup(sleepTime);
  esp_deep_sleep_start();
}