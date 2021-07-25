#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>

#define MQTT_TOPIC "moisture/values"
#define MQTT_HOST "sps"

#define ONE_SECOND (1000 * 1000)
#define ONE_HOUR (60 * 60 * ONE_SECOND)
#define HALF_HOUR (30 * 60 * ONE_SECOND)
#define NVALUES (48) // number of values stored in buffer before tx
#define SLEEPTIME (HALF_HOUR)
#define RTCMEMORYSTART 65

WiFiClient espClient;
PubSubClient client(espClient);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

#define RTCMAGIC (0x12345678)
typedef struct {
  uint32_t magic;
  uint32_t cnt;
  unsigned long time; // epoc time
  bool schedule_tx;
  uint32_t failed_tx;
  float moistures[NVALUES]; // wett: 263, dry: 460
  uint32_t measuretime[NVALUES];
} tRtcMemory;
tRtcMemory rtc;

void reconnect();

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Starting ===");

  system_rtc_mem_read(RTCMEMORYSTART, &rtc, (uint16) sizeof(tRtcMemory));
  Serial.printf("- Count: %d, Count%%NVALUES (%d): %d\n", rtc.cnt, NVALUES, rtc.cnt%NVALUES);

  if(rtc.magic != RTCMAGIC) {
    Serial.printf("- Reset RTC memory (RTCMAGIC:0x%04x)\n", rtc.magic);
    memset(&rtc, 0, sizeof(tRtcMemory));
    rtc.magic = RTCMAGIC;
  }
  
  if(rtc.cnt%NVALUES == 0 || rtc.schedule_tx) {
    Serial.printf("- Initialize Wifi\n");
    WiFiManager wifiManager;
    wifiManager.autoConnect("AutoConnectAP");
    if(WiFi.isConnected())
      Serial.printf("   is connected :)\n");
    else
      Serial.printf("   connect failed :(\n");

    /*
     * Connect MQTT
     */
    client.setBufferSize(4*1024);
    client.setServer(MQTT_HOST, 1883);
    if (!client.connected()) reconnect();
  }

  timeClient.begin();
}

int32_t getRSSI(const char* target_ssid) {
  byte available_networks = WiFi.scanNetworks();

  for (int network = 0; network < available_networks; network++) {
    if (strcmp(WiFi.SSID(network).c_str(), target_ssid) == 0) {
      return WiFi.RSSI(network);
    }
  }
  return 0;
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

static char * createJsonPayload(tRtcMemory * rtc, sint8 rssi) {
  char * buf = NULL;
  size_t len = 0;
  struct _reent reent = { 0 };
  FILE * stream = _open_memstream_r(&reent, &buf, &len);

  fprintf(stream, "{\"cnt\":%d, \"moisture\": [", rtc->cnt);
  for(unsigned idx = 0; idx < NVALUES; idx++) {
    fprintf(stream, "%.1f%s", rtc->moistures[idx], idx < NVALUES-1 ? ", " : "]");
  }
  fprintf(stream, ", \"time\":[");
  for(unsigned idx = 0; idx < NVALUES; idx++) {
    fprintf(stream, "%d%s", rtc->measuretime[idx], idx < NVALUES-1 ? ", " : "]");
  }
  if(rtc->schedule_tx) {
    fprintf(stream, ", \"rssi\": %d", rssi);
  }
  fprintf(stream, ", \"failed_tx\":%d}\n", rtc->failed_tx);
  fclose(stream);

  return buf;
}

void loop() {
  /*
   * Measure
   */
  float sensorValue = 0;
  for(unsigned idx = 0; idx < 10; idx++) {
    delay(50);
    sensorValue += analogRead(A0);
  }
  sensorValue /= 10.0;
  Serial.printf("- Sensor value: %f\n", sensorValue);

  /*
   * Sync time
   */
  if(WiFi.isConnected()) {
    timeClient.update();
    rtc.time = timeClient.getEpochTime();
  } else {
    rtc.time += SLEEPTIME/1000000;
  }

  /*
   * Store in RTC memory
   * - shift old to end of array
   * - add new
   */
  for(int idx = NVALUES-2; idx >= 0; idx--) rtc.moistures[idx+1] = rtc.moistures[idx];
  for(int idx = NVALUES-2; idx >= 0; idx--) rtc.measuretime[idx+1] = rtc.measuretime[idx];
  rtc.moistures[0] = sensorValue;
  rtc.measuretime[0] = rtc.time;

  /*
   * Schedule publish
   */
  if(rtc.cnt % NVALUES == 0) rtc.schedule_tx = 1;
  
  /*
   * Create JSON payload for MQTT
   */
  char * buf = createJsonPayload(&rtc, wifi_station_get_rssi());
  //Serial.printf(buf);
  
  if(client.connected()) {
    Serial.println("- Publish via MQTT");
    if(client.publish(MQTT_TOPIC, buf, true)) {
      client.flush();
      client.disconnect();
      rtc.schedule_tx = 0;
      Serial.println("   done");
    } else {
      rtc.failed_tx++;
      Serial.println("   failed");
    }
  }
  free(buf); buf = NULL;

  /*
   * Counter
   */
  rtc.cnt++;
  system_rtc_mem_write(RTCMEMORYSTART, &rtc, (uint16) sizeof(tRtcMemory));

  /*
   * Sleep
   */
  if(WiFi.getSleepMode() != WIFI_MODEM_SLEEP) WiFi.setSleep(true);    
  //system_deep_sleep(ONE_HOUR);  
  system_deep_sleep(SLEEPTIME);  
}