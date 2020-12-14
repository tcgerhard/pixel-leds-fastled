#include <ArduinoOTA.h>
# include <ESP8266WiFi.h>
/* Default version is 3_1_1, not compatible with our hub yet */
#define MQTT_VERSION MQTT_VERSION_3_1
# include <PubSubClient.h>
# include <EEPROM.h>

#define INCLUDE_TEMP_SENSOR
#ifdef INCLUDE_TEMP_SENSOR
# include <OneWire.h>
# include <DallasTemperature.h>
#endif

# include "GarageConfig.h"

WiFiClient espClient;
PubSubClient mqtt87(espClient);

#ifdef INCLUDE_TEMP_SENSOR
#define ONE_WIRE_BUS D4
//#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress testThermometer;
#endif

static uint8_t mac[6];
#define FASTLED_ESP8266_D1_PIN_ORDER
#include <FastLED.h>
#define CLIENTID_PREFIX "esp8266"
#define MAX_DYN_COLORS 7    // Colors that can be set individually

// EEPROM use
# define EEPROM_SIGNATURE 0x42
enum LightModes { Solid=-1, Gradient=-2, Scheme=-3, DynColors=-4 };

struct EEPROM_DATA {
   uint8_t signature;
   uint8_t eepromSize;    // total number of bytes, i.e., sizeof(struct EEPROM_DATA) 
   uint8_t brightness;    // percent,  0 - 255
   enum LightModes roofMode;        // What is our operational mode?
   enum LightModes starMode;        // What does the star look like?
   uint8_t systemMode;        // 0 = off, 1 = on
   uint8_t dynColorCount;
   CRGB dynColors[MAX_DYN_COLORS];
   
};

static char mqttClientId[sizeof(CLIENTID_PREFIX)+13];   // "ESP8266ffeeddccbbaa" . (last
static struct EEPROM_DATA eeprom_data;
static char mysteryBuffer[10];
\

#define NUM_LEDS_GABLE 98
#define NUM_LEDS_SOFFIT 101
#define NUM_LEDS (NUM_LEDS_GABLE + NUM_LEDS_SOFFIT)
#define NUM_LEDS_STAR1  60
//#define DATA_PIN_GABLE 4    // D2 
#define DATA_PIN_GABLE 7    // D7
#define DATA_PIN_SOFFIT 5   // D5
#define DATA_PIN_STAR1 6     // D6
#define MAX_COLOR_SCHEME 8
#define MAX_SCHEMES 4

#define TWINKLE_INTERVALS 7 // Number of 10ms cycles for a "twinkle"
#define CHANCE_OF_TWINKLE 1 // Higher is more likely to twinkle


// #define DATA_PIN_SOFFIT 6   // D6

# define LED_PIN 2  // onboard LED

//#define vINDEX(n) (((n)>=NUM_LEDS_GABLE) ? (n) : ((NUM_LEDS_GABLE-1)-(n)))
#define vINDEX(n) (n)

CRGB leds[NUM_LEDS];
int  twinkleStatus[NUM_LEDS];
CRGB twinkleBack[NUM_LEDS];       // Color to restore after the twinkle effect

CRGB *ledsGable = &leds[0];
CRGB *ledsSoffit = &leds[NUM_LEDS_GABLE];
CRGB ledsStar1[NUM_LEDS_STAR1];

CLEDController *ctlGable, *ctlSoffit, *ctlStar1;

CRGB roofScheme[MAX_SCHEMES][MAX_COLOR_SCHEME];
CRGB starScheme[MAX_SCHEMES][MAX_COLOR_SCHEME];
uint8_t activeColors[MAX_SCHEMES];
uint8_t activeStarColors[MAX_SCHEMES];

uint8_t activeStarScheme = 0;
bool acceptOTA = false;        // MQTT message can change this, to allow OTA update.
bool debugMode = false;

int  gateAnimation = 0;    // +1 for gate opening, -1 for gate closing
bool systemOff = false;
bool twinkling = false;
bool starMotion = false;
bool steadyMode = false; 

int maxStarScheme, maxRoofScheme;

static uint8_t defaultBrightness = 128;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400);
  Serial.print("\nIn setup\n");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Get some network love
  connectWiFi();
  
  // Start up the message bus
  Serial.printf("MQTT server: %s\n", MQTT_SERVER);
  mqtt87.setServer(MQTT_SERVER, 1883);
  
#ifdef INCLUDE_TEMP_SENSOR
  // Set up the OneWire bus
  sensors.begin();
  delay(100);

  if (!sensors.getAddress(testThermometer, 0)) {
    Serial.println("Unable to find address for OneWire Device 0"); 
    publish_status("No onewire device");
  } else {
    // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
    publish_status("Onewire found");
    sensors.setResolution(testThermometer, 9);
  }
#endif

  // get config info from EEPROM, if it appears valid
  #define READ_EEPROM
  #ifdef READ_EEPROM
  {
    int eeprom_addr = 0;
    int value;
    // Zero out the structure in case we don't get the full length
    byte *pos = (byte *)&eeprom_data;
    for (int i = 0; i <sizeof(eeprom_data); i++) {
      *pos++ = '\0';
    }
    EEPROM.begin(sizeof(struct EEPROM_DATA));
    value = EEPROM.read(eeprom_addr++);
    Serial.printf("Value %d, Signature %d\n", value, EEPROM_SIGNATURE);
    if (value == EEPROM_SIGNATURE) {
      // Looks like we have good content
      int len = EEPROM.read(eeprom_addr++);
      Serial.printf("Bytes to read: %d\n", len);
      byte *writer = &eeprom_data.brightness;
      if (len > sizeof(eeprom_data)) len = sizeof(eeprom_data);    // Sanity check
      len -= (writer - (byte *)&eeprom_data);                        // Adjust for starting partway into struct
      while (len-- > 0) {
        *writer++ = EEPROM.read(eeprom_addr++);
      }
    }
  }
  #endif

  if (eeprom_data.brightness == 0) eeprom_data.brightness = 92;
  eeprom_data.signature = EEPROM_SIGNATURE;
  eeprom_data.eepromSize = sizeof(eeprom_data);
  Serial.printf("MQTT server: %s\n", MQTT_SERVER);
  mqtt87.setServer(MQTT_SERVER, 1883);

  roofScheme[0][0] = CRGB::Red;
  roofScheme[0][1]= CRGB::Orange;
  roofScheme[0][2] = CRGB::Green;
  roofScheme[0][3] = CRGB::Blue;
  activeColors[0] = 4;

  roofScheme[1][0] = CRGB::Goldenrod;
  roofScheme[1][1] = CRGB::Goldenrod;
  roofScheme[1][2] = CRGB::Goldenrod;
  roofScheme[1][3] = CRGB::Goldenrod;
  roofScheme[1][4] = CRGB::Red;
  activeColors[1] = 5;

  roofScheme[2][0] = CRGB::Red;
  roofScheme[2][1] = CRGB::White;
  roofScheme[2][2] = CRGB::Blue;
  roofScheme[2][3] = CRGB::White;
  activeColors[2] = 3;
  maxRoofScheme = 3;

  starScheme[0][0] = CRGB::Red;
  starScheme[0][1]= CRGB::Orange;
  starScheme[0][2] = CRGB::Green;
  starScheme[0][3] = CRGB::Blue;
  activeStarColors[0] = 4;
  
  starScheme[1][0] = CRGB::Goldenrod;
  starScheme[1][1] = CRGB::Goldenrod;
  activeStarColors[1] = 2;

  starScheme[2][0] = CRGB::Red;
  starScheme[2][1] = CRGB::Green;
  activeStarColors[2] = 2;

  starScheme[3][0] = CRGB::Red;
  starScheme[3][1] = CRGB::White;
  starScheme[3][2] = CRGB::Blue;
  activeStarColors[3] = 3;
  maxStarScheme = 4;
  
  Serial.print("adding Leds\n");
  
  ctlGable = &FastLED.addLeds<WS2811, DATA_PIN_GABLE, RGB>(ledsGable, NUM_LEDS_GABLE).setCorrection(TypicalSMD5050);
  ctlSoffit = &FastLED.addLeds<WS2811, DATA_PIN_SOFFIT>(ledsSoffit, NUM_LEDS_SOFFIT).setCorrection(TypicalSMD5050);
  ctlStar1 = &FastLED.addLeds<WS2811, DATA_PIN_STAR1>(ledsStar1, NUM_LEDS_STAR1).setCorrection(TypicalSMD5050);

  // Hack 
  /*
  eeprom_data.dynColors[0] = CRGB::Red;
  eeprom_data.dynColors[1] = CRGB::White;
  eeprom_data.dynColors[2] = CRGB::Blue;
  eeprom_data.dynColorCount = 3;
  eeprom_data.roofMode = DynColors;
  */
  eeprom_data.roofMode = (enum LightModes)0;
  setRoofPattern();

  activeStarScheme  = 1;
  rotateStarPattern();
  
  // fill_solid(ledsStar1, NUM_LEDS_STAR1, CRGB::Goldenrod);
  FastLED.setBrightness(eeprom_data.brightness);
  FastLED.show();
}

int connectWiFi() {
  const char WiFiSSID[] = CFG_SSID;
  const char WiFiPSK[] = CFG_PSK;
  byte ledStatus = LOW;
  int status;
  unsigned long startMillis = millis();
  unsigned long maxWait = 20000;
  
  WiFi.mode(WIFI_STA);
  Serial.print("\nConnecting to WiFi ");
  Serial.println(WiFiSSID);
  Serial.println(WiFiPSK);
  status = WiFi.begin(WiFiSSID, WiFiPSK);
  delay(5000);                          // Allow 5 seconds for connection to establish
  Serial.print("Status from WiFi.begin: ");
  Serial.println(status);

  // Wait up to 20sec for a connection (including time above)
  while ((WiFi.waitForConnectResult() != WL_CONNECTED)
  /* while (WiFi.status() != WL_CONNECTED */ && (millis() - startMillis) < maxWait) {
    // Blink the LED
    digitalWrite(LED_PIN, ledStatus);
    Serial.print(".");
    ledStatus = (ledStatus == HIGH) ? LOW: HIGH;
    delay(200);
  }
  //acceptOTA = true;
  //ArduinoOTA.begin();                       // Enable OTA update
  //Serial.println("Accepting OTA");


  // Setup mqtt clientId based on our Mac address
  strcpy(mqttClientId, CLIENTID_PREFIX);
  WiFi.macAddress(mac);
  (void) binToHex( (unsigned char *)&mqttClientId[sizeof(CLIENTID_PREFIX)-1], mac, 6);
  
  Serial.print("\nStatus from  WiFi.status(): ");  Serial.println(WiFi.status());
  digitalWrite(LED_PIN, LOW);
  return(WiFi.status());
}

void mqttConnect() {
    int retryDelay = 5000;
    while (!mqtt87.connected()) {

    mqtt87.setServer(MQTT_SERVER, 1883);
    Serial.printf("Attempting MQTT connection %s ...", mqttClientId);
    Serial.print("\nStatus from  WiFi.status(): ");  Serial.println(WiFi.status());
    if (mqtt87.connect(mqttClientId)) {
      Serial.println("connected");
      mqtt87.subscribe(MQTT_TOPIC);
      mqtt87.setCallback(mqttCallback);
    } else {
      delay(1);
      Serial.print("failed, rc=");
      Serial.print(mqtt87.state());
      Serial.printf(" try again in %dms...\n", retryDelay);
      delay(retryDelay);
      retryDelay += 1000;
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char tmp[length];
  strncpy(tmp, (const char *)payload, length);
  tmp[length] = '\0';
  
  Serial.printf("received mqtt message: '%s'\n", tmp);
  if (strcmp(tmp, "ota") == 0 ) {
    acceptOTA = true;
    updateEeprom();
    ArduinoOTA.begin();                       // Enable OTA update
    Serial.println("Accepting OTA");
  } else if (strcmp(tmp, "debug") == 0 ) {
    debugMode = !debugMode;                   // debug displays some test patterns
    for (int i = 0; i < NUM_LEDS_GABLE/5; i++) {
      ledsGable[i] =  CRGB::White;
      ledsGable[i+NUM_LEDS_GABLE/5] = CRGB::Blue;
      ledsGable[i+(2*NUM_LEDS_GABLE/5)] = CRGB::Lime;
      ledsGable[i+(3*NUM_LEDS_GABLE/5)] =CRGB::Red;
      ledsGable[i+(4*NUM_LEDS_GABLE/5)]  = CRGB::Black;
    }
    FastLED.show();
  } else if (strcmp(tmp, "debug:1") == 0 ) {
    debugMode = !debugMode;                   // debug displays some test patterns
    for (int i = 0, c = 0; i < 255; i++) {
      ledsGable[i % NUM_LEDS_GABLE] =  c++;
      ctlGable->showLeds();
      delay(3);
      if ((i % NUM_LEDS_GABLE) == (NUM_LEDS_GABLE-1)) FastLED.delay(10000);
    }  
  } else if (strcmp(tmp, "gate:open") == 0) {
    Serial.println("Gate opening");
    gateAnimation = 1;
  } else if (strcmp(tmp, "gate:close") == 0) {
    gateAnimation = -1;
    
  } else if (strncmp(tmp, "color:", 6) == 0) {

    CRGB colors[MAX_DYN_COLORS];
    CRGB *oneSegment = NULL;
    int dynColorCount = 0;
    bool oneGroup = false;
    char *curTriplet = &tmp[6];
    // Rigid format, up to five triplets of zero-padded numbers 255,064,000;
    // Delimeters can be any non-numeric
    // TODO: make color parser more flexible
    if (strncmp(curTriplet, "gable:",6) == 0) {
      oneSegment = ledsGable;
      oneGroup = true;
      curTriplet += 6;
    } else if (strncmp(curTriplet, "soffit:",7) == 0) {
      oneSegment = ledsSoffit;
      oneGroup = true;
      curTriplet += 7;
    }

    while (isdigit(*curTriplet) && dynColorCount <= MAX_DYN_COLORS) {
      String rStr(curTriplet);
      String gStr(curTriplet+4);
      String bStr(curTriplet+8);
      int rVal = rStr.toInt();
      int gVal = gStr.toInt();
      int bVal = bStr.toInt();
      Serial.printf("r: %d, g: %d, b: %d\n", rVal, gVal, bVal);
      colors[dynColorCount++] = CRGB(rVal, gVal, bVal);
      curTriplet += 12;
    }

    if (dynColorCount >= MAX_DYN_COLORS) {
      Serial.printf("Too many dynamic colors: %d\n", dynColorCount);
      dynColorCount = MAX_DYN_COLORS - 1;
    }
    for (int i = 0; i < dynColorCount; i++) {
      eeprom_data.dynColors[i] = colors[i];
    }
    eeprom_data.dynColorCount = dynColorCount;
    eeprom_data.roofMode = DynColors;
    setDynColors(oneGroup, oneSegment);
   

  } else if (strncmp(tmp, "gradient", 8) == 0) {

    eeprom_data.roofMode = Gradient;
    setRoofPattern();
    
  } else if (strncmp(tmp, "dim:", 4) == 0) {
    // Get the characters following the ':'
    String dimStr(&tmp[4]);
    int dimVal = dimStr.toInt();
    Serial.printf("Got dim value of %s\n", tmp);
    if ( 0 <= dimVal <= 255) {
      eeprom_data.brightness = dimVal;
      FastLED.setBrightness(eeprom_data.brightness);
      FastLED.show();
    }
  } else if (strcmp(tmp, "system:off") == 0) {
    systemOff = true;
    eeprom_data.systemMode = 0;
    for (int currentBrightness = eeprom_data.brightness; currentBrightness > 0; --currentBrightness) {
      FastLED.setBrightness(currentBrightness);
      FastLED.show();
      delay(12);
      updateEeprom();
    }
  } else if (strcmp(tmp, "system:on") == 0) {
    systemOff = false;
    for  (int currentBrightness = 0; currentBrightness < eeprom_data.brightness; currentBrightness++) {
      FastLED.setBrightness(currentBrightness);
      FastLED.show();
      delay(12);
      updateEeprom();
    }
  } else if (strcmp(tmp, "twinkle:on") == 0) {
    twinkling = true;
    
  } else if (strcmp(tmp, "twinkle:off") == 0) {
    twinkling = false;

  } else if (strcmp(tmp, "pattern:roof") == 0) {
    int activeScheme = eeprom_data.roofMode;
    if (activeScheme < 0) activeScheme = 0;
    
    activeScheme = (activeScheme + 1) % maxRoofScheme;
    eeprom_data.roofMode = (enum LightModes) activeScheme;
    steadyMode = false;
    setRoofPattern();
  } else if (strcmp(tmp, "pattern:star") == 0) {
    activeStarScheme = (activeStarScheme + 1) % maxStarScheme;
    eeprom_data.starMode = (enum LightModes) activeStarScheme;
    rotateStarPattern();
  } else if (strcmp(tmp, "star:motion") == 0) {
    starMotion = !starMotion;
  }
}

void setDynColors(bool oneGroup, CRGB *oneSegment) {
  
 if (!oneGroup || oneSegment == ledsSoffit) {
      for (int i = 0; i < NUM_LEDS_SOFFIT; i++) {
        ledsSoffit[i] = eeprom_data.dynColors[i % eeprom_data.dynColorCount];
      }
      ctlSoffit->showLeds();
    }
    
    if (!oneGroup || (oneSegment == ledsGable)) {
      for (int i = 0; i < NUM_LEDS_GABLE; i++) {
        ledsGable[i] = eeprom_data.dynColors[i % eeprom_data.dynColorCount];
      }
        ctlGable->showLeds();
    }
    steadyMode = true;
 }

void loop() {
  // put your main code here, to run repeatedly:
  // Set up 4-color sequence R, Y, G, B
  // Serial.print("In loop\n");
  // digitalWrite(LED_PIN, HIGH);

  // (re)connect to Wifi

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  } 


  // connect to MQTT
  if (!mqtt87.connected()) {
      mqttConnect();
  }

  mqtt87.loop();

#ifdef INCLUDE_TEMP_SENSOR
  EVERY_N_SECONDS(20) {
     publish_sensor_reading();
     publish_status("heartbeat");
  }
#endif

  if (acceptOTA) ArduinoOTA.handle();

  if (systemOff) return;    // No further processing if we're disabled
  
  EVERY_N_MILLIS(20) {
    if (starMotion) {
      rotateStarColors();
    }
  }

  if (gateAnimation > 0) {
    Serial.print("Running gate open animation\n");
    for (int i = NUM_LEDS/2; i >= 0; i--) {
      CRGB left = leds[vINDEX(i)];
      CRGB right = leds[vINDEX(NUM_LEDS-i)];
      leds[vINDEX(i)] = CRGB::White;
      leds[vINDEX(NUM_LEDS-i)] = CRGB::White;      
      FastLED.show();
      delay(20);
      leds[vINDEX(i)] = left;
      leds[vINDEX(NUM_LEDS-i)] = right;   
    }
    gateAnimation = 0;
  }


  EVERY_N_MILLIS(60) {
    if (twinkling) {
      // Serial.print("Twinkle mode... ");
      // Select candidate to twinkle
      for (int i = 0; i < NUM_LEDS; i++) {
        if (!twinkleStatus[i] &&  ((random16(1024)) < CHANCE_OF_TWINKLE )) {
          // Serial.print(i); Serial.print(" ");
          twinkleStatus[i] = TWINKLE_INTERVALS;
          twinkleBack[i] = leds[i];             // Save old value
          leds[i] = CRGB::White;                // Make it white
        }
      }
      // Serial.print("\n");
    }
  }

  EVERY_N_MILLIS(18) {
    if (true) {                               // Don't check twinkling, because we need to allow any in-progress twinks to decay
      bool changes=false;
      for (int i = 0; i < NUM_LEDS; i++) {
        if (twinkleStatus[i]) {
          changes=true;
          leds[i].nscale8(178);               // Set to approx 70% of current value
          // Serial.printf("............... decay %d\n", i);
          twinkleStatus[i]--;
          if (twinkleStatus[i] == 0) {
            leds[i] = twinkleBack[i];         // Restore original color
          }
        }
      }
      if(changes) FastLED.show();
    }
  }
   
  if(steadyMode) return;    // don't rotate roof!
  
  // Rotate the colors every so often
  EVERY_N_SECONDS_I(timingObj, 3 ) {
    
    rotateRoofColors();
    timingObj.setPeriod(90);
    delay(100);  
  }

  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);

}

void setRoofPattern() {
    int activeScheme = eeprom_data.roofMode;
    Serial.printf("set pattern start, signature = %d, len = %d, brightness = %d roofMode = %d\n", 
      eeprom_data.signature, eeprom_data.eepromSize, eeprom_data.brightness, eeprom_data.roofMode); 
    if (activeScheme == Gradient) {
      int step=NUM_LEDS/10;
      int pos=0;
      for (int i = 0; pos < NUM_LEDS; i++) {
        CRGB c1=(i & 0x0001)?CRGB::Red : CRGB::Green;
        CRGB c2=(i & 0x0001)?CRGB::Green : CRGB::Red;
        if (pos+step >= NUM_LEDS) step = (NUM_LEDS - pos);    // Last iteration may be shorter
        fill_gradient_RGB (leds, pos, c1, pos+step, c2);
        pos += step;
      }
      
    } else if (activeScheme == DynColors) {
      
      setDynColors(false, NULL);
      
      
    } else if (activeScheme >= 0) {
    
      CRGB tmp = roofScheme[activeScheme][0];
      for (int i = 0; i < (activeColors[activeScheme] -1); i++ ) {
        roofScheme[activeScheme][i] = roofScheme[activeScheme][i+1];
      }
      roofScheme[activeScheme][activeColors[activeScheme]-1] = tmp;
      
      // Display the new colors
      for (int i=0; i < (NUM_LEDS-(activeColors[activeScheme]-1)); i+=activeColors[activeScheme]) {
        for (int j=0; j < activeColors[activeScheme]; j++) {
          leds[vINDEX(i+j)] = roofScheme[activeScheme][j];
        } 
      }  
    }
  FastLED.show();
    Serial.printf("set pattern end, signature = %d, len = %d, brightness = %d roofMode = %d\n", 
      eeprom_data.signature, eeprom_data.eepromSize, eeprom_data.brightness, eeprom_data.roofMode); 
}
void rotateRoofColors() {
#ifdef DEBUG
  Serial.print("Roof rotation!\n");
  Serial.printf("rotation start, signature = %d, len = %d, brightness = %d roofMode = %d\n", 
      eeprom_data.signature, eeprom_data.eepromSize, eeprom_data.brightness, eeprom_data.roofMode);
#endif
  CRGB tmp = leds[vINDEX(0)];
  for (int i=0; i < NUM_LEDS-1; i++) {
    leds[vINDEX(i)] = leds[vINDEX(i+1)];
  }
  leds[NUM_LEDS-1] = tmp;
  FastLED.show();
#ifdef DEBUG
    Serial.printf("rotation end,   signature = %d, len = %d, brightness = %d roofMode = %d\n", 
      eeprom_data.signature, eeprom_data.eepromSize, eeprom_data.brightness, eeprom_data.roofMode); 
#endif
}

void rotateStarPattern() {
    Serial.print("Star display\n");
    // Display the selected pattern
    for (int i=0; i < NUM_LEDS_STAR1; i++) {
      ledsStar1[i] = starScheme[activeStarScheme][i % activeStarColors[activeStarScheme]];
    }
    FastLED.show();
}

void rotateStarColors() {
  
  CRGB tmp = ledsStar1[0];
  for (int i = 0; i < NUM_LEDS_STAR1-1; i++) {
    ledsStar1[i] = ledsStar1[i+1];
  }
  ledsStar1[NUM_LEDS_STAR1-1] = tmp;
  FastLED.show();
}

void updateEeprom() {
  byte *pos = (byte *)&eeprom_data;
  int len = sizeof(eeprom_data);
  int addr = 0;
#ifdef DEBUG
  Serial.printf("Updating, signature = %d, len = %d, brightness = %d roofMode = %d\n", 
      eeprom_data.signature, eeprom_data.eepromSize, eeprom_data.brightness, eeprom_data.roofMode);
#endif
  if (eeprom_data.signature == EEPROM_SIGNATURE) {
    while (len-- > 0) {
      EEPROM.write( addr++, *pos++);
    }
    EEPROM.commit();
    }
}

#ifdef INCLUDE_TEMP_SENSOR
void publish_sensor_reading()
{
  float tempReading = -1;
  static long int priorValue = -1;
  static uint8_t sensorInit = 0;
  uint8_t mac[6];
  uint8_t topic[(1+6+1+12+1+16+1)];  // "/sensor/{mac}/{sensoraddr}"
  uint8_t *p;

  if (!sensorInit) {
        sensors.setResolution(testThermometer, 9);
        sensorInit++;
  }
  sensors.requestTemperatures();
  delay(100);
  memcpy(topic, "/sensor/", 8);
  /* Convert MAC address */
  WiFi.macAddress(mac);
  p = binToHex( &topic[8], mac, 6);
  *p++ = '/';
  
  p = binToHex( p, testThermometer, 8);

  
  /* Build topic name as /sensor/mac-last4/sensor  */
  tempReading = sensors.getTempF(testThermometer);

  /* Only publish an update if the value changes */
  /* Since we only publish two digits after the decimal, we'll only consider that in the comparison */
  if (((int) (tempReading*100)) != priorValue) {
    Serial.print("Values: ");
    Serial.print(priorValue);
    Serial.print(" / ");
    Serial.println((int) (tempReading*100));
    
    if (!mqtt87.connected()) {
      mqttConnect();
    }
    String tempFormatted = String(tempReading, 2);
    mqtt87.loop();
    mqtt87.publish((const char*)&topic[0], (const char*)tempFormatted.c_str());
    priorValue = tempReading*100;
  } else {
    priorValue = -1;    // Force a publish next time
  }
}

// Publish generic message to mqtt
void publish_status(char  *s) {
  // status/{mac}
  uint8_t mac[6];
  uint8_t topic[(1+6+1+12+1+16+1)];  // "/sensor/{mac}/{sensoraddr}"
  uint8_t *p;

  memcpy(topic, "/status/", 8);
  /* Convert MAC address */
  WiFi.macAddress(mac);
  p = binToHex( &topic[8], mac, 6);
  *p++ = '/';
  *p++ = '\0';

  if (!mqtt87.connected()) {
    mqttConnect();
  }
  mqtt87.loop();
  mqtt87.publish((const char*)&topic[0], (const char*)s);
}
#endif

/* converts a binary value to formatted hex.   Writes (byteCount*2 + 1) bytes to dest, including a termimator */
/* returns pointer to the terminator */

uint8_t *binToHex( uint8_t *dest, uint8_t *src, int byteCount) {
  uint8_t *p = dest;
  for (uint8_t i = 0; i < byteCount; i++) {
    *p++ = ((src[i] >> 4) < 10 ? ((src[i] >> 4) + '0') : ((src[i] >> 4) -10 + 'a'));
    *p++ = ((src[i] & 0x0f) < 10 ? ((src[i] & 0x0f) + '0') : ((src[i] & 0x0f) -10 + 'a'));
  }
  *p = '\0';
  return(p);
}
