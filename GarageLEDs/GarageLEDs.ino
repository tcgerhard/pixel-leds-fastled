#include <ArduinoOTA.h>

# include <ESP8266WiFi.h>
# include <PubSubClient.h>


# include "GarageConfig.h"

WiFiClient espClient;
PubSubClient mqtt87(espClient);

static uint8_t mac[6];

#define CLIENTID_PREFIX "esp8266"

static char mqttClientId[sizeof(CLIENTID_PREFIX)+13];   // "ESP8266ffeeddccbbaa" . (last

#define FASTLED_ESP8266_D1_PIN_ORDER
#include <FastLED.h>
#define NUM_LEDS_GABLE 81
#define NUM_LEDS_SOFFIT 100
#define NUM_LEDS (NUM_LEDS_GABLE + NUM_LEDS_SOFFIT)
#define NUM_LEDS_STAR1  60
//#define DATA_PIN_GABLE 4    // D2 
#define DATA_PIN_GABLE 7    // D7
#define DATA_PIN_SOFFIT 5   // D5
#define DATA_PIN_STAR1 6     // D6
#define MAX_COLOR_SCHEME 8
#define MAX_SCHEMES 3
#define TWINKLE_INTERVALS 5 // Number of 10ms cycles for a "twinkle"
#define CHANCE_OF_TWINKLE 1 // Higher is more likely to twinkle


// #define DATA_PIN_SOFFIT 6   // D6

//  Red => dull green
//  Green => dull red
//  Blue =>  bright green
//  White => yellowish green

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
int activeColors[MAX_SCHEMES];
int activeStarColors[MAX_SCHEMES];
int activeScheme = 0;
int activeStarScheme = 0;
bool acceptOTA = false;        // MQTT message can change this, to allow OTA update.
bool debugMode = false;

int gateAnimation = 0;    // +1 for gate opening, -1 for gate closing
bool systemOff = false;
bool twinkling = false;
bool starMotion = false;
bool steadyMode = false; 

int maxStarScheme, maxRoofScheme;

static int wifiIteration = 0;
static int defaultBrightness = 128;
static int currentBrightness = defaultBrightness;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("In setup\n");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

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

  roofScheme[2][0] = CRGB::Blue;
  roofScheme[2][1] = CRGB::Blue;
  roofScheme[2][2] = CRGB::Blue;
  roofScheme[2][3] = CRGB::White;
  activeColors[2] = 4;
  maxRoofScheme = 3;

  starScheme[0][0] = CRGB::Red;
  starScheme[0][1]= CRGB::Orange;
  starScheme[0][2] = CRGB::Green;
  starScheme[0][3] = CRGB::Blue;
  activeStarColors[0] = 4;
  
  starScheme[1][0] = CRGB::Goldenrod;
  activeStarColors[1] = 1;

  starScheme[2][0] = CRGB::Red;
  starScheme[2][1] = CRGB::Green;
  activeStarColors[2] = 2;
  maxStarScheme = 3;
  
  Serial.print("adding Leds\n");
  
  ctlGable = &FastLED.addLeds<WS2811, DATA_PIN_GABLE, RGB>(ledsGable, NUM_LEDS_GABLE).setCorrection(TypicalSMD5050);
  ctlSoffit = &FastLED.addLeds<WS2811, DATA_PIN_SOFFIT>(ledsSoffit, NUM_LEDS_SOFFIT).setCorrection(TypicalSMD5050);
  ctlStar1 = &FastLED.addLeds<WS2811, DATA_PIN_STAR1>(ledsStar1, NUM_LEDS_STAR1).setCorrection(TypicalSMD5050);

  for (int i = 0; i < NUM_LEDS; i++) {
   leds[i] = CRGB::Black;
   twinkleStatus[i] = 0;
  }
  fill_solid(ledsStar1, NUM_LEDS_STAR1, CRGB::Goldenrod);
  FastLED.setBrightness(currentBrightness);
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
  status = WiFi.begin(WiFiSSID, WiFiPSK);
  delay(3000);                          // Allow 3 seconds for connection to establish
  Serial.print("Status from WiFi.begin: ");
  Serial.println(status);

  // Wait up to 20sec for a connection (including time above)
  while (WiFi.status() != WL_CONNECTED && (millis() - startMillis) < maxWait) {
    // Blink the LED
    digitalWrite(LED_PIN, ledStatus);
    Serial.print(".");
    ledStatus = (ledStatus == HIGH) ? LOW: HIGH;
    delay(200);
  }

  // Setup mqtt clientId based on our Mac address
  strcpy(mqttClientId, CLIENTID_PREFIX);
  WiFi.macAddress(mac);
  (void) binToHex( (unsigned char *)&mqttClientId[sizeof(CLIENTID_PREFIX)-1], mac, 6);
  
  Serial.print("\nStatus from  WiFi.status(): ");  Serial.println(WiFi.status());
  digitalWrite(LED_PIN, LOW);
  return(WiFi.status());
}

void mqttConnect() {
  while (!mqtt87.connected()) {
    Serial.printf("Attempting MQTT connection %s ...", mqttClientId);
    if (mqtt87.connect(mqttClientId)) {
      Serial.println("connected");
      mqtt87.subscribe(MQTT_TOPIC);
      mqtt87.setCallback(mqttCallback);
    } else {
      delay(1);
      Serial.print("failed, rc=");
      Serial.print(mqtt87.state());
      Serial.println(" try again in 5 seconds...");
      delay(5000);
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
  } else if (strcmp(tmp, "debug:2") == 0 ) {
    debugMode = !debugMode;                   // debug displays some test patterns
    for (int i = 0, c = 0; i < NUM_LEDS_GABLE; i++) {
      ledsGable[i ] = (uint32_t) c++ << 8;
      FastLED.show();
      FastLED.delay(3);
      if ((i % NUM_LEDS_GABLE) == (NUM_LEDS_GABLE-1)) FastLED.delay(10000);
    }
  } else if (strcmp(tmp, "debug:3") == 0 ) {
    debugMode = !debugMode;                   // debug displays some test patterns
    for (int i = 0, c = 0; i < 255; i++) {
      ledsGable[i % NUM_LEDS_GABLE] =  (uint32_t) c++ << 16;
      FastLED.show();
      FastLED.delay(3);
      if ((i % NUM_LEDS_GABLE) == (NUM_LEDS_GABLE-1)) FastLED.delay(10000);
    }

  } else if (strcmp(tmp, "gate:open") == 0) {
    Serial.println("Gate opening");
    gateAnimation = 1;
  } else if (strcmp(tmp, "gate:close") == 0) {
    gateAnimation = -1;
  } else if (strncmp(tmp, "color:", 6) == 0) {
#define MAX_DYN_COLORS 5
    CRGB colors[MAX_DYN_COLORS];
    int dynColorCount = 0;
    char *curTriplet = &tmp[6];
    // Rigid format, up to five triplets of zero-padded numbers 255,064,000;
    // Delimeters can be any non-numeric
    // TODO: make color parser more flexible

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
    for (int i = 0; i < NUM_LEDS_SOFFIT; i++) {
      ledsSoffit[i] = colors[i % dynColorCount];
    }
    ctlSoffit->showLeds();
    
    for (int i = 0; i < NUM_LEDS_GABLE; i++) {
      ledsGable[i] = colors[i % dynColorCount];
    }
    steadyMode = true;
    
    ctlGable->showLeds();

    
  } else if (strncmp(tmp, "dim:", 4) == 0) {
    // Get the characters following the ':'
    String dimStr(&tmp[4]);
    int dimVal = dimStr.toInt();
    Serial.printf("Got dim value of %s\n", tmp);
    if ( 0 <= dimVal <= 255) {
      currentBrightness = dimVal;
      FastLED.setBrightness(currentBrightness);
      FastLED.show();
    }
  } else if (strcmp(tmp, "system:off") == 0) {
    systemOff = true;
    while (currentBrightness > 0) {
      FastLED.setBrightness(--currentBrightness);
      FastLED.show();
      delay(12);
    }
  } else if (strcmp(tmp, "system:on") == 0) {
    systemOff = false;
    while (currentBrightness < defaultBrightness) {
      FastLED.setBrightness(++currentBrightness);
      FastLED.show();
      delay(12);
    }
  } else if (strcmp(tmp, "twinkle:on") == 0) {
    twinkling = true;
    
  } else if (strcmp(tmp, "twinkle:off") == 0) {
    twinkling = false;

  } else if (strcmp(tmp, "pattern:roof") == 0) {
    activeScheme = (activeScheme + 1) % maxRoofScheme;
    steadyMode = false;
    rotateRoofColors();
  } else if (strcmp(tmp, "pattern:star") == 0) {
    activeStarScheme = (activeStarScheme + 1) % maxStarScheme;
    rotateStarPattern();
  } else if (strcmp(tmp, "star:motion") == 0) {
    starMotion = !starMotion;
  }
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

void rotateRoofColors() {
    Serial.print("Color rotation!\n");
    
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
  FastLED.show();
}

void rotateStarPattern() {
    Serial.print("Star display\n");
    // Display the selected pattern
    for (int i=0; i < NUM_LEDS_STAR1; i++) {
      ledsStar1[i] = starScheme[activeStarScheme][i % activeColors[activeStarScheme]];
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

