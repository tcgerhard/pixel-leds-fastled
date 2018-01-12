# pixel-leds-fastled
Arduino sketches and related artifacts for LED pixel projects

This sketch runs on ESP8266 and has the following functionality
* Supports 3 strings of WS8211 LEDs:
  1. Gable end (pixel #1 is on the right end)
  2. Soffit (pixel #1 is on the left)
  3. A star
  
  The gable and soffit are treated as a single logical string in some places; there is some cruft code from debugging a signalling issue with the gable string.
  The star is a 5-point star with 60 LEDS, with #1 at the top.
  
* An MQTT listener that accepts various commamnds to update the state of the system.   The topic name that is configured in GarageConfig.h
* Commands that are accepted:
1.  ota .  // Enables OTA updates to the Sketch
2. debug:1, debug:2, debug:3  . // Displays some test patterns
3. gate:open    // Triggers an animation (used when gate is opened)
4. gate:close   // Future use
5. color:rrr,ggg,bbb/   // Up to 5 triplets; rrr is 3-digit from 000-255.   The parser for this is very fragile.
6. dim:nmm              // Set the default system brightness from 0 to 255
7. system:[off|on]           // Dims the system to 0 brightness/restore default, disables/enables animations
9. twinkle:[on|off]     // randomly "twinkle" pixels on the roofline.   A work in progress
10. pattern:roof        // Pick the next sequential roof pattern
11. pattern:star        // Pick the next sequential star pattern
