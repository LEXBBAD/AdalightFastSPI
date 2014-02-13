#include "FastLED.h"

// How many leds in your strip?
#define NUM_LEDS 36

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 53
#define CLOCK_PIN 52

// Define the array of leds
CRGB leds[NUM_LEDS];

#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  _BV(PORTB5)

static const uint8_t magic[] = {'A','d','a'};
#define MAGICSIZE  sizeof(magic)
#define HEADERSIZE (MAGICSIZE + 3)

#define MODE_HEADER 0
#define MODE_HOLD   1
#define MODE_DATA   2

static const unsigned long serialTimeout = 15000; // 15 seconds


void setup() { 
      // Uncomment/edit one of the following lines for your leds arrangement.
      // FastLED.addLeds<TM1803, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<TM1804, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<TM1809, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  	 // FastLED.addLeds<NEOPIXEL, DATA_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<UCS1903, DATA_PIN, RGB>(leds, NUM_LEDS);

      //FastLED.addLeds<WS2801, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<SM16716, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<LPD8806, RGB>(leds, NUM_LEDS);

       FastLED.addLeds<WS2801, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS);
      // FastLED.addLeds<SM16716, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
      // FastLED.addLeds<LPD8806, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  uint8_t    
    buffer[256],
    indexIn       = 0,
    indexOut      = 0,
    mode          = MODE_HEADER,
    hi, lo, chk, i, spiFlag,
    rgbIndex      = 0; // 0 = red, 1 = green, 2 = blue
  int16_t
    bytesBuffered = 0,
    hold          = 0,
    ledIndex      = 0, // current LED index
    c;
  int32_t
    bytesRemaining;
  unsigned long
    startTime,
    lastByteTime,
    lastAckTime,
    t;
  LED_DDR  |=  LED_PIN; // Enable output for LED
  LED_PORT &= ~LED_PIN; // LED off
    
      for(char n=4; n>=0; n--) {
        memset(leds, 0, NUM_LEDS * 3);
        for(c=0; c<NUM_LEDS; c++) {
          switch(n){
            case 1:
              leds[c].b = 255;
              break;
            case 2:
              leds[c].g = 255;
              break;
            case 3:
              leds[c].r = 255;
              break;
            case 3:
              leds[c] = CRGB:White;
              break;
            case 0:
              leds[c] = CRGB::Black;
              break;
        }
        FastSPI_LED.show();
        delay(1); // One millisecond pause = latch
      }  
    }
    Serial.begin(115200); // Teensy/32u4 disregards baud rate; is OK!

    Serial.print("Ada\n"); // Send ACK string to host

    startTime    = micros();
    lastByteTime = lastAckTime = millis(); 
    for(;;) 
    {
      // Implementation is a simple finite-state machine.
      // Regardless of mode, check for serial input each time:
      t = millis();
      if((bytesBuffered < 256) && ((c = Serial.read()) >= 0)) 
      {
        buffer[indexIn++] = c;
        bytesBuffered++;
        lastByteTime = lastAckTime = t; // Reset timeout counters
        
      } 
      else 
      {
        // No data received.  If this persists, send an ACK packet
        // to host once every second to alert it to our presence.
        if((t - lastAckTime) > 1000) 
        {
          Serial.print("Ada\n"); // Send ACK string to host
          lastAckTime = t; // Reset counter
        }
        // If no data received for an extended time, turn off all LEDs.
        if((t - lastByteTime) > serialTimeout) 
        {
          for(c=0; c<NUM_LEDS; c++) {
              leds[c] = CRGB::Black;
          }
          FastSPI_LED.show();
          delay(1); // One millisecond pause = latch
          lastByteTime = t; // Reset counter
        }
      }
      switch(mode) {
        case MODE_HEADER:
          // In header-seeking mode.  Is there enough data to check?
          if(bytesBuffered >= HEADERSIZE)
          {
            // Indeed.  Check for a 'magic word' match.
            for(i=0; (i<MAGICSIZE) && (buffer[indexOut++] == magic[i++]););
            if(i == MAGICSIZE) 
            {
              // Magic word matches.  Now how about the checksum?
              hi  = buffer[indexOut++];
              lo  = buffer[indexOut++];
              chk = buffer[indexOut++];
              if(chk == (hi ^ lo ^ 0x55))
              {
                // Checksum looks valid.  Get 16-bit LED count, add 1
                // (# LEDs is always > 0) and multiply by 3 for R,G,B.
                bytesRemaining = 3L * (256L * (long)hi + (long)lo + 1L);
                bytesBuffered -= 3;
                ledIndex = 0;
                spiFlag        = 0;         // No data out yet
                mode           = MODE_HOLD; // Proceed to latch wait mode
                //memset(leds, 0, NUM_LEDS * 3);
                //ShowColorAndReset(CRGB::Red);
              } 
              else 
              {
                // Checksum didn't match; search resumes after magic word.
                indexOut  -= 3; // Rewind
              }
            } // else no header match.  Resume at first mismatched byte.
            bytesBuffered -= i;
          }
          
          break;
        case MODE_HOLD:
          // Ostensibly "waiting for the latch from the prior frame
          // to complete" mode, but may also revert to this mode when
          // underrun prevention necessitates a delay.
 
          if((micros() - startTime) < hold)
            break; // Still holding; keep buffering

          // Latch/delay complete.  Advance to data-issuing mode...
          LED_PORT &= ~LED_PIN;  // LED off
          mode      = MODE_DATA; // ...and fall through (no break):
          memset(leds, 0, NUM_LEDS * 3);
        case MODE_DATA:
          if(bytesRemaining > 0) {
            if(bytesBuffered > 0) {
              switch(rgbIndex++) {
                case 0:
                  leds[ledIndex].r = buffer[indexOut++];
                  break;
                case 1:
                  leds[ledIndex].g = buffer[indexOut++];
                  break;
                case 2:
                  // Set the blue component, and increment LED index
                  leds[ledIndex++].b = buffer[indexOut++];
                  rgbIndex = 0; // Reset to red for next byte
                  break;
              }
              bytesBuffered--;
              bytesRemaining--;
              spiFlag = 1;
            //ShowColorAndReset(CRGB::Blue);
            }
          }
          else 
          {
            //ShowColorAndReset(CRGB::Blue);
            FastSPI_LED.show();
            delay(1); // One millisecond pause = latch
            // End of data -- issue latch:
            startTime  = micros();
            hold       = 1000;        // Latch duration = 1000 uS
            LED_PORT  |= LED_PIN;     // LED on
            mode       = MODE_HEADER; // Begin next header search
      }
          break;
      } // end switch
      
    }
}

static void ShowColorAndReset(CRGB crgb)
{
  int16_t c;
  for(char n=1; n>=0; n--) {
    for(c=0; c<NUM_LEDS; c++) {
      switch(n){
        case 1:
          leds[c] = crgb;
          break;
        case 0:
          leds[c] = CRGB::Black;
          break;
      }
      FastSPI_LED.show();
      delay(1); // One millisecond pause = latch
    }  
  }
}
void loop() { 
}
