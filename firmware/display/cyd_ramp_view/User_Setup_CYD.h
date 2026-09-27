// User_Setup_CYD.h - TFT_eSPI configuration for the ESP32-2432S028R (Cheap Yellow Display, 2.8" ILI9341,
// resistive touch). Put it IN PLACE OF User_Setup.h in the TFT_eSPI library folder
// (Arduino/libraries/TFT_eSPI/User_Setup.h), renaming it to User_Setup.h. The cyd_ramp_view sketch
// cannot configure the library by itself: TFT_eSPI (2.5.x) reads its setup from User_Setup.h /
// User_Setup_Select.h inside the library folder, not from the sketch.
// Reference build: TFT_eSPI 2.5.43, ESP32 Arduino core 3.3.x, one particular CYD batch - the
// driver/inversion lines below may need adjusting for your panel.
#define USER_SETUP_INFO "CYD ESP32-2432S028R"

#define ILI9341_2_DRIVER          // TFT_eSPI's alternative ILI9341 init sequence; the author's panel shows wrong
                                  // colours with the plain ILI9341_DRIVER (reference build; adjust for yours)
#define TFT_WIDTH   240
#define TFT_HEIGHT  320

// If the picture looks like a negative, swap which one of these two lines is active (adjust for your build).
#define TFT_INVERSION_ON          // reference build: the panel on the author's CYD batch is inverted - without this
                                  // the background is white and the text black
// #define TFT_INVERSION_OFF

#define TFT_MISO    12
#define TFT_MOSI    13
#define TFT_SCLK    14
#define TFT_CS      15
#define TFT_DC      2
#define TFT_RST     -1            // reset is tied to EN, there is no separate pin
#define TFT_BL      21
#define TFT_BACKLIGHT_ON HIGH

// The XPT2046 touch controller on the CYD sits on a SEPARATE bus (CLK 25, MOSI 32, MISO 39, CS 33),
// so TFT_eSPI's built-in touch support (which expects it on the display's SPI bus) cannot drive it.
// The sketch does not use touch - TOUCH_CS is not defined.

#define LOAD_GLCD                 // font 1
#define LOAD_FONT2                // font 2
#define LOAD_FONT4                // font 4
#define LOAD_GFXFF
// SMOOTH_FONT is not enabled: with TFT_eSPI 2.5.x it pulls in SPIFFS, which the display does not need.

#define SPI_FREQUENCY       40000000   // reference build; lower it if your panel shows artefacts
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000   // unused here (no TOUCH_CS), kept for completeness

#define USE_HSPI_PORT             // display on HSPI - the standard CYD wiring
