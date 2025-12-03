/* 
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 */

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1,PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

uint16_t batteryInMV()
{
  int adcin = PIN_BAT_ADC;   //battery adc pin 4
  int adcvalue = 0;
  analogReadResolution(12);
  analogReference(AR_INTERNAL_3_0);
  float mv_per_lsb = 3000.0F / 4096.0F;  //12-bit ADC with 3.0V input range
  pinMode(PIN_BAT_ADC_CTL, OUTPUT);      //battery adc can be read only ctrl pin 6 set to high 
  digitalWrite(PIN_BAT_ADC_CTL, 1);      
  delay(10);
  adcvalue = analogRead(adcin);
  digitalWrite(6, 0);
  uint16_t v = (uint16_t)((float)adcvalue * mv_per_lsb * 4.9);
  return v;
}

void setup() {
    pinMode(PIN_TFT_VDD_CTL,OUTPUT);
    pinMode(PIN_TFT_LEDA_CTL,OUTPUT);
    digitalWrite(PIN_TFT_VDD_CTL,TFT_VDD_ENABLE);  //set pin 3 low to enable tft vdd
    digitalWrite(PIN_TFT_LEDA_CTL,TFT_LEDA_ENABLE);//set pin 3 low to enable tft ledA
    tft.init(135, 240);           // Init ST7789 240x135
    tft.setRotation(3);
    tft.setSPISpeed(40000000);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextWrap(false);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(2);//12*16
}


void loop() {
    String tmp="battery: "+ (String)batteryInMV() + " mV";
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(tft.width()/2-tmp.length()/2*12, tft.height()/2-16/2); // A char width is TextSize*6, height is TextSize*8
    tft.println(tmp);
    delay(1000);
}