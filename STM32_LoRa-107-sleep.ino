#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C #define BME280_ADDRESS                (0x76)

float temperature = 0;
float humidity = 0;
uint16_t tempC_int = 0;
uint8_t hum_int = 0;

//#define DEBUG
#define SLEEP

#define led      PC13
#define voltage   PA0

#define USE_SPI   2

// LoRaWAN NwkSKey, your network session key, 16 bytes (from staging.thethingsnetwork.org)
static unsigned char NWKSKEY[16] = { 0x18, 0x47, 0x12, 0x64, 0x64, 0xDB, 0x13, 0x09, 0x36, 0xFE, 0x21, 0xE6, 0x41, 0x6D, 0xB1, 0xBD };

// LoRaWAN AppSKey, application session key, 16 bytes  (from staging.thethingsnetwork.org)
static unsigned char APPSKEY[16] = { 0x59, 0x4B, 0xDB, 0x8F, 0x19, 0x4B, 0x12, 0x9A, 0x18, 0x43, 0xEE, 0x5B, 0x8E, 0xD1, 0xD9, 0x38 };

// LoRaWAN end-device address (DevAddr), ie 0x91B375AC  (from staging.thethingsnetwork.org)
static const u4_t DEVADDR = 0x260419C7 ; // <-- Change this address for every node!


// STM32 Unique Chip IDs
#define STM32_ID  ((u1_t *) 0x1FFFF7E8)

SPIClass mySPI(USE_SPI);

extern SPIClass *SPIp;
int channel = 0;
int txInterval = 60;

unsigned long sampletime_ms = 30000;  // 30 Seconds

#define RATE        DR_SF10

struct {
  uint8_t Header1[2] = {0x01,0x67};
  char temp[2];
  uint8_t Header2[2] = {0x02,0x68};
  char humid[1];
  uint8_t Header3[2] = {0x03,0x02};
  char vbat[2];  
}mydata;

  byte power;
  byte rate2;

#ifdef SLEEP

// Defined for power and sleep functions pwr.h and scb.h
#include <libmaple/pwr.h>
#include <libmaple/scb.h>

#include <RTClock.h>

RTClock rt(RTCSEL_LSI, 399); // 10 milli second alarm

// Define the Base address of the RTC registers (battery backed up CMOS Ram), so we can use them for config of touch screen or whatever.
// See http://stm32duino.com/viewtopic.php?f=15&t=132&hilit=rtc&start=40 for a more details about the RTC NVRam
// 10x 16 bit registers are available on the STM32F103CXXX more on the higher density device.
#define BKP_REG_BASE   ((uint32_t *)(0x40006C00 +0x04))

void storeBR(int i, uint32_t v) {
  BKP_REG_BASE[2 * i] = (v << 16);
  BKP_REG_BASE[2 * i + 1] = (v & 0xFFFF);
}

uint32_t readBR(int i) {
  return ((BKP_REG_BASE[2 * i] & 0xFFFF) >> 16) | (BKP_REG_BASE[2 * i + 1] & 0xFFFF);
}

bool next = false;

void sleepMode(bool deepSleepFlag)
{
  // Clear PDDS and LPDS bits
  PWR_BASE->CR &= PWR_CR_LPDS | PWR_CR_PDDS | PWR_CR_CWUF;

  // Set PDDS and LPDS bits for standby mode, and set Clear WUF flag (required per datasheet):
  PWR_BASE->CR |= PWR_CR_CWUF;
  // Enable wakeup pin bit.
  PWR_BASE->CR |=  PWR_CSR_EWUP;

  SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;

  // System Control Register Bits. See...
  // http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/Cihhjgdh.html
  if (deepSleepFlag) {
    // Set Power down deepsleep bit.
    PWR_BASE->CR |= PWR_CR_PDDS;
    // Unset Low-power deepsleep.
    PWR_BASE->CR &= ~PWR_CR_LPDS;
  } else {
    adc_disable(ADC1);
    adc_disable(ADC2);
#if STM32_HAVE_DAC
    dac_disable_channel(DAC, 1);
    dac_disable_channel(DAC, 2);
#endif
    //  Unset Power down deepsleep bit.
    PWR_BASE->CR &= ~PWR_CR_PDDS;
    // set Low-power deepsleep.
    PWR_BASE->CR |= PWR_CR_LPDS;
  }

  // Now go into stop mode, wake up on interrupt
  asm("    wfi");

  // Clear SLEEPDEEP bit so we can use SLEEP mode
  SCB_BASE->SCR &= ~SCB_SCR_SLEEPDEEP;
}

uint32 sleepTime;

void AlarmFunction () {
  // We always wake up with the 8Mhz HSI clock!
  // So adjust the clock if needed...

#if F_CPU == 8000000UL
  // nothing to do, using about 8 mA
#elif F_CPU == 16000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_2);
#elif F_CPU == 48000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_6);
#elif F_CPU == 72000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_9);
#else
#error "Unknown F_CPU!?"
#endif

  extern volatile uint32 systick_uptime_millis;
  systick_uptime_millis += sleepTime;
}

void mdelay(int n, bool mode = false)
{
  sleepTime = n;
  time_t nextAlarm = (rt.getTime() + n / 10); // Calculate from time now.
  rt.createAlarm(&AlarmFunction, nextAlarm);
  sleepMode(mode);
}

void msleep(uint32_t ms)
{
  uint32_t start = rt.getTime();

  while (rt.getTime() - start < ms) {
    asm("    wfi");
  }
}
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = PB12,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = PA8,
  .dio = {PB1, PB10, PB11}
};


bool TX_done = false;

bool joined = false;

void onEvent (ev_t ev) {
#ifdef DEBUG
  Serial.println(F("Enter onEvent"));
#endif

  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      joined = true;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      TX_done = true;
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print(F("Data Received: "));
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
        rate2 = (LMIC.frame + LMIC.dataBeg)[0];
        txInterval = (1 << rate2);
        if (LMIC.dataLen > 1) {
          switch ((LMIC.frame + LMIC.dataBeg)[1]) {
            case 7: LMIC_setDrTxpow(DR_SF7, 14); break;
            case 8: LMIC_setDrTxpow(DR_SF8, 14); break;
            case 9: LMIC_setDrTxpow(DR_SF9, 14); break;
            case 10: LMIC_setDrTxpow(DR_SF10, 14); break;
            case 11: LMIC_setDrTxpow(DR_SF11, 14); break;
            case 12: LMIC_setDrTxpow(DR_SF12, 14); break;
          }
        }
      }
      // Schedule next transmission
#ifndef SLEEP
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(txInterval), do_send);
#endif
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
#ifdef DEBUG
  Serial.println(F("Leave onEvent"));
#endif
#ifdef SLEEP
  next = true; // Always send after any event, to recover from a dead link
#endif
}

void do_send(osjob_t* j) {

#ifdef DEBUG
  Serial.println(F("Enter do_send"));
#endif

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    readData();
#ifdef SLEEP
    // Disable link check validation
    LMIC_setLinkCheckMode(0);
#endif
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
#ifdef DEBUG
  Serial.println(F("Leave do_send"));
#endif
  TX_done = false;

}

int vbat_int;

void readData()
{
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    tempC_int = temperature*10;
    hum_int = humidity*2;
    adc_enable(ADC1);
    vbat_int = 120 * 4096 / adc_read(ADC1, 17);
    adc_disable(ADC1);
    mydata.temp[0] = tempC_int >> 8;
    mydata.temp[1] = tempC_int;
    mydata.humid[0] =  hum_int ;
    mydata.vbat[0] = vbat_int >> 8;
    mydata.vbat[1] = vbat_int;    

#ifdef DEBUG
    Serial.print(temperature);Serial.print(" ");Serial.print(humidity);Serial.print(" ");Serial.println(vbat_int);
#endif   
}

void allInput()
{
  adc_disable(ADC1);
  adc_disable(ADC2);

  pinMode(PA0, INPUT_ANALOG);
  pinMode(PA1, INPUT_ANALOG);
  pinMode(PA2, INPUT_ANALOG);
  pinMode(PA3, INPUT_ANALOG);
  pinMode(PA4, INPUT_ANALOG);
  pinMode(PA5, INPUT_ANALOG);
  pinMode(PA6, INPUT_ANALOG);
  pinMode(PA7, INPUT_ANALOG);
  pinMode(PA8, INPUT_ANALOG);
  //pinMode(PA9, INPUT_ANALOG);
  //pinMode(PA10, INPUT_ANALOG);
  
  pinMode(PA11, INPUT_ANALOG);
  pinMode(PA12, INPUT_ANALOG);
  pinMode(PA13, INPUT_ANALOG);
  pinMode(PA14, INPUT_ANALOG);
  pinMode(PA15, INPUT_ANALOG);

  pinMode(PB0, INPUT_ANALOG);
  //pinMode(PB1, INPUT_ANALOG);
  pinMode(PB2, INPUT_ANALOG);
  pinMode(PB3, INPUT_ANALOG);
  pinMode(PB4, INPUT_ANALOG);
  pinMode(PB5, INPUT_ANALOG);
  //pinMode(PB6, INPUT_ANALOG);
  //pinMode(PB7, INPUT_ANALOG);
  pinMode(PB8, INPUT_ANALOG);
  pinMode(PB9, INPUT_ANALOG);
  //pinMode(PB10, INPUT_ANALOG);
  //pinMode(PB11, INPUT_ANALOG);
  pinMode(PB12, INPUT_ANALOG);
  pinMode(PB13, INPUT_ANALOG);
  pinMode(PB14, INPUT_ANALOG);
  pinMode(PB15, INPUT_ANALOG);
}

void setup_vdd_sensor() {
    adc_reg_map *regs = ADC1->regs;
    regs->CR2 |= ADC_CR2_TSVREFE; // enable VREFINT and temp sensor
    regs->SMPR1 = (ADC_SMPR1_SMP17 /* | ADC_SMPR1_SMP16 */); // sample rate for VREFINT ADC channel
}

void setup() {
allInput();
setup_vdd_sensor();
SPIp = &mySPI;
Serial.begin(9600);
   
bool status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 923600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 923800000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    
  LMIC_setupChannel(4, 924000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    
  LMIC_setupChannel(5, 924200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 
  LMIC_setupChannel(6, 924400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band          
  LMIC_setupChannel(7, 924600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    

#if F_CPU == 8000000UL
  // HSI is less accurate
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
#endif

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  //forceTxSingleChannelDr(); 
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(RATE, 14);

  // Start job
   do_send(&sendjob);
#ifdef DEBUG
  Serial.println(F("Leave setup"));
  Serial.println();
#endif   
}

void loop() {
if (next == false) {  
    LMIC.skipRX = 1; // Do NOT wait for downstream data!
    os_runloop_once();
  }else {    
    mdelay(txInterval *1000, 0);
    next = false;
    do_send(&sendjob);
 }
}
