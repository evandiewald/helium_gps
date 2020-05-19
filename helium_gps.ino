#include "teseo_liv3f_class.h"
#include <SPI.h>
#include <arduino_lmic.h>
#include <arduino_lmic_hal_boards.h>
#include <arduino_lmic_hal_configuration.h>
#include <arduino_lmic_lorawan_compliance.h>
#include <arduino_lmic_user_configuration.h>
#include <hal/hal.h>
#include <lmic.h>
#include <CayenneLPP.h>

// This is the "App EUI" in Helium. Make sure it is little-endian (lsb).
static const u1_t PROGMEM APPEUI[8] = {FILL_ME_IN};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format
// These are user configurable values and Helium console permits anything
static const u1_t PROGMEM DEVEUI[8] = {FILL_ME_IN};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This is the "App Key" in Helium. It is big-endian (msb).
static const u1_t PROGMEM APPKEY[16] = {FILL_ME_IN};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
//
// Adafruit BSPs are not consistent -- m0 express defs ARDUINO_SAMD_FEATHER_M0,
// m0 defs ADAFRUIT_FEATHER_M0
//
#if defined(ARDUINO_SAMD_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0)
// Pin mapping for Adafruit Feather M0 LoRa, etc.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8, // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};
#elif defined(ARDUINO_AVR_FEATHER32U4)
// Pin mapping for Adafruit Feather 32u4 LoRa, etc.
// Just like Feather M0 LoRa, but uses SPI at 1MHz; and that's only
// because MCCI doesn't have a test board; probably higher frequencies
// will work.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8, // LBT cal for the Adafruit Feather 32U4 LoRa, in dB
    .spi_freq = 1000000,
};
#elif defined(ARDUINO_CATENA_4551)
// Pin mapping for Murata module / Catena 4551
const lmic_pinmap lmic_pins = {
    .nss = 7,
    .rxtx = 29,
    .rst = 8,
    .dio =
        {
            25, // DIO0 (IRQ) is D25
            26, // DIO1 is D26
            27, // DIO2 is D27
        },
    .rxtx_rx_active = 1,
    .rssi_cal = 10,
    .spi_freq = 8000000 // 8MHz
};
#elif defined(MCCI_CATENA_4610)
#include "arduino_lmic_hal_boards.h"
const lmic_pinmap lmic_pins = *Arduino_LMIC::GetPinmap_Catena4610();
#elif defined(ARDUINO_DISCO_L072CZ_LRWAN1)
const lmic_pinmap lmic_pins = *Arduino_LMIC::GetPinmap_Disco_L072cz_Lrwan1();
#else
#error "Unknown target"
#endif

CayenneLPP lpp(24);



TeseoLIV3F *gps;
int incomingByte;
GNSSParser_Data_t data;
char command[32] = {0};
char msg[256];
int cmdType = 0;
uint32_t t, s;
int tracked;
GPGGA_Info_t stored_positions[64];
int status = 0;
uint32_t stime = 0;
int waitType = 0;

#ifdef ARDUINO_ARCH_STM32
HardwareSerial Serial1(PA10, PA9);
#endif

#define MSG_SZ 256
#define waitForRequest 0
#define waitForAnswer 1

#ifdef ARDUINO_SAM_DUE
#include <avr/dtostrf.h>
#endif


void setup()
{
     //Initialize serial port for user communication
   Serial.begin(115200);
   //Initialize serial port for device communication
   Serial1.begin(9600);
   Serial.println("Setup begin");
   //Create the device object passing to it the serial interface
   gps = new TeseoLIV3F(&Serial1, 7, 13);
   //Initialize the device
   gps->init();
   Serial.println("Seeking GPS fix");


    
   gps->update();
   while (data.gpgga_data.valid != 1){
        gps->update();
        printValidInfo();
        //delay(500);

      }
    delay(5000);
    Serial.println(F("Starting"));
  
  #if defined(ARDUINO_DISCO_L072CZ_LRWAN1)
    SPI.setMOSI(RADIO_MOSI_PORT);
    SPI.setMISO(RADIO_MISO_PORT);
    SPI.setSCLK(RADIO_SCLK_PORT);
    SPI.setSSEL(RADIO_NSS_PORT);
  // SPI.begin();
  #endif
  
  #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
  #endif
  
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
  
    // allow much more clock error than the X/1000 default. See:
    // https://github.com/mcci-catena/arduino-lorawan/issues/74#issuecomment-462171974
    // https://github.com/mcci-catena/arduino-lmic/commit/42da75b56#diff-16d75524a9920f5d043fe731a27cf85aL633
    // the X/1000 means an error rate of 0.1%; the above issue discusses using
    // values up to 10%. so, values from 10 (10% error, the most lax) to 1000
    // (0.1% error, the most strict) can be used.
    LMIC_setClockError(1 * MAX_CLOCK_ERROR / 40);
  
    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF8, 20);
    // Sub-band 2 - Helium Network
    LMIC_selectSubBand(1); // zero indexed
  
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);

  
  
}

void loop()
{
  os_runloop_once();

}



void printValidInfo()
{
   data = gps->getData();
   if (data.gpgga_data.valid == 1)
   {
      int lat = (int) (data.gpgga_data.xyz.lat/100.0);
      int lat_mod = (int) (data.gpgga_data.xyz.lat) - (lat * 100);
      int lon = (int) (data.gpgga_data.xyz.lon/100.0);
      int lon_mod = (int) (data.gpgga_data.xyz.lon) - (lon * 100);
      char alt[7];
      char acc[5];
      dtostrf (data.gpgga_data.xyz.alt, 3, 2, alt);
      dtostrf (data.gpgga_data.acc, 4, 1, acc);
      snprintf(msg, MSG_SZ, "UTC:\t\t\t[ %02ld:%02ld:%02ld ]\r\n",
               data.gpgga_data.utc.hh,
               data.gpgga_data.utc.mm,
               data.gpgga_data.utc.ss);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Latitude:\t\t[ %.02d' %.02d'' %c ]\r\n",
               lat,
               lat_mod,
               data.gpgga_data.xyz.ns);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Longitude:\t\t[ %.02d' %.02d'' %c ]\r\n",
               lon,
               lon_mod,
               data.gpgga_data.xyz.ew);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Satellites locked:\t[ %ld ]\r\n",
               data.gpgga_data.sats);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Position accuracy:\t[ %s ]\r\n",
               acc);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Altitude:\t\t[ %s%c ]\r\n",
               alt,
               (data.gpgga_data.xyz.mis + 32U));
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Geoid infos:\t\t[ %ld%c ]\r\n",
               data.gpgga_data.geoid.height,
               data.gpgga_data.geoid.mis);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Diff update:\t\t[ %ld ]\r\n",
               data.gpgga_data.update);
      Serial.print(msg);
   }
//   else
//   {
//      Serial.print("Last position wasn't valid.\r\n\n");
//   }
//   Serial.print("\r\n\n>");
}


void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
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
  case EV_JOIN_TXCOMPLETE:
    Serial.println(F("EV_JOIN_TXCOMPLETE"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    {
      u4_t netid = 0;
      devaddr_t devaddr = 0;
      u1_t nwkKey[16];
      u1_t artKey[16];
      LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      Serial.print("netid: ");
      Serial.println(netid, DEC);
      Serial.print("devaddr: ");
      Serial.println(devaddr, HEX);
      Serial.print("artKey: ");
      for (size_t i = 0; i < sizeof(artKey); ++i) {
        if (i != 0)
          Serial.print("-");
        Serial.print(artKey[i], HEX);
      }
      Serial.println("");
      Serial.print("nwkKey: ");
      for (size_t i = 0; i < sizeof(nwkKey); ++i) {
        if (i != 0)
          Serial.print("-");
        Serial.print(nwkKey[i], HEX);
      }
      Serial.println("");
    }
    // Disable link check validation (automatically enabled
    // during join, but because slow data rates change max TX
    // size, we don't use it in this example.
    LMIC_setLinkCheckMode(0);
    break;
  /*
  || This event is defined but not used in the code. No
  || point in wasting codespace on it.
  ||
  || case EV_RFU1:
  ||     Serial.println(F("EV_RFU1"));
  ||     break;
  */
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ack"));
    if (LMIC.dataLen) {
      Serial.println(F("Received "));
      Serial.println(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
    }
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL),
                        do_send);
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
  /*
  || This event is defined but not used in the code. No
  || point in wasting codespace on it.
  ||
  || case EV_SCAN_FOUND:
  ||    Serial.println(F("EV_SCAN_FOUND"));
  ||    break;
  */
  case EV_TXSTART:
    Serial.println(F("EV_TXSTART"));
    break;
  default:
    Serial.print(F("Unknown event: "));
    Serial.println((unsigned)ev);
    break;
  }
}


void do_send(osjob_t *j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {

    while (data.gpgga_data.valid != 1){
        gps->update();
        printValidInfo();
        //delay(500);

      }
    
    float lat = (float) (data.gpgga_data.xyz.lat/100.0);
    float lat_mod = (float) (data.gpgga_data.xyz.lat) - (lat * 100); 
    float lon = (float) (data.gpgga_data.xyz.lon/100.0);
    float lon_mod = (float) (data.gpgga_data.xyz.lon) - (lon * 100);
    float alt;
//    std::string ew = data.gpgga_data.xyz.ew;
//    std::string ns = data.gpgga_data.xyz.ns;
    Serial.println(data.gpgga_data.xyz.ew);
    Serial.println(data.gpgga_data.xyz.ns);

    if (data.gpgga_data.xyz.ns > 80) {
      lat = lat*(-1);
    }

    if (data.gpgga_data.xyz.ew > 80) {
      lon = lon*(-1);
    }

    lpp.reset();
    lpp.addGPS(1, lat, lon, alt);
    snprintf(msg, MSG_SZ, "Latitude: %.2d", lat);
    Serial.println(msg);
    snprintf(msg, MSG_SZ, "Longitude: %.2d", lon);
    Serial.println(msg);
    //LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}


namespace Arduino_LMIC {

class HalConfiguration_Disco_L072cz_Lrwan1_t : public HalConfiguration_t {
public:
  enum DIGITAL_PINS : uint8_t {
    PIN_SX1276_NSS = 37,
    PIN_SX1276_NRESET = 33,
    PIN_SX1276_DIO0 = 38,
    PIN_SX1276_DIO1 = 39,
    PIN_SX1276_DIO2 = 40,
    PIN_SX1276_RXTX = 21,
  };

  virtual bool queryUsingTcxo(void) override { return false; };
};
// save some typing by bringing the pin numbers into scope
static HalConfiguration_Disco_L072cz_Lrwan1_t myConfig;

static const HalPinmap_t myPinmap = {
    .nss = HalConfiguration_Disco_L072cz_Lrwan1_t::PIN_SX1276_NSS,
    .rxtx = HalConfiguration_Disco_L072cz_Lrwan1_t::PIN_SX1276_RXTX,
    .rst = HalConfiguration_Disco_L072cz_Lrwan1_t::PIN_SX1276_NRESET,

    .dio =
        {
            HalConfiguration_Disco_L072cz_Lrwan1_t::PIN_SX1276_DIO0,
            HalConfiguration_Disco_L072cz_Lrwan1_t::PIN_SX1276_DIO1,
            HalConfiguration_Disco_L072cz_Lrwan1_t::PIN_SX1276_DIO2,
        },
    .rxtx_rx_active = 1,
    .rssi_cal = 10,
    .spi_freq = 8000000, /* 8MHz */
    .pConfig = &myConfig};

}; // end namespace Arduino_LMIC
