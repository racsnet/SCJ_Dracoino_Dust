#include <Arduino.h>
#include <SoftwareSerial.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>
#include <DHTStable.h>

#include "config.h"
#include "devices.h"
#include "scj_reg.h"

bool trx_running = false;

SoftwareSerial Sensor(SSD011_RX, SSD011_TX); // RX, TX
DHTStable DHT;
uint8_t mydata[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

union FBConv {
  float f_value;
  uint8_t b_value[4];
};

FBConv temperature, humidity;

static osjob_t sendjob;

uint16_t TX_SENSE_TICKS = 1;
unsigned long LAST_SENSE_TX = 0;

uint16_t TX_BAT_TICKS = 40;
unsigned long LAST_BAT_TX = 0;

unsigned long ticks = 0;

void toggle_indicator_led() {
    if (digitalRead(INDICATE_LED)) {
        digitalWrite(INDICATE_LED, LOW);
    }
    else {
        digitalWrite(INDICATE_LED, HIGH);
    }
}

uint8_t get_bat() {
    return constrain(map(analogRead(A0), BAT_MIN, BAT_MAX, 1, 254), 1, 254);
}

int GetDHT22() {
    int result = 0;
    digitalWrite(DHT22_EN, HIGH);
    delay(1000);
    
    int chk = DHT.read22(DHT22_DAT);
    switch (chk)
    {
    case DHTLIB_OK:
        temperature.f_value = DHT.getTemperature();
        humidity.f_value = DHT.getHumidity();
        break;
    case DHTLIB_ERROR_CHECKSUM:
        result = DHT_ERR_CHECKSUM;
        break;
    case DHTLIB_ERROR_TIMEOUT:
        result = DHT_ERR_TIMEOUT;
        break;
    case DHTLIB_INVALID_VALUE:
        result = DHT_ERR_INVALID;
        break;
    default:
        result = DHT_ERR_UNKNOWN;
        break;
    }

    digitalWrite(DHT22_EN, LOW);
    return result;
}

bool GetSensor() {
    int counter = 0;
    byte buff[255];
    byte id1, id2, checksum;
    bool valid = false;
    digitalWrite(SDS011_EN, HIGH);
    for (int i = 0; i < 10; i++) {
        delay(1000);
        while (Sensor.available()) {
            Sensor.read();
        }
    }
    unsigned long started = millis();
    while (millis() - started < 5000) {
        if (Sensor.available()) {
            buff[counter] = Sensor.read();
            counter++;
        }
        if (counter >= 11) {
            for (int i = 0; i < counter; i++) {
                if ((buff[i] == 0xAA) && (buff[i+1] == 0xC0)) {
                    mydata[0] = buff[i+2];
                    mydata[1] = buff[i+3];
                    mydata[2] = buff[i+4];
                    mydata[3] = buff[i+5];
                    id1 = buff[i+6];
                    id2 = buff[i+7];
                    checksum = buff[i+8];
                    started = 0;
                    valid = true;
                }
            }
        }
    }

    if (checksum != (mydata[0] + mydata[1] + mydata[2] + mydata[3] + id1 + id2) % 256) {
        valid = false;
    }

    digitalWrite(SDS011_EN, LOW);
    while (Sensor.available()) {
        Sensor.read();
    }
    return valid;
}

const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {8, 9, LMIC_UNUSED_PIN},
};

void LMIC_setClockError(
    u2_t error
);

void do_send(osjob_t* j, uint8_t port, uint8_t errcode = 0x00){
    if ( !(LMIC.opmode & OP_TXRXPEND) ) {
        trx_running = true;
        if (port == SCJ_PORT_BAT) {
            uint8_t bat_val[] = {get_bat()};
            LMIC_setTxData2(SCJ_PORT_BAT, bat_val, sizeof(bat_val), 0);
        }
        else if(port == SCJ_PORT_ERR) {
            uint8_t error[] = {errcode};
            LMIC_setTxData2(SCJ_PORT_ERR, error, sizeof(error), 0);
        }
        else {
            LMIC_setTxData2(MY_PORT, mydata, sizeof(mydata), 0);
        }
    }
}

void onEvent (ev_t ev) {
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            break;
        case EV_BEACON_FOUND:
            break;
        case EV_BEACON_MISSED:
            break;
        case EV_BEACON_TRACKED:
            break;
        case EV_JOINING:
            break;
        case EV_JOINED:
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
                digitalWrite(INDICATE_LED, HIGH);
                delay(10);
                digitalWrite(INDICATE_LED, LOW);
            }
            break;
        case EV_JOIN_FAILED:

            break;
        case EV_REJOIN_FAILED:

            break;
        case EV_TXCOMPLETE:

            if (LMIC.dataLen) {
                byte payload[LMIC.dataLen];
                for (int i = 0; i < LMIC.dataLen; i++) {
                    payload[i] = LMIC.frame[LMIC.dataBeg + i];
                }
                if (payload[0] == DOWNLINK_TGL_LED) {
                    toggle_indicator_led();
                }
                else if (payload[0] == DOWNLINK_CFG_BAT_TICKS) {
                    if (sizeof(payload) == 2) {
                        TX_BAT_TICKS = (uint16_t)payload[1];
                    }
                    else if (sizeof(payload) == 3) {
                        TX_BAT_TICKS = payload[2] << 8 | payload[1];
                    }
                    else {
                        do_send(&sendjob, SCJ_PORT_ERR, SCJ_ERR_INVALID_BAT_TICKS);   
                    }

                }
                else if (payload[0] == DOWNLINK_CFG_SENSE_TICKS) {
                    if (sizeof(payload) == 2) {
                        TX_SENSE_TICKS = payload[1] << 0x00;
                    }
                    else if (sizeof(payload) == 3) {
                        TX_SENSE_TICKS = payload[1] << 8 | payload[2];
                    }
                    else {
                        do_send(&sendjob, SCJ_PORT_ERR, SCJ_ERR_INVALID_SENSE_TICKS);   
                    }
                }
                delay(100);
            }
            trx_running = false;
            break;
        case EV_LOST_TSYNC:
            break;
        case EV_RESET:
            break;
        case EV_RXCOMPLETE:
            break;
        case EV_LINK_DEAD:
            break;
        case EV_LINK_ALIVE:
            break;
        case EV_TXSTART:
            break;
        case EV_TXCANCELED:
            break;
        case EV_RXSTART:
            break;
        case EV_JOIN_TXCOMPLETE:
            break;

        default:
            break;
    }
}

void setup() {
//  initialize all unused pins as output for current saving

    #ifdef PINMODEPOWERSAVE
    pinMode(0,OUTPUT);
    pinMode(1,OUTPUT);
    pinMode(2,OUTPUT);
    pinMode(5,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
    pinMode(11,OUTPUT);
    pinMode(12,OUTPUT);
    pinMode(13,OUTPUT);
    pinMode(A1,OUTPUT);
    pinMode(A2,OUTPUT);
    pinMode(A3,OUTPUT);
    pinMode(A5,OUTPUT);
    #endif

    Sensor.begin(9600);

//  disable Brown Out Detection
    #ifdef DISABLEBOD
    MCUCR = bit (BODS) | bit (BODSE);
    MCUCR = bit (BODS); 
    #endif

    os_init();
    pinMode(INDICATE_LED, OUTPUT);
    digitalWrite(INDICATE_LED, LOW);
    for (int i = 0; i < 6; i++) {
        toggle_indicator_led();
        delay(200);
    }
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * LORA_CLK_ERR / 100);
    #ifdef LINKCHECKMODE
    LMIC_setLinkCheckMode(true);
    #endif
    #ifdef ADRMODE
    LMIC_setAdrMode(true);
    #endif
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob, SCJ_PORT_BAT);
}

void loop() {

    if ( (ticks - LAST_BAT_TX >= TX_BAT_TICKS) && !trx_running ) {
        do_send(&sendjob, SCJ_PORT_BAT);
        LAST_BAT_TX = ticks;
    }
    else if ( (ticks - LAST_SENSE_TX > TX_SENSE_TICKS) && !trx_running ) {
        if (GetSensor()) {
            int dht_result = GetDHT22();
            if (dht_result > 0) {
                do_send(&sendjob, SCJ_PORT_ERR, dht_result);
            }
            else {
                mydata[4] = temperature.b_value[0];
                mydata[5] = temperature.b_value[1];
                mydata[6] = temperature.b_value[2];
                mydata[7] = temperature.b_value[3];
                mydata[8] = humidity.b_value[0];
                mydata[9] = humidity.b_value[1];
                mydata[10] = humidity.b_value[2];
                mydata[11] = humidity.b_value[3];
                do_send(&sendjob, MY_PORT);
            }
        }
        else {
            do_send(&sendjob, SCJ_PORT_ERR);
        }
        LAST_SENSE_TX = ticks;
    }

    os_runloop_once();

    if (!trx_running) {
        delay(500);
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);       
        if (ticks == 4294967294) {
            ticks = 0;
            LAST_SENSE_TX = 0;
            LAST_BAT_TX = 0;
        }
        else {
            ticks++;
        }
    }
}