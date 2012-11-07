#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/power.h>
#include <Arduino.h>
#include <Wire.h>
#include <DS3231.h>
#include <DHT22.h>
#include <crc16.h>

#define PIN_DHT22           8

#define PIN_BATT_VOLTAGE    7
#define PIN_BATT_CHARGE     6
#define PIN_ONBOARD_LED     13
#define PIN_BEE_POWER       5
#define PIN_TF_POWER        4

#define SLEEP_SEC           300


DS3231 RTC;
DHT22 myDHT22(PIN_DHT22);

//static uint8_t prevSecond=0;
static DateTime interruptTime;

//Interrupt service routine for external interrupt on INT0 pin conntected to DS3231 /INT
void INT0_ISR()
{
    //Keep this as short as possible. Possibly avoid using function calls
    detachInterrupt(0);
    interruptTime = DateTime(interruptTime.get() + SLEEP_SEC);  //decide the time for next interrupt, configure next interrupt
}


void setup()
{
    for (byte i=0; i<20; i++) {
        pinMode(i, INPUT);        //make all pins input pins
        digitalWrite(i, HIGH);    //with pullup resistors to minimize power consumption
    }

    // specific pins
    pinMode(PIN_BATT_CHARGE, INPUT);
    pinMode(PIN_BATT_VOLTAGE, INPUT);
    pinMode(PIN_BEE_POWER, OUTPUT);
    pinMode(PIN_TF_POWER, OUTPUT);
    digitalWrite(PIN_TF_POWER, LOW); // turn off SD card controller

    pinMode(PIN_ONBOARD_LED, OUTPUT);
    digitalWrite(PIN_ONBOARD_LED, LOW);

    Serial.begin(9600);
    Wire.begin();

    RTC.begin();
    attachInterrupt(0, INT0_ISR, LOW); //Only LOW level interrupt can wake up from PWR_DOWN
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    //Enable Interrupt
    //RTC.enableInterrupts(EveryMinute); //interrupt at  EverySecond, EveryMinute, EveryHour
    // or this
    DateTime  start = RTC.now();
    interruptTime = DateTime(start.get() + SLEEP_SEC); //Add 5 mins in seconds to start time

    analogReference(INTERNAL);
    analogRead(PIN_BATT_CHARGE);
 
    //digitalWrite(PIN_ONBOARD_LED, LOW);
}


String getTimestamp()
{
    return String(RTC.now().get());
}

String getTemperature()
{
    DHT22_ERROR_t errorCode = myDHT22.readData();
    String result;
    switch (errorCode) {
    case DHT_ERROR_NONE:
        result = String(myDHT22.getTemperatureCInt()) + String(":") + String(myDHT22.getHumidityInt());
        break;
    default:
        result = String(errorCode);
        break;
    }
    return result;
}

String getChargeStatus()
{
    unsigned char status = 0;
    unsigned int ADC6 = analogRead(6);
    if (ADC6 > 900) {
        status = 0;     //sleeping
    } else if (ADC6 > 550) {
        status = 1;     //charging
    } else if (ADC6 > 350) {
        status = 2;     //done
    } else {
        status = 3;     //error
    }
    return String(status);
}

String getVoltage()
{
    unsigned int bat_read = analogRead(PIN_BATT_VOLTAGE);
    //  (1/1024)*6=0.0064453125,
    float voltage = (float)bat_read * 0.0064453125;
    return String((int)(voltage * 100));
}

String getInternalTemperature()
{
    RTC.convertTemperature();
    return String((int)(RTC.getTemperature() * 100));
}

void loop()
{
    ////////////////////// START : Application or data logging code//////////////////////////////////
    digitalWrite(PIN_ONBOARD_LED, HIGH);
    delay(2000); // required for DHT22

    String out =
            getTimestamp() +
            String("|") +
            getTemperature() +
            String("|") +
            getChargeStatus()+
            String("|") +
            getVoltage() +
            String("|") +
            getInternalTemperature();

    digitalWrite(PIN_BEE_POWER, HIGH);
    delay(150);
    Serial.print("~");
    Serial.print(out);
    Serial.print("^");
    char buff[128] = "";
    out.toCharArray(buff, 128);
    Serial.print(calc_crc16(buff, out.length()), HEX);
    Serial.print("=");

    delay(150);
    digitalWrite(PIN_BEE_POWER, LOW);

    digitalWrite(PIN_ONBOARD_LED, LOW);
    ////////////////////////END : Application code ////////////////////////////////

    RTC.clearINTStatus(); //This function call is  a must to bring /INT pin HIGH after an interrupt.
    RTC.enableInterrupts(interruptTime.hour(), interruptTime.minute(), interruptTime.second());    // set the interrupt at (h,m,s)
    attachInterrupt(0, INT0_ISR, LOW);  //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_enable();      // Set sleep enable bit
    
    sleep_bod_disable(); // Disable brown out detection during sleep. Saves more power
    sei();               // Set interrupts   
    power_all_disable(); // This shuts down ADC, TWI, SPI, Timers and USART
    sleep_cpu();         // Sleep the CPU as per the mode set earlier(power down)

    //--------------------

    sleep_disable();     // Wakes up sleep and clears enable bit. Before this ISR would have executed
    power_all_enable();  // This shuts enables ADC, TWI, SPI, Timers and USART
    delay(15);           // This delay is required to allow CPU to stabilize
}
