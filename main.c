/*
 * Based on a blog post I can't find anymore
 * The bulk of the code was created by someone I cannot locate, and mostly the main function was rewritten
 
 * Based on V-USB drivers from Objective Developments - http://www.obdev.at/products/vusb/index.html
 * Based on Flip's blog 4-key-keyboard http://blog.flipwork.nl/?x=entry:entry100224-003937
 * Analog input functions: http://correll.cs.colorado.edu/?p=1801

Working fuse setting on ATTiny45/85:

EXTENDED: 0xFF
HIGH:    0xDD
LOW:     0xC1

*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define F_CPU 12000000L /* this should probably be at 16.5MHz instead of 12MHz, but it works just fine, so... I dunno */

#include <util/delay.h>
#include <stdlib.h>

#include "usbdrv.h"

#define SWITCH_BIT PB1 /* bit for ON/OFF switch input/output */

/* ------------------------------------------------------------------------- */

static uchar reportBuffer[8] = {0,0,0,0,0,0,0,0};    /* buffer for HID reports */

/* Reportbuffer format:

    0  Modifier byte
    1  reserved
    2  keycode array (0)
    3  keycode array (1)
    4  keycode array (2)
    5  keycode array (3)
    6  keycode array (4)
    7  keycode array (5)
    
    << This is the standard usb-keyboard reportbuffer. It allows for 6 simultaneous keypresses to be detected (excl. modifier keys). In this application we only use 1, so the last 5 bytes in this buffer will always remain 0. >>
    
   Modifier byte: 8 bits, each individual bit represents one of the modifier keys.

    bit0  LEFT CTRL     (1<<0)
    bit1  LEFT SHIFT    (1<<1)
    bit2  LEFT ALT      (1<<2)
    bit3  LEFT GUI      (1<<3)
    bit4  RIGHT CTRL    (1<<4)
    bit5  RIGHT SHIFT   (1<<5)
    bit6  RIGHT ALT     (1<<6)
    bit7  RIGHT GUI     (1<<7)

    an example of a reportBuffer for a CTRL+ALT+Delete keypress:

    {((1<<0)+(1<<2)),0,76,0,0,0,0,0}

    the first byte holds both the LEFT CTRL and LEFT  modifier keys the 3rd byte holds the delete key (== decimal 76)

*/

static uchar idleRate;            /* in 4 ms units */
static uchar status;              /* 0 = not sitting, 1 = sitting */
static double threshold = 1.5;    /* minimum voltage over FSR to trigger sitting down */
static uchar change = 0;
static unsigned int statusTimer;  /* minimum time to required to change status */

static double FSRVoltage = 0;     /* stores FSR voltage */

/* ------------------------------------------------------------------------- */
/* Standard USB HID report descriptor */

PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,        // USAGE (Keyboard)
    0xa1, 0x01,        // COLLECTION (Application)
    0x05, 0x07,        //       USAGE_PAGE (Keyboard)
    0x19, 0xe0,        //       USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,        //       USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,        //       LOGICAL_MINIMUM (0)
    0x25, 0x01,        //       LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //       REPORT_SIZE (1)
    0x95, 0x08,        //       REPORT_COUNT (8)
    0x81, 0x02,        //   INPUT (Data,Var,Abs) ** Modifier Byte **
    0x95, 0x01,        //       REPORT_COUNT (1)
    0x75, 0x08,        //       REPORT_SIZE (8)
    0x81, 0x03,        //   INPUT (Cnst,Var,Abs) ** Reserved Byte **
    0x95, 0x05,        //       REPORT_COUNT (5)
    0x75, 0x01,        //       REPORT_SIZE (1)
    0x05, 0x08,        //       USAGE_PAGE (LEDs)
    0x19, 0x01,        //       USAGE_MINIMUM (Num Lock)
    0x29, 0x05,        //       USAGE_MAXIMUM (Kana)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs) ** LED Report **
    0x95, 0x01,        //       REPORT_COUNT (1)
    0x75, 0x03,        //       REPORT_SIZE (3)
    0x91, 0x03,        //   OUTPUT (Cnst,Var,Abs) ** LED Report Padding **
    0x95, 0x06,        //       REPORT_COUNT (6) ** here we define the maximum number of simultaneous keystrokes we can detect ** 
    0x75, 0x08,        //       REPORT_SIZE (8)
    0x15, 0x00,        //       LOGICAL_MINIMUM (0)
    0x25, 0x65,        //       LOGICAL_MAXIMUM (101)
    0x05, 0x07,        //       USAGE_PAGE (Keyboard)
    0x19, 0x00,        //       USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,        //       USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,        //   INPUT (Data,Ary,Abs) ** Key arrays (6 bytes) **
    0xc0               // END_COLLECTION
};

/* ADC functionality */

void initADC() {
    // Configure ADMUX register
    ADMUX =
        (1 << ADLAR)| // shift in a 1 and follow 8bit procedure
        (1 << MUX1)|  // use ADC2 or PB4 pin for Vin
        (0 << REFS0)| // set refs0 and 1 to 0 to use Vcc as Vref
        (0 << REFS1);
    // Configure ADCSRA register
    ADCSRA =
        (1 << ADEN)| // set ADEN bit to 1 to enable the ADC
        (0 << ADSC); // set ADSC to 0 to make sure no conversions are happening
}

// function that returns an analog reading
double getAIN() {
    // make sure we define result as a double so we can mult/divide w/o error
    double result = 0;
    // set ADSC pin to 1 in order to start reading the AIN value
    ADCSRA |= (1 << ADSC);
    // do nothing until the ADSC pin returns back to 0;
    while (((ADCSRA >> ADSC) & 1)) {}
    // for 8 bit precision we can just read ADCH:
    result = ADCH;
    // to properly interpret the results as a voltage we need to
    // divide the result by the maximum range: 2^(#bits precision)-1
    // and multiply the result by the Vref value, Vcc or 5V
    result = result * 5/255; // for 8bit, below for 10bit resolution
    return result;
}

/* ------------------------------------------------------------------------- */

static void timerPoll(void) {
    static unsigned int timerCnt;

    if(TIFR & (1 << TOV1)) {
        TIFR = (1 << TOV1); /* clear overflow */
        if(++timerCnt >= 5) { // 5/63 sec delay for switch debouncing - may not be accurate
            timerCnt = 0;
        }
    }
}

static void buildReport(int notEmpty) {
    if (!notEmpty) // if empty, build an empty report
        reportBuffer[2] = 0;
    else
        reportBuffer[2] = 0x2C; // spacebar
}

static void checkFSR(void) {
    FSRVoltage = getAIN();
    if (FSRVoltage > threshold) { //if status has changed
        if (status == 1 && change == 1) {
            change = 0;
        } else if (status == 0) {
            change = 1;
        }
    } else { // not sitting
        if (status == 0 && change == 1) {
            change = 0;
        } else if (status == 1) {
            change = 1;
        }
    }
}

static void resetStatusTimer(void) {
    // for some reason the timer reaches zero twice as fast when status = 1
    if (status)
        statusTimer = 62000;
    else
        statusTimer = 31000;
}

/* ------------------------------------------------------------------------- */

static void timerInit(void) {
    TCCR1 = 0x0b; /* select clock: 16.5M/1k -> overflow rate = 16.5M/256k = 62.94 Hz  - this may not be accurate */
}

/* -------------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* -------------------------------------------------------------------------------- */

uchar   usbFunctionSetup(uchar data[8]) {
    usbRequest_t *rq = (void *)data;
    usbMsgPtr = reportBuffer;
    if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) { /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){ /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            buildReport(0);
            return sizeof(reportBuffer);
        } else if (rq->bRequest == USBRQ_HID_GET_IDLE) {
            usbMsgPtr = &idleRate;
            return 1;
        } else if (rq->bRequest == USBRQ_HID_SET_IDLE) {
            idleRate = rq->wValue.bytes[1];
        }
    } else {
        /* no vendor specific requests implemented */
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void) {
    uchar step = 128;
    uchar trialValue = 0, optimumValue;
    int x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do {
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength(); /* proportional to current real frequency */
        if(x < targetValue) /* frequency still too low */
            trialValue += step;
        step >>= 1;
    } while (step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for (OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++) {
        x = usbMeasureFrameLength() - targetValue;
        if (x < 0)
            x = -x;
        if (x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void hadUsbReset(void) {
    calibrateOscillator();
    eeprom_write_byte(0, OSCCAL); /* store the calibrated value in EEPROM byte 0*/
}

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int main(void) {
    uchar i;
    uchar calibrationValue;

    do {} while (!eeprom_is_ready());
    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if (calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }
    
    usbInit();
    usbDeviceDisconnect(); /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while (--i) { /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();

    wdt_enable(WDTO_2S);

    /* turn on internal pull-up resistor for the switch */
    PORTB |= _BV(SWITCH_BIT);

    initADC();

    FSRVoltage = getAIN();
    if (FSRVoltage > threshold)
        status = 1; // sitting
    else
        status = 0; // not sitting

    resetStatusTimer();
    timerInit();
    sei();

    for(;;) { /* main event loop */
        wdt_reset();
        usbPoll();
        if (change) // if a change was detected, decrease timer
            statusTimer -= 5;
        else
            resetStatusTimer();

        checkFSR();
        if (statusTimer <= 0) { // if timer reached zero, toggle status and send keys
            status ^= 1;
            change = 0;
            resetStatusTimer();
            if (bit_is_set(PINB, SWITCH_BIT)) { // if device is switched on
                if (usbInterruptIsReady()) { /* we can send another report */
                    buildReport(1);
                    usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
                    while (!usbInterruptIsReady())
                        timerPoll();
                    buildReport(0);
                    usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
                }
            }
        }
        timerPoll();
    }
    return 0;
}