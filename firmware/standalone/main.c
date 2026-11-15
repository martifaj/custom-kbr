/*
 * Standalone USB HID Keyboard + Media Keys firmware for custom-kbr (ATmega32U4 @ 16 MHz)
 * Matches firmware/qmk/keyboards/custom_kbr/keyboard.json & keymaps/default/keymap.json
 */
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* --- HID Keycodes --- */
#define KC_NO   0x00
#define KC_TRNS 0xFF

#define KC_A    0x04
#define KC_B    0x05
#define KC_C    0x06
#define KC_D    0x07
#define KC_E    0x08
#define KC_F    0x09
#define KC_G    0x0A
#define KC_H    0x0B
#define KC_I    0x0C
#define KC_J    0x0D
#define KC_K    0x0E
#define KC_L    0x0F
#define KC_M    0x10
#define KC_N    0x11
#define KC_O    0x12
#define KC_P    0x13
#define KC_Q    0x14
#define KC_R    0x15
#define KC_S    0x16
#define KC_T    0x17
#define KC_U    0x18
#define KC_V    0x19
#define KC_W    0x1A
#define KC_X    0x1B
#define KC_Y    0x1C
#define KC_Z    0x1D

#define KC_1    0x1E
#define KC_2    0x1F
#define KC_3    0x20
#define KC_4    0x21
#define KC_5    0x22
#define KC_6    0x23
#define KC_7    0x24
#define KC_8    0x25
#define KC_9    0x26
#define KC_0    0x27

#define KC_ENT  0x28
#define KC_ESC  0x29
#define KC_BSPC 0x2A
#define KC_TAB  0x2B
#define KC_SPC  0x2C
#define KC_MINS 0x2D
#define KC_EQL  0x2E
#define KC_LBRC 0x2F
#define KC_RBRC 0x30
#define KC_BSLS 0x31
#define KC_SCLN 0x33
#define KC_QUOT 0x34
#define KC_GRV  0x35
#define KC_COMM 0x36
#define KC_DOT  0x37
#define KC_SLSH 0x38
#define KC_CAPS 0x39

#define KC_F1   0x3A
#define KC_F2   0x3B
#define KC_F3   0x3C
#define KC_F4   0x3D
#define KC_F5   0x3E
#define KC_F6   0x3F
#define KC_F7   0x40
#define KC_F8   0x41
#define KC_F9   0x42
#define KC_F10  0x43
#define KC_F11  0x44
#define KC_F12  0x45

#define KC_PSCR 0x46
#define KC_INS  0x49
#define KC_HOME 0x4A
#define KC_PGUP 0x4B
#define KC_DEL  0x4C
#define KC_END  0x4D
#define KC_PGDN 0x4E
#define KC_RGHT 0x4F
#define KC_LEFT 0x50
#define KC_DOWN 0x51
#define KC_UP   0x52

/* Modifiers */
#define KC_LCTL 0xE0
#define KC_LSFT 0xE1
#define KC_LALT 0xE2
#define KC_LGUI 0xE3
#define KC_RCTL 0xE4
#define KC_RSFT 0xE5
#define KC_RALT 0xE6
#define KC_RGUI 0xE7

/* Special / Custom Keycodes */
#define QK_GESC 0xF0
#define MO_1    0xF1
#define QK_BOOT 0xF2
#define KC_MUTE 0xF3
#define KC_VOLD 0xF4
#define KC_VOLU 0xF5
#define KC_MPLY 0xF6

#define N_ROWS 5
#define N_COLS 14

/* Keymap: 2 layers x 5 rows x 14 cols */
static const uint8_t PROGMEM keymaps[2][N_ROWS][N_COLS] = {
    /* Layer 0: Base 60% ANSI */
    {
        { QK_GESC, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC },
        { KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC, KC_BSLS },
        { KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT, KC_NO,   KC_ENT  },
        { KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_NO,   KC_NO,   KC_RSFT },
        { KC_LCTL, KC_LGUI, KC_LALT, KC_NO,   KC_NO,   KC_NO,   KC_SPC,  KC_NO,   KC_NO,   KC_NO,   KC_RALT, KC_RGUI, MO_1,    KC_RCTL }
    },
    /* Layer 1: Fn layer */
    {
        { KC_GRV,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  KC_DEL  },
        { KC_TRNS, KC_HOME, KC_UP,   KC_END,  KC_PGUP, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_PSCR, KC_INS,  QK_BOOT },
        { KC_TRNS, KC_LEFT, KC_DOWN, KC_RGHT, KC_PGDN, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_NO,   KC_TRNS },
        { KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_MUTE, KC_VOLD, KC_VOLU, KC_TRNS, KC_NO,   KC_NO,   KC_TRNS },
        { KC_TRNS, KC_TRNS, KC_TRNS, KC_NO,   KC_NO,   KC_NO,   KC_MPLY, KC_NO,   KC_NO,   KC_NO,   KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS }
    }
};

/* Row pins on PORTF: F6, F5, F4, F1, F0 */
static const uint8_t row_masks[N_ROWS] = {
    (1 << 6), (1 << 5), (1 << 4), (1 << 1), (1 << 0)
};

static void matrix_init(void) {
    /* Disable JTAG so PF4, PF5, PF6, PF7 work as digital GPIO */
    uint8_t sreg = SREG;
    cli();
    MCUCR |= (1 << JTD);
    MCUCR |= (1 << JTD);
    SREG = sreg;

    /* Rows: PF6, PF5, PF4, PF1, PF0 as Hi-Z inputs with pull-ups */
    uint8_t all_rows = (1 << 6) | (1 << 5) | (1 << 4) | (1 << 1) | (1 << 0);
    DDRF  &= ~all_rows;
    PORTF |=  all_rows;

    /* Cols:
     * D7, B4, B5, B6, C6, C7, F7, E6, B1, B2, B3, B7, D1, D2
     * Set all as inputs with internal pull-ups enabled.
     */
    DDRB  &= ~((1 << 4) | (1 << 5) | (1 << 6) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 7));
    PORTB |=  ((1 << 4) | (1 << 5) | (1 << 6) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 7));

    DDRC  &= ~((1 << 6) | (1 << 7));
    PORTC |=  ((1 << 6) | (1 << 7));

    DDRD  &= ~((1 << 7) | (1 << 1) | (1 << 2));
    PORTD |=  ((1 << 7) | (1 << 1) | (1 << 2));

    DDRE  &= ~(1 << 6);
    PORTE |=  (1 << 6);

    DDRF  &= ~(1 << 7);
    PORTF |=  (1 << 7);
}

static uint16_t read_cols(void) {
    uint8_t pinb = PINB;
    uint8_t pinc = PINC;
    uint8_t pind = PIND;
    uint8_t pine = PINE;
    uint8_t pinf = PINF;

    uint16_t cols = 0;
    if (!(pind & (1 << 7))) cols |= (1U << 0);
    if (!(pinb & (1 << 4))) cols |= (1U << 1);
    if (!(pinb & (1 << 5))) cols |= (1U << 2);
    if (!(pinb & (1 << 6))) cols |= (1U << 3);
    if (!(pinc & (1 << 6))) cols |= (1U << 4);
    if (!(pinc & (1 << 7))) cols |= (1U << 5);
    if (!(pinf & (1 << 7))) cols |= (1U << 6);
    if (!(pine & (1 << 6))) cols |= (1U << 7);
    if (!(pinb & (1 << 1))) cols |= (1U << 8);
    if (!(pinb & (1 << 2))) cols |= (1U << 9);
    if (!(pinb & (1 << 3))) cols |= (1U << 10);
    if (!(pinb & (1 << 7))) cols |= (1U << 11);
    if (!(pind & (1 << 1))) cols |= (1U << 12);
    if (!(pind & (1 << 2))) cols |= (1U << 13);
    return cols;
}

static void select_row(uint8_t r) {
    uint8_t m = row_masks[r];
    DDRF  |=  m;
    PORTF &= ~m;
}

static void unselect_row(uint8_t r) {
    uint8_t m = row_masks[r];
    DDRF  &= ~m;
    PORTF |=  m;
}

static void jump_to_bootloader(void) {
    UDCON = 1;
    USBCON = (1 << FRZCLK);
    UCSR1B = 0;
    _delay_ms(50);
    cli();
    EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
    TIMSK0 = 0; TIMSK1 = 0; TIMSK3 = 0; TIMSK4 = 0; TWCR = 0;
    DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
    PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
    /* Jump to Atmel DFU bootloader at byte address 0x7000 (word address 0x3800) */
    ((void (*)(void))0x3800)();
    for (;;) {}
}

/* --- USB Descriptors --- */
#define EP0_SIZE 64
#define KBD_EP   1
#define MEDIA_EP 2

static const uint8_t PROGMEM device_desc[] = {
    18,                 /* bLength */
    0x01,               /* bDescriptorType (Device) */
    0x00, 0x02,         /* bcdUSB 2.00 */
    0x00,               /* bDeviceClass (per interface) */
    0x00,               /* bDeviceSubClass */
    0x00,               /* bDeviceProtocol */
    EP0_SIZE,           /* bMaxPacketSize0 */
    0xED, 0xFE,         /* idVendor 0xFEED */
    0x65, 0x65,         /* idProduct 0x6565 */
    0x00, 0x01,         /* bcdDevice 1.00 */
    1,                  /* iManufacturer */
    2,                  /* iProduct */
    0,                  /* iSerialNumber */
    1                   /* bNumConfigurations */
};

static const uint8_t PROGMEM kbd_hid_report_desc[] = {
    0x05, 0x01,         /* Usage Page (Generic Desktop) */
    0x09, 0x06,         /* Usage (Keyboard) */
    0xA1, 0x01,         /* Collection (Application) */
    0x05, 0x07,         /*   Usage Page (Key Codes) */
    0x19, 0xE0,         /*   Usage Minimum (224) */
    0x29, 0xE7,         /*   Usage Maximum (231) */
    0x15, 0x00,         /*   Logical Minimum (0) */
    0x25, 0x01,         /*   Logical Maximum (1) */
    0x75, 0x01,         /*   Report Size (1) */
    0x95, 0x08,         /*   Report Count (8) */
    0x81, 0x02,         /*   Input (Data, Variable, Absolute) - Modifier byte */
    0x95, 0x01,         /*   Report Count (1) */
    0x75, 0x08,         /*   Report Size (8) */
    0x81, 0x01,         /*   Input (Constant) - Reserved byte */
    0x95, 0x05,         /*   Report Count (5) */
    0x75, 0x01,         /*   Report Size (1) */
    0x05, 0x08,         /*   Usage Page (LEDs) */
    0x19, 0x01,         /*   Usage Minimum (1) */
    0x29, 0x05,         /*   Usage Maximum (5) */
    0x91, 0x02,         /*   Output (Data, Variable, Absolute) - LED report */
    0x95, 0x01,         /*   Report Count (1) */
    0x75, 0x03,         /*   Report Size (3) */
    0x91, 0x01,         /*   Output (Constant) - LED padding */
    0x95, 0x06,         /*   Report Count (6) */
    0x75, 0x08,         /*   Report Size (8) */
    0x15, 0x00,         /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00,   /*   Logical Maximum (255) */
    0x05, 0x07,         /*   Usage Page (Key Codes) */
    0x19, 0x00,         /*   Usage Minimum (0) */
    0x29, 0xFF,         /*   Usage Maximum (255) */
    0x81, 0x00,         /*   Input (Data, Array) - 6 keycodes */
    0xC0                /* End Collection */
};

static const uint8_t PROGMEM media_hid_report_desc[] = {
    0x05, 0x0C,         /* Usage Page (Consumer) */
    0x09, 0x01,         /* Usage (Consumer Control) */
    0xA1, 0x01,         /* Collection (Application) */
    0x15, 0x00,         /*   Logical Minimum (0) */
    0x26, 0xFF, 0x03,   /*   Logical Maximum (0x03FF) */
    0x19, 0x00,         /*   Usage Minimum (0) */
    0x2A, 0xFF, 0x03,   /*   Usage Maximum (0x03FF) */
    0x75, 0x10,         /*   Report Size (16) */
    0x95, 0x01,         /*   Report Count (1) */
    0x81, 0x00,         /*   Input (Data, Array, Absolute) */
    0xC0                /* End Collection */
};

#define CONFIG_DESC_SIZE (9 + 9 + 9 + 7 + 9 + 9 + 7)
static const uint8_t PROGMEM config_desc[CONFIG_DESC_SIZE] = {
    /* Configuration Descriptor */
    9, 0x02, CONFIG_DESC_SIZE, 0x00, 2, 1, 0, 0xA0, 250,

    /* Interface 0: Boot Keyboard */
    9, 0x04, 0, 0, 1, 0x03, 0x01, 0x01, 0,
    /* HID Descriptor (Interface 0) */
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, sizeof(kbd_hid_report_desc), 0x00,
    /* Endpoint 1 IN Descriptor */
    7, 0x05, 0x80 | KBD_EP, 0x03, 8, 0x00, 1,

    /* Interface 1: Consumer Control (Media Keys) */
    9, 0x04, 1, 0, 1, 0x03, 0x00, 0x00, 0,
    /* HID Descriptor (Interface 1) */
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, sizeof(media_hid_report_desc), 0x00,
    /* Endpoint 2 IN Descriptor */
    7, 0x05, 0x80 | MEDIA_EP, 0x03, 8, 0x00, 1
};

static const uint8_t PROGMEM str_lang[] = { 4, 0x03, 0x09, 0x04 };
static const uint8_t PROGMEM str_mfg[]  = {
    22, 0x03,
    'c',0,'u',0,'s',0,'t',0,'o',0,'m',0,'-',0,'k',0,'b',0,'r',0
};

static volatile uint8_t usb_config = 0;
static volatile uint8_t kbd_protocol = 1;
static volatile uint8_t kbd_idle = 0;
static uint8_t kbd_report[8] = {0};
static uint16_t media_report = 0;

static void usb_configure_endpoints(void) {
    UENUM = KBD_EP;
    UECONX = (1 << EPEN);
    UECFG0X = (1 << EPTYPE1) | (1 << EPTYPE0) | (1 << EPDIR);
    UECFG1X = (1 << ALLOC);

    UENUM = MEDIA_EP;
    UECONX = (1 << EPEN);
    UECFG0X = (1 << EPTYPE1) | (1 << EPTYPE0) | (1 << EPDIR);
    UECFG1X = (1 << ALLOC);
}

static void usb_init(void) {
    UHWCON = (1 << UVREGE);
    USBCON = (1 << USBE) | (1 << FRZCLK);
    PLLCSR = (1 << PINDIV) | (1 << PLLE);
    while (!(PLLCSR & (1 << PLOCK))) {}
    USBCON = (1 << USBE) | (1 << OTGPADE);
    UDCON = 0;
    usb_config = 0;
    UDIEN = (1 << EORSTE);
    sei();
}

static void usb_send_pgm(const uint8_t *data, uint16_t len, uint16_t wLength) {
    if (len > wLength) len = wLength;
    while (len > 0) {
        while (!(UEINTX & ((1 << TXINI) | (1 << RXOUTI) | (1 << RXSTPI)))) {}
        if (UEINTX & ((1 << RXOUTI) | (1 << RXSTPI))) return;
        uint8_t n = len > EP0_SIZE ? EP0_SIZE : len;
        for (uint8_t i = 0; i < n; i++) {
            UEDATX = pgm_read_byte(data++);
        }
        len -= n;
        UEINTX = ~(1 << TXINI);
    }
    while (!(UEINTX & ((1 << RXOUTI) | (1 << RXSTPI)))) {}
    if (UEINTX & (1 << RXOUTI)) {
        UEINTX = ~(1 << RXOUTI);
    }
}

ISR(USB_GEN_vect) {
    uint8_t intf = UDINT;
    UDINT = 0;
    if (intf & (1 << EORSTI)) {
        UENUM = 0;
        UECONX = (1 << EPEN);
        UECFG0X = 0;
        UECFG1X = (1 << EPSIZE1) | (1 << EPSIZE0) | (1 << ALLOC);
        UEIENX = (1 << RXSTPE);
        usb_config = 0;
    }
}

ISR(USB_COM_vect) {
    UENUM = 0;
    if (!(UEINTX & (1 << RXSTPI))) return;

    uint8_t bmRequestType = UEDATX;
    uint8_t bRequest      = UEDATX;
    uint16_t wValue       = UEDATX | ((uint16_t)UEDATX << 8);
    uint16_t wIndex       = UEDATX | ((uint16_t)UEDATX << 8);
    uint16_t wLength      = UEDATX | ((uint16_t)UEDATX << 8);
    UEINTX = ~((1 << RXSTPI) | (1 << RXOUTI) | (1 << TXINI));

    if (bRequest == 0x06) { /* GET_DESCRIPTOR */
        uint8_t desc_type = wValue >> 8;
        uint8_t desc_idx  = wValue & 0xFF;
        if (desc_type == 0x01) {
            usb_send_pgm(device_desc, sizeof(device_desc), wLength);
            return;
        } else if (desc_type == 0x02) {
            usb_send_pgm(config_desc, sizeof(config_desc), wLength);
            return;
        } else if (desc_type == 0x03) {
            if (desc_idx == 0) {
                usb_send_pgm(str_lang, sizeof(str_lang), wLength);
                return;
            } else if (desc_idx == 1 || desc_idx == 2) {
                usb_send_pgm(str_mfg, sizeof(str_mfg), wLength);
                return;
            }
        } else if (desc_type == 0x21) { /* HID_DESCRIPTOR */
            if (wIndex == 0) {
                usb_send_pgm(config_desc + 18, 9, wLength);
                return;
            } else if (wIndex == 1) {
                usb_send_pgm(config_desc + 43, 9, wLength);
                return;
            }
        } else if (desc_type == 0x22) { /* HID_REPORT */
            if (wIndex == 0) {
                usb_send_pgm(kbd_hid_report_desc, sizeof(kbd_hid_report_desc), wLength);
                return;
            } else if (wIndex == 1) {
                usb_send_pgm(media_hid_report_desc, sizeof(media_hid_report_desc), wLength);
                return;
            }
        }
    } else if (bmRequestType == 0x00 && bRequest == 0x05) { /* SET_ADDRESS */
        UDADDR = wValue & 0x7F;
        UEINTX = ~(1 << TXINI);
        while (!(UEINTX & (1 << TXINI))) {}
        UDADDR |= (1 << ADDEN);
        return;
    } else if (bmRequestType == 0x00 && bRequest == 0x09) { /* SET_CONFIGURATION */
        usb_config = wValue;
        UEINTX = ~(1 << TXINI);
        usb_configure_endpoints();
        return;
    } else if (bmRequestType == 0x80 && bRequest == 0x08) { /* GET_CONFIGURATION */
        while (!(UEINTX & (1 << TXINI))) {}
        UEDATX = usb_config;
        UEINTX = ~(1 << TXINI);
        while (!(UEINTX & (1 << RXOUTI))) {}
        UEINTX = ~(1 << RXOUTI);
        return;
    } else if (bmRequestType == 0x80 && bRequest == 0x00) { /* GET_STATUS */
        while (!(UEINTX & (1 << TXINI))) {}
        UEDATX = 0; UEDATX = 0;
        UEINTX = ~(1 << TXINI);
        while (!(UEINTX & (1 << RXOUTI))) {}
        UEINTX = ~(1 << RXOUTI);
        return;
    } else if ((bmRequestType & 0x7F) == 0x21) { /* Class Interface requests */
        if (bRequest == 0x01) { /* GET_REPORT */
            while (!(UEINTX & (1 << TXINI))) {}
            if (wIndex == 0) {
                for (uint8_t i = 0; i < 8; i++) UEDATX = kbd_report[i];
            } else {
                UEDATX = media_report & 0xFF;
                UEDATX = media_report >> 8;
            }
            UEINTX = ~(1 << TXINI);
            while (!(UEINTX & (1 << RXOUTI))) {}
            UEINTX = ~(1 << RXOUTI);
            return;
        } else if (bRequest == 0x09) { /* SET_REPORT (LEDs) */
            while (!(UEINTX & (1 << RXOUTI))) {}
            (void)UEDATX;
            UEINTX = ~(1 << RXOUTI);
            UEINTX = ~(1 << TXINI);
            return;
        } else if (bRequest == 0x0A) { /* SET_IDLE */
            kbd_idle = wValue >> 8;
            UEINTX = ~(1 << TXINI);
            return;
        } else if (bRequest == 0x02) { /* GET_IDLE */
            while (!(UEINTX & (1 << TXINI))) {}
            UEDATX = kbd_idle;
            UEINTX = ~(1 << TXINI);
            while (!(UEINTX & (1 << RXOUTI))) {}
            UEINTX = ~(1 << RXOUTI);
            return;
        } else if (bRequest == 0x0B) { /* SET_PROTOCOL */
            kbd_protocol = wValue;
            UEINTX = ~(1 << TXINI);
            return;
        } else if (bRequest == 0x03) { /* GET_PROTOCOL */
            while (!(UEINTX & (1 << TXINI))) {}
            UEDATX = kbd_protocol;
            UEINTX = ~(1 << TXINI);
            while (!(UEINTX & (1 << RXOUTI))) {}
            UEINTX = ~(1 << RXOUTI);
            return;
        }
    }

    /* Stall unhandled requests */
    UECONX = (1 << STALLRQ) | (1 << EPEN);
}

static void send_kbd_report(const uint8_t new_report[8]) {
    if (!usb_config) return;
    if (memcmp(kbd_report, new_report, 8) == 0) return;
    memcpy(kbd_report, new_report, 8);

    uint8_t sreg = SREG;
    cli();
    UENUM = KBD_EP;
    uint16_t timeout = 5000;
    while (!(UEINTX & (1 << RWAL)) && --timeout) {
        _delay_us(10);
    }
    if (UEINTX & (1 << RWAL)) {
        for (uint8_t i = 0; i < 8; i++) {
            UEDATX = kbd_report[i];
        }
        UEINTX = 0x3A; /* Clear FIFOCON and TXINI */
    }
    SREG = sreg;
}

static void send_media_report(uint16_t new_media) {
    if (!usb_config) return;
    if (media_report == new_media) return;
    media_report = new_media;

    uint8_t sreg = SREG;
    cli();
    UENUM = MEDIA_EP;
    uint16_t timeout = 5000;
    while (!(UEINTX & (1 << RWAL)) && --timeout) {
        _delay_us(10);
    }
    if (UEINTX & (1 << RWAL)) {
        UEDATX = media_report & 0xFF;
        UEDATX = media_report >> 8;
        UEINTX = 0x3A;
    }
    SREG = sreg;
}

int main(void) {
    MCUSR &= ~(1 << WDRF);
    wdt_disable();
    clock_prescale_set(clock_div_1);

    matrix_init();
    _delay_ms(5);

    /* Bootmagic Lite: if Esc (Row 0, Col 0) is held at power-on, jump to DFU bootloader */
    select_row(0);
    _delay_us(30);
    uint16_t boot_cols = read_cols();
    unselect_row(0);
    if (boot_cols & (1U << 0)) {
        jump_to_bootloader();
    }

    usb_init();

    uint16_t matrix_debounced[N_ROWS] = {0};
    uint16_t matrix_raw[N_ROWS] = {0};
    uint8_t  debounce_cnt[N_ROWS] = {0};

    for (;;) {
        for (uint8_t r = 0; r < N_ROWS; r++) {
            select_row(r);
            _delay_us(30);
            uint16_t cols = read_cols();
            unselect_row(r);
            _delay_us(5);

            if (cols != matrix_raw[r]) {
                matrix_raw[r] = cols;
                debounce_cnt[r] = 5;
            } else if (debounce_cnt[r] > 0) {
                debounce_cnt[r]--;
                if (debounce_cnt[r] == 0) {
                    matrix_debounced[r] = matrix_raw[r];
                }
            }
        }

        /* Check if MO(1) is active (Row 4, Col 12) */
        bool fn_active = (matrix_debounced[4] & (1U << 12)) != 0;

        uint8_t next_report[8] = {0};
        uint8_t key_idx = 2;
        uint16_t next_media = 0;
        bool gesc_pressed = false;

        for (uint8_t r = 0; r < N_ROWS; r++) {
            uint16_t row_bits = matrix_debounced[r];
            if (!row_bits) continue;
            for (uint8_t c = 0; c < N_COLS; c++) {
                if (!(row_bits & (1U << c))) continue;

                uint8_t kc = KC_NO;
                if (fn_active) {
                    kc = pgm_read_byte(&keymaps[1][r][c]);
                }
                if (kc == KC_TRNS || kc == KC_NO) {
                    kc = pgm_read_byte(&keymaps[0][r][c]);
                }

                if (kc == KC_NO || kc == MO_1) {
                    continue;
                } else if (kc == QK_BOOT) {
                    jump_to_bootloader();
                } else if (kc >= KC_LCTL && kc <= KC_RGUI) {
                    next_report[0] |= (1 << (kc - KC_LCTL));
                } else if (kc == QK_GESC) {
                    gesc_pressed = true;
                } else if (kc == KC_MUTE) {
                    next_media = 0x00E2;
                } else if (kc == KC_VOLU) {
                    next_media = 0x00E9;
                } else if (kc == KC_VOLD) {
                    next_media = 0x00EA;
                } else if (kc == KC_MPLY) {
                    next_media = 0x00CD;
                } else if (key_idx < 8) {
                    next_report[key_idx++] = kc;
                }
            }
        }

        if (gesc_pressed && key_idx < 8) {
            /* QK_GESC sends KC_GRV when Shift or GUI is held, else KC_ESC */
            bool shift_or_gui = (next_report[0] & ((1 << 1) | (1 << 5) | (1 << 3) | (1 << 7))) != 0;
            next_report[key_idx++] = shift_or_gui ? KC_GRV : KC_ESC;
        }

        send_kbd_report(next_report);
        send_media_report(next_media);
        _delay_ms(1);
    }
}
