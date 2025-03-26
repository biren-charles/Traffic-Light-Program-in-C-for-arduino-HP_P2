#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL

// Pin Definitions
#define AUTO_ROT     (1 << PD2)  // Pin 2
#define AUTO_GELB    (1 << PD3)  // Pin 3
#define AUTO_GRUEN   (1 << PD4)  // Pin 4
#define FUSS_ROT     (1 << PD7)  // Pin 7
#define FUSS_GRUEN   (1 << PB0)  // Pin 8
#define SENSOR       (1 << PD5)  // Pin 2

void init_ports(void) {

    DDRD |= (AUTO_ROT | AUTO_GELB | AUTO_GRUEN | FUSS_ROT);

    DDRB |= FUSS_GRUEN;

    // Initializsiere lichter
    PORTD &= ~(AUTO_ROT | AUTO_GELB | AUTO_GRUEN | FUSS_ROT);
    PORTB &= ~FUSS_GRUEN;

    
    DDRD &= ~SENSOR;        
    PORTD |= SENSOR;        
}

void setze_ampel(uint8_t zustandD, uint8_t zustandB) {
    
    PORTD = (PORTD & ~(AUTO_ROT | AUTO_GELB | AUTO_GRUEN | FUSS_ROT)) | (zustandD & (AUTO_ROT | AUTO_GELB | AUTO_GRUEN | FUSS_ROT));

    PORTB = (PORTB & ~FUSS_GRUEN) | (zustandB & FUSS_GRUEN);
}

void delay_ms(uint16_t ms) {
    while (ms--) {
        _delay_ms(1);
    }
}

int main(void) {
    uint8_t anforderung_aktiv = 0;

    init_ports();

    while (1) {
        // Normal state: Car green, pedestrian red
        setze_ampel(AUTO_GRUEN | FUSS_ROT, 0);

        // Check sensor
        if (!(PIND & SENSOR) && !anforderung_aktiv) {
            anforderung_aktiv = 1;

            delay_ms(2000);

            // auto gelb
            setze_ampel(AUTO_GELB | FUSS_ROT, 0);
            delay_ms(1000);

            // auto rot
            setze_ampel(AUTO_ROT | FUSS_ROT, 0);
            delay_ms(1000);

            // fuss grün
            setze_ampel(AUTO_ROT, FUSS_GRUEN);
            delay_ms(5000);

            // alles rot
            setze_ampel(AUTO_ROT | FUSS_ROT, 0);
            delay_ms(2000);

            // auto gelb
            setze_ampel(AUTO_ROT | AUTO_GELB | FUSS_ROT, 0);
            delay_ms(1000);

            // normal
            setze_ampel(AUTO_GRUEN | FUSS_ROT, 0);
            delay_ms(1000);

            anforderung_aktiv = 0;
        }

        delay_ms(100);  
    }

    return 0;
}
