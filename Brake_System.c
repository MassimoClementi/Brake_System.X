//PROGRAM: Brake_System
//WRITTEN BY: Massimo Clementi
//DATA: 26/11/2015
//VERSION: 1.0
//FILE SAVED AS: Brake_System.c
//FOR PIC: 18F4480
//CLOCK FREQUENCY: 16 MHz
//PROGRAM FUNCTION: Centralina che gestisce la frenata del veicolo secondo i
// valori che vengono forniti dalle altre centraline attraverso il protocollo 
// CANbus. Predisposizione di led di malfunzionamento del CANbus, arresto e
// trimmer su scheda per il controllo della correzione della frenata.

////////////////////////////            //////////////////////////////
//          I/O           //            //  COMBINAZIONI BIT CANBus //
//  RA0 => Trimmer (ADC)  //            //   00 = Nessuna frenata   //
//  RB0 => Warning LED    //            //   01 = Livello basso     //
//  RC0 => PWM TMR0       //            //   10 = Livello medio     //
//  CANRX/CANTX => CANBus //            //   11 = Livello alto      //
////////////////////////////            //////////////////////////////

// TIMER0 => PWM
// 800us  => 0�
// 1500us => 90�
// 2200us => 180�
// 500ns ad ogni singolo incremento (prescaler 1:2)

// TIMER3 => delay

#define USE_AND_MASKS

#include <xc.h>
#include "PIC18F4480_config.h"
#include "CANlib.h"
#include "delay.c"
#include "delay.h"
#include "pwm.h"
#include "timers.h"
#define _XTAL_FREQ 16000000
#define HIGH 1
#define LOW 0

//////////////////////////////
//  POSIZIONE HOME => 127   //
//  POSIZIONE BRAKE => 142  //
//////////////////////////////

#define brake_signal 0b00000000000000000000000000110 //(!!) impostare
#define status_id 0b00000000000000000000000000100 //(!!) impostare

//Prototitpi delle funzioni
void board_initialization(void);
void status_ok(void);
void ADC_Read(void);

//////////////////
//Declarations  //
//////////////////
//CANbus
CANmessage msg;
bit remote_frame;
bit Tx_retry;
unsigned long id = 0;
BYTE status_array [8] = 0; //!!MODIFICARE

//Delay TMR0
unsigned long timer_on = 0;
unsigned long timer_off = 0;

//Delay TMR3
unsigned long TMR3_counter = 0;
unsigned long TMR3_stored = 0;
unsigned char wait_time = 0; //in ms (10-20-30..)

//ADC
unsigned char i = 0;
unsigned char number_of_measures = 8; //!! decidere
unsigned char read = 0;
unsigned int partial_sum = 0;
unsigned int correction_factor = 0;

//Program variables
unsigned char brake_signal_CAN;
unsigned char brake_value_inc = 0; //0-256 (fattore 17)
unsigned char brake_value = 0; //0-15
unsigned char brake_value_degree = 0; //0-180
unsigned char home_position = 0;
unsigned char ADC_wait_counter = 0;
unsigned char gap = 0;
unsigned char inc = 0;
unsigned char ramp_speed = 0;

//////////////////////
//    INTERRUPT     //
//  Gestione CANbus //
//////////////////////

__interrupt(high_priority) void ISR_Alta(void) {
    if ((PIR3bits.RXB0IF == HIGH) || (PIR3bits.RXB1IF == HIGH)) {
        if (CANisRxReady()) {
            CANreceiveMessage(&msg);
            if (msg.RTR == HIGH) {
                id = msg.identifier;
                remote_frame = HIGH;
            }
            if (msg.identifier == brake_signal) {
                brake_signal_CAN = msg.data[0];
            }
        }
        PIR3bits.RXB0IF = LOW;
        PIR3bits.RXB1IF = LOW;
    }
    if (INTCONbits.TMR0IF == HIGH) {
        PORTCbits.RC0 = ~PORTCbits.RC0;
        if (PORTCbits.RC0 == 1) {
            timer_on = (((1400 * brake_value_degree) / 180) + 800)*2; //incrementi TMR0
            timer_on = 65536 - timer_on; //interrupt per overflow
            timer_off = 40000 - timer_on;
            WriteTimer0(timer_on);
        } else {
            WriteTimer0(timer_off);
        }
        INTCONbits.TMR0IF = 0;
    }
}

//////////////////////////
//    INTERRUPT         //
//  Gestione TMR0 delay //
//////////////////////////

__interrupt(low_priority) void ISR_Bassa(void) {
    if (PIR2bits.TMR3IF) {
        TMR3_counter++;
        TMR3H = 0x63;
        TMR3L = 0xC0;
        PIR2bits.TMR3IF = 0;
    }
}

//////////////
//   MAIN   //
//////////////

int main(void) {
    board_initialization();
    ADC_Read();

    TMR0H = 0xFF; //<= FORZIAMO IL PRIMO INTERRUPT
    TMR0L = 0xFE; //<= DEL TIMER0 PER LE CONFIGURAZIONI
    T0CONbits.TMR0ON = 1; //<= DEI REGISTRI PER IL PWM

    while (1) {
        if ((remote_frame == HIGH)&&(Tx_retry = HIGH)) {
            status_ok();
        }

        if ((CANisTXwarningON() == HIGH) || (CANisRXwarningON() == HIGH)) {
            PORTBbits.RB0 = HIGH; //accendi led errore
        }

        if (ADC_wait_counter == 50) { //polling trimmer ogni 50 cicli
            ADC_Read();
            ADC_wait_counter = 0;
        }

        wait_time = 20;

        if ((TMR3_counter - TMR3_stored) > (wait_time / 10)) {

            if ((brake_signal_CAN == 00)&&((brake_value_inc / 2) > 1)) {
                brake_value_inc = brake_value_inc / 2;
                brake_value = (brake_value_inc / 17) + home_position;
                brake_value_degree = (255 * brake_value) / 180;
            }

            if ((brake_signal_CAN != 00)&&((255 - brake_value) != 0)) {
                if (brake_signal_CAN == 01) { //LOW
                    ramp_speed = 20; //verificare valore
                }
                if (brake_signal_CAN == 10) { //MEDIUM
                    ramp_speed = 10; //verificare valore
                }
                if (brake_signal_CAN == 11) { //HIGH
                    ramp_speed = 5; //verificare valore
                }

                gap = 255 - brake_value_inc;
                inc = ((gap / ramp_speed)*(gap / ramp_speed));
                if (inc < 1) {
                    brake_value_inc = 255;
                    brake_value = (brake_value_inc / 17) + home_position;
                    brake_value_degree = (255 * brake_value) / 180;
                } else {
                    brake_value_inc = brake_value_inc + inc;
                    brake_value = (brake_value_inc / 17) + home_position;
                    brake_value_degree = (255 * brake_value) / 180;
                }
            }
            TMR3_stored = TMR3_counter;
        }
        ADC_wait_counter++;
    }
}

void status_ok(void) {
    if (CANisTxReady()) {
        if (id == status_id) {
            CANsendMessage(id, status_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            if (TXB0CONbits.TXABT || TXB1CONbits.TXABT) {
                Tx_retry = HIGH;
            } else {
                Tx_retry = LOW;
            }
        }
    } else {
        Tx_retry = HIGH;
    }
    remote_frame = LOW;
}

void ADC_Read(void) {
    for (i = 0; i < number_of_measures; i++) {
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO);
        read = ADRESH;
        partial_sum = partial_sum + read;
    }
    correction_factor = (partial_sum / number_of_measures) - 127;
    home_position = correction_factor / 4 + 127; //RICONTROLLARE /4
}

void board_initialization(void) { //(!!)completare
    //Configurazione I/O
    LATA = 0x00;
    TRISA = 0xFF; //ALL IN
    LATB = 0x00;
    TRISB = 0b11111110; //RBO OUTPUT
    LATC = 0x00;
    TRISC = 0b11111110; //RC0 OUTPUT
    LATD = 0x00;
    TRISD = 0xFF;
    LATE = 0x00;
    TRISE = 0xFF;

    ADCON1 = 0x11111110;

    //Configurazione CANbus
    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON);
    RCONbits.IPEN = 1;

    //Azzero Flag Interrupts
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0
    INTCONbits.TMR0IF = 0;
    PIR2bits.TMR3IF = 0; //resetta flag interrupt timer 3

    //Config. Priorit�
    IPR3bits.RXB1IP = 1; //interrupt alta priorit� per can
    IPR3bits.RXB0IP = 1; //interrupt alta priorit� per can
    INTCON2bits.TMR0IP = 1; //interrupt alta priorit� timer0
    IPR2bits.TMR3IP = 0; //interrupt bassa priorit� timer 3

    //Config. Registri
    T0CON = 0x80; //imposta timer0, prescaler 1:2
    TMR3H = 0x63; //<= VALORI PER AVERE UN
    TMR3L = 0xC0; //<= INTERRUPT OGNI 10ms
    INTCON2bits.INTEDG0 = 1; //interrupt su fronte di salita [SERVE?]

    //Enable Interrupts
    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    INTCONbits.TMR0IE = 1; //abilita interrupt timer 0
    PIE2bits.TMR3IE = 1; //abilita interrupt timer 3
    INTCONbits.GIEH = 1; //abilita interrupt alta priorit�
    INTCONbits.GIEL = 1; //abilita interrupt bassa priorit� periferiche

    //Configurazione ADC
    ADCON1 = 0b01110111;
    ADCON0bits.CHS2 = 0; //<--|
    ADCON0bits.CHS1 = 0; //<--|- CANALE 0 => RB0
    ADCON0bits.CHS0 = 0; //<--|
    ADCON2bits.ACQT2 = 1;
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 0;
    ADCON2bits.ADCS2 = 1;
    ADCON2bits.ADCS1 = 0;
    ADCON2bits.ADCS0 = 1;
    ADCON2bits.ADFM = 0; //Right Justified
    ADCON0bits.ADON = 1;

    T3CON = 0x01; //abilita timer
    delay_ms(2);
}