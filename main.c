/*
 * File:   PWM.c
 * Author: Cristian Catú, Jose Gonzales, Frank Martínez
 * LAB 9
 *
 * Created on 27 april 2022, 19:04
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 4000000
#define IN_MIN 0           
#define IN_MAX 255        
#define OUT_MIN 15       
#define OUT_MAX 75


/*------------------------------------------------------------------------------
 * VARIABLES
 ------------------------------------------------------------------------------*/
unsigned short CCPR = 0;        // Variable para almacenar ancho de pulso al hacer la interpolación lineal
uint8_t pot4 = 0;
uint8_t pot3 = 0;
uint8_t pot2 = 0;
uint8_t pot1 = 0;
uint8_t bandera = 0;
uint8_t CCPR_2 = 0;
uint8_t estado = 1;
uint8_t entrada_terminal = 1;
uint8_t bits_envio = 0;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max,
            unsigned short out_min, unsigned short out_max);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.ADIF){                      // Interrupción ADC
        if(estado == 1){
            if(ADCON0bits.CHS == 0){            // Verificamos sea AN0 el canal seleccionado
                pot1 = ADRESH;
                CCPR_2 = map(pot1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR2L = (uint8_t)(CCPR_2>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP2CONbits.DC2B0 = CCPR_2 & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                CCP2CONbits.DC2B1 = CCPR_2>>1 & 0b1;

            }
            else if(ADCON0bits.CHS == 1){            // Verificamos sea AN0 el canal seleccionado
                pot2 = ADRESH;
                CCPR = map(pot2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B

            }
            else if(ADCON0bits.CHS == 2){            // Verificamos sea AN0 el canal seleccionado
                pot3 = ADRESH;
            }
            else if(ADCON0bits.CHS == 3){            // Verificamos sea AN0 el canal seleccionado
                pot4 = ADRESH;
            }
        }
        PIR1bits.ADIF = 0;                  // Limpiamos bandera de interrupción
    }
        
    if(PIR1bits.RCIF){
        entrada_terminal = RCREG;
        if (entrada_terminal < 5){
            if (entrada_terminal == 4){
                estado++;
                if(estado == 4)
                    estado = 1;
            }
            if(estado == 1){
                if(entrada_terminal == 3){                     // Verificamos si fue RB0 quien generó la interrupción
                    write_EEPROM(1, pot1); //Escribir cuando nos dormimos
                }
                if(entrada_terminal == 2){                     // Verificamos si fue RB0 quien generó la interrupción
                    write_EEPROM(2, pot2); //Escribir cuando nos dormimos
                }
                if(entrada_terminal == 1){                     // Verificamos si fue RB0 quien generó la interrupción
                    write_EEPROM(3, pot3); //Escribir cuando nos dormimos
                }
                if(entrada_terminal == 0){                     // Verificamos si fue RB0 quien generó la interrupción
                    write_EEPROM(4, pot4); //Escribir cuando nos dormimos
                }
            }
            else if(estado == 2){
                if(entrada_terminal == 3){                     // Verificamos si fue RB0 quien generó la interrupción
                    pot1 = read_EEPROM(1);
                    CCPR_2 = map(pot1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                    CCPR2L = (uint8_t)(CCPR_2>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                    CCP2CONbits.DC2B0 = CCPR_2 & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                    CCP2CONbits.DC2B1 = CCPR_2>>1 & 0b1;
                }
                if(entrada_terminal == 2){                     // Verificamos si fue RB0 quien generó la interrupción
                    pot2 = read_EEPROM(2); // Mostrar siempre la lectura del eeprom en el puerto C
                    CCPR = map(pot2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                    CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                    CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
                }
                if(entrada_terminal == 1){                     // Verificamos si fue RB0 quien generó la interrupción
                    pot3 = read_EEPROM(3); // Mostrar siempre la lectura del eeprom en el puerto C
                }
                if(entrada_terminal == 0){                     // Verificamos si fue RB0 quien generó la interrupción
                    pot4 = read_EEPROM(4); // Mostrar siempre la lectura del eeprom en el puerto C
                }
            }
        }
    }
    if(INTCONbits.RBIF){            // Fue interrupción del PORTB  
        if(!PORTBbits.RB0){                     // Verificamos si fue RB0 quien generó la interrupción
            estado++;
            if(estado == 4)
                estado = 1;
        }
        if(estado == 1){
            if(!PORTBbits.RB1){                     // Verificamos si fue RB0 quien generó la interrupción
                write_EEPROM(1, pot1); //Escribir cuando nos dormimos
            }
            if(!PORTBbits.RB2){                     // Verificamos si fue RB0 quien generó la interrupción
                write_EEPROM(2, pot2); //Escribir cuando nos dormimos
            }
            if(!PORTBbits.RB3){                     // Verificamos si fue RB0 quien generó la interrupción
                write_EEPROM(3, pot3); //Escribir cuando nos dormimos
            }
            if(!PORTBbits.RB4){                     // Verificamos si fue RB0 quien generó la interrupción
                write_EEPROM(4, pot4); //Escribir cuando nos dormimos
            }
        }
        else if(estado == 2){
            if(!PORTBbits.RB1){                     // Verificamos si fue RB0 quien generó la interrupción
                pot1 = read_EEPROM(1);
                CCPR_2 = map(pot1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR2L = (uint8_t)(CCPR_2>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP2CONbits.DC2B0 = CCPR_2 & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                CCP2CONbits.DC2B1 = CCPR_2>>1 & 0b1;
            }
            if(!PORTBbits.RB2){                     // Verificamos si fue RB0 quien generó la interrupción
                pot2 = read_EEPROM(2); // Mostrar siempre la lectura del eeprom en el puerto C
                CCPR = map(pot2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
            }
            if(!PORTBbits.RB3){                     // Verificamos si fue RB0 quien generó la interrupción
                pot3 = read_EEPROM(3); // Mostrar siempre la lectura del eeprom en el puerto C
            }
            if(!PORTBbits.RB4){                     // Verificamos si fue RB0 quien generó la interrupción
                pot4 = read_EEPROM(4); // Mostrar siempre la lectura del eeprom en el puerto C
            }
        }
        INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupción del puerto B
    }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        __delay_ms(10);
        if(PIR1bits.TXIF){ 
            if(bandera == 1){
                TXREG = 0b00000001 | pot3; //identificamos el valor enviado para el potenciometro 3
                bandera = 0;
            }
            else{
                TXREG = 0b11111110 & pot4;// identificamos el valor enviado para el potenciometro 4
                bandera = 1;
            } 
        }
        if(estado == 1){
            if(ADCON0bits.GO == 0){             // No hay proceso de conversion
                // *usando mas de un canal analógico
                if(ADCON0bits.CHS == 0b0000)
                    ADCON0bits.CHS = 0b0001;    // Cambio de canal 1
                else if(ADCON0bits.CHS == 0b0001)
                    ADCON0bits.CHS = 0b0010;    // Cambio de canal 2
                else if(ADCON0bits.CHS == 0b0010)
                    ADCON0bits.CHS = 0b0011;    // Cambio de canal 2
                else if(ADCON0bits.CHS == 0b0011)
                    ADCON0bits.CHS = 0b0000;    // Cambio de canal 2
                __delay_us(40);                 // Tiempo adquisición
                ADCON0bits.GO = 1;              // proceso de conversión
            }
            PORTE = 1;
        }
        else if(estado == 2){
            PORTE = 2;
        }
        else if(estado == 3){
            PORTE = 4;
            PORTD = entrada_terminal;
            if (entrada_terminal > 4){
                bits_envio = 0b00000011 & entrada_terminal;
                if (bits_envio == 0){
                    pot1 = entrada_terminal;
                    CCPR_2 = map(pot1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                    CCPR2L = (uint8_t)(CCPR_2>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                    CCP2CONbits.DC2B0 = CCPR_2 & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                    CCP2CONbits.DC2B1 = CCPR_2>>1 & 0b1;
                }
                else if(bits_envio == 1){
                    pot2 = entrada_terminal; // Mostrar siempre la lectura del eeprom en el puerto C
                    CCPR = map(pot2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                    CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                    CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
                }
                else if(bits_envio == 2){
                    pot3 = entrada_terminal;
                }
                else if(bits_envio == 3){
                    pot4 = entrada_terminal;
                }
            }
        }
        
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0xF;                // AN0, AN1 y AN2 como entrada analógica
    ANSELH = 0;                 // I/O digitales
    TRISA = 0xF;                // AN0, AN1 y AN2 como entrada
    PORTA = 0;
    TRISB = 0b00011111;
    TRISD = 0;
    PORTD = 0;
    TRISE = 0;
    PORTE = estado;

    OSCCONbits.IRCF = 0b011;    // 500MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 12;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time

    // Configuración PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    PR2 = 155;                  // periodo de 2ms

    // Configuración CCP
    CCP1CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM

    CCPR1L = 30>>2;
    CCP1CONbits.DC1B = 30 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo

    TRISCbits.TRISC1 = 1;       // Deshabilitamos salida de CCP1
    CCP2CON = 0;                // Apagamos CCP1
    CCP2CONbits.CCP2M = 0b1100; // PWM
    CCPR2L = 30>>2;
    CCP2CONbits.DC2B0 = 30 & 0b1; // Guardamos los 2 bits menos significativos en DC2B
    CCP2CONbits.DC2B1 = 30>>1 & 0b1;

    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente

    TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
    TRISCbits.TRISC1 = 0;       // Habilitamos salida de PWM
    TRISCbits.TRISC3 = 0;       // Habilitamos salida de PWM
    PORTC = 0;

    // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    PIE1bits.RCIE = 1;
    
    OPTION_REGbits.nRBPU = 0; 
    WPUB = 0b00011111; 
    INTCONbits.RBIE = 1;   
    IOCB = 0b00011111;         
    INTCONbits.RBIF = 0;


}
// función para el mapeo
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
uint8_t read_EEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;  // Lectura en la EEPROM
    EECON1bits.RD = 1;       // Conseguimos dato de la EEPROM
    return EEDAT;              // Regresamos ese dato leido 
}

//Función para escribir
void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0; // Escritura en la EEPROM
    EECON1bits.WREN = 1;  // Habilitamos la escritura a la EEPROM
    
    INTCONbits.GIE = 0;    // Deshabilitamos las interrupciones
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;    // Se inicia la escritura
    
    EECON1bits.WREN = 0;     // se deshabilita escritura en la EEPROM
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;   // Habilitamos las interrupciones
}