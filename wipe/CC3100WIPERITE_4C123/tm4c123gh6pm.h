// Minimal TM4C123GH6PM register definitions used by this project
// This is a pared-down header that only defines the addresses and bitfields
// referenced in the current source files. For full device support, use TI's
// official header from the TivaWare SDK.

#ifndef TM4C123GH6PM_MIN_H_
#define TM4C123GH6PM_MIN_H_

#include <stdint.h>

// --------------------------- System Control ------------------------------
#define SYSCTL_RCGC0_R            (*((volatile uint32_t *)0x400FE100))
#define SYSCTL_RCGC1_R            (*((volatile uint32_t *)0x400FE104))
#define SYSCTL_RCGC2_R            (*((volatile uint32_t *)0x400FE108))
#define SYSCTL_RCC_R              (*((volatile uint32_t *)0x400FE060))
#define SYSCTL_RCGCGPIO_R         (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCPWM_R          (*((volatile uint32_t *)0x400FE640))
#define SYSCTL_PRGPIO_R           (*((volatile uint32_t *)0x400FEA08))

// RCGC1 peripheral enables (legacy)
#define SYSCTL_RCGC1_UART0        0x00000001
#define SYSCTL_RCGC1_UART1        0x00000002

// RCGC2 GPIO bits (legacy)
#define SYSCTL_RCGC2_GPIOA        0x00000001
#define SYSCTL_RCGC2_GPIOB        0x00000002
#define SYSCTL_RCGC2_GPIOC        0x00000004
#define SYSCTL_RCGC2_GPIOD        0x00000008 
#define SYSCTL_RCGC2_GPIOE        0x00000010
#define SYSCTL_RCGC2_GPIOF        0x00000020

// RCC PWM divider
#define SYSCTL_RCC_USEPWMDIV      0x00100000
#define SYSCTL_RCC_PWMDIV_M       0x000E0000
#define SYSCTL_RCC_PWMDIV_2       0x00000000  // /2

// RCGCPWM bits
#define SYSCTL_RCGCPWM_R1         0x00000002  // PWM Module 1

// RCGCGPIO bits
#define SYSCTL_RCGCGPIO_R3        0x00000008  // GPIO Port D

// ------------------------------- SysTick ---------------------------------
#define NVIC_ST_CTRL_R            (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R          (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R         (*((volatile uint32_t *)0xE000E018))

#define NVIC_ST_CTRL_ENABLE       0x00000001
#define NVIC_ST_CTRL_CLK_SRC      0x00000004
#define NVIC_ST_CTRL_COUNT        0x00010000

// ------------------------------- NVIC ------------------------------------
#define NVIC_EN0_R                (*((volatile uint32_t *)0xE000E100))
#define NVIC_PRI1_R               (*((volatile uint32_t *)0xE000E404))
#define NVIC_PRI7_R               (*((volatile uint32_t *)0xE000E41C))

// ------------------------------- GPIO ------------------------------------
// Port A (APB)
#define GPIO_PORTA_DATA_R         (*((volatile uint32_t *)0x400043FC))
#define GPIO_PORTA_DIR_R          (*((volatile uint32_t *)0x40004400))
#define GPIO_PORTA_AFSEL_R        (*((volatile uint32_t *)0x40004420))
#define GPIO_PORTA_PUR_R          (*((volatile uint32_t *)0x40004510))
#define GPIO_PORTA_DEN_R          (*((volatile uint32_t *)0x4000451C))
#define GPIO_PORTA_AMSEL_R        (*((volatile uint32_t *)0x40004528))
#define GPIO_PORTA_PCTL_R         (*((volatile uint32_t *)0x4000452C))

// Port B (APB)
#define GPIO_PORTB_DATA_R         (*((volatile uint32_t *)0x400053FC))
#define GPIO_PORTB_DIR_R          (*((volatile uint32_t *)0x40005400))
#define GPIO_PORTB_IS_R           (*((volatile uint32_t *)0x40005404))
#define GPIO_PORTB_IBE_R          (*((volatile uint32_t *)0x40005408))
#define GPIO_PORTB_IEV_R          (*((volatile uint32_t *)0x4000540C))
#define GPIO_PORTB_IM_R           (*((volatile uint32_t *)0x40005410))
#define GPIO_PORTB_RIS_R          (*((volatile uint32_t *)0x40005414))
#define GPIO_PORTB_ICR_R          (*((volatile uint32_t *)0x4000541C))
#define GPIO_PORTB_AFSEL_R        (*((volatile uint32_t *)0x40005420))
#define GPIO_PORTB_PUR_R          (*((volatile uint32_t *)0x40005510))
#define GPIO_PORTB_DEN_R          (*((volatile uint32_t *)0x4000551C))
#define GPIO_PORTB_LOCK_R         (*((volatile uint32_t *)0x40005520))
#define GPIO_PORTB_CR_R           (*((volatile uint32_t *)0x40005524))
#define GPIO_PORTB_AMSEL_R        (*((volatile uint32_t *)0x40005528))
#define GPIO_PORTB_PCTL_R         (*((volatile uint32_t *)0x4000552C))

// Port C (APB)
#define GPIO_PORTC_DATA_R         (*((volatile uint32_t *)0x400063FC))
#define GPIO_PORTC_DIR_R          (*((volatile uint32_t *)0x40006400))
#define GPIO_PORTC_AFSEL_R        (*((volatile uint32_t *)0x40006420))
#define GPIO_PORTC_PUR_R          (*((volatile uint32_t *)0x40006510))
#define GPIO_PORTC_DEN_R          (*((volatile uint32_t *)0x4000651C))
#define GPIO_PORTC_AMSEL_R        (*((volatile uint32_t *)0x40006528))
#define GPIO_PORTC_PCTL_R         (*((volatile uint32_t *)0x4000652C))

// Port D (APB)
#define GPIO_PORTD_DATA_R         (*((volatile uint32_t *)0x400073FC))
#define GPIO_PORTD_DIR_R          (*((volatile uint32_t *)0x40007400))
#define GPIO_PORTD_AFSEL_R        (*((volatile uint32_t *)0x40007420))
#define GPIO_PORTD_PUR_R          (*((volatile uint32_t *)0x40007510))
#define GPIO_PORTD_DEN_R          (*((volatile uint32_t *)0x4000751C))
#define GPIO_PORTD_AMSEL_R        (*((volatile uint32_t *)0x40007528))
#define GPIO_PORTD_PCTL_R         (*((volatile uint32_t *)0x4000752C))

// Port E (APB)
#define GPIO_PORTE_DATA_R         (*((volatile uint32_t *)0x400243FC))
#define GPIO_PORTE_DIR_R          (*((volatile uint32_t *)0x40024400))
#define GPIO_PORTE_AFSEL_R        (*((volatile uint32_t *)0x40024420))
#define GPIO_PORTE_PUR_R          (*((volatile uint32_t *)0x40024510))
#define GPIO_PORTE_DEN_R          (*((volatile uint32_t *)0x4002451C))
#define GPIO_PORTE_AMSEL_R        (*((volatile uint32_t *)0x40024528))
#define GPIO_PORTE_PCTL_R         (*((volatile uint32_t *)0x4002452C))

// Port F (APB)
#define GPIO_PORTF_DATA_R         (*((volatile uint32_t *)0x400253FC))
#define GPIO_PORTF_DIR_R          (*((volatile uint32_t *)0x40025400))
#define GPIO_PORTF_IS_R           (*((volatile uint32_t *)0x40025404))
#define GPIO_PORTF_IBE_R          (*((volatile uint32_t *)0x40025408))
#define GPIO_PORTF_IEV_R          (*((volatile uint32_t *)0x4002540C))
#define GPIO_PORTF_IM_R           (*((volatile uint32_t *)0x40025410))
#define GPIO_PORTF_RIS_R          (*((volatile uint32_t *)0x40025414))
#define GPIO_PORTF_ICR_R          (*((volatile uint32_t *)0x4002541C))
#define GPIO_PORTF_AFSEL_R        (*((volatile uint32_t *)0x40025420))
#define GPIO_PORTF_PUR_R          (*((volatile uint32_t *)0x40025510))
#define GPIO_PORTF_DEN_R          (*((volatile uint32_t *)0x4002551C))
#define GPIO_PORTF_LOCK_R         (*((volatile uint32_t *)0x40025520))
#define GPIO_PORTF_CR_R           (*((volatile uint32_t *)0x40025524))
#define GPIO_PORTF_AMSEL_R        (*((volatile uint32_t *)0x40025528))
#define GPIO_PORTF_PCTL_R         (*((volatile uint32_t *)0x4002552C))

#define GPIO_LOCK_KEY             0x4C4F434B

// ------------------------------- PWM1 ------------------------------------
#define PWM1_ENABLE_R             (*((volatile uint32_t *)0x40029008))
#define PWM_ENABLE_PWM0EN         0x00000001
#define PWM_ENABLE_PWM1EN         0x00000002
#define PWM_ENABLE_PWM2EN         0x00000004
#define PWM_ENABLE_PWM3EN         0x00000008
#define PWM_ENABLE_PWM4EN         0x00000010
#define PWM_ENABLE_PWM5EN         0x00000020
#define PWM_ENABLE_PWM6EN         0x00000040
#define PWM_ENABLE_PWM7EN         0x00000080

// Generator 2 (PWM1_2)
#define PWM1_2_CTL_R              (*((volatile uint32_t *)0x400290C0))
#define PWM1_2_GENA_R             (*((volatile uint32_t *)0x400290E0))
#define PWM1_2_LOAD_R             (*((volatile uint32_t *)0x400290D0))
#define PWM1_2_CMPA_R             (*((volatile uint32_t *)0x400290D8))
// additional registers for Generator 2
#define PWM1_2_CMPB_R             (*((volatile uint32_t *)0x400290DC))
#define PWM1_2_GENB_R             (*((volatile uint32_t *)0x400290E4))

// Generator 3 (PWM1_3)
#define PWM1_3_CTL_R              (*((volatile uint32_t *)0x40029100))
#define PWM1_3_GENA_R             (*((volatile uint32_t *)0x40029120))
#define PWM1_3_LOAD_R             (*((volatile uint32_t *)0x40029110))
#define PWM1_3_CMPA_R             (*((volatile uint32_t *)0x40029118))

// ------------------------------- PWM0 ------------------------------------
#define PWM0_ENABLE_R             (*((volatile uint32_t *)0x40028008))

// Generator 0 (PWM0_0)
#define PWM0_0_CTL_R              (*((volatile uint32_t *)0x40028040))
#define PWM0_0_LOAD_R             (*((volatile uint32_t *)0x40028050))
#define PWM0_0_CMPA_R             (*((volatile uint32_t *)0x40028058))
#define PWM0_0_CMPB_R             (*((volatile uint32_t *)0x4002805C))
#define PWM0_0_GENA_R             (*((volatile uint32_t *)0x40028060))
#define PWM0_0_GENB_R             (*((volatile uint32_t *)0x40028064))

// Generator 1 (PWM0_1)
#define PWM0_1_CTL_R              (*((volatile uint32_t *)0x40028080))
#define PWM0_1_LOAD_R             (*((volatile uint32_t *)0x40028090))
#define PWM0_1_CMPA_R             (*((volatile uint32_t *)0x40028098))
#define PWM0_1_CMPB_R             (*((volatile uint32_t *)0x4002809C))
#define PWM0_1_GENA_R             (*((volatile uint32_t *)0x400280A0))
#define PWM0_1_GENB_R             (*((volatile uint32_t *)0x400280A4))

// Generator 2 (PWM0_2)
#define PWM0_2_CTL_R              (*((volatile uint32_t *)0x400280C0))
#define PWM0_2_LOAD_R             (*((volatile uint32_t *)0x400280D0))
#define PWM0_2_CMPA_R             (*((volatile uint32_t *)0x400280D8))
#define PWM0_2_CMPB_R             (*((volatile uint32_t *)0x400280DC))
#define PWM0_2_GENA_R             (*((volatile uint32_t *)0x400280E0))
#define PWM0_2_GENB_R             (*((volatile uint32_t *)0x400280E4))

// ------------------------------- UART0/1 ---------------------------------
// UART0 base 0x4000C000, UART1 base 0x4000D000
#define UART0_DR_R                (*((volatile uint32_t *)0x4000C000))
#define UART0_FR_R                (*((volatile uint32_t *)0x4000C018))
#define UART0_IBRD_R              (*((volatile uint32_t *)0x4000C024))
#define UART0_FBRD_R              (*((volatile uint32_t *)0x4000C028))
#define UART0_LCRH_R              (*((volatile uint32_t *)0x4000C02C))
#define UART0_CTL_R               (*((volatile uint32_t *)0x4000C030))
#define UART0_IFLS_R              (*((volatile uint32_t *)0x4000C034))
#define UART0_IM_R                (*((volatile uint32_t *)0x4000C038))

#define UART1_DR_R                (*((volatile uint32_t *)0x4000D000))
#define UART1_FR_R                (*((volatile uint32_t *)0x4000D018))
#define UART1_IBRD_R              (*((volatile uint32_t *)0x4000D024))
#define UART1_FBRD_R              (*((volatile uint32_t *)0x4000D028))
#define UART1_LCRH_R              (*((volatile uint32_t *)0x4000D02C))
#define UART1_CTL_R               (*((volatile uint32_t *)0x4000D030))

// UART control bits
#define UART_CTL_UARTEN           0x00000001
#define UART_CTL_TXE              0x00000100
#define UART_CTL_RXE              0x00000200

// UART line control
#define UART_LCRH_WLEN_8          0x00000060
#define UART_LCRH_FEN             0x00000010

// UART flag register bits
#define UART_FR_RXFE              0x00000010
#define UART_FR_TXFF              0x00000020

// UART interrupt FIFO level select (receive mask + 1/8 level)
#define UART_IFLS_RX_M            0x00000038
#define UART_IFLS_RX1_8           0x00000000

// UART interrupt mask bits
#define UART_IM_RXIM              0x00000010
#define UART_IM_TXIM              0x00000020
#define UART_IM_RTIM              0x00000040

#endif // TM4C123GH6PM_MIN_H_
