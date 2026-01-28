#include "servo.h"
#include "stm32f0xx_it.h"
#include "stm32f0xx.h"
#include "main.h"


void servo_uart_init(void)
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA9 and PA10 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER10_Msk);
	GPIOA->MODER |=  (0x02 <<GPIO_MODER_MODER9_Pos) | (0x02 <<GPIO_MODER_MODER10_Pos);

	// Set PA9 and PA10 to AF1 (USART1)
	GPIOA->AFR[1] &= ~(0x00000FF0);
	GPIOA->AFR[1] |=  (0x00000110);

	// Enable USART1 clock
	RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;

	// Clear USART1 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART1->CR1 = 0x00000000;
	USART1->CR2 = 0x00000000;
	USART1->CR3 = 0x00000000;

	// Select PCLK (APB1) as clock source
	// PCLK -> 48 MHz
	RCC->CFGR3 &= ~RCC_CFGR3_USART1SW_Msk;

	// Baud Rate = 115200
	// With OVER8=0 and Fck=48MHz, USARTDIV =   48E6/115200 = 416.6666
	// BRR = 417 -> Actual BaudRate = 115107.9137 -> 0.08% error
	//
	// With OVER8=1 and Fck=48MHz, USARTDIV = 2*48E6/115200 = 833.3333
	// BRR = 833 -> Actual BaudRate = 115246.0984 -> 0.04% error (better choice)

	USART1->CR1 |= USART_CR1_OVER8;
	USART1->BRR = 833;

	// Enable both Transmitter and Receiver
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;

	// Enable USART1
	USART1->CR1 |= USART_CR1_UE;
}

void uart_send_byte(uint8_t data)
{
	while (!(USART1->ISR & USART_ISR_TXE));
	USART1->TDR = data;
}

uint8_t uart_receive_byte(void)
{
    //On attend que le drapeau RXNE 1
    while (!(USART1->ISR & USART_ISR_RXNE));

    // On LIT le registre RDR (Receive Data Register)
    return (uint8_t)(USART1->RDR); 
}


void herkulex_ram_write(uint8_t servo_id, uint8_t address, uint8_t length, uint8_t *data)
{
    uint8_t packet_size = 9 + length;
    uint8_t packet[packet_size];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = packet_size;
    packet[3] = servo_id;
    packet[4] = 0x03; // RAM_WRITE

    packet[7] = address;
    packet[8] = length;

    for (uint8_t i = 0; i < length; i++)
    {
        packet[9 + i] = data[i];
    }

    uint8_t cksum1 = packet[2] ^ packet[3] ^ packet[4] ^ packet[7] ^ packet[8];
    for (uint8_t i = 0; i < length; i++) {
        cksum1 ^= data[i];
    }
    cksum1 &= 0xFE;

    packet[5] = cksum1;
    packet[6] = (~cksum1) & 0xFE;

    for (uint8_t j = 0; j < packet_size; j++) {
        uart_send_byte(packet[j]);
    }
}

void herkulex_ram_read(uint8_t servo_id, uint8_t address, uint8_t length)
{
    uint8_t packet[10];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0x0A;
    packet[3] = servo_id;
    packet[4] = 0x04;  // RAM_READ
    packet[7] = address;
    packet[8] = length;

    uint8_t cksum1 = packet[2] ^ packet[3] ^ packet[4] ^ packet[7] ^ packet[8];
    cksum1 &= 0xFE;
    packet[5] = cksum1;
    packet[6] = (~cksum1) & 0xFE;

    my_printf("Tram :");
    for (uint8_t i = 0; i < 10; i++) {
        uart_read_byte(packet[i]);
    }
}

void herkulex_torque_on(uint8_t servo_id)
{
    uint8_t data[1] = {0x60};  // Torque ON
    herkulex_ram_write(servo_id, 0x34, 1, data);
}

void herkulex_led_red(uint8_t servo_id)
{
    uint8_t data[1] = {0x04};  // LED rouge
    herkulex_ram_write(servo_id, 0x35, 1, data);
}

void herkulex_led_green(uint8_t servo_id)
{
    uint8_t data[1] = {0x01}; //LED green
    herkulex_ram_write(servo_id, 0x35, 1, data);
}

void herkulex_led_cyan(uint8_t servo_id)
{
    uint8_t data[1] = {0x07}; //LED green
    herkulex_ram_write(servo_id, 0x35, 1, data);
}


//void herkulex_set_position2(uint8_t  id1, uint16_t position1,uint8_t  id2, uint16_t position2,uint32_t  playtime)
//{
//    const uint8_t count = 2;
//    const uint8_t packet_size = 7 + 5 * count;   // 7 header + 5 bytes/servo = 17
//    uint8_t packet[packet_size];
//
//    // Header
//    packet[0] = 0xFF;
//    packet[1] = 0xFF;
//    packet[2] = packet_size;   // 17 = 0x11
//    packet[3] = 0xFE;          // broadcast
//    packet[4] = 0x05;          // I_JOG
//
//    // ---- Servo 1 ----
//    uint8_t pos1_lsb = position1 & 0xFF;
//    uint8_t pos1_msb = (position1 >> 8) & 0x03;
//
//    uint8_t base1 = 7;         // premier bloc de 5 octets
//    packet[base1 + 0] = pos1_lsb;      // JOG LSB
//    packet[base1 + 1] = pos1_msb;      // JOG MSB
//    packet[base1 + 2] = 0x04;          // SET (mêmes flags que ta version 1-servo)
//    packet[base1 + 3] = id1;           // ID servo 1
//    packet[base1 + 4] = playtime;      // même playtime pour les deux
//
//    // ---- Servo 2 ----
//    uint8_t pos2_lsb = position2 & 0xFF;
//    uint8_t pos2_msb = (position2 >> 8) & 0x03;
//
//    uint8_t base2 = 7 + 5;     // deuxième bloc de 5 octets → index 12
//    packet[base2 + 0] = pos2_lsb;
//    packet[base2 + 1] = pos2_msb;
//    packet[base2 + 2] = 0x04;          // SET
//    packet[base2 + 3] = id2;           // ID servo 2
//    packet[base2 + 4] = playtime;
//
//    // ---- Checksum ----
//    uint8_t cksum1 = 0;
//    for (uint8_t i = 2; i < packet_size; i++) {
//        if (i == 5 || i == 6) continue;
//        cksum1 ^= packet[i];
//    }
//    cksum1 &= 0xFE;
//    packet[5] = cksum1;
//    packet[6] = (~cksum1) & 0xFE;
//
//    // ---- Envoi ----
//    for (uint8_t j = 0; j < packet_size; j++) {
//        uart_send_byte(packet[j]);
//    }
//}


void herkulex_set_position(uint8_t servo_id, uint16_t position, uint32_t playtime)
{
    const uint8_t packet_size = 12;   // 7 header + 5 data *nbservo
    uint8_t packet[packet_size];

    uint8_t pos_lsb = position & 0xFF;
    uint8_t pos_msb = (position >> 8) & 0x03; // 10 bits

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = packet_size;          // 0x0C
    packet[3] = 0xFE;                 // ou servo_id si tu ne veux pas broadcast
    packet[4] = 0x5;                 // I_JOG

    //les cksums

    packet[7]  = pos_lsb;             // JOG LSB
    packet[8]  = pos_msb;             // JOG MSB
    packet[9]  = 0x04;                // SET : position control, LED off
    packet[10] = servo_id;            // ID du servo ciblé
    packet[11] = playtime;            // playtime (11.2ms * value)

    // Checksum : XOR de SIZE..dernier byte de data, en sautant 5 et 6
    uint8_t cksum1 = 0;
    for (uint8_t i = 2; i < packet_size; i++) {
        if (i == 5 || i == 6) continue;
        cksum1 ^= packet[i];
    }
    cksum1 &= 0xFE;
    packet[5] = cksum1;
    packet[6] = (~cksum1) & 0xFE;

    for (uint8_t j = 0; j < packet_size; j++) {
        uart_send_byte(packet[j]);
    }
}





