
//#define debugSemohosting

#define mainQUEUE_LENGTH          ( 10 )

#define TicksPerMS configTICK_RATE_HZ/1000

#define PCONP_PCTIM1    ((uint32_t)(1<<2))
#define PCONP_PCSPI     ((uint32_t)(1<<8))
#define PCONP_PCADC     ((uint32_t)(1<<12))
#define PCONP_PCGPIO    ((uint32_t)(1<<15))

#define SPCR_MSTR       ((uint32_t)(1<<5))

#define PCLK_TIMER1(n)  ((uint32_t)(n<<4))
#define PCLK_SPI(n)     ((uint32_t)(n<<16))
#define PCLK_ADC(n)     ((uint32_t)(n<<24))

#define CCLK_DIV4  ((uint32_t)(0))
#define CCLK_DIV1  ((uint32_t)(1))
#define CCLK_DIV2  ((uint32_t)(2))
#define CCLK_DIV8  ((uint32_t)(3))

#define PORT_0 ((0)) /**< PORT 0*/
#define PORT_1 ((1)) /**< PORT 1*/
#define PORT_2 ((2)) /**< PORT 2*/
#define PORT_3 ((3)) /**< PORT 3*/
#define PORT_4 ((4)) /**< PORT 4*/

#define FUNC_0 ((0)) /**< default function */
#define FUNC_1 ((1)) /**< first alternate function */
#define FUNC_2 ((2)) /**< second alternate function */
#define FUNC_3 ((3)) /**< third or reserved alternate function */

#define PINMODE_PULLUP ((0)) /**< Internal pull-up resistor */
#define PINMODE_TRISTATE ((2)) /**< Tri-state */
#define PINMODE_PULLDOWN ((3)) /**< Internal pull-down resistor */
#define PINMODE_NORMAL ((0)) /**< Pin is in the normal (not open drain) mode */
#define PINMODE_OPENDRAIN ((1)) /**< Pin is in the open drain mode */


#define PIN(n) ((uint32_t)(1 << (n)))
#define BIT(n) ((uint32_t)(1 << (n)))


/* Advanced define with if...
#define DATABIT_15 SPI_SPCR_BITS(0x0F)
#define SPCR_BITS(n) ((n==0) ? ((uint32_t)0) : ((uint32_t)((n&0x0F)<<8)))

#pragma message ("Warning goes here")

#if (SETTING_A == 0) && (SETTING_B == 0)
#error SETTING_A and SETTING_B can't both be 0!
#endif

*/

#define KEY0 		0
#define KEY1 		1
#define KEY2 		2
#define KEY3 		3
#define KEY4 		4
#define KEY5 		5
#define KEY6		6
#define KEY7 		7
#define KEY8		8
#define KEY9 		9
#define KEYHOME 	10
#define KEYUP 		22
#define KEYDOWN		21
#define KEYBACK 	91
#define KEYOK 		90
#define KEYSTART	99
#define KEYPWR 		100
#define KEYSTOP		199
