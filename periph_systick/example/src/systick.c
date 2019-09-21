/*
 * @brief Blinky example using sysTick
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include "nlms.h"
#include "math.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TICKRATE_HZ (1000)	/* 1000 ticks per second */

#define DATOS_SIMULADOS (1)
#define FEEDFORWARD (2)
#define FEEDBACK (3)

#define TICKRATE_10HZ (10)				/* 10 ticks per second */
#define TICKRATE_100HZ (100)			/* 100 ticks per second */
#define TICKRATE_1000HZ (1000)			/* 1000 ticks per second */
#define TICKRATE_HZ1 (TICKRATE_1000HZ)
#define LED_TOGGLE_250MS (250)
#define LED_TOGGLE_500MS (500)
#define LED_TOGGLE_MS1 (LED_TOGGLE_250MS * 1000 / TICKRATE_1000HZ)
#define BUTTON_DEBOUNCE_25MS (25)
#define BUTTON_DEBOUNCE_50MS (50)
#define BUTTON_DEBOUNCE_MS1 (BUTTON_DEBOUNCE_50MS * 1000 / TICKRATE_1000HZ)


#define TEST (FEEDBACK)

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
volatile bool LED_Time_Flag = false;
volatile bool BUTTON_Status_Flag = false;
volatile bool TOGGLE_Sense_Flag = false;
volatile bool BUTTON_Time_Flag = false;
/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

#if (TEST == DATOS_SIMULADOS)

#define LPC_UART LPC_USART2
#define UARTx_IRQn  USART2_IRQn
#define UARTx_IRQHandler UART2_IRQHandler
#define _GPDMA_CONN_UART_Tx GPDMA_CONN_UART2_Tx
#define _GPDMA_CONN_UART_Rx GPDMA_CONN_UART2_Rx


#define BUFFER_LENGTH 10		// 2 char + 2 float
bool msgCompleteFlag;			// flag que avisa que se recibió todo el mensaje
char msgBuffer[BUFFER_LENGTH];

int sendFloat(float2bytes);
int readFloat(float2bytes*);
static void App_Interrupt_Init(void);
//void UARTx_IRQHandler(void);

/*int sendFloat(float2bytes f2send)
{
	printf("%c%c%c%c", f2send.b[0], f2send.b[1], f2send.b[2], f2send.b[3]);
	return 0;
}
*/

int readFloat(float2bytes *f2read)
{
	f2read->b[3] = (char)DEBUGIN();
	f2read->b[2] = (char)DEBUGIN();
	f2read->b[1] = (char)DEBUGIN();
	f2read->b[0] = (char)DEBUGIN();
	return 0;
}


/* Initialize Interrupt for UART */
static void App_Interrupt_Init(void)
{
	/* Enable UART Rx & line status interrupts */
	/*
	 * Do not enable transmit interrupt here, since it is handled by
	 * UART_Send() function, just to reset Tx Interrupt state for the
	 * first time
	 */
	//Chip_UART_IntEnable(DEBUG_UART, (UART_IER_RBRINT | UART_IER_RLSINT));
	Chip_UART_IntEnable(DEBUG_UART, (UART_IER_RBRINT));

	/* Enable Interrupt for UART channel */
	/* Priority = 1 */
	NVIC_SetPriority(UARTx_IRQn, 1);
	/* Enable Interrupt for UART channel */
	NVIC_EnableIRQ(UARTx_IRQn);
}


/**
 * @brief	UART interrupt handler sub-routine
 * @return	Nothing
 */
void UARTx_IRQHandler(void)
{
	static int bufferCounter=0;

	msgBuffer[bufferCounter] = (char)DEBUGIN();		// se puede usar con while(! EOF)
	bufferCounter++;
	if(bufferCounter==BUFFER_LENGTH)
	{
		bufferCounter = 0;
		msgCompleteFlag = true;
	}
}

// TIENE QUE EXISTIR PARA SALIR DE WFI()... PORQUE?????
void SysTick_Handler(void)
{
	LED_Time_Flag = true;
}

/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void)
{
	/* Generic Initialization */
	SystemCoreClockUpdate();
	Board_Init();

	/* Enable and setup SysTick Timer at a periodic rate */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ1);

	// Arrays donde se van a guardar las muestras y los coeficientes del filtro
	float u[Mw];		// referencia de ruido
	float w[Mw];		// filtro adaptivo
	float d;			// señal deseada
	float e;			// error NLMS
	vector_initialize(u, Mw, 0.0);
	vector_initialize(w, Mw, 0.0);

	// identificador del dato de entrada
	char id;

	float2bytes f2b;
	//float float2send = 1.23456;
	//f2b.f = float2send;
	int cont;

	msgCompleteFlag = false;
	App_Interrupt_Init();

	while (1)
	{
		if(msgCompleteFlag==true)
		{
			msgCompleteFlag=false;
			//printf("%c", msgBuffer[0]);
			//printf("eh");
			f2b.b[3] = msgBuffer[1];
			f2b.b[2] = msgBuffer[2];
			f2b.b[1] = msgBuffer[3];
			f2b.b[0] = msgBuffer[4];
			//sendFloat(f2b);
			vector_shift(u, Mw);
			u[0] = f2b.f;


			//printf("%c", msgBuffer[5]);
			//printf("ew");
			f2b.b[3] = msgBuffer[6];
			f2b.b[2] = msgBuffer[7];
			f2b.b[1] = msgBuffer[8];
			f2b.b[0] = msgBuffer[9];
			//sendFloat(f2b);
			d = f2b.f;

			// Ejecuto el NLMS
			e = nlms(d, u, w, Mw, 0.005, 1e-5);

			// mando los resultados
			f2b.f = e;
			printf("ew");
			sendFloat(f2b);
			printf("tw");
			for(cont=0; cont<Mw; cont++){
				f2b.f = w[cont];
				sendFloat(f2b);
			}
		}
/*
		id = (char)DEBUGIN();
		//printf("%c ", id);
		if(id=='u')
		{
			readFloat(&f2b);
		}
		printf("eh");
		sendFloat(f2b);

		id = (char)DEBUGIN();
		//printf("%c ", id);
		if(id=='d')
		{
			readFloat(&f2b);
		}
		printf("ew");
		sendFloat(f2b);
*/
/*		printf("th");
		for(cont=0; cont<Mh; cont++){
			sendFloat(f2b);
		}
		printf("ew");
		sendFloat(f2b);
		printf("tw");
		for(cont=0; cont<Mw; cont++){
			sendFloat(f2b);
		}
*/

	}
}
#endif

#if (TEST == FEEDFORWARD)
/*
// ADC
#define ADC_D LPC_ADC0
#define ADC_U LPC_ADC1
#define CANAL_ADC_D ADC_CH2
#define CANAL_ADC_U ADC_CH3
#define _GPDMA_CONN_ADC_D GPDMA_CONN_ADC_0
#define _GPDMA_CONN_ADC_U GPDMA_CONN_ADC_1
*/
#define _ADC_CHANNLE ADC_CH2
#define _LPC_ADC_ID LPC_ADC0
#define _LPC_ADC_IRQ ADC0_IRQn
#define _GPDMA_CONN_ADC GPDMA_CONN_ADC_0
#define _ADC_CHANNLE2 ADC_CH3
#define _LPC_ADC_ID2 LPC_ADC1
#define _GPDMA_CONN_ADC2 GPDMA_CONN_ADC_1

/* The APB clock (PCLK_ADC0) is divided by (CLKDIV+1) to produce the clock for
   A/D converter, which should be less than or equal to 4.5MHz.
   A fully conversion requires (bits_accuracy+1) of these clocks.
   ADC Clock = PCLK_ADC0 / (CLKDIV + 1);
   ADC rate = ADC clock / (the number of clocks required for each conversion);
 */
#define V_REF (3.3)
#define V_MAX (V_REF/2)
#define V_MIN (-V_REF/2)
#define ADC2VOLTS (3.3/1024)
#define VOLTS2DAC (1024/3.3)
#define CH2_OFFSET (511)		// Offset de los conversores AD
#define CH3_OFFSET (511)

static ADC_CLOCK_SETUP_T ADCSetup, ADCSetup2;
static volatile uint8_t Burst_Mode_Flag = 0, Interrupt_Continue_Flag;
static volatile uint8_t ADC_Interrupt_Done_Flag, channelTC, dmaChannelNum;
static volatile uint8_t channelTC2, dmaChannelNum2;
#define F_SAMPLE (100000)
#define N_SAMPLES (100)
#define N_OFFSET (20000)
#define N_IDENTIFICACION (10000)
uint32_t DMAbuffer[N_SAMPLES];
uint32_t DMAbuffer2[N_SAMPLES];

/**
 * @brief	DMA interrupt handler sub-routine
 * @return	Nothing
 */
void DMA_IRQHandler(void)
{
	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaChannelNum) == SUCCESS) {
		channelTC++;
	}
	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaChannelNum2) == SUCCESS) {
		channelTC2++;
	}
	else {
		/* Process error here */
	}
}

/*
    Generador de ruido blanco obtenido de
    https://os.mbed.com/questions/2886/Why-is-the-rand-function-not-the-least-b/
    rand() genera una secuencia repetitiva que se manifestaba como tonos
*/
#define MAX_RND (0xffffffff)
uint32_t m_z=12434, m_w=33254;
uint32_t rnd()
{
    m_z = 36969 * (m_z & 65535) + (m_z >>16);
    m_w = 18000 * (m_w & 65535) + (m_w >>16);
    return ((m_z <<16) + m_w);
}


/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void)
{
	uint32_t dataDAC=0;
	uint32_t dataADC=0;
	uint32_t dataADC2=0;
	float dataADC0_Offset=0.0;
	float dataADC1_Offset=0.0;
	uint8_t cont=0;
	uint16_t cont_uart=0;
	uint32_t cont_nlms=0;
	uint32_t _bitRate = F_SAMPLE;

	// Arrays donde se van a guardar las muestras y los coeficientes del filtro
	// Declaro volatile porque sino se pisan las posiciones de memoria
	// dependiendo del orden de las operaciones
	float u[Mh];			// referencia de ruido
	float h[Mh];			// respuesta del camino secundario (parlante-mic de error)
	float w[Mw];			// respuesta del camino primario (fuentede ruido-mic de error)
	float uw[Mw];			// ruido de referencia filtrado por h ṕara Filtered-X NLMS
	float d;				// señal deseada
	float e;				// error NLMS
	float mu;				// paso del algoritmo NLMS
	float u2 = 0.0;			// módulo al cuadrado de u
	vector_initialize(u, Mh, 0.0);
	vector_initialize(uw, Mw, 0.0);
	vector_initialize(h, Mh, 0.0);
	vector_initialize(w, Mw, 0.0);
	float2bytes f2b;	// estructura para convertir floats a char para enviar

	SystemCoreClockUpdate();
	Board_Init();
	Chip_UART_SetBaudFDR(DEBUG_UART, 921600);
	Board_DAC_Init(LPC_DAC);
	Board_ADC_Init();

	/* DAC */
	Chip_DAC_Init(LPC_DAC);
	/* set time out for DAC*/
	Chip_DAC_SetDMATimeOut(LPC_DAC, 0xFFFF);
	Chip_DAC_ConfigDAConverterControl(LPC_DAC, (DAC_CNT_ENA | DAC_DMA_ENA));

	/* ADC */
	Chip_ADC_Init(_LPC_ADC_ID, &ADCSetup);
	Chip_ADC_Init(_LPC_ADC_ID2, &ADCSetup2);
	Chip_ADC_EnableChannel(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);
	Chip_ADC_EnableChannel(_LPC_ADC_ID2, _ADC_CHANNLE2, ENABLE);
	ADCSetup.burstMode = true;
	ADCSetup2.burstMode = true;
	Chip_ADC_SetSampleRate(_LPC_ADC_ID, &ADCSetup, _bitRate);
	Chip_ADC_SetSampleRate(_LPC_ADC_ID2, &ADCSetup2, _bitRate);


	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);
	/* Setting GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);
	Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);
	Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID2, _ADC_CHANNLE2, ENABLE);
	/* Get the free channel for DMA transfer */
	dmaChannelNum = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, _GPDMA_CONN_ADC);
	dmaChannelNum2 = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, _GPDMA_CONN_ADC2);

	/* Enable burst mode if any, the AD converter does repeated conversions
	   at the rate selected by the CLKS field in burst mode automatically */
	Chip_ADC_SetBurstCmd(LPC_ADC0, ENABLE);
	Chip_ADC_SetBurstCmd(LPC_ADC1, ENABLE);

	/*******************************************************
	 * Calcula el offset de los canales ADC
	 *******************************************************/

	for(cont_nlms=0; cont_nlms<N_OFFSET; cont_nlms++)
	{
		channelTC = 0;
		channelTC2 = 0;
		Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum,
						  _GPDMA_CONN_ADC,
						  (uint32_t) &DMAbuffer,
						  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
						  N_SAMPLES);
		Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum2,
								  _GPDMA_CONN_ADC2,
								  (uint32_t) &DMAbuffer2,
								  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
								  N_SAMPLES);
		/* Waiting for reading ADC value completed */
		while ((channelTC == 0) && (channelTC2 == 0)) {}

		/* Get the ADC value fron Data register*/
		for(cont=0; cont<N_SAMPLES; cont++){
			dataADC += ADC_DR_RESULT(DMAbuffer[cont]);
			dataADC2 += ADC_DR_RESULT(DMAbuffer2[cont]);
		}
		dataADC0_Offset += (float)((int32_t)dataADC)*ADC2VOLTS/N_SAMPLES;
		dataADC1_Offset += (float)((int32_t)dataADC2)*ADC2VOLTS/N_SAMPLES;
		dataADC = 0;
		dataADC2 = 0;
	}
	dataADC0_Offset = dataADC0_Offset/N_OFFSET-V_MAX;
	dataADC1_Offset = dataADC1_Offset/N_OFFSET-V_MAX;

	/****************************************************************
	 * Fin Cálculo de Offset de canales ADC
	 ****************************************************************/


	/****************************************************************
	 * Identificación H(z)
	 ****************************************************************/
	for (cont_nlms=1; cont_nlms<N_IDENTIFICACION; cont_nlms++)
	{
		// Genero ruido blanco
		vector_shift(u, Mh);
		u[0] = V_REF*(((float)rnd())/MAX_RND-0.5);

		// Ruido al DAC
		dataDAC = (uint32_t)(u[0]*VOLTS2DAC+CH3_OFFSET);
		Chip_DAC_UpdateValue(LPC_DAC, dataDAC);

		// Espero datos del ADC por DMA
		channelTC = 0;
		Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum,
						  _GPDMA_CONN_ADC,
						  (uint32_t) &DMAbuffer,
						  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
						  N_SAMPLES);
		/* Waiting for reading ADC value completed */
		while (channelTC == 0) {}

		/* Get the ADC value fron Data register*/
		for(cont=0; cont<N_SAMPLES; cont++){
			dataADC += ADC_DR_RESULT(DMAbuffer[cont]);
		}
		d = (float)((int32_t)dataADC-CH2_OFFSET*N_SAMPLES)*ADC2VOLTS/N_SAMPLES-dataADC0_Offset;
		dataADC = 0;

		// Algoritmo NLMS
		u2 += u[0]*u[0];
		e = d-vector_dot(h, u, Mh);         			// calcula e = d - w'*u
		vector_add_scaled(h, u, Mh, MU_H*e/(1.0e-7+u2));  // calcula w = w + mu*e/(a + ||u||^2) * u
		u2 -= u[Mh-1]*u[Mh-1];

		// Envío algunos datos cada tanto
		f2b.f = e;
		printf("eh");
		sendFloat(f2b);

/*		if(cont_uart==10){
			cont_uart=0;
			f2b.f = e;
			printf("eh");
			sendFloat(f2b);
			f2b.f = u[0];
			printf("ew");
			sendFloat(f2b);
		}
		else{
			cont_uart++;
		}
*/
	}

	// Mando la respuesta al impulso estimada
	printf("th");
	for(cont_uart=0; cont_uart<Mh; cont_uart++){
		f2b.f = h[cont_uart];
		sendFloat(f2b);
	}

	/**************************************************************
	 * Fin Identificación H(z)
	 **************************************************************/

	// Delay
	for(cont_nlms=0; cont_nlms<10000000; cont_nlms++){ }


	/**************************************************************
	 * Cancelación de ruido
	 **************************************************************/
	vector_initialize(u, Mh, 0.0);	// reseteo u para empezar a cargar el ruido
	mu = MU;
	volatile float y = 0.0;			// dato que va al parlante
	volatile float uw2 = 0.0;		// norma al cuadrado de uw
	for(cont_nlms=1; cont_nlms<100000; cont_nlms++)
	{
		channelTC = 0;
		channelTC2 = 0;
		Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum,
						  _GPDMA_CONN_ADC,
						  (uint32_t) &DMAbuffer,
						  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
						  N_SAMPLES);
		Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum2,
								  _GPDMA_CONN_ADC2,
								  (uint32_t) &DMAbuffer2,
								  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
								  N_SAMPLES);
		/* Waiting for reading ADC value completed */
		while ((channelTC == 0) && (channelTC2 == 0)) {}

		/* Get the ADC value fron Data register*/
		for(cont=0; cont<N_SAMPLES; cont++){
			dataADC += ADC_DR_RESULT(DMAbuffer[cont]);
			dataADC2 += ADC_DR_RESULT(DMAbuffer2[cont]);
		}
		// Señal de referencia
		u[0] = (float)((int32_t)dataADC2-CH3_OFFSET*N_SAMPLES)*ADC2VOLTS/N_SAMPLES-dataADC1_Offset;
		// genero la salida y
		y = vector_dot(u, w, Mw);
		dataDAC = (int32_t)(y*VOLTS2DAC+511);
		if(dataDAC<0){dataDAC=0;}
		else if(dataDAC>1023){dataDAC=1023;}
		Chip_DAC_UpdateValue(LPC_DAC, (uint32_t)dataDAC);

		// Señal de error
		e = (float)((int32_t)dataADC-CH2_OFFSET*N_SAMPLES)*ADC2VOLTS/N_SAMPLES-dataADC0_Offset;
		dataADC = 0;
		dataADC2 = 0;

		// paso la referencia u por h antes de mandarlo al NLMS
		uw[0] = vector_dot(u, h, Mh);

		// Algoritmo NLMS
		uw2 += uw[0]*uw[0];
		vector_scale(w, Mw, 1.0-MU*GAMMA);
		vector_add_scaled(w, uw, Mw, mu*e/(A+uw2));  // calcula w = w + mu*e/(a + ||u||^2) * u
		uw2 -= uw[Mw-1]*uw[Mw-1];

		vector_shift(u, Mh);
		vector_shift(uw, Mw);

		// Envío algunos datos cada tanto
		f2b.f = e;
		printf("eh");
		sendFloat(f2b);
		f2b.f = y;
		printf("ew");
		sendFloat(f2b);

/*		if(cont_uart==10){
			cont_uart=0;
			f2b.f = e;
			printf("eh");
			sendFloat(f2b);
			f2b.f = u[0];
			printf("ew");
			sendFloat(f2b);
		}
		else{
			cont_uart++;
		}
*/
	}

	// Mando la respuesta al impulso estimada
	printf("tw");
	for(cont_uart=0; cont_uart<Mw; cont_uart++){
		f2b.f = w[cont_uart];
		sendFloat(f2b);
	}



	/* Disable interrupts, release DMA channel */
	Chip_GPDMA_Stop(LPC_GPDMA, dmaChannelNum);
	Chip_GPDMA_Stop(LPC_GPDMA, dmaChannelNum2);
	NVIC_DisableIRQ(DMA_IRQn);
	/* Disable burst mode if any */
	Chip_ADC_SetBurstCmd(LPC_ADC0, DISABLE);
	Chip_ADC_SetBurstCmd(LPC_ADC1, DISABLE);

	return 0;
}
#endif


#if (TEST == FEEDBACK)
/*
// ADC
#define ADC_D LPC_ADC0
#define ADC_U LPC_ADC1
#define CANAL_ADC_D ADC_CH2
#define CANAL_ADC_U ADC_CH3
#define _GPDMA_CONN_ADC_D GPDMA_CONN_ADC_0
#define _GPDMA_CONN_ADC_U GPDMA_CONN_ADC_1
*/
#define _ADC_CHANNLE ADC_CH2
#define _LPC_ADC_ID LPC_ADC0
#define _LPC_ADC_IRQ ADC0_IRQn
#define _GPDMA_CONN_ADC GPDMA_CONN_ADC_0

/* The APB clock (PCLK_ADC0) is divided by (CLKDIV+1) to produce the clock for
   A/D converter, which should be less than or equal to 4.5MHz.
   A fully conversion requires (bits_accuracy+1) of these clocks.
   ADC Clock = PCLK_ADC0 / (CLKDIV + 1);
   ADC rate = ADC clock / (the number of clocks required for each conversion);
 */
#define V_REF (3.3)
#define V_MAX (V_REF/2)
#define V_MIN (-V_REF/2)
#define ADC2VOLTS (3.3/1024)
#define VOLTS2DAC (1024/3.3)

static ADC_CLOCK_SETUP_T ADCSetup;
static volatile uint8_t ADC_Interrupt_Done_Flag, channelTC, dmaChannelNum;
#define F_SAMPLE (100000)
#define N_SAMPLES (100)
#define N_OFFSET (20000)
#define N_IDENTIFICACION (10000)
volatile uint32_t DMAbuffer[N_SAMPLES];

/**
 * @brief	DMA interrupt handler sub-routine
 * @return	Nothing
 */
void DMA_IRQHandler(void)
{
	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaChannelNum) == SUCCESS) {
		channelTC++;
	}
	else {
		/* Process error here */
	}
}

/*
    Generador de ruido blanco obtenido de
    https://os.mbed.com/questions/2886/Why-is-the-rand-function-not-the-least-b/
    rand() genera una secuencia repetitiva que se manifestaba como tonos
*/
#define MAX_RND (0xffffffff)
uint32_t m_z=12434, m_w=33254;
uint32_t rnd()
{
    m_z = 36969 * (m_z & 65535) + (m_z >>16);
    m_w = 18000 * (m_w & 65535) + (m_w >>16);
    return ((m_z <<16) + m_w);
}


/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void)
{
	uint32_t dataDAC=0;
	uint32_t dataADC=0;
	volatile float dataADC0_Offset=0.0;
	uint8_t cont=0;
	uint16_t cont_uart=0;
	uint32_t cont_nlms=0;
	uint32_t _bitRate = F_SAMPLE;

	// Arrays donde se van a guardar las muestras y los coeficientes del filtro
	// Declaro volatile porque sino se pisan las posiciones de memoria
	// dependiendo del orden de las operaciones
	volatile float u[Mh];			// referencia de ruido
	volatile float h[Mh];			// respuesta del camino secundario (parlante-mic de error)
	volatile float w[Mw];			// respuesta del camino primario (fuentede ruido-mic de error)
	volatile float uw[Mw];			// ruido de referencia filtrado por h ṕara Filtered-X NLMS
	volatile float y[Mh];			// vector de salidas
	volatile float d;				// señal deseada
	volatile float e;				// error NLMS
	volatile float u2 = 0.0;			// módulo al cuadrado de u
	vector_initialize(u, Mh, 0.0);
	vector_initialize(y, Mh, 0.0);
	vector_initialize(h, Mh, 0.0);
	vector_initialize(uw, Mw, 0.0);
	vector_initialize(w, Mw, 0.0);
	float2bytes f2b;	// estructura para convertir floats a char para enviar

	SystemCoreClockUpdate();
	Board_Init();
	Chip_UART_SetBaudFDR(DEBUG_UART, 921600);
	Board_DAC_Init(LPC_DAC);
	Board_ADC_Init();

	/* DAC */
	Chip_DAC_Init(LPC_DAC);
	/* set time out for DAC*/
	Chip_DAC_SetDMATimeOut(LPC_DAC, 0xFFFF);
	Chip_DAC_ConfigDAConverterControl(LPC_DAC, (DAC_CNT_ENA | DAC_DMA_ENA));

	/* ADC */
	Chip_ADC_Init(_LPC_ADC_ID, &ADCSetup);
	Chip_ADC_EnableChannel(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);
	ADCSetup.burstMode = true;
	Chip_ADC_SetSampleRate(_LPC_ADC_ID, &ADCSetup, _bitRate);

	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);
	/* Setting GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);
	Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);
	/* Get the free channel for DMA transfer */
	dmaChannelNum = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, _GPDMA_CONN_ADC);

	/* Enable burst mode if any, the AD converter does repeated conversions
	   at the rate selected by the CLKS field in burst mode automatically */
	Chip_ADC_SetBurstCmd(LPC_ADC0, ENABLE);

	/*******************************************************
	 * Calcula el offset de los canales ADC
	 *******************************************************/

	for(cont_nlms=0; cont_nlms<N_OFFSET; cont_nlms++)
	{
		channelTC = 0;
		Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum,
						  _GPDMA_CONN_ADC,
						  (uint32_t) &DMAbuffer,
						  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
						  N_SAMPLES);
		/* Waiting for reading ADC value completed */
		while (channelTC == 0) {}

		/* Get the ADC value fron Data register*/
		for(cont=0; cont<N_SAMPLES; cont++){
			dataADC += ADC_DR_RESULT(DMAbuffer[cont]);
		}
		dataADC0_Offset += (float)((int32_t)dataADC)*ADC2VOLTS/N_SAMPLES;
		dataADC = 0;
	}
	dataADC0_Offset = dataADC0_Offset/N_OFFSET-V_MAX;

	/****************************************************************
	 * Fin Cálculo de Offset de canales ADC
	 ****************************************************************/


	/****************************************************************
	 * Identificación H(z)
	 ****************************************************************/
	channelTC = 0;
	Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum,
				  _GPDMA_CONN_ADC,
				  (uint32_t) &DMAbuffer,
				  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
				  N_SAMPLES);
	for (cont_nlms=1; cont_nlms<N_IDENTIFICACION; cont_nlms++)
	{
		// Genero ruido blanco
		vector_shift(u, Mh);
		u[0] = V_REF*(((float)rnd())/MAX_RND-0.5);

		// Ruido al DAC
		dataDAC = (uint32_t)(u[0]*VOLTS2DAC+511);
		Chip_DAC_UpdateValue(LPC_DAC, dataDAC);

		/* Waiting for reading ADC value completed */
		while (channelTC == 0) {}

		/* Get the ADC value fron Data register*/
		for(cont=0; cont<N_SAMPLES; cont++){
			dataADC += ADC_DR_RESULT(DMAbuffer[cont]);
		}
		// Reactivo DMA para que cargue datos mientras proceso
		channelTC = 0;
		Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum,
					  _GPDMA_CONN_ADC,
					  (uint32_t) &DMAbuffer,
					  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
					  N_SAMPLES);

		d = (float)((int32_t)dataADC-511*N_SAMPLES)*ADC2VOLTS/N_SAMPLES-dataADC0_Offset;
		dataADC = 0;

		// Algoritmo NLMS
		u2 += u[0]*u[0];
		e = d-vector_dot(h, u, Mh);         			// calcula e = d - w'*u
		vector_add_scaled(h, u, Mh, MU_H*e/(1.0e-7+u2));  // calcula w = w + mu*e/(a + ||u||^2) * u
		u2 -= u[Mh-1]*u[Mh-1];

		// Envío algunos datos cada tanto
		f2b.f = e;
		printf("eh");
		sendFloat(f2b);

/*		if(cont_uart==10){
			cont_uart=0;
			f2b.f = e;
			printf("eh");
			sendFloat(f2b);
			f2b.f = u[0];
			printf("ew");
			sendFloat(f2b);
		}
		else{
			cont_uart++;
		}
*/
	}

	// Mando la respuesta al impulso estimada
	printf("th");
	for(cont_uart=0; cont_uart<Mh; cont_uart++){
		f2b.f = h[cont_uart];
		sendFloat(f2b);
	}

	/**************************************************************
	 * Fin Identificación H(z)
	 **************************************************************/

	// Delay
	for(cont_nlms=0; cont_nlms<10000000; cont_nlms++){ }

	/**************************************************************
	 * Cancelación de ruido
	 **************************************************************/
	vector_initialize(u, Mh, 0.0);	// reseteo u para empezar a cargar el ruido
	uw[0]=0.0;
	volatile float uw2 = 0.0;		// norma al cuadrado de uw
	//for(cont_nlms=1; cont_nlms<300000; cont_nlms++)
	cont_nlms=9;

	channelTC = 0;
	Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum,
				  _GPDMA_CONN_ADC,
				  (uint32_t) &DMAbuffer,
				  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
				  N_SAMPLES);

	while(1)
	{
		cont_nlms++;
		/* Waiting for reading ADC value completed */
		while (channelTC == 0)
		{
			//printf("00");
		}

		/* Get the ADC value fron Data register*/
		for(cont=0; cont<N_SAMPLES; cont++){
			dataADC += ADC_DR_RESULT(DMAbuffer[cont]);
		}

		// Vuelvo a activar DMA para que cargue muestras mientras proceso
		channelTC = 0;
		Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum,
				 _GPDMA_CONN_ADC,
				 (uint32_t) &DMAbuffer,
				 GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
				 N_SAMPLES);

		// genero la salida y
		vector_shift(y,Mh);
		y[0] = -vector_dot(u, w, Mw);
		if(y[0]>V_MAX){y[0]=V_MAX;}
		else if(y[0]<V_MIN){y[0]=V_MIN;}
		// Anti-ruido al DAC
		dataDAC = (int32_t)(y[0]*VOLTS2DAC+511);
		if(dataDAC<0){dataDAC=0;}
		else if(dataDAC>1023){dataDAC=1023;}
		Chip_DAC_UpdateValue(LPC_DAC, (uint32_t)dataDAC);

		// Señal de referencia u = e-y*ḧ
		e = -(float)((int32_t)dataADC-511*N_SAMPLES)*ADC2VOLTS/N_SAMPLES-dataADC0_Offset;
		dataADC = 0;
		vector_shift(u, Mh);
		u[0] = e-vector_dot(y,h,Mh);

		// Algoritmo NLMS
		// paso la referencia u por h antes de mandarlo al NLMS
		vector_shift(uw, Mw);
		uw[0] = vector_dot(u, h, Mh);
		uw2 += uw[0]*uw[0];
		//vector_scale(w, Mw, 1.0-NU);					// factor de olvido
		//vector_add_scaled(w, uw, Mw, MU_NLMS*e/(A+uw2));  // calcula w = w + mu*e/(a + ||u||^2) * uw
		vector_add_scaled(w, uw, Mw, MU*e);  // calcula w = w + mu*e * uw
		uw2 -= uw[Mw-1]*uw[Mw-1];


		// Envío algunos datos cada tanto
		// envío el error
		f2b.f = e;
		printf("eh");
		sendFloat(f2b);
		// envío el control
		f2b.f = y[0];
		printf("ew");
		sendFloat(f2b);
		// envío un coeficiente de w
		f2b.f = w[(cont_nlms-1)%Mw];
		printf("tw");
		printf("%c", (cont_nlms-1)%Mw);
		sendFloat(f2b);

	}


	/* Disable interrupts, release DMA channel */
	Chip_GPDMA_Stop(LPC_GPDMA, dmaChannelNum);
	NVIC_DisableIRQ(DMA_IRQn);
	/* Disable burst mode if any */
	Chip_ADC_SetBurstCmd(LPC_ADC0, DISABLE);

	return 0;
}
#endif
