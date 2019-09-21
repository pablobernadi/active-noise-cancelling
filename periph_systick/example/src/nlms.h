#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


#define Mh (256)
#define Mw (64)
#define MU_H (0.05)
#define MU (0.000005)			// 75Hz
//#define MU (0.00025)			// 75Hz
#define MU_NLMS (0.00003)		// No funciona
#define NU (0.0)				// 0 < GAMMA < 1/MU
#define A (1.0)

typedef union
{
	volatile float f;
	char b[4];
} float2bytes;

int vector_initialize(volatile float *, uint32_t, volatile float);                // inicializa un vector en ceros
int vector_copy(volatile float *, volatile float *, uint32_t);                    // copia dos vectores
volatile float vector_dot(volatile float *, volatile float *, uint32_t);                   // producto interno entre dos vectores del mismo tamaño
int vector_add(volatile float *, volatile float *, uint32_t);                     // suma dos vectores
int vector_scale(volatile float *, uint32_t, volatile float);                     // multiplica un vector por un escalar
int vector_shift(volatile float *, uint32_t);                            // desplaza todos los elementos de un vector
int vector_add_scaled(volatile float *, volatile float *, uint32_t, volatile float);   // suma dos vectores, el segundo escalado
volatile float nlms(volatile float, volatile float *, volatile float*, uint32_t, volatile float, volatile float);     // algoritmo NLMS
float rng(void);                                                // genera ruido blanco uniforme
int sendFloat(float2bytes);
