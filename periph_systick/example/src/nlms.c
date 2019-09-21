#include "nlms.h"

/*
    inicializa un vector en ceros
*/
int vector_initialize(volatile float* v, uint32_t M, volatile float c)
{
    uint32_t i;
    for(i=0; i<M; i++)
    {
        v[i] = c;
    }
    return 0;
}

/*
    copia dos vectores (copia v2 a v1)
*/
int vector_copy(volatile float* v1, volatile float* v2, uint32_t M)
{
    uint32_t i;
    for(i=0; i<M; i++)
    {
        v1[i] = v2[i];
    }
    return 0;
}

/*
    Producto interno de dos vectores
*/
volatile float vector_dot(volatile float* v1, volatile float* v2, uint32_t M)
{
    uint32_t i;
    volatile float r = 0.0;
    for(i=0; i<M; i++)
    {
        r += v1[i]*v2[i];
    }
    return r;
}

/*
    Suma dos vectores (resultado queda en v1)
*/
int vector_add(volatile float* v1, volatile float* v2, uint32_t M)
{
    uint32_t i;
    for(i=0; i<M; i++)
    {
        v1[i] += v2[i];
    }
    return 0;
}

/*
    Escala un vector
*/
int vector_scale(volatile float* v, uint32_t M, volatile float c)
{
    uint32_t i;
    for(i=0; i<M; i++)
    {
        v[i] *= c;
    }
    return 0;
}

/*
    Desplaza un vector (libera posición 0)
*/
int vector_shift(volatile float* v, uint32_t M)
{
    uint32_t i;
    for(i=(M-1); i>0; i--)
    {
        v[i] = v[i-1];
    }
    return 0;
}

/*
 * Suma dos vectores, el segundo escalado, de esta forma se evita hacer una copia
 * de un vector para escalarlo y sumarlo (como se hacía usando vector_scale y vector_add)
 */

int vector_add_scaled(volatile float *v1, volatile float *v2, uint32_t M, volatile float c)
{
    uint32_t i;
    for(i=0; i<M; i++)
    {
        v1[i] += (v2[i]*c);
    }
    return 0;
}


/*
    Algoritmo NLMS
    - (YA SE CORRIGIO CON vector_add_scaled) pasar una copia de u[n] porque lo modifica!
*/
volatile float nlms(volatile float d, volatile float* u, volatile float* w, uint32_t M, volatile float mu, volatile float a)
{
	//static float u2 = 0.0;
	//u2 += u[0]*u[0];
	volatile float e = d - vector_dot(w, u, M);         // calcula e = d - w'*u
	volatile float c = mu*e/(a+vector_dot(u, u, M));    // calcula c = mu*e/(a + ||u||^2)
    //float c = mu*e/(a+u2);
	vector_add_scaled(w, u, M, c);             // calcula w = w + mu*e/(a + ||u||^2) * u
    //u2 -= u[M]*u[M];
    return e;
}


/*
 * Envía un float por puerto serie usando la estructura definida en nlms.h
 */
int sendFloat(float2bytes f2send)
{
	printf("%c%c%c%c", f2send.b[0], f2send.b[1], f2send.b[2], f2send.b[3]);
	return 0;
}
