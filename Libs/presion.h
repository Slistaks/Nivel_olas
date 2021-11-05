#ifndef PRESION_H_INCLUDED
#define PRESION_H_INCLUDED

#define SENSOR_PRESION_ADDRESS  0x28




/**
 * Estructura usada junto con la funcion PRESION_read_processedData(.) para guardar la media de las muestras, la desviacion y
 * la relacion entre muestras utiles respecto al total de muestras.
 **/
typedef struct media_deviation_relation_pressure{
    float media;
    float desviacion;
    float relation;
}media_deviation_relation_struct_pressure;




/**
 * PRESION_init: Inicializa el sensor de presion.
 * 
 * Parametros:
 *      file: File descriptor al sensor de presion.
 * 
 * Retorna:
 *       0: Exito
 *      -1: Fallo en ioctl
 * 
 **/
int PRESION_init(int file);





/**
 * PRESION_leer_presion_bar: Toma una sola medida de presion, en bar.
 * 
 * Parametros:
 *      presion_p: Puntero a la variable donde se guardara la medida.
 * 
 * Retorna:
 *       0: Exito
 *      -1: Fallo, dispositivo en modo comandos
 *      -3: Loss of sense element connection or short circuit of sense element
 *      -4: Fallo en lectura
 * 
 **/
int PRESION_leer_presion_bar(double* presion_p);







/**
 * PRESION_read_processedData: Toma un set de medidas y calcula la media y la desviacion.
 *                             Descarta las muestras que esten fuera de la banda +-desviacion
 *                             y recalcula. Repite hasta que la desviacion sea menor a la desviacion
 *                             aceptable pasada como parametro o hasta que se hayan descartado todas las muestras.
 * 
 * Parametros:
 *      desviacion_aceptable_bar: La desviacion maxima que se acepta despues de procesar las medidas.
 * 
 *      vectorSize: Cantidad de muestras a tomar.
 * 
 *      salidaStruct_p: Puntero a la estructura donde se guardan la media, desviacion y relacion: (muestras no descartadas / muestras totales)
 * 
 * Retorna:
 *        0: Exito
 *       -6: Fallo en lectura
 *       -9: Fallo en calcularDesviacion
 *      -10: Fallo en calcularMediaUnsigned
 * 
 **/
// presion media medida en bar
int PRESION_read_processedData(float desviacion_aceptable_bar, int vectorSize, struct media_deviation_relation_pressure *salidaStruct_p);



#endif // PRESION_H_INCLUDED
