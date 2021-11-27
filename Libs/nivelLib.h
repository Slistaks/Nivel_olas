#ifndef NIVELLIB_H_INCLUDED
#define NIVELLIB_H_INCLUDED


#define FDC1004ADDRESS 0x50

enum sample_rate{
    CIEN_Ss=0x400,
    DOSCIENTAS_Ss=0x800,
    CUATROCIENTAS_Ss=0xC00
};



enum pinCIN{
    pinCIN1= 0,
    pinCIN2= 1,
    pinCIN3= 2,
    pinCIN4= 3
};



/// Direcciones de los registros MSB donde se almacenan los resultados de las mediciones:
enum reg_resultado{
    reg_RESULTADO_NIVEL= 0,                        // Uso el MEAS1
    reg_RESULTADO_DIFERENCIAL=2,                   // Uso el MEAS2
    reg_RESULTADO_MEAS3= 4,                        // MEAS3
    reg_RESULTADO_MEAS4= 6                        // MEAS4
};




enum tipo_medida{
    medidaNIVEL= 8,             // lo uso como mascara en funcion done. Nivel, registro meas1.
    medidaDIFERENCIAL= 4,       // diferencial, registro meas2.
    medidaMEAS3= 2,
    medidaMEAS4= 1
}tipoMedida;



/**
 * Estructura usada junto con la funcion read_processedData_mm(.) para guardar la media de las muestras y
 * la relacion entre muestras utiles respecto al total de muestras.
 **/
typedef struct media_confiabilidad_nivel_struct {

    float media;
    float confiabilidad;// relacion entre cantidad de muestras utiles y cantidad de muestras totales.

}media_confiabilidad_nivel;




//
//
//
//
//
//
//
//
//
int capacidad_autooffset(float* capacidad, enum tipo_medida tipoMedida);    // modif del orig. agregue tipoMedida.




//
//
//
//
//
//
//
//
int multiMedidasEnable(unsigned char enable);




//
//
//
//
//
//
//
//
int MEASn_capdac_config(int capdac_offset, enum tipo_medida tipoMedida);



//
//
//
//
//
//
//
//
int MEASn_capdac_config_pin(int capdac_offset, enum pinCIN pin);







// capacimeter_init: Inicializa y configura el integrado fdc1004.
//
// Parametros:
//      file_p			: Puntero al bus I2C.
//		capdac_offset	: Offset de capacidad, 3.125pF por unidad. 5 bits (maximo offset: 96.875pF)
//		sampleRate 		: Tasa de muestreo, posibles: 100, 200 y 400 muestras por segundo. Usar enume, ver nivelLib.h.
// Retorna:
//      0 : No hubo error.
//     -1 : Fallo ioctl
//     -2 : Fallo capacimeter_config 
int capacimeter_init(int* file_p, int capdac_offset, enum sample_rate sampleRate);





// capacimeter_config: Configura el offset del capdac y la tasa de muestreo. Se puede usar en cualquier momento
//					   en que se quiera cambiar esos atributos.
//
// Parametros:
//		capdac_offset: El offset a restar a la capacidad a medir. El offset en pF es capdac_offset*3.125pF.
//		sampleRate   : La tasa de muestreo. Posibles: 100, 200 y 400 muestras por segundo. Usar enum de nivelLib.h.
// Retorna:
//		terminar
//int capacimeter_config(int capdac_offset, enum sample_rate sampleRate, enum tipo_medida tipoMedida);  //original
int capacimeter_config(enum sample_rate sampleRate, enum tipo_medida tipoMedida);





// capacidad_medida_single: Toma una sola medida de capacidad en pF.
//
// VER "NOTA 2" EN SU DEFINICION. En funcion de comentar o no una linea, se puede obtener como salida la
//								  capacidad absoluta, o la capacidad relativa al offset.
//
// Parametros:
//		cap: Puntero a la variable donde se guardara la capacidad en pF.
//
// Retorna:
//		terminar.
int capacidad_medida_single(float* cap, enum tipo_medida tipoMedida);






// Realiza una cantidad <vectorSize> de mediciones, calcula la media y la desviacion estandar muestral, si esta ultima
// es mayor a un valor que recibe como parametro <desviacion_aceptable_pF>, elimina todas las muestras que se dispersan mas que
// una desviacion estandar muestral, y recalcula la media y la desviacion. Repite el procedimiento hasta
// que la desviacion sea menor que el valor maximo que recibe o se hayan descartado todas las muestras. Luego, convierte la media
// a pF y calcula la relacion entre muestras utiles (las que no fueron descartadas) y muestras totales (la cantidad
// inicial de muestras) y guarda esos datos en una estructura <*salidaStruct_p>.
//
// VER "NOTA 1" EN SU DEFINICION. En funcion de comentar o no una linea, se puede obtener como salida la
//								  capacidad absoluta, o la capacidad relativa al offset.
//
//
// Retorna:
//       0 : Exito
//      -7 : Fallo capacimeter_read
//      -8 : Fallo reset
//      -9 : Fallo capacimeter_done
//      -10: Flag conversion completa no se setea despues de reintento
//      -11: Fallo calcularDesviacion
//      -12: Fallo calcularMedia
//
int read_processed_cap_pF(enum tipo_medida tipoMedida, float desviacion_aceptable_pF, int vectorSize, struct media_confiabilidad_nivel_struct *salidaStruct_p);






/**
 * nivel_medida_single: Toma una medida de nivel.
 * 
 * Parametros:
 *      nivel: Puntero a la variable donde se guarda el nivel.
 * 
 * Retorna:
 * 
 **/
int nivel_medida_single(float* nivel);





/**
 * reset: Resetea el integrado fdc1004.
 * 
 * Parametros:
 *      file_p: Puntero al file descriptor correspondiente.
 * 
 * Retorna:
 *       0: Exito
 *      -1: Fallo capacimeter_write
 *      -2: Fallo en capacimeter_config
 * 
 **/
int reset(int* file_p);



#endif // NIVELLIB_H_INCLUDED
