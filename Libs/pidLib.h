#ifndef PRESIONPIDLIB_H_INCLUDED
#define PRESIONPIDLIB_H_INCLUDED



/**
 * enum de los tipos de errores que puede indicar el cryocooler cuando
 * se usa la funcion read_cryo_error(.).
 **/
typedef enum{
	NO_ERROR= 0,
    TEMPERATURE_SENSOR_ERROR= 1, 
    WATCHDOG_ERROR= 2,
    NON_VOLATILE_MEM_ERROR= 3,
    SERIAL_COMMUNICATION_ERROR= 4,
    JUMPER_ERROR= 5,
    OVER_CURRENT= 6,
}errores_cryo;



/** read_cryo_error: Lee los errores del cryocooler, la cantidad maxima
 *                   de errores simultaneos es 6. Los errores leidos se
 *                   guardan en el array <errores> de la siguiente forma:
 * 
 *  Si hubo como ejemplo errores: WATCHDOG_ERROR y OVER_CURRENT:
 * 
 *      errores[0]= NO_ERROR            (corresponde a TEMPERATURE_SENSOR_ERROR)
 *      errores[1]= WATCHDOG_ERROR
 *      errores[2]= NO_ERROR            (corresponde a NON_VOLATILE_MEM_ERROR)
 *      errores[3]= NO_ERROR            (corresponde a SERIAL_COMMUNICATION_ERROR)
 *      errores[4]= NO_ERROR            (corresponde a JUMPER_ERROR)
 *      errores[5]= OVER_CURRENT
 * 
 * 
 *  Parametros: 
 *      errores -> Puntero a un array de 6 elementos del tipo errores_cryo.
 *      fs      -> file descriptor del cryocooler.
 * 
 *  Retorna:
 *       0: Exito
 *      -1: Fallo en lectura
 *      -2: Fallo en escritura
 *      -3: Respuesta invalida
 *      -4: No se encontro la cadena "ERROR" dentro de la respuesta
 *      -5: Respondio "ERROR" pero no la linea con los errores.
 *      -6: El tamaño de los errores leidos no es correcto
 * 
 **/
int read_cryo_error(int fs, errores_cryo* errores);	// hay hasta 6 errores, var errores tiene que ser de tamaño 6.







/** pid_presion_cryo_ini: Inicializa el pid, el sensor de presion y el cryocooler.
 * 
 *  Parametros:
 *      fileDescriptorPRESION: File descriptor del sensor de presion.
 * 
 *      fileDescriptorCRYO: File descriptor del cryocooler.
 * 
 *  Retorna:
 *       0: Exito
 *      -1: Fallo inicializando cryocooler
 *      -2: Fallo configurando cryocooler en control por potencia
 *      -3: Fallo configurando SSTOPM
 *      -4: Fallo en el comando de encendido del cryocooler
 *      -5: Fallo en PRESION_init
 *
**/
int pid_presion_cryo_ini(int fileDescriptorPRESION, int fileDescriptorCRYO);



/**
 * @brief Check if PID loop needs to run
 *
 * Determines if the PID control algorithm should compute a new output value,
 * if this returs true, the user should call the pid_compute() function.
 *
 * @return return Return true if PID control algorithm is required to run
 */
bool pid_need_compute();



/**
 * pid_compute: Calcula y comanda la salida.
 * 
 * Retorna:
 *       0: Exito
 *      -1: PID deshabilitado
 *      -2: Fallo lectura de presion
 *      -3: Error comandando potencia
 * 
 */
int pid_compute();



#endif // PRESIONPIDLIB_H_INCLUDED
