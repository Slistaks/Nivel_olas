

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include "presion.h"
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include "pidLib.h"


#define TICKS_PER_SECOND            1000000        // pid usa esto junto con tick_get para calcular el tiempo transcurrido.
#define RESPUESTA_LARGO_MAX         32

//Control por potencia
#define MAX_OUTPUT_PID              240  //potencia maxima en W del cryocooler.
#define MIN_OUTPUT_PID              70   //potencia minima en W del cryocooler.

#define PID_SAMPLE_TIME_ms          2000    // Sample time pid.
#define PID_SETPOINT_BAR            0.1     // Presion a la que controla el pid.
#define PID_KP                      35269   //recalculada para bar y corregido el error del autotuning
#define PID_KI                      16
#define PID_KD                      4

#define CONTROL_POT                 0           // Parametro que se envia al cryocooler para controlar su potencia.
#define CONTROL_TEMP                2



typedef struct pid_controller * pidController_t;

/**
 * Defines if the controler is direct or reverse
 */
enum pid_control_directions {
    E_PID_DIRECT,
    E_PID_REVERSE,
};





int fs_cryo;    // file descriptors globales, entonces se pasan en inicializacion unicamente.

int filePRESION;

// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata;     // ctrldata: contendra los parametros del pid

pidController_t pid;                //      pid: puntero que apuntara a ctrldata

char respuesta[RESPUESTA_LARGO_MAX];

typedef enum { TC_cryo, SET_PID_EQ_cryo, SET_PID_cryo, MODE_cryo, RESET_EQ_F_cryo, P_cryo, E_cryo, ERROR_cryo, STATE_cryo, SET_SSTOPM_EQ_cryo, SET_SSTOPM_cryo, SET_SSTOP_EQ_cryo, SET_SSTOP_cryo, SET_PWOUT_EQ_cryo, SET_PWOUT_cryo, SET_TTARGET_EQ_cryo, SET_TTARGET_cryo, SET_TBAND_EQ_cryo, SET_TBAND_cryo, SET_TSTATM_EQ_cryo, SET_TSTATM_cryo, TSTAT_cryo, SET_MIN_EQ_cryo, SET_MIN_cryo, SET_MAX_EQ_cryo, SET_MAX_cryo, SHOW_MX_cryo} comandos;


uint32_t tick_get();

int cryocooler_ini(int serial0_fs);

int leer(char* rx, int* largo);

int send_command(comandos comando, double parametro);

int process_output(float output);



/*-------------------------------------------------------------*/
/*      Function prototypes             */
/*-------------------------------------------------------------*/

/**
 * @brief Creates a new PID controller
 *
 * Creates a new pid controller and initializes internal
 * variables. Also we set the tuning parameters
 *
 * @param pid A pointer to a pid_controller structure
 * @param set Float with the process setpoint value
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Diferential gain
 *
 * @return returns a pidController_t controller handle
 */
pidController_t pid_create(pidController_t pid, float set, float kp, float ki, float kd);




/**
 * @brief Sets new PID tuning parameters
 *
 * Sets the gain for the Proportional (Kp), Integral (Ki) and Derivative (Kd)
 * terms.
 *
 * @param pid The PID controller instance to modify
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void pid_tune(pidController_t pid, float kp, float ki, float kd);

/**
 * @brief Sets the pid algorithm period
 *
 * Changes the between PID control loop computations.
 *
 * @param pid The PID controller instance to modify
 * @param time The time in milliseconds between computations
 */
void pid_sample(pidController_t pid, uint32_t time);

/**
 * @brief Sets the limits for the PID controller output
 *
 * @param pid The PID controller instance to modify
 * @param min The minimum output value for the PID controller
 * @param max The maximum output value for the PID controller
 */
void pid_limits(pidController_t pid, float min, float max);

/**
 * @brief Enables automatic control using PID
 *
 * Enables the PID control loop. If manual output adjustment is needed you can
 * disable the PID control loop using pid_manual(). This function enables PID
 * automatic control at program start or after calling pid_manual()
 *
 * @param pid The PID controller instance to enable
 */
void pid_auto(pidController_t pid);

/**
 * @brief Disables automatic process control
 *
 * Disables the PID control loop. User can modify the value of the output
 * variable and the controller will not overwrite it.
 *
 * @param pid The PID controller instance to disable
 */
void pid_manual(pidController_t pid);

/**
 * @brief Configures the PID controller direction
 *
 * Sets the direction of the PID controller. The direction is "DIRECT" when a
 * increase of the output will cause a increase on the measured value and
 * "REVERSE" when a increase on the controller output will cause a decrease on
 * the measured value.
 *
 * @param pid The PID controller instance to modify
 * @param direction The new direction of the PID controller
 */
void pid_direction(pidController_t pid, enum pid_control_directions dir);

/**
 * Structure that holds PID all the PID controller data, multiple instances are
 * posible using different structures for each controller
 */
struct pid_controller {
    
    float setpoint; //!< Controller Setpoint
    // Tuning parameters
    float Kp; //!< Stores the gain for the Proportional term
    float Ki; //!< Stores the gain for the Integral term
    float Kd; //!< Stores the gain for the Derivative term
    // Output minimum and maximum values
    float omin; //!< Maximum value allowed at the output
    float omax; //!< Minimum value allowed at the output
    // Variables for PID algorithm
    float iterm; //!< Accumulator for integral term
    float lastin; //!< Last input value for differential term
    // Time related
    uint32_t lasttime; //!< Stores the time when the control loop ran last time
    uint32_t sampletime; //!< Defines the PID sample time
    // Operation mode
    uint8_t automode; //!< Defines if the PID controller is enabled or disabled
    enum pid_control_directions direction;
};




                


int pid_presion_cryo_ini(int fileDescriptorPRESION, int fileDescriptorCRYO){
    

    filePRESION= fileDescriptorPRESION;     // guardo en la variable global el file descriptor.
    fs_cryo= fileDescriptorCRYO;
    ///INICIALIZACION CRYO

    int error;
    error= cryocooler_ini(fs_cryo);
    if(error!=0){
        printf("error surgido inicializando cryocooler.\n");
        return -1;
    }
    
    


    
    usleep(1000);       // ms entre comandos, en el datasheet no especifica tiempo minimo.
    error= send_command(SET_SSTOPM_EQ_cryo, 0);     // set sstopm=0 -> sstop se manipula por comando.
    if(error!=0){
        printf("error surgido configurando sstop mode.\n");
        return -3;
    }
    
    
    
    usleep(1000);
    error= send_command(SET_PID_EQ_cryo, CONTROL_POT);    // cryo en modo potencia.
    if(error!=0){
        printf("error surgido configurando cryocooler en modo control potencia.\n");
        return -2;
    }
    
    
    usleep(1000);       // ms entre comandos, en el datasheet no especifica tiempo minimo.
    error= send_command(SET_SSTOP_EQ_cryo, 0);  // set sstop=0 -> enciende el cryocooler.
    if(error!=0){
        printf("error surgido encendiendo el cryocooler.\n");
        return -4;
    }

    
    
    
    ///INICIALIZACION PRESION

    error= PRESION_init(filePRESION);
    if(error!=0){
        printf("error surgido inicializando medidor de presion.\n");
        return -5;
    }
    
    
    ///INICIALIZACION PID
    // Prepare PID controller for operation
    pid = pid_create(&ctrldata, PID_SETPOINT_BAR, PID_KP, PID_KI, PID_KD);        // pid es global

    
    return 0;

}










int process_output(float output)	//errores->6 elementos
{
    
    float potencia;
    int err=0;
    
    
    ///CONTROL POR POTENCIA
    potencia= MAX_OUTPUT_PID + MIN_OUTPUT_PID - output;     // pid reverse. (podria haber usado la funcion de la lib de pid.)
    
    usleep(1000);
    err= send_command(SET_PWOUT_EQ_cryo, potencia);
    if(err!=0){
	printf("error surgido enviando comando de potencia\n");
	return -3;
    }
    
    return 0;

}



int cryocooler_ini(int serial0_fs)
{

    ///config
    struct termios options;
    
    if(tcgetattr(serial0_fs, &options)==-1){
    	printf("Error en tcgetattr.\n");
    	return -1;
    }
    
    
    
    options.c_cflag = B4800 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;   // Ignore parity errors
    options.c_oflag&= ~OPOST;
    options.c_lflag&= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 10;
    
    
    if(tcflush(serial0_fs, TCIFLUSH)==-1){
    	printf("fallo en tcflush.\n");
    	return -2;
    }
    
    if(tcsetattr(serial0_fs, TCSANOW, &options)==-1){
    	printf("fallo en tcsetattr.\n");
    	return -3;
    }
    ///config

    return 0;
}


int send_command(comandos comando, double parametro)
{


    ///Corroboro que el parametro este dentro del rango:
    if( (999.99<parametro) || (parametro<-999.99))             // parametro iii.dd
    {
        printf("Error, el parametro esta fuera de rango.\n");
        return -6;
    }
    ///Corroboro que el parametro este dentro del rango.






    char tx_buffer[32];
    char parametro_str[10];     // Parte de la cadena a enviar.

    ///obtengo el string en la forma adecuada:
							// 2 decimales.
    sprintf(parametro_str, "%03.02f", parametro);
    ///obtengo el string en la forma adecuada.






    /// Segun el comando, el buffer de Tx se carga con lo que se envia:
    switch(comando)
    {
    case TC_cryo:
        strcpy(tx_buffer, "TC\r");
        break;
    case SET_PID_EQ_cryo:
        sprintf(tx_buffer, "SET PID=%s\r", parametro_str);
        break;

    case SET_PID_cryo:
        strcpy(tx_buffer, "SET PID\r");
        break;

    case MODE_cryo:
        strcpy(tx_buffer, "MODE\r");
        break;

    case RESET_EQ_F_cryo:
        strcpy(tx_buffer, "RESET=F\r");
        break;

    case P_cryo:
        strcpy(tx_buffer, "P\r");
        break;

    case E_cryo:
        strcpy(tx_buffer, "E\r");
        break;

    case SET_SSTOPM_EQ_cryo:
        sprintf(tx_buffer, "SET SSTOPM=%s\r", parametro_str);
        break;

    case SET_SSTOPM_cryo:
        strcpy(tx_buffer, "SET SSTOPM\r");
        break;

    case SET_SSTOP_EQ_cryo:
        sprintf(tx_buffer, "SET SSTOP=%s\r", parametro_str);
        break;

    case SET_SSTOP_cryo:
        strcpy(tx_buffer, "SET SSTOP\r");
        break;

    case SET_PWOUT_EQ_cryo:
        sprintf(tx_buffer, "SET PWOUT=%s\r", parametro_str);
        break;

    case SET_PWOUT_cryo:
        strcpy(tx_buffer, "SET PWOUT\r");
        break;

    case SET_TTARGET_EQ_cryo:
        sprintf(tx_buffer, "SET TTARGET=%s\r", parametro_str);
        break;

    case SET_TTARGET_cryo:
        strcpy(tx_buffer, "SET TTARGET\r");
        break;

    default:
        printf("Error, comando ingresado no existe.\n");
        return -5;

    }



    if(tcflush(fs_cryo, TCIOFLUSH)==-1){
    	printf("fallo en tcflush.\n");
    	return -1;
    }
    //Envio el comando:
    if( write(fs_cryo, tx_buffer, strlen(tx_buffer)) != strlen(tx_buffer)) 		//Filestream, bytes to write, number of bytes to write
    {
        printf("Error escribiendo.\n");
        return -2;
    }
    
    
    // Corroboro que no haya error:
    usleep(50000);   // espero a que termine de enviarse la respuesta del comando anterior ( @ baudrate=4800 )
    errores_cryo err_cryocooler[6];
    
    int error= read_cryo_error(fs_cryo, err_cryocooler);
    if(error!=0){
        printf("error leyendo errores cryocooler.\n");
        return -6;
    }
    for(int i=0; i<6; i++){
        if(err_cryocooler[i]!=NO_ERROR){
            printf("cryocooler respondio con error.\n");
            return -7;
        }
    }
    // Corroboro que no haya error
    
    
    
    
    return 0;
}



///pid
















pidController_t pid_create(pidController_t pid, float set, float kp, float ki, float kd)
{
    pid->setpoint = set;
    pid->automode = true;      //pid habilitado

    pid_limits(pid, MIN_OUTPUT_PID, MAX_OUTPUT_PID);

    // Set default sample time to PID_SAMPLE_TIME_ms ms
    pid->sampletime = PID_SAMPLE_TIME_ms * (TICKS_PER_SECOND / 1000);     //sampletime esta medido en ticks.

    pid_direction(pid, E_PID_DIRECT);
    pid_tune(pid, kp, ki, kd);

    pid->lasttime = tick_get() - pid->sampletime;

    return pid;
}







bool pid_need_compute()     // ya no toma como parametro el puntero al pid, hay un solo pid.
{
    // Check if the PID period has elapsed
    return(tick_get() - pid->lasttime >= pid->sampletime) ? true : false;
}






int pid_compute()      // mide la presion, calcula la salida y comanda la salida. 
{
    int err;
    double presion;

    // Check if control is enabled
    if (!pid->automode)
        return -1;
	

    err= PRESION_leer_presion_bar(&presion);
    if(err!=0){
        printf("error leyendo presion en pid_compute.\n");
        return -2;
    }
    float in= presion;

    // Compute error
    float error = (pid->setpoint) - in;
	
    // Compute integral
    pid->iterm += (pid->Ki * error);
    if (pid->iterm > pid->omax)
        pid->iterm = pid->omax;
    else if (pid->iterm < pid->omin)
        pid->iterm = pid->omin;
	
    // Compute differential on input
    float dinput = in - pid->lastin;
	
    // Compute PID output
    float out = pid->Kp * error + pid->iterm - pid->Kd * dinput;   // El signo del termino derivativo es porque d/dt(e)=-d/dt(input)
	
    // Apply limit to output value
    if (out > pid->omax)
        out = pid->omax;
    else if (out < pid->omin)
        out = pid->omin;

    // Output
    err= process_output(out);
    if(err!=0){
        printf("error comandando salida en pid_compute.\n");
        return -3;
    }

    // Keep track of some variables for next execution
    pid->lastin = in;
    pid->lasttime = tick_get();
    
    return 0;
}






void pid_tune(pidController_t pid, float kp, float ki, float kd)
{
	// Check for validity
	if (kp < 0 || ki < 0 || kd < 0)		// estan definidas, corroboracion no hace falta.
	    return;
	
	//Compute sample time in seconds
	float ssec = ((float) pid->sampletime) / ((float) TICKS_PER_SECOND);

	pid->Kp = kp;
	pid->Ki = ki * ssec;
	pid->Kd = kd / ssec;

	if (pid->direction == E_PID_REVERSE) {
	    pid->Kp = 0 - pid->Kp;
	    pid->Ki = 0 - pid->Ki;
	    pid->Kd = 0 - pid->Kd;
	}
}

void pid_sample(pidController_t pid, uint32_t time) //time entra en milisegundos
{
	if (time > 0) {
	    float ratio = (float) (time * (TICKS_PER_SECOND / 1000)) / (float) pid->sampletime;
	    pid->Ki *= ratio;
	    pid->Kd /= ratio;
	    pid->sampletime = time * (TICKS_PER_SECOND / 1000);
	}
}

void pid_limits(pidController_t pid, float min, float max)
{
    pid->omin = min;
    pid->omax = max;
}




void pid_direction(pidController_t pid, enum pid_control_directions dir)
{
	if (pid->automode && pid->direction != dir) {
	    pid->Kp = (0 - pid->Kp);
	    pid->Ki = (0 - pid->Ki);
	    pid->Kd = (0 - pid->Kd);
	}
	pid->direction = dir;
}




uint32_t tick_get(){            // devuelve ticks, cada tick= 1 useg -> TICKS_PER_SECOND= 1000 000.
    uint32_t usec;
    struct timeval tiempoFuncionTick;
    gettimeofday(&tiempoFuncionTick, NULL);
    usec= tiempoFuncionTick.tv_sec*1000000 + tiempoFuncionTick.tv_usec;
    return usec;
}




int read_cryo_error(int fs, errores_cryo* errores)
{
    
    
    char comando_error[]= "ERROR\r";
    char respuesta[256];			// responde: "ERROR\r\nxxxxxx\r\n"
    int leidos;
    int largo;
    
    tcflush(fs, TCIOFLUSH);
    usleep(1000);
    //Envio el comando:
    if( write(fs, comando_error, strlen(comando_error)) != strlen(comando_error))
    {
        printf("Error escribiendo.\n");
        return -2;
    }
    
    
    
    
    //_______________________________________________
    // Fragmento modificado para lectura no canonica:
    //_______________________________________________
    
    usleep(100000);	// ver
    //leer:
    int err= leer(respuesta, &largo);
    if(err<0){
        printf("error leyendo.\n");
        return -1;
    }
    
    
    
    //:
    //chequeo si respondio "ERROR":
    char* ret= strstr(respuesta, "ERROR");
    if(ret==NULL){
        printf("no se encontro ""ERROR"" en la respuesta.\n");
        return -4;
    }
    //. ret apunta donde dentro de <respuesta> donde empieza "ERROR\r\n..."
    
    
    //obtengo la primera linea de la cadena:		(deberia ser "ERROR")
    char* tokens[2];
    char* save_pointer;		//usado por strtok_r.
    
    //tokens[0]= strtok_r(respuesta, "\r\n", &save_pointer);        //debug
    tokens[0]= strtok_r(ret, "\r\n", &save_pointer);
    
    if(tokens[0]==NULL){
        printf("no se pudo leer respuesta.\n");
        return -3;
    }
    
    
    //Obtengo la segunda linea, deberian ser los 6 flags de errores "xxxxxx" (x = 0 o 1)
    tokens[1]= strtok_r(NULL, "\r\n", &save_pointer);
    if(tokens[1]==NULL){
        printf("respondio ""ERROR"" pero no se leyo correctamente la siguiente linea.\n");
        return -5;
    }
    
    if(strlen(tokens[1])!=6){
        printf("la linea de los flags de errores no se leyo correctamente, su tamaño no concuerda.\n");
        return -6;
    }
    //_______________________________________________
    // Fragmento modificado para lectura no canonica.
    //_______________________________________________
    
    
    
    
    
    //guardar en <errores>, en el lugar correspondiente el numero de error segun el enum.
    for(int i=0; i<6; i++){
        if( *(tokens[1]+i) =='1'){
            errores[i]= i+1;		// errores del enum van de 1 a 6
        }else{
            errores[i]= NO_ERROR;
        }
    }
    
    
    return 0;
}







//rx no debe ser de menor tamaño que 256.
int leer(char* rx, int* largo){		//largo tambien cuenta el '\0'
    
    
    ///lectura no canonica
    
    int datosLeidos=0;
    int err;
    
    
    datosLeidos= read(fs_cryo, rx, 255);
    if(datosLeidos<0){
	printf("Error en lectura.\n");
	return -2;
    }
    if(datosLeidos==0){
	printf("No contesto.\n");
	return -3;
    }
    
    
    rx[datosLeidos]='\0';
    
    //debug:
    //printf("cadena leida: |%s|\n", rx);
    //debug.
    
    ///lectura no canonica
    
    *largo= datosLeidos+1;
    
    return 0;
}



