

#include "presion.h"
#include "time.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#ifndef I2C_M_RD
#include <linux/i2c.h>
#endif

#include <math.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#define kPaToPSI 0.14503773800722
#define kPaToBar 0.01

float calcularMediaUnsigned(int* muestras, int tam);

float calcularDesviacion(int* muestras, int tam);

int PRESION_leer_presion_raw(int* file_p, double* presion_p, int* presion_raw);

int PRESION_leer_presion_temperatura(double* presion_p, double* temperatura_p);



int fs_presion;



int errorr;
const double presion_minima= -103.42; //en kilo pascales.
const double presion_maxima=  103.42;









int PRESION_init(int file){

    fs_presion= file;
    // 0x28 es la dir del sensor de presion.
    if (ioctl(file, I2C_SLAVE, SENSOR_PRESION_ADDRESS) == -1) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        return -1;
    }
    
    usleep(5000);                       // Startup time max 5ms

    
    return 0;
}

















int PRESION_leer_presion_temperatura(double* presion_p, double* temperatura_p){
    
    int* file_p= &fs_presion;
    //usleep(1000);                       // Response time               
    char rxBuffer[4];
    int file= *file_p;
    char status;            // Estado de la conversion.
    usleep(20);
    if(read(file,rxBuffer,4) != 4) {
        printf("Failed to read from the i2c bus.\n");
        return -4;
    }else{

        status= ( *(rxBuffer) & 0xC0 )>>6;

        switch(status){             // En caso que haya error, imprime el error y retorna. Caso contrario, sigue ejecutando.

            case 1:
                printf("\nError, device in command mode.\n");
                return -1;

            //case 2:
                //printf("stale data: data that has already been fetched since the last measurement cycle,"
                //"or data fetched before the first measurement has been completed");
                //return -2;

            case 3:
                printf("Loss of sense element connection or short circuit of sense element.\n");
                return -3;

            default:
                break;
        }
        //Si sigue por aca, no hubo error.
        *presion_p= ( ((*rxBuffer) & 0x3f)<<8 ) | *(rxBuffer+1);
        *temperatura_p= (*(rxBuffer+2))<<3 | ( ( *(rxBuffer+3) & 0xe0 )>>5 );

        //Conversion a unidades:
        *presion_p    = ((*presion_p-1638)*(presion_maxima-presion_minima)/(14745-1638)+presion_minima)*kPaToBar;
        *temperatura_p= (*temperatura_p)*200/2047-50;


    }


	return 0;
}





int PRESION_leer_presion_raw(int* file_p, double* presion_p, int* presion_raw){
    
    //usleep(1000);           // Response time
    char rxBuffer[4];
    char status;            // Estado de la conversion.
    usleep(20);
    if(read(*file_p,rxBuffer,2) != 2) {
        printf("Failed to read from the i2c bus.\n");
        return -4;
    }else{

        status= ( *(rxBuffer) & 0xC0 )>>6;

        switch(status){             // En caso que haya error, imprime el error y retorna. Caso contrario, sigue ejecutando.

            case 1:
                printf("\nError, device in command mode.\n");
                return -1;

            //case 2:
                //printf("stale data: data that has already been fetched since the last measurement cycle,"
                //"or data fetched before the first measurement has been completed");
                //return -2;

            case 3:
                printf("Loss of sense element connection or short circuit of sense element.\n");
                return -3;

            default:
                break;
        }
        //Si sigue por aca, no hubo error.
        *presion_p= ( ((*rxBuffer) & 0x3f)<<8 ) | *(rxBuffer+1);
        *presion_raw= *presion_p;

        //Conversion a unidades:
        *presion_p    = ((*presion_p-1638)*(presion_maxima-presion_minima)/(14745-1638)+presion_minima)*kPaToBar;

    }

    return 0;

}









int PRESION_read_processedData(float desviacion_aceptable_bar, int vectorSize, struct media_deviation_relation_pressure *salidaStruct_p){
    
    int* file_p= &fs_presion;
    char rxBuffer[4];
    int intentos;
    int muestras[vectorSize];
    int presion_raw;
    int err;
    double presion;
    float desviacion_aceptable= (desviacion_aceptable_bar/kPaToBar)*(14745-1638)/(presion_maxima-presion_minima);    //Convierto presion en kPa a cuentas.



    // TOMO LAS N MUESTRAS:
    for(int i=0; i<vectorSize; i++){

        err= PRESION_leer_presion_raw(file_p, &presion, &presion_raw);
        if(err!=0){
            printf("sin respuesta.\n");
            return -6;
        }
        usleep(1000);           // Response time

        muestras[i]= presion_raw;

    }               //YA ESTAN TOMADAS LAS MUESTRAS. PROCESAR:



    float media;
    float desviacion;
    int muestrasUtiles[vectorSize];       // Inicialmente es el vector de medidas.
    for(int i=0; i<vectorSize; i++){
        muestrasUtiles[i]= muestras[i];
    }
    int muestrasUtilesBuffer[vectorSize];
    int cantMuestrasUtiles=vectorSize;         //Inicialmente, todas son utiles.

    int j=0;
    while(1){      //Se hace iteraciones hasta obtener el vector final con desviacion<desviacion_aceptable.

        if( (desviacion=calcularDesviacion(muestrasUtiles,cantMuestrasUtiles)) < 0 ){
            printf("Error en calculo de desviacion en PRESION_read_processedData.\n");
            return -9;
        }
        errorr=0;
        media=calcularMediaUnsigned(muestrasUtiles, cantMuestrasUtiles);
        if(errorr){
            printf("Error en calculo de media en PRESION_read_processedData.\n");
            return -10;
        }

        if(desviacion<desviacion_aceptable)
            break;

        for(int i=0; i<cantMuestrasUtiles; i++){
            muestrasUtilesBuffer[i]=muestrasUtiles[i];
        }

        j=0;

        for(int i=0; i<cantMuestrasUtiles; i++){

            if( abs(muestrasUtilesBuffer[i]-media)<desviacion ){
                muestrasUtiles[j]=muestrasUtilesBuffer[i];
                j++;
            }
        }
        if(cantMuestrasUtiles==j){
            break;
        }
        cantMuestrasUtiles=j;
        if(cantMuestrasUtiles<=2)
            break;

    }

    salidaStruct_p->media     = ((media-1638)*(presion_maxima-presion_minima)/(14745-1638)+presion_minima)*kPaToBar;
    salidaStruct_p->desviacion= ((desviacion)*(presion_maxima-presion_minima)/(14745-1638))*kPaToBar;
    salidaStruct_p->relation  = cantMuestrasUtiles*100/vectorSize;    // (cantMuestrasUtiles/totales)*100%.

    return 0;
}










float calcularMediaUnsigned(int* muestras, int tam){

	errorr= 0;
    if(tam<1){
        printf("Error intento de division por cero en calcularMedia.\n");
        errorr=1;
        return -11;
    }

    float media= 0;

    for(int i=0; i<tam; i++){
        media+= *(muestras+i);
    }

    return (media/tam);
}





int PRESION_leer_presion_bar(double* presion_p){
    
    int* file_p= &fs_presion;
    //usleep(1000);           // Response time
    char rxBuffer[4];
    char status;            // Estado de la conversion.
    usleep(20);
    if(read(*file_p, rxBuffer, 2) != 2) {
        printf("Failed to read from the i2c bus.\n");
        return -4;
    }else{

        status= ( *(rxBuffer) & 0xC0 )>>6;

        switch(status){             // En caso que haya error, imprime el error y retorna. Caso contrario, sigue ejecutando.

            case 1:
                printf("\nError, device in command mode.\n");
                return -1;

            //case 2:
                //printf("stale data: data that has already been fetched since the last measurement cycle,"
                //"or data fetched before the first measurement has been completed");
                //return -2;

            case 3:
                printf("Loss of sense element connection or short circuit of sense element.\n");
                return -3;

            default:
                break;
        }
        //Si sigue por aca, no hubo error.
        *presion_p= ( ((*rxBuffer) & 0x3f)<<8 ) | *(rxBuffer+1);

        //Conversion a unidades:
        *presion_p= (*presion_p-1638)*(presion_maxima-presion_minima)/(14745-1638)+presion_minima;
        *presion_p= (*presion_p)*kPaToBar;
    }

    return 0;

}










float calcularMedia(int* muestras, int tam){

	errorr= 0;
    if(tam<1){
        printf("Error intento de division por cero en calcularMedia.\n");
        errorr=1;
        return -11;
    }

    float media= 0;
    int muestra_signada;

    for(int i=0; i<tam; i++){
        if( *(muestras+i)&0x800000 ){
            muestra_signada= ~(*(muestras+i))+1;
            muestra_signada= muestra_signada & 0xffffff;
            muestra_signada= -muestra_signada;
        }else{
            muestra_signada= *(muestras+i);
        }
        media+= muestra_signada;
    }

    return (media/tam);
}






float calcularDesviacion(int* muestras, int tam){

    if(tam<2){
        printf("Error de division por cero en calcularDesviacion.\n");
        return -11;
    }

    float aux_sumatoria= 0;
    errorr=0;
    float media_muestral= calcularMedia(muestras, tam);
    if(errorr){
        printf("Error en calcularMedia en calcularDesviacion.\n");
        return -12;
    }
    int muestra_signada;
    for(int i=0; i<tam; i++){

        if( *(muestras+i)&0x800000 ){
            muestra_signada= ~(*(muestras+i))+1;
            muestra_signada= muestra_signada & 0xffffff;
            muestra_signada= -muestra_signada;
        }else{
            muestra_signada= *(muestras+i);
        }

        aux_sumatoria+= ( muestra_signada-media_muestral) * ( muestra_signada-media_muestral);
    }
    aux_sumatoria= aux_sumatoria/(tam-1);     // Media muestral-> /(n-1)

    return sqrt(aux_sumatoria);
}


