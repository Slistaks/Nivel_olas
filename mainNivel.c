#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include "Libs/nivelLib.h"

int Log(const char* nombre, float valor);
int capacidad_autooffset(float* capacidad);

int main()
{


    // file descriptor para probar la biblioteca, el file descriptor que pasaria usuario
    int file= open("/dev/i2c-1", O_RDWR);
    if (file < 0) {
        perror("Failed to open /dev/i2c-1");
        return -1;
    }
    //fin file descriptor para probar biblioteca




    int capdac_offset= 0;		// el offset que se resta a lo que se medira: (valor real[pF]= valor medido[pF] + capdac_offset*3.125pF)








    // Inicializacion:
    int err= capacimeter_init(&file, capdac_offset, CUATROCIENTAS_Ss); // CUATROCIENTAS_Ss: sample rate, esta en enum en nivelLib.h.
    if(err!= 0){
        printf("error inicializando.\n");
        return -1;
    }
    usleep(100000);



    // En caso que se quiera cambiar la configuracion en cualquier momento se puede ejecutar:
    capdac_offset= 10;		// Nuevo offset a configurar.
    capacimeter_config(capdac_offset, CUATROCIENTAS_Ss, medidaNIVEL);	// sample rate puede ser cualquier elemento del enum de nivelLib.h.



    /*_____________ Si se quiere logguear:

	
	Log("nombreDelLog.txt", capacidad);
	
	la funcion anterior guarda en modo apendice separando por comas. Por ejemplo si se hace lo siguiente:
	
	cap= 1;
	Log("log.txt", cap);
	cap= 2;
	Log("log.txt", cap);
	cap= 4;
	Log("log.txt", cap);

	Se creara en caso que no exista un archivo "log.txt", y lo ultimo que contendra es: "1,2,4,".

	Como toma como parametro el nombre del archivo, se puede logguear simultaneamente distintas variables en
	distintos logs. Como no sobreescribe, puede ser usado dentro de un bucle (como los bucles de ejemplo siguientes).

	*/

    float capacidad;
    
    




    multiMedidasEnable(1);  //debug

    int offset_autoEncontrado;
    
    printf("MEDIDA CON AUTO-OFFSET:\n");
    for(int i=0; i<2; i++){
        offset_autoEncontrado= capacidad_autooffset(&capacidad);
        printf("CAPACIDAD: %f, OFFSET: %d    <<<<<<<<<<<<<<<<\n", capacidad+offset_autoEncontrado*3.125, offset_autoEncontrado);
        usleep(500000);
    }
    
    
    printf("\n\n");
    
    












    ///TOMAR MEDIDA DE CAPACIDAD sin autoOffset:
    
    printf("MEDIDA UNICA SIN AUTO-OFFSET:\n");
    capdac_offset= offset_autoEncontrado;		// Nuevo offset a configurar.   //comento para debug
    //capdac_offset= 1;  //debug
    capacimeter_config(capdac_offset, CUATROCIENTAS_Ss, medidaNIVEL);                          // la comente para debug, para ver si deja de funcionar mal.
    
    usleep(8000);   //debug
    
    for(int i=0; i<2; i++){

        capacidad_medida_single(&capacidad, medidaNIVEL);
        
        
        //printf("capacidad sin offset saturada?: %f\n", capacidad);   //debug
        
        
        //chequeo si no saturo:
        if(15.98<capacidad){
            printf("Saturo. Capacidad muy alta para el offset seteado.\n");
        }else if(capacidad<-15.98){
            printf("Saturo. Capacidad muy baja para el offset seteado.\n");
        }else
            printf("capacidad sin filtro con offset manual: %.2fpF\n", capacidad+capdac_offset*3.125);

        usleep(500000);

    }



    printf("\n\n");













    // MEDIDAS DIFERENCIALES:

    printf("MEDIDAS DIFERENCIALES CIN2-CIN3:\n");

    multiMedidasEnable(1);  // si se usa capacimeter_config(., ., medidaDIFERENCIAL) arriba, no hace falta esta funcion.
    usleep(8000);

    for(int i=0; i<2; i++){
        capacidad_medida_single(&capacidad, medidaDIFERENCIAL);
        printf("capacidad DIFERENCIAL= %fpF\n", capacidad);
        usleep(500000);
    }


    printf("\n\n");













    multiMedidasEnable(0);  //debug
    ///BLOQUE QUE MUESTRA MEDIDAS FILTRADAS.

    printf("MEDIDAS PROMEDIADAS:\n");

    media_confiabilidad_nivel medidaProcesada;   // En esta estructura se guarda la media, la desviacion y el porcentaje de muestras utiles
    float desviacion_aceptable= 0.1;  // en [pF].// respecto del total, al llamar a read_processedData.
    int cantidadMuestras      = 20;                       // Cantidad de muestras que se leeran y procesaran.

    for(int i=0; i<2; i++){
        
        usleep(500000);
        // LEER "cantidadMuestras" Y PROCESAR:
        err= read_processed_cap_pF(medidaNIVEL, desviacion_aceptable, cantidadMuestras, &medidaProcesada);
        
        //chequeo si no saturo:
        if(15.98<medidaProcesada.media){
            printf("Saturo. Capacidad muy alta para el offset seteado.\n");
        }else if(medidaProcesada.media<-15.98){
            printf("Saturo. Capacidad muy baja para el offset seteado.\n");
        }else{
            if(err==0){
                printf("capacidad promediada= %.2fpF\n", medidaProcesada.media+capdac_offset*3.125);  //usar para nivel
                //printf("capacidad promediada= %.2fpF\n", medidaProcesada.media);        // usar para diferencial.
                printf("confiabilidad       = %.0f%%\n\n", medidaProcesada.confiabilidad);
            }
        }
        
        
        
        
        //sleep(1);
    }
    ///FIN DE BLOQUE QUE MUESTRA MEDIDAS FILTRADAS.







    close(file);

    return 0;
}









int Log(const char* nombre, float valor){


    char coma= ',';
    char str_log[9];
    int j=0;

    // Abro fichero
    FILE *fichero;
    if( (fichero = fopen(nombre, "a+")) == NULL ){
        printf("Error al intentar abrir fichero del log general.\n");
        return -8;
    }


    sprintf(str_log, "%.3f", valor);


    while(str_log[j]!='\0'){
        fwrite(&str_log[j], sizeof(char), 1, fichero);
        j++;
    }

    fwrite(&coma, sizeof(coma), 1, fichero);

    fclose(fichero);

    return 0;
}

