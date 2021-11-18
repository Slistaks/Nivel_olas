#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include "Libs/nivelLib.h"


//Log generico para lo que sea que se necesite logguear.
int Log(const char* nombre, float valor);



int main()
{



    float capacidad;
    int capdac_offset= 10;
    enum tipo_medida tipoDeMedida= medidaNIVEL;
    //tipoDeMedida= medidaMEAS4;


    // file descriptor para probar la biblioteca, el file descriptor que pasaria usuario
    int file= open("/dev/i2c-1", O_RDWR);
    if (file < 0) {
        perror("Failed to open /dev/i2c-1");
        return -1;
    }
    //fin file descriptor para probar biblioteca








    // Inicializacion:
    int err= capacimeter_init(&file, capdac_offset, CUATROCIENTAS_Ss); // CUATROCIENTAS_Ss: sample rate, esta en enum en nivelLib.h.
    if(err!= 0){
        printf("error inicializando.\n");
        return -1;
    }
    usleep(100000);


    



    
    







    capacimeter_config(CUATROCIENTAS_Ss, tipoDeMedida);

    int offset_autoEncontrado;
    
    printf("MEDIDA CON AUTO-OFFSET:\n");
    for(int i=0; i<4; i++){
        offset_autoEncontrado= capacidad_autooffset(&capacidad, tipoDeMedida);
        printf("CAPACIDAD: %f, OFFSET: %d    <<<<<<<<<<<<<<<<\n", capacidad+offset_autoEncontrado*3.125, offset_autoEncontrado);
        usleep(500000);
    }
    
    printf("\n\n");
    
    












    ///TOMAR MEDIDA DE CAPACIDAD sin autoOffset:
    
    printf("MEDIDA UNICA SIN AUTO-OFFSET:\n");
    
    MEASn_capdac_config(offset_autoEncontrado, tipoDeMedida);
    capacimeter_config(CUATROCIENTAS_Ss, tipoDeMedida);   // nueva

    usleep(8000);   //debug
    
    for(int i=0; i<2; i++){

        capacidad_medida_single(&capacidad, tipoDeMedida);
        
        //chequeo si no saturo:
        if(15.98<capacidad){
            printf("Saturo. Capacidad muy alta para el offset seteado.\n");
        }else if(capacidad<-15.98){
            printf("Saturo. Capacidad muy baja para el offset seteado.\n");
        }else
            printf("capacidad sin filtro con offset manual: %.2fpF\n", capacidad+offset_autoEncontrado*3.125);

        usleep(500000);

    }
    printf("\n\n");













    // MEDIDAS DIFERENCIALES:

    printf("MEDIDAS DIFERENCIALES CIN2-CIN3:\n");

    capacimeter_config(CUATROCIENTAS_Ss, medidaDIFERENCIAL);
    usleep(8000);

    for(int i=0; i<2; i++){
        capacidad_medida_single(&capacidad, medidaDIFERENCIAL);
        printf("capacidad DIFERENCIAL= %fpF\n", capacidad);
        usleep(500000);
    }


    printf("\n\n");












    
    ///BLOQUE QUE MUESTRA MEDIDAS FILTRADAS.

    printf("MEDIDAS PROMEDIADAS:\n");

    MEASn_capdac_config(offset_autoEncontrado, tipoDeMedida);
    capacimeter_config(CUATROCIENTAS_Ss, tipoDeMedida);

    media_confiabilidad_nivel medidaProcesada;   // En esta estructura se guarda la media, la desviacion y el porcentaje de muestras utiles
    float desviacion_aceptable= 0.1;  // en [pF].// respecto del total, al llamar a read_processedData.
    int cantidadMuestras      = 20;                       // Cantidad de muestras que se leeran y procesaran.

    for(int i=0; i<2; i++){
        
        usleep(500000);
        // LEER "cantidadMuestras" Y PROCESAR:
        err= read_processed_cap_pF(tipoDeMedida, desviacion_aceptable, cantidadMuestras, &medidaProcesada);
        
        //chequeo si no saturo:
        if(15.98<medidaProcesada.media){
            printf("Saturo. Capacidad muy alta para el offset seteado.\n");
        }else if(medidaProcesada.media<-15.98){
            printf("Saturo. Capacidad muy baja para el offset seteado.\n");
        }else{
            if(err==0){
                printf("capacidad promediada= %.2fpF\n", medidaProcesada.media+offset_autoEncontrado*3.125);  //usar para nivel
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

