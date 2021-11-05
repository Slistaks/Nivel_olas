
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include "Libs/presion.h"


int Log(const char* nombre, float valor);

int main()
{

    // file descriptor para probar la biblioteca, el file descriptor que pasaria usuario
    int filePRESION= open("/dev/i2c-1", O_RDWR);
    if (filePRESION < 0) {
        perror("Failed to open /dev/i2c-1");
        return -1;
    }
    //fin file descriptor para probar biblioteca





    int err= PRESION_init(filePRESION);
    if(err!=0){
        printf("error inicializando presion.\n");
        return -1;
    }


                             /// PRESION PROCESADA:

   float desviacionAceptable_bar= 0.001;
   int cantMuestrasPRESSURE= 100;
   media_deviation_relation_struct_pressure estructuraEjemploPresion;
    
    printf("Presion varias muestras:\n\n");
   for(int j=0; j<2; j++){

       PRESION_read_processedData(desviacionAceptable_bar, cantMuestrasPRESSURE, &estructuraEjemploPresion);

       printf("Media: %.2f bar\n", estructuraEjemploPresion.media);
       printf("Confianza: %.2f\n\n", estructuraEjemploPresion.relation);
       sleep(1);

   }                         
                            /// PRESION PROCESADA.




                            /// PRESION SIN PROCESAR:
    printf("Presion de a una sola medida:\n\n");
   double presion;
   for(int m=0; m<2; m++){
       PRESION_leer_presion_bar(&presion);
       printf("presion: %.4f\n", presion);
       sleep(1);
   }
                           /// PRESION SIN PROCESAR.




    close(filePRESION);

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



