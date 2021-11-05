#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include "Libs/pidLib.h"
#include "Libs/presion.h"


int Log(const char* nombre, float valor);       // usada para loguear y corroborar el funcionamiento del pid. No necesaria.


int main(){



    /*
     * Este bloque no es parte de la biblioteca, es para testearla:
     *
    */
    double presion; // no es necesario para el pid, lo uso para leer la presion y controlar.

    //file descriptor cryocooler para probar la biblioteca, el file descriptor que pasaria usuario
    int fs_cryo= -1;
    char puerto[]="/dev/ttyUSB0";
    fs_cryo = open(puerto, O_RDWR | O_NOCTTY);
    if (fs_cryo == -1)
    {
        printf("Error abriendo %s.\n", puerto);
        return -1;
    }

    //file descriptor sensor presion para probar la biblioteca, el file descriptor que pasaria usuario
   int fs_presion= open("/dev/i2c-1", O_RDWR);
   if (fs_presion < 0) {
       perror("Failed to open /dev/i2c-2");
       return -1;
   }
   //fin file descriptor para probar biblioteca

   /*
     * Este bloque no es parte de la biblioteca, es para testearla.
     *
    */









    if(pid_presion_cryo_ini(fs_presion, fs_cryo)!=0){  // Inicializa el pid
        printf("fallo inicializando.\n");
        return -1;
    }
    sleep(1);
    
    
    errores_cryo err_cryo[6];
    if(read_cryo_error(fs_cryo, err_cryo)!=0){
        printf("error leyendo errores.\n");
        return -1;
    }
    
    
    
    // Una vez inicializado, ejecutar pid_compute() cada cierto tiempo (tiempo de muestreo= 2 seg por defecto,
    //  definido en macro en pidLib.c). pid_compute() calcula y comanda la salida hacia el cryocooler.


    

    for(int i=0; i<900; i++)        // 900 iteraciones de 1 segundo cada una son 15 minutos durante los que controla.
    {

        if (pid_need_compute()){                // chequea si ya paso un tiempo mayor al tiempo de muestreo.
            pid_compute();                      // Calcula y comanda la salida.
        }



        // no necesario, usado para ver que el controlador funcione correctamente:
        PRESION_leer_presion_bar(&presion);
        Log("log_presion.txt", presion);
        printf("presion: %.2lf\n", presion);
        sleep(1);
        // no necesario, usado para ver que el controlador funcione correctamente.
    
    }


    close(fs_cryo);
    close(fs_presion);

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
