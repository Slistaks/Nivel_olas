


#include <math.h>
#include "nivelLib.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

#ifndef I2C_M_RD
#include <linux/i2c.h>
#endif

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>


#define REPEAT_CONVERSION               0x100       // termina una conversion e inicia otra automaticamente
#define RST_FLAG                        (1<<7)
#define CAP_A_NIVEL_PENDIENTE           15.173      //calculado de ensayo. mm/pF.
#define CAP_A_NIVEL_OFFSET              373.49      //calculado de ensayo.es negativo, pero el menos esta en la expresion donde se lo usa. mm.



// Usar este enum como parametros de entrada para las funciones cuya descripcion lo aclara.
enum cap_config{SINGLE=0, REPEAT=0x100};


unsigned char multiMedidas_f;     // cuando se mide capacitor nivel y ademas capacitores referencia liquido aire.
int multiMedidasEnable(unsigned char enable); // habilita que se mida primero el capacitor de nivel y despues el de referencia.
    

float capacidad_pF_a_nivel_mm(float capacidad);








// Lee el registro recibido como parametro.
//      file_p              : Puntero al bus I2C.
//      rxBuffer            : Puntero al array de datos donde se guarda lo recibido por I2C.
//      txBuffer            : Puntero al array de datos a enviar por I2C.
//      registro            : Registro fuente de la lectura.
// Retorna:
//      0 : No hubo error.
//     -3 : Fallo en la escritura i2c.
//     -4 : Fallo en la lectura i2c.
int capacimeter_read(int* file_p, char* rxBuffer_p, char* txBuffer_p, enum reg_resultado reg);
//int capacimeter_read(int* file_p, char* rxBuffer_p, char* txBuffer_p, unsigned char reg);




// Lee el flag de conversion completa, y retorna su valor, o informacion del error en caso que ocurra.
//      file_p              : Puntero al bus I2C.
// Retorna:
//      0 : La medida no esta lista para ser leida.
//      1 : La medida esta lista para ser leida.
//     -3 : Fallo en la escritura i2c.
//     -4 : Fallo en la lectura i2c.
int capacimeter_done(int* file_p, enum tipo_medida tipoMedida);






// Escribe en un registro del FDC1004.
// Parametros:
//      file_p              : Puntero al bus I2C.
//      rxBuffer            : Puntero al array de datos donde se guarda lo recibido por I2C.
//      txBuffer            : Puntero al array de datos a enviar por I2C.
//      reg                 : Registro destino de la escritura.
// Retorna:
//      0 : No hubo error.
//     -3 : Fallo en la escritura i2c.
int capacimeter_write(int* file_p, char* rxBuffer_p, char* txBuffer_p, int reg);




/*
// Usar en su lugar el log definido en el main (mainNivel.c).
// Realiza la cantidad de medidas que se le pase como parametro, y las escribe en un archivo de texto
// llamado "Log.txt", al cual sobreescribe en cada llamada a la funcion (si se desean tomar varios vectores
// de datos, se debe guardar el archivo "Log.txt" antes de realizar otra llamada a la funcion).
//
// Parametros:
//      file_p                  : Puntero al bus I2C.
//      cantidad_medidas        : Cantidad de medidas a realizar.
// Retorna:
//      0 : No hubo error.
//     -6 : La conversion no se completa en un tiempo razonable.
//     -7 : Error en lectura de registro.
//     -8 : Error al intentar abrir fichero del log.
int readAndLog_cap(int* file_p, int cantidad_medidas);
*/







// Calcula la desviacion estandar muestral, para un vector "muestras" de datos de tama�o "tam".
// Parametros:
//      muestras: Puntero al array de datos.
//      tam     : Tama�o del array de datos.
// Retorna:
//      >0 : Desviacion estandar muestral: . |float|
//      -11: Error de division por cero.
//      -12: Error en calcularMedia en calcularDesviacion.
float calcularDesviacion(int* muestras, int tam);









// Calcula la media muestral, para un vector "muestras" de datos de tama�o "tam".
// Parametros:
//      muestras: Puntero al array de datos.
//      tam     : Tama�o del array de datos.
// Retorna:
//      >0 : media muestral: float.
//      -11: Error de division por cero.
float calcularMedia(int* muestras, int tam);




int errorr;









int fs_nivel;

int mode_g, sampleRate_g, capdac_offset_g;  // Globales, para recargar la configuracion anterior en reset.


int capacimeter_init(int* file_p, int capdac_offset, enum sample_rate sampleRate){
    
    fs_nivel= *file_p;
    if (ioctl(fs_nivel, I2C_SLAVE, FDC1004ADDRESS) == -1) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        return -1;
    }
    
    if(capacimeter_config(capdac_offset, sampleRate, medidaNIVEL)!=0){              // por defecto inicia habilitada una sola medicion.
    	printf("capacimeter_config failed.\n");
    	return -2;
    }

    return 0;
}




int capacimeter_read(int* file_p, char* rxBuffer_p, char* txBuffer_p, enum reg_resultado reg){         // Parametro es el reg a leer
//int capacimeter_read(int* file_p, char* rxBuffer_p, char* txBuffer_p, unsigned char reg){         // Parametro es el reg a leer//debug, la de arriba es la real
                            
                            //ESCRIBO EL POINTER REG ANTES DE LEER:
                            
    (*txBuffer_p) = reg;
    if (write(*file_p, txBuffer_p, 1) != 1) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n");
        return -3;
    }
    usleep(50);

                            //LEO LO APUNTADO POR POINTER REG:
    if (read(*file_p,rxBuffer_p,2) != 2) {
        printf("Failed to read from the i2c bus.\n");
        return -4;
    }
    usleep(50);
                                                // FIN LEER.
    return 0;
    
}




int capacimeter_write(int* file_p, char* rxBuffer_p, char* txBuffer_p, int reg){          //Llenar el buffer antes-> tx[1]= MSB, tx[2]=LSB (tx[0] es el reg, parametro recibido)

                            //ESCRIBO EL POINTER REG ANTES DE LEER:
    *txBuffer_p = reg;

    if (write(*file_p,txBuffer_p,3) != 3) {
        // ERROR HANDLING: i2c transaction failed
        printf("Failed to write to the i2c bus.\n");
        return -3;
    }
    usleep(50);

    return 0;
}





int capacimeter_done(int* file_p, enum tipo_medida tipoMedida){ // retorna el estado de la conversion (1 o 0) y <0 en caso de error.
    
    char txBuffer[3];
    char rxBuffer[3];
    *txBuffer= 0x0C; //[0]
    
    usleep(50);
    if (write(*file_p,txBuffer,1) != 1) {
        // ERROR HANDLING: i2c transaction failed
        printf("Failed to write to the i2c bus.\n");
        return -3;
    }else{
        usleep(50);
        if (read(*file_p,rxBuffer,2) != 2) {
            printf("Failed to read from the i2c bus.\n");
            return -4;
        } else {

            //return ( (multiMedidas_f==1) ? (rxBuffer[1] & 0b100) : (rxBuffer[1] & 0b1000) );    //antiguo
            return tipoMedida & rxBuffer[1];

        }
    }
}




int capacimeter_config(int capdac_offset, enum sample_rate sampleRate, enum tipo_medida tipoMedida){
    
    char txBuffer[3];
    char rxBuffer[3];
    multiMedidas_f= (tipoMedida==medidaDIFERENCIAL) ? 1 : 0;    // Este flag lo uso por fuera para saber en que
                                                                // modo esta.
    ///config global para todos los tipos de mediciones:
    mode_g= REPEAT;
    sampleRate_g= sampleRate;
    
    
    
    
    ///config meas1 (medicion de cap de nivel):
    capdac_offset_g= capdac_offset;
    
    u_int16_t reg_CONF_MEAS1_= capdac_offset<<5;
    reg_CONF_MEAS1_|= 0x1000; // capdac enabled.

    //Configuracion para mediciones:
    txBuffer[1]= (reg_CONF_MEAS1_ & 0xFF00)>>8; //MSB     //txBuffer[1]
    txBuffer[2]= (reg_CONF_MEAS1_ & 0xFF  );    //LSB     //Configuro el registro 0x08. txBuffer[2]
    
    //capacimeter(WRITE, 0x08);
    if( capacimeter_write(&fs_nivel, rxBuffer, txBuffer, 0x08) != 0 ){
        printf("Failed capacimeter_write first call, in capacimeter_config.\n");
        return -5;
    }
    



    // AGREGO OTRO CANAL PARA MEDIDA ABSOLUTA: CANAL 4


/*

    ///config meas4 (medicion de cap de nivel):
    capdac_offset_g= capdac_offset;
    
    u_int16_t reg_CONF_MEAS1_= capdac_offset<<5;
    reg_CONF_MEAS1_|= 0x1000; // capdac enabled.

    //Configuracion para mediciones:
    txBuffer[1]= (reg_CONF_MEAS1_ & 0xFF00)>>8; //MSB     //txBuffer[1]
    txBuffer[2]= (reg_CONF_MEAS1_ & 0xFF  );    //LSB     //Configuro el registro 0x08. txBuffer[2]
    
    //capacimeter(WRITE, 0x08);
    if( capacimeter_write(&fs_nivel, rxBuffer, txBuffer, 0x08) != 0 ){
        printf("Failed capacimeter_write first call, in capacimeter_config.\n");
        return -5;
    }


*/
    //terminar. agregar a la funcion leer para que lea el canal que se seleccione.s

    // FIN DE AGREGAR OTRO CANAL




    
    
    
    ///config meas2 (medicion diferencial capacitor liquido - capacitor aire):
    u_int16_t reg_CONF_MEAS2_= (0b001<<13) | (0b010<<10); //| (CHA->cin2) | (CHB->cin3)    // modifique y no tenia que. corregir.

    //Configuracion para mediciones:
    txBuffer[1]= (reg_CONF_MEAS2_ & 0xFF00)>>8; //MSB     //txBuffer[1]
    txBuffer[2]= (reg_CONF_MEAS2_ & 0xFF  );    //LSB     //Configuro el registro . txBuffer[2]
    
    //capacimeter(WRITE, );
    if( capacimeter_write(&fs_nivel, rxBuffer, txBuffer, 0x09) != 0 ){
        printf("Failed capacimeter_write first call, in capacimeter_config.\n");
        return -5;
    }
    
    
    
    


    ///inicio las mediciones:
    //config de reg 0x0C
    u_int16_t reg_0x0C_= sampleRate | REPEAT | ( (tipoMedida==medidaDIFERENCIAL) ? (0x80 | 0x40) : 0x80 );      // 0x80->meas1 enable. 0x40->meas2 enable.
    
    //printf("reg C en config: %d\n\n", reg_0x0C_);   //debug
    txBuffer[1]= (reg_0x0C_ & 0xFF00)>>8; //MSB
    txBuffer[2]= (reg_0x0C_ & 0xFF  );    //LSB         //Configuro el registro 0x0C
    //capacimeter(WRITE, 0x0C);
    if( capacimeter_write(&fs_nivel, rxBuffer, txBuffer, 0x0C) != 0 ){
        printf("Failed capacimeter_write call, in capacimeter_config.\n");
        return -5;
    }

    return 0;
}




int multiMedidasEnable(unsigned char enable){ // habilita que se mida primero el cap de nivel y despues el de referencia, o deshabilita y mide solo el primer cap.
    
    
    if(! ((enable==1)||(enable==0)) ){
        printf("Parametro de funcion multiMedidasEnable invalido.\n");
        return -1;
    }

    
    unsigned char txBuffer[3];
    unsigned char rxBuffer[3];
    
    multiMedidas_f= enable;        // variable global que se usa fuera para esperar o leer lo que corresponda.
    //config de reg 0x0C
    u_int16_t reg_0x0C_= sampleRate_g | REPEAT | 0x80 | (enable<<6);      // 0x80 -> meas1 enable. (enable<<6) = enable*0x40 -> meas2 enable.
    txBuffer[1]= (reg_0x0C_ & 0xFF00)>>8; //MSB
    txBuffer[2]= (reg_0x0C_ & 0xFF  );    //LSB         //Configuro el registro 0x0C
    
    //capacimeter(WRITE, 0x0C);
    if( capacimeter_write(&fs_nivel, rxBuffer, txBuffer, 0x0C) != 0 ){
        printf("Failed capacimeter_write second call, in capacimeter_config.\n");
        return -5;
    }
    
    
    return 0;
}




/*

int readAndLog_cap(int* file_p, int cantidad_medidas){
    
    char txBuffer[3];
    char rxBuffer[3];
    FILE *fichero;
    if( (fichero = fopen("Log.txt", "a+")) == NULL ){
        printf("Error al intentar abrir fichero del log.\n");
        return -8;
    }


    int medida=0;
    char coma= ',';
    char str[9];
    char strcap[9];
    int j=0;
    int err;
    int ret;
    float capacidadRaw;

    for(int i=0; i<cantidad_medidas; i++){
    
    
        ret= capacimeter_done(&fs_nivel);  //cambiar
        if(ret<0){
            printf("error leyendo flag done.\n");
            return -9;
        }
        if(ret==0){
            usleep(3000);   //esperar 3ms y reintentar
            ret= capacimeter_done(&fs_nivel);  //cambiar
            if(ret==0){
                printf("flag conversion completa no se setea.\n");
                return -10;
            }
            if(ret<0){
                printf("error leyendo flag done.\n");
                err= reset(&fs_nivel);
                if(err!=0){
                    printf("error en reset.\n");
                    return -8;
                }
                return -9;
            }
        }
        
        
        
        //Leo:
        //capacimeter(READ, 0);       //Leo registro alto reg0
        if( (capacimeter_read(file_p, rxBuffer, txBuffer, reg_RESULTADO_NIVEL)) != 0 ){             //cambiar
            printf("Error en lectura de registro en readAndLog function.\n");
            return -7;
        }
        medida= (*(rxBuffer) << 16) + (*(rxBuffer+1) << 8);

        //capacimeter(READ, 1);        //Leo registro bajo reg1
        if( (capacimeter_read(file_p, rxBuffer, txBuffer, reg_RESULTADO_NIVEL+1)) != 0 ){           //cambiar
            printf("Error en lectura de registro en readAndLog function.\n");
            return -7;
        }
        medida= medida + *rxBuffer;   //Los 8LSB son reservados, no son parte del resultado (rxBuffer[1] no interesa).


        // CONVERTIR MEDIDA A STRING:

        //SIGNO:
        if( medida & 0x800000 ){
            medida= ~medida+1;
            medida= medida & 0xffffff;
            medida= -medida;
        }
        //SIGNO.
        capacidadRaw= (float)medida/(1<<19);


        sprintf(str, "%d", medida);
        sprintf(strcap, "%.3f", capacidadRaw);
        //printf("%s\n", strcap);                   DEBUG, descomentar desp
        j=0;
        while(strcap[j]!='\0'){
            fwrite(&strcap[j], sizeof(char), 1, fichero);
            j++;
        }
        fwrite(&coma, sizeof(coma), 1, fichero);

        //sleep(1);                                   // Poner el tiempo entre medidas (es para debug, despues se borra).
    }

    //fwrite("\n", 1, 1, fichero);
    fclose(fichero);

    return 0;
}
*/



int read_processed_cap_pF(enum tipo_medida tipoMedida, float desviacion_aceptable_pF, int vectorSize, struct media_confiabilidad_nivel_struct *salidaStruct_p){
    
    char txBuffer[3];
    char rxBuffer[3];
    int medida=0;
    int err;
    int ret;
    int muestras[vectorSize];
    float desviacion_aceptable= (desviacion_aceptable_pF)*(1<<19);    //Convierto pF a cuentas del adc.


    unsigned char regResultado= reg_RESULTADO_NIVEL; // cambiar
    if(tipoMedida==medidaDIFERENCIAL){
        regResultado= reg_RESULTADO_DIFERENCIAL;
    }


    // TOMO LAS N MUESTRAS:
    for(int i=0; i<vectorSize; i++){

        
        
        ret= capacimeter_done(&fs_nivel, tipoMedida);  //cambiar
        if(ret<0){
            printf("error leyendo flag done.\n");
            return -9;
        }
        if(ret==0){
            usleep( multiMedidas_f ? 5500 : 3000);   //esperar 3ms y reintentar (si se toman dos tipos de medidas) (cambiar)
            ret= capacimeter_done(&fs_nivel, tipoMedida);  //cambiar
            if(ret==0){
                printf("flag conversion completa no se setea.\n");
                err= reset(&fs_nivel);
                if(err!=0){
                    printf("error en reset.\n");
                    return -8;
                }
                return -10;
            }
            if(ret<0){
                printf("error leyendo flag done.\n");
                err= reset(&fs_nivel);
                if(err!=0){
                    printf("error en reset.\n");
                    return -8;
                }
                return -9;
            }
        }
        
        
        
        //Leo:
        //capacimeter(READ, 0);       //Leo registro alto reg0
        if((capacimeter_read(&fs_nivel, rxBuffer, txBuffer, regResultado)) != 0){    //cambiar
            printf("Error en lectura de registro en read_processedData.\n");
            return -7;
        }
        medida= (rxBuffer[0] << 16) + (rxBuffer[1] << 8);

        //capacimeter(READ, 1);        //Leo registro bajo reg1
        if( (capacimeter_read(&fs_nivel, rxBuffer, txBuffer, regResultado+1)) != 0 ){    //cambiar
            printf("Error en lectura de registro en read_processedData.\n");
            return -7;
        }
        medida= medida + *rxBuffer;   //Los 8LSB son reservados, no son parte del resultado (rxBuffer[1] no interesa).

        muestras[i]= medida;

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
    int muestra_signada;
    while(1){      //Se hace iteraciones hasta obtener el vector final con desviacion<desviacion_aceptable.

        if( (desviacion=calcularDesviacion(muestrasUtiles,cantMuestrasUtiles)) < 0 ){
            printf("Error en calculo de desviacion en read_processedData.\n");
            return -11;
        }
        errorr=0;
        media=calcularMedia(muestrasUtiles, cantMuestrasUtiles);
        if(errorr){
            printf("Error en calculo de media en read_processedData.\n");
            return -12;
        }

        if(desviacion<desviacion_aceptable)
            break;
        if(cantMuestrasUtiles<=2)
            break;


        for(int i=0; i<cantMuestrasUtiles; i++){
            muestrasUtilesBuffer[i]=muestrasUtiles[i];
        }
        j=0;

        for(int i=0; i<cantMuestrasUtiles; i++){

            if( muestrasUtilesBuffer[i] & 0x800000 ){
                muestra_signada= ~muestrasUtilesBuffer[i]+1;
                muestra_signada= muestra_signada & 0xffffff;
                muestra_signada= -muestra_signada;
            }else{
                muestra_signada= muestrasUtilesBuffer[i];
            }


            if( abs(muestra_signada-media)<desviacion ){
                muestrasUtiles[j]=muestrasUtilesBuffer[i];
                j++;
            }
            
        }
        cantMuestrasUtiles=j;
    }
    
    
    float capacidad= (float)media/(1<<19);
    

    // Nota 1: Si se descomenta la linea de abajo, la salida es la capacidad absoluta.
    //capacidad= capacidad + 3.125*capdac_offset_g;

    salidaStruct_p->media         = capacidad;
    salidaStruct_p->confiabilidad = cantMuestrasUtiles*100/vectorSize;    // (cantMuestrasUtiles/totales)*100%.
    
    return 0;
}






int capacidad_autooffset(float* capacidad){             // la medida diferencial no usa capdac (offset interno) por lo que no tiene sentido ejecutar
                                                        // esta funcion para medidas diferenciales.
    

    

    float cap;
    unsigned char saltoOffset= 0b01000;                 // busqueda dicotomica, salto se va a ir dividiendo por 2
    unsigned char capdac_offset= 0b10000;               //    0 (0pF) < capdac (5 bits) < 31 (96.875pF)
    unsigned char multiMedidas= multiMedidas_f;         // si antes estaba configurado para multiples medidas, guardo aca y al finalizar la funcion restauro la config.
    unsigned char fueraDeRangoSuperior;                 // si se va de rango completo, con esto se si fue fuera del limite superior o inferior.

    capacimeter_config(capdac_offset, CUATROCIENTAS_Ss, medidaNIVEL); //comentado por debug.
    // la linea anterior, si se descomenta, el bit done se setea pero la capacidad que mide da mal, da mucho mayor.
    


    usleep(3000);   //tiempo mayor a un Ts. // primer medida da muy alto, no funciona. descartar primer medida? : lo de descartar la primer medida funciona.
    capacidad_medida_single(&cap, medidaNIVEL);  //descarto la primer muestra -> funciono.
    
    saltoOffset*=2; //debug. desplazo a izda para compenzar el primer desplazamiento a dcha.

    while(1){     //mientras este saturado, corrige offset.
        

        //debug:
        if(saltoOffset==0){
            *capacidad= cap;
            // reconfiguro tipo de medicion que habia antes de entrar a esta funcion:
            multiMedidasEnable(multiMedidas);
            return capdac_offset;
        }

        saltoOffset/= 2;
        capacimeter_config(capdac_offset, CUATROCIENTAS_Ss, medidaNIVEL);
        usleep(3000);   //debug
        //debug.

        capacidad_medida_single(&cap, medidaNIVEL);
        
        //printf("\n___________\ncapacidad= %f\ncapdac_offset= %d\nsaltoOffset= %d\n___________\n", cap, capdac_offset, saltoOffset);   //debug   //debug
        

        //chequeo si no saturo, si saturo modifico el offset:
        if(15.998<cap){
            capdac_offset+= saltoOffset;
        }else if(cap<-15.998){
            capdac_offset-= saltoOffset;
            
        }else{
            *capacidad= cap;     // si entro aca, no saturo, fin.

            // reconfiguro en la forma que habia antes de entrar a esta funcion:
            multiMedidasEnable(multiMedidas);
            return capdac_offset;
        }


    }

    
}







int capacidad_medida_single(float* cap, enum tipo_medida tipoMedida){
    
    
    char txBuffer[3];
    char rxBuffer[3];
    int ret;
    int err;
    int medida=0;
    
    
    unsigned char registroResultado= reg_RESULTADO_NIVEL;

    if(tipoMedida==medidaDIFERENCIAL){
        registroResultado= reg_RESULTADO_DIFERENCIAL;
        //printf("registro configurado en diferencial, deberia ser: 2. si es 0 esta mal\n");  //debug
    }
    
    //printf("registroResultado= %d\n #######", registroResultado);   ////debug
    
    ret= capacimeter_done(&fs_nivel, tipoMedida);  //cambiar
    if(ret<0){
        printf("error leyendo flag done.\n");
        return -9;
    }
    if(ret==0){
        
        usleep( multiMedidas_f ? 5500 : 3000 );   //esperar 3ms y reintentar
        
        ret= capacimeter_done(&fs_nivel, tipoMedida);  //cambiar
        if(ret==0){
            printf("flag conversion completa no se setea.\n");
            err= reset(&fs_nivel);
            if(err!=0){
                printf("error en reset.\n");
                return -8;
            }
            return -10;
        }
        if(ret<0){
            printf("error leyendo flag done.\n");
            err= reset(&fs_nivel);
            if(err!=0){
                printf("error en reset.\n");
                return -8;
            }
            return -9;
        }
    }
    
    
    
    //Leo:
    //capacimeter(READ, );       //Leo registro alto
    if( (capacimeter_read(&fs_nivel, rxBuffer, txBuffer, registroResultado)) != 0 ){  //cambiar
        printf("Error en lectura de registro.\n");
        return -7;
    }
    medida= (*(rxBuffer) << 16) + (*(rxBuffer+1) << 8);

    //capacimeter(READ, 1);        //Leo registro bajo
    if( (capacimeter_read(&fs_nivel, rxBuffer, txBuffer, registroResultado+1)) != 0 ){    //cambiar
        printf("Error en lectura de registro.\n");
        return -7;
    }
    medida= medida + *rxBuffer;   //Los 8LSB son reservados, no son parte del resultado (rxBuffer[1] no interesa).
    
    // CONVERTIR MEDIDA A STRING:
    
    //SIGNO:
    if( medida & 0x800000 ){
        medida= ~medida+1;
        medida= medida & 0xffffff;
        medida= -medida;
    }
    //SIGNO.
    
    *cap= (float)medida/(1<<19);

    // Nota 2: Si se descomenta la linea de abajo, la salida es la capacidad absoluta.
    //*cap= *cap + 3.125*capdac_offset_g;

    return 0;
    
}























/*
int nivel_medida_single(float* nivel){
    
    
    float capacidad;
    if(capacidad_medida_single(&capacidad, )!=0){
        printf("Error leyendo capacidad para obtener nivel.\n");
        return -1;
    }
    
    *nivel= (capacidad + 3.125*capdac_offset_g)*CAP_A_NIVEL_PENDIENTE - CAP_A_NIVEL_OFFSET;
    
    return 0;
    
}
*/
















float capacidad_pF_a_nivel_mm(float capacidad){
    
    return (capacidad + 3.125*capdac_offset_g)*CAP_A_NIVEL_PENDIENTE - CAP_A_NIVEL_OFFSET;
    
}





int reset(int* file_p){
    
    char txBuffer[3];
    char rxBuffer[3];
    
    // Reset:
    txBuffer[1] = RST_FLAG;     //msb= rst
    txBuffer[2] = 0x80;         //lsb no importa, tras el reset hay que reconfigurar el ic.
    int err= capacimeter_write(file_p, rxBuffer, txBuffer, 0x0C);    // Envia reset.
    if(err!=0){
        printf("error reseteando.\n");
        return -1;
    }
    usleep(100);
            
    err= capacimeter_config(capdac_offset_g, sampleRate_g, (multiMedidas_f ? medidaDIFERENCIAL : medidaNIVEL));  // reconfiguro por el reset.
    if(err!=0){
        printf("error reconfigurando.\n");
        return -2;
    }
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
