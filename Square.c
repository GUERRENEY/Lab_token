#include <stdio.h>
#include <sys/time.h>



struct timeval stop, start;
unsigned long tiempo; //4 bytes, entonces la clave de 4 bytes
unsigned long clave = 0xF10ABCD0;
unsigned long tokenLocal=0;

void imprimeCuadrado();

int main(){
    printf("Pulse enter para sincronizar.....");
    getchar();
    imprimeCuadrado();
    gettimeofday(&start,NULL);
    printf("Ingrse su TOKEN>>>");
    getchar();
    gettimeofday(&stop,NULL);
    tiempo = ((stop.tv_sec -start.tv_sec)*1000000 + stop.tv_usec -start.tv_usec);
    tiempo/=1000.0;
    tokenLocal = tiempo^clave;
    printf("Tiempo %lu ms\n",tiempo);
    printf("Tiempo %08X \n",tiempo);
    printf("Clave %08X \n",clave);
    printf("Token %08X \n, tokenLocal");


}

void imprimeCuadrado(){
     unsigned char dato = 219;
     int i = 0;
     for(i=1; i < 289; i++){
        printf("%c",dato );
       if(0==(i%24))printf("\n");
     }

}