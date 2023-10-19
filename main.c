#include <stdio.h>
#include "pico/stdlib.h"
#include "mpu9265.h"

int main(void) {
    stdio_init_all();
    //asigno gpio_4 como linea sda
	//gpio_set_function(4, GPIO_FUNC_I2C);
	//asigno gpio_5 como linea scl
	//gpio_set_function(5, GPIO_FUNC_I2C);
	//inicializo i2c	
	//i2c_init(i2c0, 100 * 1000);
	//i2c_set_baudrate(i2c0, 100000);

	//Variables de aceleracion de los tres ejes
	float ax, ay, az;

	//Almacena la salida del acelerometro asignada de 16-Bit
	int16_t accelCount[3];  
	
	//inicializo acelerometro 
	//mpu9265_setup();

	while (1){
		//Obtengo los valores del acelerometro
	//	readAccelData(accelCount);  // Leo the x/y/z adc valores
	//	getAres();
		//Calculo los valores de aceleracion 
	//	ax = (float)accelCount[0]*aRes; // - accelBias[0];
	//	ay = (float)accelCount[1]*aRes; // - accelBias[1];
	//	az = (float)accelCount[2]*aRes; // - accelBias[2];
	//	printf("ax: %f\n\r", ax);
    //    printf("ay: %f\n\r", ay);
    //    printf("az: %f\n\r", az);
        printf("Hello world!\r\n");
        sleep_ms(500);

    }



}

