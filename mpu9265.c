#include "mpu9265.h"

enum Ascale_values {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale_values {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale_values {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;

uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution

float beta = 0.8660254 * GyroMeasError; //sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = 0.8660254 * GyroMeasDrift; //sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

float deltat = 0, sum = 0;        // integration interval for both filter schemes

float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer

float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias

float magBias[3] = {0 ,0 ,0}, magScale[3] = {0, 0, 0};

float aRes, gRes;      // scale resolutions per LSB for the sensor

float   SelfTest[6];    // holds results of gyro and accelerometer self test

void mpu9265_setup()
{     	
  printf("MPU9265\n\r");
  printf("9-DOF 16-bit\n\r");
  printf("motion sensor\n\r");
  printf("60 ug LSB\n\r");
  sleep_ms(800);

  // Read the WHO_AM_I register, this is a good test of communication
  //readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  uint8_t whoami = 0;
  uint8_t datitos19[1] = {WHO_AM_I_MPU9265}; 
  i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos19, 1, false);  // Read WHO_AM_I register for MPU-9250
  printf("MPU9265\n\r I AM %x I should be 0x71\n\r" , whoami);
  ////  display.setCursor(20,0);
  ////  display.setCursor(20,0);
//  UART_printf("MPU9250");
////  display.setCursor(0,10);
//  UART_printf("I AM");
////  display.setCursor(0,20);
//  UART_printf(c, HEX);  
////  display.setCursor(0,30);
//  UART_printf("I Should Be");
////  display.setCursor(0,40);
//  UART_printf(0x71, HEX); 
//  display.display();
  sleep_ms(800); 
  if (whoami == 0x71) // WHO_AM_I should always be 0x68
  {  
    printf("MPU9250 is online...\n\r");
    MPU9265SelfTest(SelfTest); // Start by performing self test and reporting values
    printf("x-axis self test: acceleration trim within : %d\% of factory value\n\r" 	, SelfTest[0]);
    printf("y-axis self test: acceleration trim within : %d\% of factory value\n\r" 	, SelfTest[1]);
    printf("z-axis self test: acceleration trim within : %d\% of factory value\n\r" 	, SelfTest[2]);
    printf("x-axis self test: gyration trim within : %d\% of factory value\n\r" 	, SelfTest[3]); 
    printf("y-axis self test: gyration trim within : %d\% of factory value\n\r"	, SelfTest[4]);
    printf("z-axis self test: gyration trim within : %d\% of factory value\n\r"	, SelfTest[5]);

    calibrateMPU9265(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

    printf("MPU9265 bias\n\r");

    printf(" x   y   z\n\r");


    printf("%d %d %d mg\n\r" , (int)(1000*accelBias[0]) , (int)(1000*accelBias[1]) , (int)(1000*accelBias[2])); 

    printf("%d %d %d °/s\n\r" , gyroBias[0] , gyroBias[1] , gyroBias[2]); 

    sleep_ms(1000); 
    initMPU9265(); 
    printf("MPU9265 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    uint8_t BypassTrue = 0;
    while(BypassTrue == 0)
      {
        uint8_t datitos1[4] = {INT_PIN_CFG, 0x22, INT_ENABLE, 0x01 }; 
        i2c_write_blocking(i2c0, MPU9265_ADDRESS,  datitos1, 4, false);  // Enable data ready (bit 0) interrup
        uint8_t datitos20[1] = {INT_PIN_CFG};
        uint8_t PinCFG = i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos20, 1, false);	//0x22);
        uint8_t datitos21[1] = {I2C_MST_CTRL};
        uint8_t MasterDis = i2c_read_blocking(i2c0, MPU9265_ADDRESS,  datitos21, 1, false); //0x00); // Disable I2C master
        uint8_t datitos22[1] = {INT_ENABLE};
        uint8_t IntEna = i2c_read_blocking(i2c0, MPU9265_ADDRESS,datitos22, 1, false); //0x01); // Disable I2C master
        whoami = 0;     
        uint8_t datitos23[1] = {WHO_AM_I_MPU9265};
        whoami = i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos23, 1, false);  // Read WHO_AM_I register for MPU-9250
        printf("PinCFG: %x = 0x22 MasterDisable %x = 0x00 Interrupts %x = 0x01 Whoami %x = 0x71\n\r" , PinCFG, MasterDis, IntEna, whoami);
        if(PinCFG == 0x22 && MasterDis == 0x00 && IntEna == 0x01)
        {
          BypassTrue = 1;
        }
        sleep_ms(800); 
      }
  }
  else
  {
      printf("Could not connect to MPU9250: 0x%x\n\r" , whoami);
  }
  printf("ARRANCO!\n\r");
}

  


void getGres() 
{
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() 
{
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  i2c_read_blocking(i2c0, MPU9265_ADDRESS, rawData, 6, false);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  i2c_read_blocking(i2c0, MPU9265_ADDRESS, rawData, 6, false);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void initMPU9265()
{  
 // wake up device
  uint8_t datitos2[2] = {PWR_MGMT_1, 0x00};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos2, 2, false); // Clear sleep mode bit (6), enable all sensors 
  sleep_ms(100); // Wait for all registers to reset 

 // get stable time source
  uint8_t datitos3[2] = {PWR_MGMT_1, 0x01};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos3, 2, false);  // Auto select clock source to be PLL gyroscope reference if ready else
  sleep_ms(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum sleep_ms time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  uint8_t datitos4[4] = {CONFIG, 0x03, SMPLRT_DIV, 0x04};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos4, 4, false);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t datitos24[1] = {GYRO_CONFIG};
  uint8_t c = i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos24, 1, false);
//  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  uint8_t datitos5[6] = {GYRO_CONFIG, c & ~0x02, GYRO_CONFIG, c & ~0x18, GYRO_CONFIG,c | Gscale << 3 };
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos5, 6, false); // Clear Fchoice bits [1:0] 
 // Clear AFS bits [4:3]
 // Set full scale range for the gyro
 // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  
 // Set accelerometer full-scale range configuration
 uint8_t datitos25[1] = {ACCEL_CONFIG};
  c = i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos25, 1, false);
//  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  uint8_t datitos6[4] = {ACCEL_CONFIG, c & ~0x18, ACCEL_CONFIG, c | Ascale << 3};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos6, 4, false); // Clear AFS bits [4:3]
  // Set full scale range for the accelerometer 

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
 uint8_t datitos26[1] = {ACCEL_CONFIG2};
  c = i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos26, 1, false);
  uint8_t datitos7[4] = {ACCEL_CONFIG2, c & ~0x0F, ACCEL_CONFIG2, c | 0x03};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos7, 4, false); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
 // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
  uint8_t datitos8[4] = {INT_PIN_CFG, 0x22, INT_ENABLE, 0x01};
   i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos8, 4, false);    
  // Enable data ready (bit 0) interrupt
   sleep_ms(100);
}
// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9265(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
 // reset device
  uint8_t datitos9[2] = {PWR_MGMT_1, 0x80};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos9, 2, false); // Write a one to bit 7 reset bit; toggle reset device
  sleep_ms(100);
   
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
 // else use the internal oscillator, bits 2:0 = 001
  uint8_t datitos10[4] = {PWR_MGMT_1, 0x01, PWR_MGMT_2, 0x00};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos10, 4, false);  
  sleep_ms(200);                                    

// Configure device for bias calculation
  uint8_t datitos11[12] = {INT_ENABLE, 0x00, FIFO_EN, 0x00, PWR_MGMT_1, 0x00, I2C_MST_CTRL, 0x00, USER_CTRL, 0x00, USER_CTRL, 0x0C};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos11, 12, false);   // Disable all interrupts
    // Disable FIFO
   // Turn on internal clock source
   // Disable I2C master
    // Disable FIFO and I2C master modes
   // Reset FIFO and DMP
  sleep_ms(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  uint8_t datitos12[8] = {CONFIG, 0x01, SMPLRT_DIV, 0x00, GYRO_CONFIG, 0x00, ACCEL_CONFIG, 0x00};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos12, 8, false);      // Set low-pass filter to 188 Hz
  // Set sample rate to 1 kHz
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
 // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    uint8_t datitos13[4] = {USER_CTRL, 0x40, FIFO_EN, 0x78};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos13, 4, false);   // Enable FIFO  
       // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  sleep_ms(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  uint8_t datitos14[2] = {FIFO_EN, 0x00};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos14, 2, false);        // Disable gyro and accelerometer sensors for FIFO
   uint8_t datitos27[1] = {FIFO_COUNTH};
  i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos27, 2, false); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    uint8_t datitos28[1] = {FIFO_R_W};
    i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos28, 12, false); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
  uint8_t datitos[12] = {XG_OFFSET_H,data[0],XG_OFFSET_L,data[1],YG_OFFSET_H,data[2], YG_OFFSET_L,data[3],ZG_OFFSET_H,data[4],ZG_OFFSET_L,data[5]};
// Push gyro biases to hardware registers
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos, 12, false);
  
// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  uint8_t datitos29[1] = {XA_OFFSET_H};
  i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos29, 2, false); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  uint8_t datitos30[1] = {YA_OFFSET_H}; 
  i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos30, 2, false);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  uint8_t datitos31[1] = {ZA_OFFSET_H}; 
  i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos31, 2, false);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
  uint8_t datitos15[12] = {XA_OFFSET_H, data[0], XA_OFFSET_L, data[1], YA_OFFSET_H, data[2], YA_OFFSET_L, data[3], ZA_OFFSET_H, data[4], ZA_OFFSET_L, data[5]};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos15, 12, false);

// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
   

}

   
// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9265SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
   float factoryTrim[6];
   uint8_t FS = 0;
   uint8_t datitos16[10] = {SMPLRT_DIV, 0x00, CONFIG, 0x02, GYRO_CONFIG, 1<<FS, ACCEL_CONFIG2, 0x02, ACCEL_CONFIG, 1<<FS};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos16, 10, false);    // Set gyro sample rate to 1 kHz
    // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  // Set full scale range for the gyro to 250 dps
   // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
 // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) 
  {  // get average current values of gyro and acclerometer
  
    uint8_t datitos32[1] = {ACCEL_XOUT_H};
    i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos32, 6, false);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

    uint8_t datitos33[1] = {GYRO_XOUT_H};
    i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos33, 6, false);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) 
  {  // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }
  
  // Configure the accelerometer for self-test
  uint8_t datitos17[4] = {ACCEL_CONFIG, 0xE0, GYRO_CONFIG, 0xE0};
  i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos17, 4, false); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  sleep_ms(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) 
  {  // get average self-test values of gyro and acclerometer
    uint8_t datitos34[1] = {ACCEL_XOUT_H};  
    i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos34, 6, false);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    uint8_t datitos35[1] = {GYRO_XOUT_H};
    i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos35, 6, false);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) 
  {  // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }   
  
   // Configure the gyro and accelerometer for normal operation
   uint8_t datitos18[4] = {ACCEL_CONFIG, 0x00, GYRO_CONFIG, 0x00};
   i2c_write_blocking(i2c0, MPU9265_ADDRESS, datitos18, 4, false);    
   sleep_ms(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   uint8_t datitos36[1] = {SELF_TEST_X_ACCEL};
   i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos36 , 1 , false); // X-axis accel self-test results
   uint8_t datitos37[1] = {SELF_TEST_Y_ACCEL}; 
   i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos37, 1 , false); // Y-axis accel self-test results
   uint8_t datitos38[1] = {SELF_TEST_Z_ACCEL};
   selfTest[2] = i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos38, 1, false); // Z-axis accel self-test results
   uint8_t datitos39[1] = {SELF_TEST_X_GYRO};
   selfTest[3] = i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos39, 1, false);  // X-axis gyro self-test results
   uint8_t datitos40[1] = {SELF_TEST_Y_GYRO};
   selfTest[4] = i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos40, 1, false);  // Y-axis gyro self-test results
   uint8_t datitos41[1] = {SELF_TEST_Z_GYRO};
   selfTest[5] = i2c_read_blocking(i2c0, MPU9265_ADDRESS, datitos41, 1, false);  // Z-axis gyro self-test results

   // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
   // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
   // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) 
   {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
   }
   
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float * quaternionBuffer)
{
	float q1 = quaternionBuffer[0], q2 = quaternionBuffer[1], q3 = quaternionBuffer[2], q4 = quaternionBuffer[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	quaternionBuffer[0] = q1 * norm;
	quaternionBuffer[1] = q2 * norm;
	quaternionBuffer[2] = q3 * norm;
	quaternionBuffer[3] = q4 * norm;

}





