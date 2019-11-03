/* GYRO PINOUT
 * VCC    |   5v
 * GND    |   GND
 * SDA    |   A4
 * SCL    |   A5
 */

/* ULTRASONIC PINOUT
 * VCC    |   5v
 * GND    |   GND
 * TRIG   |   D11
 * ECHO   |   D12
 */

#include <NewPing.h>
#include <Wire.h>


//Gyro Variables
float elapsedTime250ms, elapsedTime5min, time;        //Variables for time control
float timePrev250ms = 0;
float timePrev5min = 0;
int 5minuteTimeFrame = 0;
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;     //Here we store the raw data read 
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data

//Acc Variables
float Acc_rawX, Acc_rawY, Acc_rawZ;    //Here we store the raw data read 
float Acc_angle_x, Acc_angle_y;          //Here we store the angle value obtained with Acc data

float Total_angle_x, Total_angle_y;

float Total_height_to_average_250ms;
float Total_height_to_average_5min;
int Number_of_readings_250ms;
int Number_of_readings_5min;

float Average_height;
float Highest_reading = 0;
float Lowest_reading = 0;
float Data_storage[][];
float Data_to_send[][];



void setup() {
  Wire.begin();                           //begin the wire comunication
  
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true); 

  Serial.begin(9600);                     //Remember to set this same baud rate to the serial monitor  
  time = millis();                        //Start counting time in milliseconds
}

void loop() {                        // the previous time is stored before the actual time read
  time = millis();                        // actual time read
  elapsedTime250ms = (time - timePrev250ms);        // elapsed time in ms
  elapsedTime5min = (time - timePrev5min);

  //////////////////////////////////////Gyro read/////////////////////////////////////

    Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
    Wire.write(0x43);                        //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true);           //We ask for just 4 registers
        
    Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum
    Gyr_rawY=Wire.read()<<8|Wire.read();
    /*Now in order to obtain the gyro data in degrees/seconds we have to divide first
    the raw value by 32.8 because that's the value that the datasheet gives us for a 1000dps range*/
    /*---X---*/
    Gyr_rawX = (Gyr_rawX/32.8); 
    /*---Y---*/
    Gyr_rawY = (Gyr_rawY/32.8);
    
    /*Now we integrate the raw value in degrees per seconds in order to obtain the angle
    * If you multiply degrees/seconds by seconds you obtain degrees */
    /*---X---*/
    Gyro_angle_x = Gyr_rawX*elapsedTime;
    /*---X---*/
    Gyro_angle_y = Gyr_rawY*elapsedTime;


    
  
  //////////////////////////////////////Acc read/////////////////////////////////////

  Wire.beginTransmission(0x68);     //begin, Send the slave adress (in this case 68) 
  Wire.write(0x3B);                 //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);      //keep the transmission and next
  Wire.requestFrom(0x68,6,true);    //We ask for next 6 registers starting withj the 3B  
  /*We have asked for the 0x3B register. The IMU will send a brust of register.
  * The amount of register to read is specify in the requestFrom function.
  * In this case we request 6 registers. Each value of acceleration is made out of
  * two 8bits registers, low values and high values. For that we request the 6 of them  
  * and just make then sum of each pair. For that we shift to the left the high values 
  * register (<<) and make an or (|) operation to add the low values.
  If we read the datasheet, for a range of+-8g, we have to divide the raw values by 4096*/    
  Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
  Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
  Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ; 
 /*Now in order to obtain the Acc angles we use euler formula with acceleration values
 after that we substract the error value found before*/  
 /*---X---*/
 Acc_angle_x = radToDeg(atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2))));
 /*---Y---*/
 Acc_angle_y = radToDeg(atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2))));


 //////////////////////////////////////Total angle and filter/////////////////////////////////////
 /*---X axis angle---*/
 Total_angle_x = 0.98 *(Total_angle_x + Gyro_angle_x) + 0.02*Acc_angle_x;
 /*---Y axis angle---*/
 Total_angle_y = 0.98 *(Total_angle_y + Gyro_angle_y) + 0.02*Acc_angle_y;
   

 //float set_height = 1000.00;
 Serial.print("Yº: ");
 Serial.println(Total_angle_y);
 //float delta_height = calculateDeltaHeight(Total_angle_y,300);
 //Serial.print("Height difference: ");
 //Serial.println(delta_height);
 //Serial.print("Waterheight: ");
 //Serial.println(set_height+delta_height);
 Serial.println(" ");
 //delay(250);
if(elapsedTime250ms >= 250){
  Total_height_to_average_250ms = Total_height_to_average_250ms + Total_angle_y;
  Number_of_readings_250ms++;
  Data_storage[5minTimeFrame][Data_storage_counter] = Total_height_to_average_250_ms/Number_of_readings;

  if (Lowest_reading == 0){
    Lowest_reading = Data_storage[5minTimeFrame][Data_storage_counter];
  }
  else if (Data_storage[5minTimeFrame][Data_storage_counter] > Highest_reading){
    Highest_reading = Data_storage[5minTimeFrame][Data_storage_counter];
  }
  else if(Data_storage[5minTimeFrame][Data_storage_counter] < Lowest_reading){
    Lowest_reading = Data_storage[5minTimeFrame][Data_storage_counter];
  }
  
  
  Total_height_to_average_5min = Total_height_to_average_5min + Total_angle_y;
  Number_of_readings_5min++;
  
  Data_storage_counter++;
  
  Total_height_to_average_250ms = 0;
  Number_of_readings_250ms = 0;
  timePrev250ms = time;
}
else {
  Total_height_to_average_250ms = Total_height_to_average_250ms + Total_angle_y;
  Number_of_readings_250ms++; 

  Total_height_to_average_5min = Total_height_to_average5min + Total_angle_y;
  Number_of_readings_5min++; 
}
if elapsedTime5min >= 300000){
  Data_to_send[5minTimeFrame][0] = {Total_height_to_average5min/Number_of_readings_5min};
  
  
  5minTimeFrame++;
  timePrev5min = time;
  Lowest_reading = 0;
  Highest_reading = 0;
  Number_of_readings_5min = 0;
  
} 
}
  
float hoogste20(float high, float low){ 
  return (high - low)*0,8+low;
}
float laagste20(float high, float low){ 
  return (high - low)*0,2+low;
}

float calculateDeltaHeight(float angle, int armLength){
  return armLength*sin(degToRad(angle));
}

float radToDeg(float input){
  return input * (180/PI);
}

float degToRad(float input){
  return input * (PI/180);
}
