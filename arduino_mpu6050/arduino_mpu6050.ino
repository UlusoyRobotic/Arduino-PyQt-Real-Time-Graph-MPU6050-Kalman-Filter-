// Calibration for MadgwickAHRS & MahonyAHRS
#include<Wire.h> 
#include <Kalman.h>

Kalman kalmanX;

double kalAngleX;

unsigned long previousMillis = 0;                 //Önceki millis değeri için değişken
const long interval = 200;                         //Ölçümler arasındaki süre (ms)
float elapsedTime=0.2; // time/1000

const int MPU_ADDR=0x68;  // I2C address of the MPU-6050 

long AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; 
float T_AcX,T_AcY,T_AcZ,T_Tmp,T_GyX,T_GyY,T_GyZ; 
long Cal_AcX, Cal_AcY, Cal_AcZ, Cal_Tmp, Cal_GyX, Cal_GyY, Cal_GyZ; 

float acc_ang_x;  //X ekseni ile yapılan açı için değişken
float acc_ang_y;  //Y ekseni ile yapılan açı için değişken
float acc_ang_z;  //Z ekseni ile yapılan açı için değişken
float gyro_ang_x;  //X ekseni ile yapılan açı için değişken
float gyro_ang_y;  //Y ekseni ile yapılan açı için değişken
float gyro_ang_z;  //Z ekseni ile yapılan açı için değişken

float roll, pitch, yaw;

// kalman variables
//float varVolt = 1.12184278324081E-05;  // variance determined using excel and reading samples of raw sensor data
float varVolt = 0.00962361;  //  datasheet variance determined using excel and reading samples of raw sensor data
float varProcess = 1e-8;
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

void MPU6050_calibrating();

void setup(){ 
  Wire.begin(); 
  
  init_MPU6050(); 
   
  Serial.begin(115200); 
  pinMode(13, OUTPUT);

  //MPU6050_calibrating();

   //reading data from sensor
   Wire.beginTransmission(MPU_ADDR); 
   Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H) 
   Wire.endTransmission(false); 
   Wire.requestFrom(MPU_ADDR,14,true);  // request a total of 14 registers 
   AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)      
   AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L) 
   AcZ=(Wire.read()<<8|Wire.read()) ;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) 
   Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) 
   GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L) 
   GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L) 
   GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

   T_AcX = AcX /16384.0;
   T_AcY = AcY/16384.0;
   T_AcZ = AcZ/16384.0;
   T_GyX = GyX/131.0;
   T_GyY = GyY/131.0;
   T_GyZ = GyZ/131.0;
        
   acc_ang_x=(atan(T_AcY / sqrt(pow(T_AcX, 2) + pow(T_AcZ, 2))) * 180 / PI) - 0.58;
   acc_ang_y= (atan(-1 * T_AcX / sqrt(pow(T_AcY, 2) + pow(T_AcZ, 2))) * 180 / PI) - 1.58;
   acc_ang_z= (atan( T_AcZ / sqrt(pow(T_AcX, 2) + pow(T_AcZ, 2))) * 180 / PI);
   
   kalmanX.setAngle(acc_ang_x);
} 

void loop(){ 

   unsigned long currentMillis = millis(); 
   if (currentMillis - previousMillis >= interval) {                           //İnterval süresi geçtiğinde içeri girilir
    previousMillis = currentMillis;
    
      //reading data from sensor
      Wire.beginTransmission(MPU_ADDR); 
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H) 
      Wire.endTransmission(false); 
      Wire.requestFrom(MPU_ADDR,14,true);  // request a total of 14 registers 
      AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)      
      AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L) 
      AcZ=(Wire.read()<<8|Wire.read()) ;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) 
      Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) 
      GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L) 
      GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L) 
      GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

      //Calibration 
       /* AcX = AcX-Cal_AcX; 
        AcY = AcY-Cal_AcY;
        AcZ = AcZ-Cal_AcZ;
        GyX = GyX-Cal_GyX;
        GyY = GyY-Cal_GyY;
        GyZ = GyZ-Cal_GyZ;*/

        T_AcX = AcX /16384.0;
        T_AcY = AcY/16384.0;
        T_AcZ = AcZ/16384.0;
        T_GyX = GyX/131.0;
        T_GyY = GyY/131.0;
        T_GyZ = GyZ/131.0;
         
       // ang_x = atan(T_AcY/(sqrt(pow(T_AcX,2)+pow(T_AcZ,2)))) * 57296 / 1000; //Euler Açı formülüne göre açı hesabı. (X-Ekseni)
        //ang_y = atan(T_AcX/(sqrt(pow(T_AcY,2)+pow(T_AcZ,2)))) * 57296 / 1000; //Euler Açı formülüne göre açı hesabı. (Y-Ekseni)

        //accelorometer datas aprovval
        
        acc_ang_x=(atan(T_AcY / sqrt(pow(T_AcX, 2) + pow(T_AcZ, 2))) * 180 / PI) - 0.58;
        acc_ang_y= (atan(-1 * T_AcX / sqrt(pow(T_AcY, 2) + pow(T_AcZ, 2))) * 180 / PI) - 1.58;
        acc_ang_z= (atan( T_AcZ / sqrt(pow(T_AcX, 2) + pow(T_AcZ, 2))) * 180 / PI);

        
         T_GyX = T_GyX + 0.56; // GyroErrorX ~(-0.56)
         T_GyY = T_GyY - 2; // GyroErrorY ~(2)
         T_GyZ = T_GyZ + 0.79; // GyroErrorZ ~ (-0.8)
        //Gyro daas

        /*gyro_ang_x = gyro_ang_x + T_GyX * elapsedTime; // deg/s * s = deg
        gyro_ang_y = gyro_ang_y + T_GyY * elapsedTime;
        yaw =  yaw + T_GyZ * elapsedTime;*/

        gyro_ang_x =  T_GyX * elapsedTime; // deg/s * s = deg
        gyro_ang_y = T_GyY * elapsedTime;
        yaw =   T_GyZ * elapsedTime;

        roll = 0.96 * (gyro_ang_x+roll) + 0.04 * acc_ang_x;
        pitch = 0.96 *( gyro_ang_y+pitch) + 0.04 * acc_ang_y;

        //Basic Filter
        
        //Kalman filter manuel
        Pc = P + varProcess;
        G = Pc/(Pc + varVolt);    // kalman gain
        P = (1-G)*Pc;
        Xp = Xe;
        Zp = Xp;
        Xe = G*(acc_ang_x-Zp)+Xp;   // the kalman estimate of the sensor voltage

        //Kalman filter by library
        kalAngleX = kalmanX.getAngle(acc_ang_x,T_AcX , 0.2);

        
     // Print Data Angle
     //Serial.println("Angle of Accelorrometer");
      Serial.print(","); 
      Serial.print(acc_ang_x);
      Serial.print(","); 
      Serial.print(Xe);
      Serial.print(",");
      Serial.print(kalAngleX);
      Serial.println(",");
      
      /*Serial.print("AcX = "); Serial.print(AcX); 
      Serial.print(" | AcY = "); Serial.print(AcY); 
      Serial.print(" | AcZ = "); Serial.print(T_AcZ); 
      //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet 
      Serial.print(" | GyX = "); Serial.print(T_GyX); 
      Serial.print(" | GyY = "); Serial.print(T_GyY); 
      Serial.print(" | GyZ = "); Serial.println(T_GyZ);*/

      //Serial.print("Gyro Ang X= "); Serial.print(gyro_ang_x);
      //Serial.print("Gyro Ang Y = "); Serial.println(gyro_ang_y);

      /*Serial.print(" Yaw : "); Serial.print(yaw); 
      Serial.print(" Pitch : "); Serial.print(pitch); 
      Serial.print(" Roll : "); Serial.println(roll);*/

   }

} 

void init_MPU6050(){ 
  //MPU6050 Initializing & Reset 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x6B);  // PWR_MGMT_1 register 
  Wire.write(0);     // set to zero (wakes up the MPU-6050) 
  Wire.endTransmission(true); 

  //MPU6050 Clock Type 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x6B);  // PWR_MGMT_1 register 
  Wire.write(0x03);     // Selection Clock 'PLL with Z axis gyroscope reference' 
  Wire.endTransmission(true); 

  //MPU6050 Gyroscope Configuration Setting 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x1B);  // Gyroscope Configuration register 
  //Wire.write(0x00);     // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec] 
  //Wire.write(0x08);     // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec] 
  //Wire.write(0x10);     // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec] 
  Wire.write(0x18);     // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec] 
  Wire.endTransmission(true); 

  //MPU6050 Accelerometer Configuration Setting 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x1C);  // Accelerometer Configuration register 
  Wire.write(0x00);     // AFS_SEL=0, Full Scale Range = +/- 2 [g] 
  //Wire.write(0x08);     // AFS_SEL=1, Full Scale Range = +/- 4 [g] 
  //Wire.write(0x10);     // AFS_SEL=2, Full Scale Range = +/- 8 [g] 
  //Wire.write(0x18);     // AFS_SEL=3, Full Scale Range = +/- 10 [g] 
  Wire.endTransmission(true); 

  //MPU6050 DLPF(Digital Low Pass Filter) 
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x1A);  // DLPF_CFG register 
  Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz  
  //Wire.write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz  
  //Wire.write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz  
  //Wire.write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz  
  //Wire.write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz  
  //Wire.write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz  
  //Wire.write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz  
  Wire.endTransmission(true); 
}
void MPU6050_calibrating()
{
  for(int i = 0 ; i < 1000 ; i++) { 

    if(i % 200 == 0) 
    {
    Serial.println("Calculating ....."); 
    digitalWrite(13, !digitalRead(13));
    }
     
    Wire.beginTransmission(MPU_ADDR); 
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H) 
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU_ADDR,14,true);  // request a total of 14 registers 
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)      
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L) 
    AcZ=(Wire.read()<<8|Wire.read()) - 4096;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) 
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) 
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L) 
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L) 
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) 
    delay(5); 

    // Sum data 
    Cal_AcX += AcX; 
    Cal_AcY += AcY; 
    Cal_AcZ += AcZ; 
    Cal_GyX += GyX; 
    Cal_GyY += GyY; 
    Cal_GyZ += GyZ; 

  } 

  // Average Data 
  Cal_AcX /= 1000; 
  Cal_AcY /= 1000; 
  Cal_AcZ /= 1000; 
  Cal_GyX /= 1000; 
  Cal_GyY /= 1000; 
  Cal_GyZ /= 1000; 

  // Print Data 
  Serial.println("End of Calculation"); 
  Serial.print("AcX = "); Serial.print(Cal_AcX); 
  Serial.print(" | AcY = "); Serial.print(Cal_AcY); 
  Serial.print(" | AcZ = "); Serial.print(Cal_AcZ); 
  //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet 
  Serial.print(" | GyX = "); Serial.print(Cal_GyX); 
  Serial.print(" | GyY = "); Serial.print(Cal_GyY); 
  Serial.print(" | GyZ = "); Serial.println(Cal_GyZ); 
}
