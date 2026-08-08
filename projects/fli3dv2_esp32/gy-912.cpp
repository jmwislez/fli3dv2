// Support for GY-912 providing ICM20948 9-axis motion sensor and BMP388 pressure sensor

#include <Wire.h>
#include <ICM20948_WE.h>
#include <BMP388_DEV.h>
#include "fli3dv2.h"

#define ICM20948_ADDR 0x69
#define BMP388_ADDR 0x77

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
BMP388_DEV myBMP(I2C_SDA_PIN, I2C_SCL_PIN);

bool setup_icm20948() {
    Wire.begin();

    myBMP.begin();                                 // Default initialisation, place the BMP388 into SLEEP_MODE 
    myBMP.setTimeStandby(TIME_STANDBY_1280MS);     // Set the standby time to 1.2 seconds
    myBMP.startNormalConversion();  

    if(!myIMU.init()){
        sprintf(buffer, "Motion unit ICM20948 does not respond on I2C address 0x%02X", ICM20948_ADDR);
        publish_event(STS_THIS, SS_THIS, EVENT_ERROR, buffer);
        return false;
    }
    else {
        /*  This is a method to calibrate. You have to determine the minimum and maximum 
        *  raw acceleration values of the axes determined in the range +/- 2 g. 
        *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
        *  The parameters are floats. 
        *  The calibration changes the slope / ratio of raw acceleration vs g. The zero point is 
        *  set as (min + max)/2.
        */
        //myIMU.setAccOffsets(-16330.0, 16450.0, -16600.0, 16180.0, -16520.0, 16690.0);

        /* The starting point, if you position the ICM20948 flat, is not necessarily 0g/0g/1g for x/y/z. 
        * The autoOffset function measures offset. It assumes your ICM20948 is positioned flat with its 
        * x,y-plane. The more you deviate from this, the less accurate will be your results.
        * It overwrites the zero points of setAccOffsets, but keeps the correction of the slope.
        * The function also measures the offset of the gyroscope data. The gyroscope offset does not   
        * depend on the positioning.
        * This function needs to be called after setAccOffsets but before other settings since it will 
        * overwrite settings!
        * You can query the offsets with the functions:
        * xyzFloat getAccOffsets() and xyzFloat getGyrOffsets()
        * You can apply the offsets using:
        * setAccOffsets(xyzFloat yourOffsets) and setGyrOffsets(xyzFloat yourOffsets)
        */
        //  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
        //  delay(1000);
        //  myIMU.autoOffsets();
        //  Serial.println("Done!"); 

        /*  The gyroscope data is not zero, even if you don't move the ICM20948. 
        *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
        *  using the +/- 250 degrees/s range. 
        *  Use either autoOffset or setGyrOffsets, not both.
        */
        //myIMU.setGyrOffsets(-115.0, 130.0, 105.0);

        /*  ICM20948_ACC_RANGE_2G      2 g   (default)
        *  ICM20948_ACC_RANGE_4G      4 g
        *  ICM20948_ACC_RANGE_8G      8 g   
        *  ICM20948_ACC_RANGE_16G    16 g
        */
        myIMU.setAccRange(ICM20948_ACC_RANGE_2G);

        /*  Choose a level for the Digital Low Pass Filter or switch it off.  
        *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
        *  
        *  IMPORTANT: This needs to be ICM20948_DLPF_7 if DLPF is used in cycle mode!
        *  
        *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
        *    0              246.0               1125/(1+ASRD) 
        *    1              246.0               1125/(1+ASRD)
        *    2              111.4               1125/(1+ASRD)
        *    3               50.4               1125/(1+ASRD)
        *    4               23.9               1125/(1+ASRD)
        *    5               11.5               1125/(1+ASRD)
        *    6                5.7               1125/(1+ASRD) 
        *    7              473.0               1125/(1+ASRD)
        *    OFF           1209.0               4500
        *    
        *    ASRD = Accelerometer Sample Rate Divider (0...4095)
        *    You achieve lowest noise using level 6  
        */
        myIMU.setAccDLPF(ICM20948_DLPF_6);    

        /*  Acceleration sample rate divider divides the output rate of the accelerometer.
        *  Sample rate = Basic sample rate / (1 + divider) 
        *  It can only be applied if the corresponding DLPF is not off!
        *  Divider is a number 0...4095 (different range compared to gyroscope)
        *  If sample rates are set for the accelerometer and the gyroscope, the gyroscope
        *  sample rate has priority.
        */
        //myIMU.setAccSampleRateDivider(10);

        /*  ICM20948_GYRO_RANGE_250       250 degrees per second (default)
        *  ICM20948_GYRO_RANGE_500       500 degrees per second
        *  ICM20948_GYRO_RANGE_1000     1000 degrees per second
        *  ICM20948_GYRO_RANGE_2000     2000 degrees per second
        */
        //myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);

        /*  Choose a level for the Digital Low Pass Filter or switch it off. 
        *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
        *  
        *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
        *    0              196.6               1125/(1+GSRD) 
        *    1              151.8               1125/(1+GSRD)
        *    2              119.5               1125/(1+GSRD)
        *    3               51.2               1125/(1+GSRD)
        *    4               23.9               1125/(1+GSRD)
        *    5               11.6               1125/(1+GSRD)
        *    6                5.7               1125/(1+GSRD) 
        *    7              361.4               1125/(1+GSRD)
        *    OFF          12106.0               9000
        *    
        *    GSRD = Gyroscope Sample Rate Divider (0...255)
        *    You achieve lowest noise using level 6  
        */
        myIMU.setGyrDLPF(ICM20948_DLPF_6);  

        /*  Gyroscope sample rate divider divides the output rate of the gyroscope.
        *  Sample rate = Basic sample rate / (1 + divider) 
        *  It can only be applied if the corresponding DLPF is not OFF!
        *  Divider is a number 0...255
        *  If sample rates are set for the accelerometer and the gyroscope, the gyroscope
        *  sample rate has priority.
        */
        //myIMU.setGyrSampleRateDivider(10);

        /*  Choose a level for the Digital Low Pass Filter. 
        *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
        *  
        *  DLPF          Bandwidth [Hz]      Output Rate [Hz]
        *    0             7932.0                    9
        *    1              217.9                 1125
        *    2              123.5                 1125
        *    3               65.9                 1125
        *    4               34.1                 1125
        *    5               17.3                 1125
        *    6                8.8                 1125
        *    7             7932.0                    9
        *                 
        *    
        *    GSRD = Gyroscope Sample Rate Divider (0...255)
        *    You achieve lowest noise using level 6  
        */
        myIMU.setTempDLPF(ICM20948_DLPF_6);
        sprintf(buffer, "Motion unit ICM20948 initialized on I2C address 0x%02X", ICM20948_ADDR);
        publish_event(STS_THIS, SS_THIS, EVENT_INIT, buffer);
    }

    if(!myIMU.initMagnetometer()){
        sprintf(buffer, "Magnetometer ICM20948 does not respond on I2C address 0x%02X", ICM20948_ADDR);
        publish_event(STS_THIS, SS_THIS, EVENT_WARNING, buffer);
    }
    else {
        /* You can set the following modes for the magnetometer:
        * AK09916_PWR_DOWN          Power down to save energy
        * AK09916_TRIGGER_MODE      Measurements on request, a measurement is triggered by 
        *                           calling setMagOpMode(AK09916_TRIGGER_MODE)
        * AK09916_CONT_MODE_10HZ    Continuous measurements, 10 Hz rate
        * AK09916_CONT_MODE_20HZ    Continuous measurements, 20 Hz rate
        * AK09916_CONT_MODE_50HZ    Continuous measurements, 50 Hz rate
        * AK09916_CONT_MODE_100HZ   Continuous measurements, 100 Hz rate (default)
        */
        myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);
        // delay(50); // add a delay of 1000/magRate to avoid first mag value being zero 
        sprintf(buffer, "Magnetometer ICM20948 initialized on I2C address 0x%02X", ICM20948_ADDR);
        publish_event(STS_THIS, SS_THIS, EVENT_INIT, buffer);
    }
    //tm_motion.accel_range = myIMU.getAccRange();
    //tm_motion.gyro_range = myIMU.getGyrRange();
    return true;
}

bool setup_bmp388() {
    Wire.begin();
    if(!myBMP.begin()){
        sprintf(buffer, "Pressure unit BMP388 does not respond on I2C address 0x%02X", BMP388_ADDR);
        publish_event(STS_THIS, SS_THIS, EVENT_ERROR, buffer);
        return false;
    }
    else {
        myBMP.setTimeStandby(TIME_STANDBY_80MS);     // Set the standby time to 1.2 seconds
        myBMP.startNormalConversion();  
        tm_esp32.pressure_enabled = true;
        sprintf(buffer, "Pressure unit BMP388 initialized on I2C address 0x%02X", BMP388_ADDR);
        publish_event(STS_THIS, SS_THIS, EVENT_INIT, buffer);
    }
    return true;
}

bool acquire_IMU() {
    xyzFloat gValue;
    xyzFloat gyr;
    xyzFloat magValue;

    myIMU.readSensor();
    myIMU.getGValues(&gValue);
    myIMU.getGyrValues(&gyr);
    myIMU.getMagValues(&magValue);
    float temp = myIMU.getTemperature();
    Serial.printf("ICM20948: Accel: %.2f, %.2f, %.2f g; Gyro: %.2f, %.2f, %.2f deg/s; Mag: %.2f, %.2f, %.2f uT; Temp: %.2f C\n", gValue.x, gValue.y, gValue.z, gyr.x, gyr.y, gyr.z, magValue.x, magValue.y, magValue.z, temp);
    tm_motion.accel_x = int16_t(gValue.x * 100); // convert to cm/s2
    tm_motion.accel_y = int16_t(gValue.y * 100);
    tm_motion.accel_z = int16_t(gValue.z * 100);
    tm_motion.gyro_x = int16_t(gyr.x * 100); // convert to cdeg/s    
    tm_motion.gyro_y = int16_t(gyr.y * 100);
    tm_motion.gyro_z = int16_t(gyr.z * 100);
    tm_motion.magn_x = int16_t(magValue.x);  
    tm_motion.magn_y = int16_t(magValue.y);
    tm_motion.magn_z = int16_t(magValue.z);

    tm_motion.tilt = int16_t(myIMU.getPitch()*100); // convert to cdeg
    tm_motion.g = myIMU.getResultantG(&gValue);
    //tm_motion.a = myIMU.getA();
    //tm_motion.rpm = myIMU.getRPM();
    return true;
}

bool acquire_BMP() {
    float pressure, temperature, height;
    myBMP.getMeasurements(temperature, pressure, height);
    Serial.printf("BMP388: Pressure: %.2f Pa, Temperature: %.2f C, Height: %.2f m\n", pressure, temperature, height);
    tm_pressure.temperature = int16_t(temperature * 100); // convert to cdegC
    tm_pressure.pressure = uint32_t(pressure * 100); // convert to Pa
    tm_pressure.height = int16_t(height * 100); // convert to cm
    return true;
}