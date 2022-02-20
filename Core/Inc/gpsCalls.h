/*
 * gpsCalls.h
 *
 *  Created on: Feb 18, 2022
 *      Author: adrian
 */

#ifndef INC_GPSCALLS_H_
#define INC_GPSCALLS_H_

SFE_UBLOX_GPS myGPS;
byte i2cDataXX[] = { 0x00, 0x00 };
int i = 0;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

uint16_t Hour = 0;
uint16_t Minute = 0;
uint16_t Second = 0;


//void setupGPS();
//void loopGPS();

void setupGPS() {

	HAL_I2C_Master_Transmit( &hi2c1, ( 0x15 << 1 ), i2cDataXX, 1, 10 );
  while (myGPS.begin(0x42) == false) //Connect to the Ublox module using Wire port
  {
    HAL_Delay(1);
//	APP_LOG(TS_ON, VLEVEL_M, "Enters SETUP GPS in function \r\n");
		//HAL_I2C_Master_Transmit( &hi2c2, ( 0x16 << 1 ), i2cDataXX, 1, 10 );
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

}

void loopGPS() {
  bool timeValid = false, dateValid = false;
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if ( HAL_GetTick() - lastTime > 1000 ) {
    lastTime = HAL_GetTick(); //Update the timer
    long latitude = myGPS.getLatitude();
	long longitude = myGPS.getLongitude();
    long altitude = myGPS.getAltitude();
    long accuracy = myGPS.getPositionAccuracy();

		//printf( "latitude: %ld longitude: %ld \n", latitude, longitude );

    	APP_LOG(TS_ON, VLEVEL_M, "Lati: %d, Long: %d \r\n", latitude, longitude);

    byte SIV = myGPS.getSIV();

    uint16_t Year = myGPS.getYear();
    uint16_t Month = myGPS.getMonth();
    uint16_t Day = myGPS.getDay();
    Hour = myGPS.getHour();
    Minute = myGPS.getMinute();
    Second = myGPS.getSecond();

//		printf( "Hour: %2d Minute: %2d Second: %2d\n", Hour, Minute, Second );

//  timeValid = myGPS.getTimeValid();
//  dateValid = myGPS.getDateValid();
  }
}


#endif /* INC_GPSCALLS_H_ */
