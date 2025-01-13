/*********************************************************************************************/
void calculateSun(time_t utcTime)  // Find the last and next sun set and rise times.
/*********************************************************************************************/
{
  //  Serial.print("Local time "); Serial.print(ctime(&utcTime));
  //  Serial.println("\r\n--->calcSun info");
  //  Serial.print("Local time "); Serial.print(ctime(&utcTime));

  theSun.calculate(latitude, longitude, utcTime);
  sunVisible = theSun.isVisible; sunHasRise = theSun.hasRise; sunHasSet  = theSun.hasSet;
  // Where the sun will rise/set in degrees from North.
  sunRiseAz  = theSun.riseAz; sunSetAz   = theSun.setAz;

  sunQueryTime = theSun.queryTime; sunRiseTime  = theSun.riseTime;
  sunSetTime   = theSun.setTime;
  //  Serial.printf("sunQueryTime %lu, iOffset %i\r\n", sunQueryTime, iOffset);
  //  Serial.printf("Nearest Sun Rise: %.24s, Azimuth %.2f\n",
  //                ctime(&sunRiseTime), theSun.riseAz);


  //  Serial.printf("Nearest Sun Set: %.24s, Azimuth %.2f\n",
  //                ctime(&sunSetTime), sunSetAz);
  //  if (theSun.isVisible && theMoon.isVisible)  // If sun and moon are overhead, may be trouble.
  //    Serial.println("\nThe moon is in the sky but may be washed out by the sun.\r\n");

  //  tmlocalTime = localtime(&sunQueryTime);
  //  strftime (work_char, sizeof(work_char), "(strftime) Sun query Time %R", tmlocalTime);
  //  Serial.println(work_char);

  tmlocalTime = localtime(&sunRiseTime);
  strftime (work_char, sizeof(work_char), "%R", tmlocalTime);
  //  Serial.print("(strftime) Sun Rise  Time "); Serial.println(work_char);
  lv_label_set_text(ui_SunRiseData, work_char);

  tmlocalTime = localtime(&sunSetTime);
  strftime (work_char, sizeof(work_char), "%R", tmlocalTime);
  //  Serial.print("(strftime) Sun Set   Time "); Serial.println(work_char);
  lv_label_set_text(ui_SunSetData, work_char);

}
