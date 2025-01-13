/*********************************************************************************************/
void updateMainScreen()
//Display new GPS info
/*********************************************************************************************/
{
  //available() returns the number of new bytes available from the GPS module
  while (myI2CGPS.available()) gps.encode(myI2CGPS.read()); //Feed the GPS parser
  if (prevSec == gps.time.second()) return;
  prevSec = gps.time.second();
  //  Serial.printf("prev %02i, now %02i\r\n", prevSec, gps.time.second());
  if (selectedTZ == tzPH)
    convertGPSTimeToLocalTime(PHtime_zone, false);  // Philippines
  else if (selectedTZ == tzCA)
    convertGPSTimeToLocalTime(CAtime_zone, false); // California

  /**********************/
  /* Latitude           */
  /**********************/
  showLatOnScreen();

  /**********************/
  /* Longitude           */
  /**********************/
  showLngOnScreen();

  /**********************/
  /* Distance           */
  /**********************/
  // GPS_Movement will be kept in kilometers and converted to feet/miles when needed.
  // Meters moved this second.
  //  Serial.printf("1 Distance input Lat %.1f, Lng %.1f, pLat %.1f, pLng %.1f, Dist %.1f M, "
  //                "Total %.1f M\r\n",
  //                latitude, longitude, prevLat, prevLng, GPS_Movement, totMovement);
  GPS_Movement = gps.distanceBetween(latitude, longitude, prevLat, prevLng);
  //  prevLat = latitude; prevLng = longitude;
  if (GPS_Movement < 1.) GPS_Movement = 0;  // Ignore movement less than .5 meters/second
  if (prevLat > 500 || prevLng > 500) GPS_Movement = 0;
  prevLat = latitude;  prevLng = longitude;
  totMovement += GPS_Movement;  // Kept in total in meters moved.
  //  Serial.printf("2 Distance input Lat %.1f, Lng %.1f, pLat %.1f, pLng %.1f, Dist %.1f M, "
  //                "Total %.1f M\r\n",
  //                latitude, longitude, prevLat, prevLng, GPS_Movement, totMovement);
  if (distUnit == IMPERIAL) {
    //Convert from meters to miles
    snprintf(dist_char, sizeof(dist_char), "%0.1f Mi.", totMovement * 0.00062137112);
  } else {
    // Leave as meters.
    snprintf(dist_char, sizeof(dist_char), "%0.1f Mt.", totMovement);
  }
  lv_label_set_text(ui_DistData, dist_char);

  /**********************/
  /* Course             */
  /**********************/
  if (gps.course.isUpdated()) {
    snprintf(heading_char, sizeof(heading_char), "%.0f° %s",
             gps.course.deg(), gps.cardinal(gps.course.deg()));
    lv_label_set_text(ui_HeadingData, heading_char);
  }

  /**********************/
  /* Time & Date        */
  /**********************/
  showTimeAndDate();

  /**********************/
  /* HDOP               */
  /**********************/
  hdop = gps.hdop.hdop();
  // Convert the number of HDOP to a string
  snprintf(hdop_char, sizeof(hdop_char), "%.1f", hdop);
  lv_label_set_text(ui_HDOPData, hdop_char);
  //lv_label_set_text(ui_HeadingData, "ASDF");  // Example.  Fails there but works here. ???
  //lv_label_set_text(ui_HDOPData, heading_char);

  /**********************/
  /* Heading            */
  /**********************/
  heading = gps.course.deg();  // Note that this is a double
  // Buffer to hold the converted heading value
  //  char headingStr[10];
  //  dtostrf(heading, 6, 0, headingStr);  // Convert double to string
  //  Serial.println(headingStr); Serial.println();
  //  lv_label_set_text(ui_Direction, headingStr);

  /**********************/
  /* Sats in Use/View   */
  /**********************/
  showSatsInUseView();

  /**********************/
  /* Speed              */
  /**********************/
  if (spdUnit == IMPERIAL) {
    work_int = int(gps.speed.mph());
    snprintf(speed_char, sizeof(speed_char),  "%i mph", work_int);
  } else {
    work_int = int(gps.speed.kmph());
    snprintf(speed_char, sizeof(speed_char),  "%i kmph", work_int);
  }
  lv_label_set_text(ui_SpeedData, speed_char);

  /**********************/
  /* Altitude           */
  /**********************/

  if (altUnit == IMPERIAL) {
    int dAltitude = int(gps.altitude.isValid() ? gps.altitude.feet() : 0);
    snprintf(altitude_char, sizeof(altitude_char), "%i", dAltitude);
    strcat(altitude_char, " ft");
    lv_label_set_text(ui_AltData, altitude_char);
  }  else {
    int dAltitude = int(gps.altitude.isValid() ? gps.altitude.meters() : 0);
    snprintf(altitude_char, sizeof(altitude_char), "%i", dAltitude);
    strcat(altitude_char, " Mt");
    lv_label_set_text(ui_AltData, altitude_char);
  }
  // Convert heading to an integer if necessary
  angle = int(gps.course.deg());
  // Rotate the image lv_img_set_angle(img, angle)
  // LVGL uses 0.1 degree units, hence multiply by 10. That makes 3600 in a circle.
  angle *= 10; angle = 3600 - angle;   // Subtract 3600 from the angle to correct rotation.
  lv_img_set_angle(ui_ImgCompass, int(angle));  // Update rotate compass

  /**********************/
  /* Battery State      */
  /**********************/
  showBattLevel();
}
/*********************************************************************************************/
void updateInfoScreen()
/*********************************************************************************************/
{
  while (myI2CGPS.available()) {
    N = myI2CGPS.read();
    // if (N == NQ) Serial.println(); Serial.write(N);
    gps.encode(N);

    if (GPGSV_MessageNumber.isUpdated()) fetchGSx_Data(GPGSV_UPDATED);
    if (GLGSV_MessageNumber.isUpdated()) fetchGSx_Data(GLGSV_UPDATED);
    if (GNRMC_Time.isUpdated()) {
      if (waitforGSV1) return;
      // printsatsArray100();
      showSatsPositions();  // This will update the screen,
      //  but it is not being shown right now.
      waitforGSV1 = true;
    }
  }
  //  if (!gps.time.isUpdated()) return;
  if (prevSec == gps.time.second()) return;
  prevSec = gps.time.second();
  //  Serial.printf("\r\nNew Second %i, Fix Quality: %i\r\n",
  //                prevSec, atoi(fixQuality.value()));

  if (selectedTZ == tzPH)
    convertGPSTimeToLocalTime(PHtime_zone, false);  // Philippines
  else if (selectedTZ == tzCA)
    convertGPSTimeToLocalTime(CAtime_zone, false); // California

  // Serial.println("Updating stats boxes");

  fmtWithCommas(gps.charsProcessed());
  lv_label_set_text(ui_CharData, comma_fmt_char);
  fmtWithCommas(gps.sentencesWithFix());
  lv_label_set_text(ui_SentFixedData, comma_fmt_char);
  fmtWithCommas(gps.failedChecksum());
  lv_label_set_text(ui_FailedCheckSumData, comma_fmt_char);
  fmtWithCommas(gps.passedChecksum());
  lv_label_set_text(ui_PassCheckSumData, comma_fmt_char);

  ltaLatitude += gps.location.lat(); ltaLongitude += gps.location.lng();
  ltaCount++;
  if (llFormat == llDeg) {
    snprintf(latitude_char, sizeof(latitude_char), "%.6f", ltaLatitude / ltaCount);
    strcat(latitude_char, "°");
  }
  if (llFormat == llDMS) {
    work_str = DecimalToDMS(ltaLatitude / ltaCount);
    work_str.toCharArray(latitude_char, sizeof(latitude_char));
  }
  // Send Latitude data to SL screen varible
  lv_label_set_text(ui_LatAvgData, latitude_char);

  //-----------
  if (llFormat == llDeg) {
    snprintf(longitude_char, sizeof(longitude_char), "%.6f", ltaLongitude / ltaCount);
    strcat(longitude_char, "°");
  }
  if (llFormat == llDMS) {
    work_str = DecimalToDMS(ltaLongitude / ltaCount);
    work_str.toCharArray(longitude_char, sizeof(longitude_char));
  }
  // Send Latitude data to SL screen varible
  lv_label_set_text(ui_LonAvgData, longitude_char);

  sprintf(work_char, "%i", ltaCount);  // Make something that set_text likes.
  lv_label_set_text(ui_CountData, work_char);

  // Calculate and show sun rise & set times
  calculateMoon(utcTime);  

  // Calculate and show moon rise & set times
  calculateSun(utcTime);   
  
  // Calculate and show the date of the next full moon (approximately)
  FindNextFM(utcTime);     

  lv_task_handler();  // And show on screen.
}
/***************************************************************************/
void FindNextFM(time_t Epoch)
/***************************************************************************/
{
  tmlocalTime = localtime(&Epoch);
  for (int i = 0; i < 708; i++)  // Test the next 29.5 days
  {
    tmlocalTime = localtime(&Epoch);
    moon = moonPhase.getPhase(Epoch);
    if (moon.angle >= 180 && moon.angle < 186) {
      strftime (work_char, sizeof(work_char), "%x", tmlocalTime);
      Serial.printf("Full moon on: %03i - %s. Moon phase %03i\r\n",
                    Epoch, work_char, moon.angle);
      break;  // Found it.  No need to continue the loop.
    }
    Epoch += 3600;  // Check next hour.
  }
lv_label_set_text(ui_txtNextFullMoon, work_char);
}
/*********************************************************************************************/
void updateSatsScreen()
/*********************************************************************************************/
{
  while (myI2CGPS.available()) {
    N = myI2CGPS.read();
    if (N == NQ) Serial.println();
    Serial.write(N); gps.encode(N);
    //    Serial.printf("P %i, L %i\r\n",
    //                  atoi(GPGSV_MessageNumber.value()), atoi(GLGSV_MessageNumber.value()));
    if (GPGSV_MessageNumber.isUpdated()) {
      // Serial.printf("\r\n%i GPGSV_MessageNumber.value() %i\r\n",
      //               millis(), atoi(GPGSV_MessageNumber.value()));
      fetchGSx_Data(GPGSV_UPDATED);
    }
    if (GLGSV_MessageNumber.isUpdated()) {
      // Serial.printf("\r\n%i GLGSV_MessageNumber.value() %i\r\n",
      //               millis(), atoi(GLGSV_MessageNumber.value()));
      fetchGSx_Data(GLGSV_UPDATED);
    }
    if (gps.time.minute() % 2) {
      if (selectedTZ == tzPH)
        convertGPSTimeToLocalTime(PHtime_zone, false);  // Philippines
      else if (selectedTZ == tzCA)
        convertGPSTimeToLocalTime(CAtime_zone, false); // California
      //    if (GNRMC_Time.isUpdated()) {
      if (waitforGSV1) return;
      Serial.println("Now fill the screen with sats, please. PLEASE!!!");
      printsatsArray100();
      showSatsPositions();
      waitforGSV1 = true;

      // Reset all sats to inactive status.
      for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i) satsArray100[i].active = false;

    }
  }
  //  if (!gps.time.isUpdated()) return;
  //  if (prevSec == gps.time.second()) return;
  //  prevSec = gps.time.second();
  //  Serial.printf("\r\nNew Second %i, Fix Quality: %i\r\n", prevSec, atoi(fixQuality.value()));
}
/*********************************************************************************************/
void fetchGSx_Data(int iWhich)
/*********************************************************************************************/
{
  // Serial.println();  // Only if showing NMEA data bytes.
  if (currentMessageP == 1) {
    // Show what we have collected lately then clear it out for a new collection cycle.
    //    Serial.println("Now fill the screen with sats, please. PLEASE!!!");
    //    printsatsArray100();
    //    showSatsPositions();
    // The updating of the sats screen is now triggered by reception of a GNRMC statement.
    waitforGSV1 = false;  // I'm here!  No more waiting!!!

    // Now, we start all over and collect position information.
    totSatsInView = 0;
    //    for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i) satsArray100[i].active = false;
  }
  if (iWhich == GPGSV_UPDATED) {
    /* Testing */
    currentMessageP = atoi(GPGSV_MessageNumber.value());
    Serial.printf("\r\nNow in fetchGSx_Data, currentMessageP is %i\r\n", currentMessageP);
    /* End Testing */
    for (PRN_Group = 0; PRN_Group < 4; ++PRN_Group)
    {
      arraySlotNo = atoi(satNumberP[PRN_Group].value()) - 1;
      if (arraySlotNo > -1 && arraySlotNo < SATS_INFO_ARRAY_SIZE_100)  // Array bounds check
      {
        satsArray100[arraySlotNo].elevation = atoi(elevationP[PRN_Group].value());
        satsArray100[arraySlotNo].azimuth   = atoi(azimuthP[PRN_Group].value());
        satsArray100[arraySlotNo].snr       = atoi(snrP[PRN_Group].value());
        satsArray100[arraySlotNo].active    = true;
      }
    }
    // Serial.println("Printing Sats array after fill by GPGSV");
    // printsatsArray100();
    return;  // Mostly redundant but saves a little time.
  }
  // Now time for GNSS (the rest of the world other than GPS).
  if (iWhich == GLGSV_UPDATED) {
    /* Testing */
    currentMessageL = atoi(GLGSV_MessageNumber.value());
    Serial.printf("\r\nNow in fetchGSx_Data, currentMessageL is %i\r\n", currentMessageL);
    /* End Testing */
    for (int PRN_Group = 0; PRN_Group < 4; ++PRN_Group)
    {
      arraySlotNo = atoi(satNumberL[PRN_Group].value()) - 1;
      if (arraySlotNo > -1  && arraySlotNo < SATS_INFO_ARRAY_SIZE_100)  // Array bounds check
      {
        satsArray100[arraySlotNo].elevation = atoi(elevationL[PRN_Group].value());
        satsArray100[arraySlotNo].azimuth   = atoi(azimuthL[PRN_Group].value());
        satsArray100[arraySlotNo].snr       = atoi(snrL[PRN_Group].value());
        satsArray100[arraySlotNo].active    = true;
      }
    }
    // Serial.println("Printing Sats array after fill by GLGSV");
    // printsatsArray100();
  }
}
/*********************************************************************************************/
void showLatOnScreen()
/*********************************************************************************************/
{
  latitude = gps.location.lat();

  /* Testing */
  //  latitude = 38.;
  /* Testing */

  latArray[latPtr++] = latitude;
  if (latPtr > LL_TRAIL_LEN) latPtr = 0;
  // Keeping trailing numbers for the trailing average.
  if (latCtr < LL_TRAIL_LEN) latCtr++;
  llAvg = 0;
  for (llCt = 0; llCt < latCtr; llCt++) llAvg += latArray[llCt];
  latitude = llAvg / latCtr;
  if (llFormat == llDeg) {
    snprintf(latitude_char, sizeof(latitude_char), "%.6f", latitude);
    strcat(latitude_char, "°");
  }
  if (llFormat == llDMS) {
    work_str = DecimalToDMS(latitude);
    work_str.toCharArray(latitude_char, sizeof(latitude_char));
  }
  // Send Latitude data to SL screen varible
  lv_label_set_text(ui_LatData, latitude_char);
}
/*********************************************************************************************/
void showLngOnScreen()
/*********************************************************************************************/
{
  longitude = gps.location.lng();
  /* Testing */
  //  longitude = -122.;
  /* Testing */
  lngArray[lngPtr++] = longitude;
  if (lngPtr > LL_TRAIL_LEN) lngPtr = 0;
  // Keeping trailing numbers for the trailing average.
  if (lngCtr < LL_TRAIL_LEN) lngCtr++;
  llAvg = 0;
  for (llCt = 0; llCt < lngCtr; llCt++) llAvg += lngArray[llCt];
  longitude = llAvg / lngCtr;
  if (llFormat == llDeg) {
    snprintf(longitude_char, sizeof(longitude_char), "%.6f", longitude);
    strcat(longitude_char, "°");
  }
  if (llFormat == llDMS) {
    work_str = DecimalToDMS(longitude);
    work_str.toCharArray(longitude_char, sizeof(longitude_char));
  }
  // Send Latitude data to SL screen varible
  lv_label_set_text(ui_LonData, longitude_char);
}
/*********************************************************************************************/
void showTimeAndDate()
/*********************************************************************************************/
{
  //  Serial.printf("showTimeAndDate sees %02i:%02i:%02i\r\n",
  //                gps.time.hour(), gps.time.minute(), gps.time.second());
  //  computeLocalTime();

  if (selectedTZ == tzPH)  // Philippines
    convertGPSTimeToLocalTime(PHtime_zone, false);
  else if (selectedTZ == tzCA)   // California
    convertGPSTimeToLocalTime(CAtime_zone, false);

  lv_label_set_text(ui_TimeData, time_char);
  lv_label_set_text(ui_DateData, date_char);
}
/*********************************************************************************************/
void showSatsInUseView()
/*********************************************************************************************/
{
  sats = gps.satellites.value();  // Sats in use only.
  // Convert the number of satellites to a string
  snprintf(sats_char, sizeof(sats_char), "%i", sats);

  work_int =  atoi(satsInViewP.value()) + atoi(satsInViewL.value());  // Sats in view
  snprintf(work_char, sizeof(work_char),  "%i", work_int);

  strcat(sats_char, "/"); strcat(sats_char, work_char);  // Build up the output char string.
  lv_label_set_text(ui_SatData, sats_char);

  /* Testing */
  //  Serial.print("Sats char "); Serial.print(sats_char);
  //  Serial.println(" gps.satellites.value()");
  /* End Testing */
}
/*********************************************************************************************/
void showBattLevel()
/*********************************************************************************************/
{
  battVoltage = amoled.getBattVoltage();
  if (battVoltage < 1000.) {  // Essentially 0.
    lv_label_set_text(ui_LabelBatteryLevel, "No Batt");
  }
  if (battVoltage > 3300.) {  // 3.3 volts
    lv_label_set_text_fmt(ui_LabelBatteryLevel, "%.2f v", battVoltage / 1000.);
  } else {
    lv_obj_add_flag(ui_Flash, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(ui_LabelBatteryLevel, "Lo Batt");
  }

  if (amoled.isCharging()) {
    lv_obj_clear_flag(ui_Flash, LV_OBJ_FLAG_HIDDEN);
//    amoled.XPowersPPM::enableStatLed();  // Turn on the flashing charge LED.
  } else {
    lv_obj_add_flag(ui_Flash, LV_OBJ_FLAG_HIDDEN);
//    amoled.XPowersPPM::disableStatLed();  // Turn off the flashing charge LED.
  }
}
/*********************************************************************************************/
void savePrefs()
/*********************************************************************************************/
{
  preferences.begin("lvglGPS", false);  // Open it, this time, in read/write mode.

  work_int = preferences.getInt("brightness", -1);  // -1 Says the key is not there yet.
  //  Serial.printf("Read Brightness value of %i\r\n", work_int);
  if (work_int != brightness) {
    preferences.putInt("brightness", brightness);
    Serial.printf("Saving changed brightness value of %i\r\n", brightness);
  }

  work_int = preferences.getInt("dateFmt", -1);  // -1 Says the key is not there yet.
  if (work_int != dateFormat) {
    preferences.putInt("dateFmt", dateFormat);
    Serial.printf("Saving changed dateFormat value of %i\r\n", dateFormat);
  }

  work_int = preferences.getInt("TZ", -1);  // -1 Says the key is not there yet.
  //  Serial.printf("Current saved TZ is %i, displayed is %i\r\n",
  //                work_int, selectedTZ);
  if (work_int != selectedTZ) {
    preferences.putInt("TZ", selectedTZ);
    Serial.printf("Saving changed timeZone value of %i\r\n", selectedTZ);
  }

  //  work_int = preferences.getInt("TZ",  PHtime);
  //  Serial.printf("Verifying saved selected TZ %i\r\n", work_int);

  work_int = preferences.getInt("llFormat", -1);  // -1 Says the key is not there yet.
  if (work_int != llFormat) {
    preferences.putInt("llFormat", llFormat);
    Serial.printf("Saving changed llFormat value of %i\r\n", llFormat);
  }

  work_int = preferences.getInt("altUnit",  -1);  // -1 Says the key is not there yet.
  if (work_int != altUnit) {
    preferences.putInt("altUnit", altUnit);
    Serial.printf("Saving changed altUnit value of %i\r\n", altUnit);
  }

  work_int = preferences.getInt("spdUnit",  -1);  // -1 Says the key is not there yet.
  if (work_int != spdUnit) {
    preferences.putInt("spdUnit", spdUnit);
    Serial.printf("Saving changed spdUnit value of %i\r\n", spdUnit);
  }

  work_int = preferences.getInt("distUnit", -1);  // -1 Says the key is not there yet.
  if (work_int != altUnit) {
    preferences.putInt("distUnit", distUnit);
    Serial.printf("Saving changed distUnit value of %i\r\n", distUnit);
  }

  preferences.end();  // Close the preferences locker.
}
/*********************************************************************************************/
void readPreferences()
/*********************************************************************************************/
{
  // The begin() method opens a storage space with a defined namespace.
  // The false argument means that we will use it in read/write mode.
  // Use true to open or create the namespace in read-only mode.
  // Name the "folder" we will use and set for read/write.
  // Pick up user preferences from last time.

  Serial.println("\r\nReading saved preferences.");
  preferences.begin("lvglGPS", true);  // Open it, this time, in read-only mode.

  brightness = preferences.getInt("brightness", 125);
  if (brightness < 50) brightness = 50;  // Not sure if I like this but trying it for now.
  Serial.printf("Brightness value read: %i\r\n", brightness);

  dateFormat = preferences.getInt("dateFormat", dtFormatMDY);
  Serial.printf("Date format read: %i\r\n",     dateFormat);

  selectedTZ = preferences.getInt("TZ",         PHtime);
  Serial.printf("Timezone preference: %i\r\n",  selectedTZ);
  //  setTimeZone();
  if (selectedTZ == tzPH)  // Philippines
    convertGPSTimeToLocalTime(PHtime_zone, true);
  else if (selectedTZ == tzCA)   // California
    convertGPSTimeToLocalTime(CAtime_zone, true);

  llFormat   = preferences.getInt("llFormat",   llDeg);
  Serial.printf("Lat/Long format read: %i\r\n", llFormat);

  altUnit    = preferences.getInt("altUnit",    IMPERIAL);
  Serial.printf("Altitude unit read: %i\r\n",   altUnit);

  spdUnit    = preferences.getInt("spdUnit",    IMPERIAL);
  Serial.printf("Speed unit read: %i\r\n",      spdUnit);

  distUnit   = preferences.getInt("distUnit",   IMPERIAL);
  Serial.printf("Distance unit read: %i\r\n",   distUnit);

  preferences.end();  // Close the preferences locker.
}
/*********************************************************************************************/
void printfcomma (int n)
/*********************************************************************************************/
{
  if (n < 0) {
    Serial.printf ("-");
    n = -n;
  }
  printfcomma2 (n);
}
/*********************************************************************************************/
void printfcomma2 (int n)
/*********************************************************************************************/
{
  if (n < 1000) {
    Serial.printf ("%d", n);
    return;
  }
  printfcomma2 (n / 1000);
  Serial.printf (",%03d", n % 1000);
}
/*********************************************************************************************/
void fmtWithCommas (int n)
/*********************************************************************************************/
{
  comma_fmt_char[0] = 0;
  if (n < 0) {
    strcat(comma_fmt_char, "-");
    n = -n;
  }
  fmtWithCommas2(n);
}
/*********************************************************************************************/
void fmtWithCommas2(int n)
/*********************************************************************************************/
{
  if (n < 1000) {
    sprintf(work_char, "%d", n);
    strcat(comma_fmt_char, work_char);
    return;
  }
  fmtWithCommas2(n / 1000);
  sprintf(work_char, ",%03d", n % 1000);
  strcat(comma_fmt_char, work_char);
}
/*********************************************************************************************/
void showSatsPositions()
/*********************************************************************************************/
{
  /* Testing */
  //  delay(5000);
  //  lv_obj_set_style_text_color(screenSats[0], lv_color_hex(0x0000FF),
  //                              LV_PART_MAIN | LV_STATE_DEFAULT );
  //  lv_task_handler(); delay(5000);
  //
  //  lv_obj_set_style_text_color(screenSats[0], lv_color_hex(0xFFFFFF),
  //                              LV_PART_MAIN | LV_STATE_DEFAULT );
  //  lv_task_handler(); delay(5000);
  //
  //  lv_obj_set_style_bg_color(screenSats[0], lv_color_hex(0x800000),
  //                            LV_PART_MAIN | LV_STATE_DEFAULT );
  //  lv_task_handler(); delay(5000);
  //
  //  lv_obj_set_style_bg_opa(screenSats[0], 80,
  //                          LV_PART_MAIN | LV_STATE_DEFAULT);
  //  lv_task_handler(); delay(5000);
  //
  //  lv_label_set_text(screenSats[currScreenSat], "123");
  //  lv_task_handler(); delay(5000);
  //
  //  lv_label_set_text(screenSats[currScreenSat], "00");
  //  lv_task_handler(); delay(5000);

  //  if (firstPass) delay(15000);
  //  firstPass = false;
  /* End Testing */

  for (currScreenSat = 0; currScreenSat < SCREEN_SATS_INFO_ARRAY_SIZE_33; currScreenSat++)
    lv_obj_add_flag(screenSats[currScreenSat], LV_OBJ_FLAG_HIDDEN);

  /*
      Go through all 33 sat slots looking for active sats from the big array to setup
       sats on the screen.  If we get to 33, then quit.  Or, if we get through all 100
       without getting to 33 (more likely), also quit.  Either can just do a return to
       go back to loop.
  */
  currArray100Sat = -1;   // 0 to 100, stepped in subroutine seeking full array.active sats.

  for (currScreenSat = 0; currScreenSat < SCREEN_SATS_INFO_ARRAY_SIZE_33; currScreenSat++) {
    // Skipping the one we just found, scan the big array looking for .active status.
    currArray100Sat = findNextActive(currArray100Sat + 1);
    // If -1 returned, we ran off the end.  Show progress and return.
    if (currArray100Sat == -1) {
      lv_task_handler();  // Show what we have for this pass.
      return;  // We are through here, exit and return.
    }
    currPRN = currArray100Sat + 1;  // That's the way they are loaded into the array. At PRN-1.
    //    Serial.printf("\r\nNext sat in view is in 100 array slot %i, PRN is %i, going into "
    //                  "screen sat %i\r\n", currArray100Sat, currPRN, currScreenSat);

    //    Serial.printf("Clear hidden for screenSat %i\r\n", currScreenSat);  // Show it.
    lv_obj_clear_flag(screenSats[currScreenSat], LV_OBJ_FLAG_HIDDEN);   // Show it.

    dAngle = satsArray100[currArray100Sat].azimuth; // In degrees.
    dAngle *= 0.0174533; // Radians now

    // Range the distance from Elevation to screen pixels
    satEl = satRanger - (satsArray100[currArray100Sat].elevation / 90. * satRanger);

    satX = int(sin(dAngle) * satEl);
    satY = 0 - int(cos(dAngle) * satEl);
    //     Serial.printf("PRN %i: ScreenSat %i,  X %i, Y %i\r\n", currPRN,
    //     currScreenSat, satX, satY);

    lv_obj_set_x(screenSats[currScreenSat], satX);
    lv_obj_set_y(screenSats[currScreenSat], satY);
    if (satsArray100[currScreenSat].snr > 0) {
      lv_obj_set_style_text_color(screenSats[currScreenSat], lv_color_hex(0xFFFFFF),
                                  LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_label_set_text_fmt(screenSats[currScreenSat], "%3i", currPRN);
      //      lv_obj_set_style_bg_color(screenSats[currScreenSat], lv_color_hex(0x400000),
      //                                LV_PART_MAIN | LV_STATE_DEFAULT );
      //      lv_obj_set_style_bg_opa(screenSats[currScreenSat], 255,
      //                              LV_PART_MAIN | LV_STATE_DEFAULT);
    } else {
      lv_obj_set_style_text_color(screenSats[currScreenSat], lv_color_hex(0x808080),
                                  LV_PART_MAIN | LV_STATE_DEFAULT );
      lv_label_set_text_fmt(screenSats[currScreenSat], "%3i", currPRN);
      //      lv_obj_set_style_bg_color(screenSats[currScreenSat], lv_color_hex(0x010101),
      //                                LV_PART_MAIN | LV_STATE_DEFAULT );
      //      lv_obj_set_style_text_opa(screenSats[currScreenSat], 0,
      //                                LV_PART_MAIN | LV_STATE_DEFAULT);
    }
  }
  Serial.println("-----------------------------------");
  lv_task_handler();
}
/*********************************************************************************************/
int findNextActive(int currArray100Sat)  // Find next active sat after currArray100Sat.
/*********************************************************************************************/
{
  //  Serial.printf("Looking for next active sat in the 100 array starting with slot %i\r\n",
  //                currArray100Sat);
  for (work_int = currArray100Sat; work_int < SATS_INFO_ARRAY_SIZE_100; work_int++) {
    if (satsArray100[work_int].active) {
      //      Serial.printf("Next active sat in the 100 array is %i\r\n", work_int);
      return work_int;
    }
  }
  //  Serial.println("There are no more active sats now. Returning -1");
  return -1;  // We are done here.
}
/*********************************************************************************************/
void printsatsArray100()
/*********************************************************************************************/
{
  for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i)
    if (satsArray100[i].active) totSatsInView++;

  Serial.printf("%02i:%02i:%02i Sats = %i / %i, HDOP %.1f\r\n",
                gps.time.hour(), gps.time.minute(), gps.time.second(),
                gps.satellites.value(), totSatsInView, gps.hdop.hdop());

  Serial.print("Array Slot ");
  for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i)
    if (satsArray100[i].active) Serial.printf("%3i|", i);

  Serial.print("\r\nPRNs       ");
  for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i)
    if (satsArray100[i].active) Serial.printf("%3i|", i + 1);

  Serial.print("\r\nAzimuths   ");
  for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i)
    if (satsArray100[i].active)
      Serial.printf("%3i|", satsArray100[i].azimuth);

  Serial.print("\r\nElevations ");
  for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i)
    if (satsArray100[i].active)
      Serial.printf("%3i|", satsArray100[i].elevation);

  Serial.print("\r\nSNRs       ");
  for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i)
    if (satsArray100[i].active)
      Serial.printf("%3i|", satsArray100[i].snr);

  Serial.println();
}
/*********************************************************************************************/
String DecimalToDMS(double decimalDegrees)
/*********************************************************************************************/
{
  double absDegrees;

  isNegative = (decimalDegrees < 0);
  absDegrees = abs(decimalDegrees);

  int myDegrees = int(absDegrees);
  double fractionalPart = absDegrees - double(myDegrees);
  fractionalPart *= 60.;

  int myMinutes = int(fractionalPart);
  if (myMinutes < 9) myResult += "0";
  fractionalPart -= myMinutes;

  int mySeconds = fractionalPart * 60.0;

  myResult = "";  // Clear the String for reuse.
  if (isNegative) myResult += "-";
  myResult += String(myDegrees) + "°" + String(myMinutes) + "'" + String(mySeconds) + "\"";
  //  Serial.println(myResult);
  return myResult;
}
/*********************************************************************************************/
void BtnLat(lv_event_t * e)
/*********************************************************************************************/
{
  Serial.println("Lat/Lon format changed by BtnLat.");
  llFormat++; if (llFormat > 1) llFormat = 0;
}
/*********************************************************************************************/
void BtnLon(lv_event_t * e)
/*********************************************************************************************/
{
  Serial.println("Lat/Lon format changed by BtnLon.");
  llFormat++; if (llFormat > 1) llFormat = 0;
}
/*********************************************************************************************/
void BtnAlt(lv_event_t * e)
/*********************************************************************************************/
{
  altUnit++; if (altUnit > METRIC) altUnit = 0;
}
/*********************************************************************************************/
void BtnSpeed(lv_event_t * e)
/*********************************************************************************************/
{
  spdUnit++; if (spdUnit > METRIC) spdUnit = 0;
}
///*********************************************************************************************/
//void readGPS(bool showNMEA)
///*********************************************************************************************/
//{
//  // Get them here then use the variable, later. These are "reset on read".
//  timeUpdated   = false;
//  locUpdated    = false;
//  courseUpdated = false;
//
//  while (!timeUpdated || !locUpdated || !courseUpdated) {
//    while (myI2CGPS.available()) {
//      N = myI2CGPS.read();
//      if (showNMEA) {
//        if (N == NQ) Serial.println();
//        Serial.write(N);
//      }
//      gps.encode(N);
//    }
//    timeUpdated   = gps.time.isUpdated();
//    locUpdated    = gps.location.isValid();
//    courseUpdated = gps.course.isUpdated();
//  }
//}
