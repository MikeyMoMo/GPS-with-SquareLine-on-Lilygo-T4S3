/*********************************************************************************************/
void calculateMoon(time_t utcTime)  // Find the last and next moon set and rise times.
/*********************************************************************************************/
{
  //  Serial.print("Local time "); Serial.print(ctime(&utcTime));
  //  Serial.printf("Time for moon rise/set calculations: %lu\r\n", utcTime);
  //  Serial.println("\r\n--->calcMoon info");

  theMoon.calculate(latitude, longitude, utcTime);
  moonVisible = theMoon.isVisible; moonHasRise = theMoon.hasRise; moonHasSet = theMoon.hasSet;
  // Where the moon will rise/set in degrees from the North.
  moonRiseAz  = theMoon.riseAz; moonSetAz   = theMoon.setAz;

  moonQueryTime = theMoon.queryTime; moonRiseTime = theMoon.riseTime;
  moonSetTime   = theMoon.setTime;

  //  Serial.println("Query time "); Serial.print(moonQueryTime);
  //  The next two will be zero if there is no rise or set within the window.
  //  Serial.println("Rise  time "); Serial.print(moonRiseTime);
  //  Serial.println("Set   time "); Serial.print(moonSetTime);

  //  Serial.printf("Moon rise/set nearest %.24s for latitude %.2f longitude %.2f:\n",
  //                ctime(&theMoon.queryTime), latitude, longitude);

  //  Serial.printf("Preceding event: ");
  //  if ((!theMoon.hasRise || (theMoon.hasRise && theMoon.riseTime > theMoon.queryTime)) &&
  //      (!theMoon.hasSet || (theMoon.hasSet && theMoon.setTime > theMoon.queryTime)))
  //    Serial.printf("No moon rise or set during preceding %d hours\n", MR_WINDOW);
  //  if (theMoon.hasRise && theMoon.riseTime < theMoon.queryTime)
  //    Serial.printf("Moon rise at %.24s, Azimuth %.2f\n", ctime(&moonRiseTime), moonRiseAz);
  //  if (theMoon.hasSet && theMoon.setTime < theMoon.queryTime)
  //    Serial.printf("Moon set at %.24s, Azimuth %.2f\n", ctime(&moonSetTime), moonSetAz);

  //  Serial.printf("Succeeding event: ");
  //  if ((!theMoon.hasRise || (theMoon.hasRise && theMoon.riseTime < theMoon.queryTime)) &&
  //      (!theMoon.hasSet || (theMoon.hasSet && theMoon.setTime < theMoon.queryTime)))
  //    Serial.printf("No moon rise or set during succeeding %d hours\n", MR_WINDOW);
  //  if (theMoon.hasRise && moonRiseTime > moonQueryTime)
  //    Serial.printf("Moon rise at %.24s, Azimuth %.2f\n", ctime(&moonRiseTime), moonRiseAz);
  //  if (theMoon.hasSet && moonSetTime > moonQueryTime)
  //    Serial.printf("Moon set at %.24s, Azimuth %.2f\n", ctime(&moonSetTime), moonSetAz);

  //  if (theMoon.isVisible)
  //    Serial.printf("Moon visible.\n");
  //  else
  //    Serial.printf("Moon not visible.\n");

  moon = moonPhase.getPhase(utcTime);

  Serial.print("Moon phase angle: ");
  Serial.print(moon.angle);           // angle is a integer from 0 to 359°.

  lv_obj_add_flag(moonPhases[moonShowing], LV_OBJ_FLAG_HIDDEN);  // Hide previous showing one.
  switch ((int)(moon.angle + .5))
  {
    // (0 and 29 being hidden and 14 is full)
    case   0 ...  29: moonShowing =  0; break;
    case  30 ...  59: moonShowing =  1; break;
    case  60 ...  89: moonShowing =  2; break;
    case  90 ... 119: moonShowing =  3; break;
    case 120 ... 149: moonShowing =  4; break;
    case 150 ... 179: moonShowing =  5; break;
    case 180 ... 209: moonShowing =  6; break;
    case 210 ... 239: moonShowing =  7; break;
    case 240 ... 269: moonShowing =  8; break;
    case 270 ... 299: moonShowing =  9; break;
    case 300 ... 329: moonShowing = 10; break;
    case 330 ... 359: moonShowing = 11; break;
  }
  lv_obj_clear_flag(moonPhases[moonShowing], LV_OBJ_FLAG_HIDDEN);   // Show new phase.

  Serial.print("°");
  Serial.print(" / Moon surface lit: ");
  Serial.printf("%.2f%%\r\n", moon.percentLit * 100.); // percentLit is a real between 0-1

  //  tmlocalTime = localtime(&moonQueryTime);
  //  strftime (work_char, sizeof(work_char), "(strftime) Moon query Time %R", tmlocalTime);
  //  Serial.println(work_char);

  //  Serial.println(moonRiseTime);  // time_t of Moon Rise Time
  if (moonRiseTime == 0) {
    strncpy(work_char, noMoonRise, sizeof(work_char));
  } else {
    tmlocalTime = localtime(&moonRiseTime);
    strftime (work_char, sizeof(work_char), "%R", tmlocalTime);
  }
  //  Serial.print("(strftime) Moon Rise  Time "); Serial.println(work_char);
  lv_label_set_text(ui_MoonRiseData, work_char);

  //  Serial.println(moonSetTime);  // time_t of Moon Set Time
  if (moonSetTime == 0) {
    strncpy(work_char, noMoonSet, sizeof(work_char));
  } else {
    tmlocalTime = localtime(&moonSetTime);
    strftime (work_char, sizeof(work_char), "%R", tmlocalTime);
  }
  //  Serial.print("(strftime) Moon Set   Time "); Serial.println(work_char);
  lv_label_set_text(ui_MoonSetData, work_char);

  //-----------------------------------

  //  int currMoonPhase = (t - moonepoch) % 2551443;
  float fcurrMoonPhase = (float)(utcTime - moonepoch);
  //  Serial.printf("\n%f", fcurrMoonPhase);
  fcurrMoonPhase = fmod(fcurrMoonPhase, 2551443.);
  //  Serial.printf("\n%f", fcurrMoonPhase);

  //  int currMoonAge = floor(currMoonPhase / (24 * 3600)) + 1;
  float fcurrMoonAge = (fcurrMoonPhase / (24. * 3600.));  // + 1.;
  //  Serial.printf("\ncurrMoonAge %i, fcurrMoonAge %f", currMoonAge, fcurrMoonAge);

  //  if (currMoonAge == 30) currMoonAge = 0;
  //  if (fcurrMoonAge >= 29.) fcurrMoonAge = 0.;  // Covers 29 = 0 and 30 out of range
  //  Serial.printf("\nMoon Phase# %i - ", currMoonAge);
  Serial.printf("Moon Age (Actual) %.2f, (Rounded) %i - ",
                fcurrMoonAge, int(fcurrMoonAge + .5));
  switch ((int)(fcurrMoonAge + .5))
  {
    // (0 and 29 being hidden and 14 is full)
    case  0:
      lv_label_set_text(ui_MoonPhases, "New Moon");
      Serial.printf("New Moon\r\n");
      break;
    case  1 ... 6:
      lv_label_set_text(ui_MoonPhases, "Waxing Crescent");
      Serial.printf("Waxing Crescent\r\n");
      break;
    case  7 ... 8:
      lv_label_set_text(ui_MoonPhases, "");
      Serial.printf("First Quarter\r\n");
      break;
    case  9 ... 13:
      lv_label_set_text(ui_MoonPhases, "Waxing Gibbus");
      Serial.printf("Waxing Gibbus\r\n");
      break;
    case 14 ... 16:
      lv_label_set_text(ui_MoonPhases, "Full Moon");
      Serial.printf("Full Moon\r\n");
      break;
    case 17 ... 21:
      lv_label_set_text(ui_MoonPhases, "Waning Gibbus");
      Serial.printf("Waning Gibbus\r\n");
      break;
    case 22 ... 22:       lv_label_set_text(ui_MoonPhases, "Last Quarter");
      Serial.printf("Last Quarter\r\n");
      break;
    case 23 ... 28:
      lv_label_set_text(ui_MoonPhases, "Waning Crescent");
      Serial.printf("Waning Crescent\r\n");
      break;
    case 29 ... 30:
      lv_label_set_text(ui_MoonPhases, "New Moon");
      Serial.printf("New Moon\r\n");
      break;
  }
}
