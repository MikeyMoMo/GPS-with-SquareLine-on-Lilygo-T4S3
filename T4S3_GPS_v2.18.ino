#define CONFIG_FOR_JOE
#define HIDE_CHARGE_LED  // Hide the flashing charge LED.
/*
   Note: ctime() is strange.  It adds a CR/LF at the end of the data.
   It should not!
   Print it with Serial.print, not println or use a %.24s in printf
   to chop off the CR/LF.
*/
#include <LilyGo_AMOLED.h>
#include <LV_Helper.h>
#include "ui.h"           // Path to SquareLine studio export
#include "lvgl.h"

LilyGo_Class amoled;

#include "Preferences.h"
Preferences preferences;

//#include <Wire.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;  // GPS data parser setup

// $GPGSV sentence, third element  // GPS constellation
TinyGPSCustom satsInViewP(gps, "GPGSV", 3);  

// $GPGSV sentence, third element  // GLONASS constellation
TinyGPSCustom satsInViewL(gps, "GLGSV", 3);  

static const int SATS_INFO_ARRAY_SIZE_100 = 100;
/* $GPGSV,2,1,08,02,74,042,45,04,18,190,36,07,67,279,42,12,29,323,36*77

   The GSV sentence contains the following fields:
    The sentence type
    The number of sentences in the sequence
    The number of this sentence
    The number of satellites
    The satellite number, elevation, azimuth, and signal to noise ratio for each satellite
    The checksum validation value (in hexadecimal)

*/
// $GPGSV sentences this group. First element
TinyGPSCustom totalGPGSV_Messages(gps, "GPGSV", 1);
// $GPGSV this sentence.        Second element
TinyGPSCustom GPGSV_MessageNumber(gps, "GPGSV", 2);

// $GLGSV sentences this group. First element
TinyGPSCustom totalGLGSV_Messages(gps, "GLGSV", 1);
// $GLGSV this sentence.        Second element
TinyGPSCustom GLGSV_MessageNumber(gps, "GLGSV", 2);

// This is used to trigger off the sats screen update.
//  I put this here because it is taking too much time to do the screen update
//  between GxGSV sentence 1 and 2 and I am losing data because of missing characters.
//  It may not fix it all, but I am trying to fix this problem. See also waitforGSV1.
// %GNRMC time value.           First element
TinyGPSCustom GNRMC_Time   (gps, "GNRMC", 1);

TinyGPSCustom fixQuality   (gps, "GNGGA", 6); // $GPGGA sentence, 6th element, Fix Quality
#define fixQualityInvalid  0   // No fix yet or it was lost due to no sats visible.
#define fixQualitySPS      1   // 3 sat fix. Position, time and date valid.
#define fixQuality3D       2   // 3D fix.  All fields should be valid.
#define fixQualityDR       6   // Dead Reckoning Fix.  Don't know how to use this.

// Capture GPS info (to be initialized later, with code)
TinyGPSCustom satNumberP[4];
TinyGPSCustom elevationP[4];
TinyGPSCustom azimuthP[4];
TinyGPSCustom snrP[4];

// Capture GPS info (to be initialized later, with code)
TinyGPSCustom satNumberL[4];
TinyGPSCustom elevationL[4];
TinyGPSCustom azimuthL[4];
TinyGPSCustom snrL[4];

int no, totSatsInView;
int currentMessageP, currentMessageL;
int currScreenSat, currArray100Sat;
int PRN_Group, arraySlotNo;

#define SCREEN_SATS_INFO_ARRAY_SIZE_33 33
lv_obj_t* screenSats[SCREEN_SATS_INFO_ARRAY_SIZE_33];  // Pointers into SL SATs info.
#define MOON_PHASES 12
lv_obj_t* moonPhases[MOON_PHASES];

struct
{ bool active;
  int  elevation;
  int  azimuth;
  int  snr;
} satsArray100[SATS_INFO_ARRAY_SIZE_100];

// This is trying to set the range of all sats to fit in the outer circle.
//  If any are falling outside of the outermost ring, then make this a bit smaller
//  until they all fit.  Those on the outermost ring are actually on the horizon.
//  So it should show that way.  Then, all of the rest should be at their
//  approximate positions in the rings. That's the plan, anyway...
#define satRanger 200.  // Pixels to the horizon

int moonepoch = 614100;  // No idea but it is needed.

double   GPS_Movement, totMovement = 0.;

int      work_int;
int      actSat, currPRN, satX, satY;
int      angle, sats;
int      lastDot, lastV;
float    hdop, battVoltage;
bool     isNegative, rslt = false;

char     time_char[20];       // Buffers to hold the formatted strings (really overkill!)
char     date_char[20];       //    .     .   .   .      .        .
char     hdop_char[5];        //    .     .   .   .      .        .
char     sats_char[20];       //    .     .   .   .      .        .
char     heading_char[100];   //    .     .   .   .      .        .
char     dist_char[100];      //    .     .   .   .      .        .
char     speed_char[100];     //    .     .   .   .      .        .
char     work_char[200];      // Utility infielder
char     comma_fmt_char[50];  // More overkill!  But, we have lots of memory, so it's OK.
char     altitude_char[20];
String   myResult;
String   sFile, sVer;

uint8_t  brightness, level;
double   heading;
double   latitude;           // Declare varibles for GPS data
double   ltaLatitude, ltaLongitude;  // Long term averaging of latitude and longitude
char     latitude_char[20];  // Declare varibles to convert data to String for SL screen

double   longitude;
double   prevLat = 1000, prevLng = 1000, llAvg;
double   dAngle, satEl;
char     longitude_char[20];

long int ltaCount;
#define  LL_TRAIL_LEN 10  // Running average, length 10.
double   latArray[LL_TRAIL_LEN], lngArray[LL_TRAIL_LEN];
byte     latPtr = 0, lngPtr = 0, llCt;  // Work counter
byte     latCtr = 0, lngCtr = 0;  // These should be the same all the time, actually...

int      whichGSV;
#define  GPGSV_UPDATED 0
#define  GLGSV_UPDATED 1

#define  ScrMain    0
#define  ScrSats    1
#define  ScrStats   2
int      showingScreen = ScrMain;
lv_obj_t* active_screen;

//int      timeZone;
// Add all of the time zones you wish.
#define  tzPH       0
const char *PHtime_zone = "PHT-8";
#define  tzCA       1
const char *CAtime_zone = "PST8PDT,M3.2.0,M11.1.0";

int      dateFormat;
#define  dtFormatMDY 0
#define  dtFormatDMY 1

int      selectedTZ;
#define  PHtime 0
#define  CAtime 1

int      llFormat;
#define  llDeg  0
#define  llDMS  1

// Each of these can be set differently... just in case the user wants it that way.
int      altUnit  = 0;
int      spdUnit  = 0;
int      distUnit = 0;
#define  IMPERIAL 0
#define  METRIC  1

static lv_obj_t *slider_label;

#include <SparkFun_I2C_GPS_Arduino_Library.h>
I2CGPS myI2CGPS;    // Instantiate and hook object to the library
int sda_pin = 6;    // GPIO6 as I2C SDA
int scl_pin = 7;    // GPIO7 as I2C SCL

byte N;
byte NQ = '$';  // Start of next NMEA sentence. Time for a
//                    CR/LF for readability.
bool timeUpdated = false, locUpdated = false, courseUpdated = false;
// Initially waiting on a GSV sentence 1 to come along.
//  No plotting of sats yet.
bool waitforGSV1 = true;
bool prefsCkNeeded = false;

struct tm tm;
struct tm* tmlocalTime;
int    iOffset;
int    thisSec, prevSec = -1;
time_t utcTime, workTime;

#include "MoonRise.h"
MoonRise theMoon;
int    moonShowing = 0;
bool   moonVisible,   moonHasRise,  moonHasSet;
float  moonRiseAz,    moonSetAz;
time_t moonQueryTime, moonRiseTime, moonSetTime;
char   noMoonSet[]  = "None";
char   noMoonRise[] = "None";
String work_str;

#include <moonPhase.h>
moonPhase moonPhase; // include a MoonPhase instance
moonData_t moon; // variable to receive the data

#include <SunRise.h>
SunRise theSun;
bool   sunVisible,   sunHasRise,  sunHasSet;
float  sunRiseAz,    sunSetAz;
time_t sunQueryTime, sunRiseTime, sunSetTime;

//const char *PHtime_zone  = "PHT-8                 ";
//const char *CAtime_zone  = "PST8PDT,M3.2.0,M11.1.0";
const char *UTCtime_zone = "UTC0                  ";

bool   showNMEA = true, skipNMEA = false;

/*********************************************************************************************/
void setup()
/*********************************************************************************************/
{
  Serial.begin(115200); delay(2000);
  Serial.println("This is: Mike & Joe's SL/LVGL/.ino GPS app Version 1.07.");
  Serial.println("Running from:");
  Serial.println(__FILE__);
  Serial.print("Compiled on "); Serial.print(__DATE__);
  Serial.print(" "); Serial.println(__TIME__);
  Serial.printf("Board ID: %i\r\n", amoled.getBoardID());

  sFile = String(__FILE__);
  lastDot = sFile.lastIndexOf(".");
  if (lastDot > -1) {  // Found a dot.  Thank goodness!
    lastV = sFile.lastIndexOf("v");  // Find start of version number
    if (lastV > -1) {  // Oh, good, found version number, too
      sVer = sFile.substring(lastV + 1, lastDot); // Pick up version number
      lastV = sVer.lastIndexOf("\\");
      if (lastV > -1) sVer = sVer.substring(0, lastV);
      sVer = "Vers " + sVer + " " + __DATE__;
    } else {
      sVer = "0.00";  // Unknown version.
    }
  } else {
    sVer = "n/a";  // Something badly wrong here!
  }
  Serial.println(sVer);

  //  Wire.setPins();  // Set the I2C pins before begin
  //  Wire.begin(uint8_t slaveAddr, int sda, int scl, uint32_t frequency);
  if (!Wire.begin(sda_pin, scl_pin, 400000)) {
    // join i2c bus (address optional for master)
    Serial.println("Could not initialize Wire. Please check connections.");
    while (1);  //Freeze!
  }

  readPreferences();

  //rslt = amoled.beginAMOLED_147();   // Begin LilyGo  1.47 Inch AMOLED board class
  //rslt =  amoled.beginAMOLED_191();  // Begin LilyGo  1.91 Inch AMOLED board class
  //rslt =  amoled.beginAMOLED_241();  // Begin LilyGo  2.41 Inch AMOLED board class
  // ...or...
  rslt = amoled.begin();  // Automatically determine the access device

  if (!rslt) {
    while (1) {
      Serial.println("The board model cannot be detected, please raise the "
                     "Core Debug Level to an error.");
      delay(1000);
    }
  }
  beginLvglHelper(amoled);
  //#if defined HIDE_CHARGE_LED
  //  amoled.XPowersPPM::disableStatLed();  // Turn off the flashing charge LED.
  //  amoled.XPowersPPM::enableStatLed();  // Turn on the flashing charge LED.
  //#endif

  ui_init();  // Start SL & lvgl by call in ui.c

  lv_label_set_text(ui_txtTitle, "Mike & Joe's GPS");
  lv_label_set_text(ui_txtStatus1, "T4S3 w/XA1110 GPS");
  lv_label_set_text(ui_txtVersion, sVer.c_str());

  lv_obj_add_event_cb(ui_SliderBrightness, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
  lv_slider_set_value(ui_SliderBrightness, brightness, LV_ANIM_OFF);
  snprintf(work_char, sizeof(work_char), "%i", brightness);
  //  amoled.setBrightness(brightness);  // See below for actual code for this.

  if (myI2CGPS.begin() == false) {
    Serial.println("GPS Module failed to respond. Please check wiring.");
    lv_label_set_text(ui_txtStatus2, "GPS Module failed to respond. Please check wiring.");
    lv_label_set_long_mode(ui_txtStatus2, LV_LABEL_LONG_SCROLL_CIRCULAR);
    // Load and show the Splash screen.
    lv_disp_load_scr(ui_scrSplash);  lv_task_handler();  
    while (1) lv_task_handler();  //Freeze!
  }
  Serial.println("GPS module found!");
  lv_label_set_text(ui_txtStatus2, "GPS module found!");

  // The Splash screen shows some stats about the sketch (.ino files) and the 
  //  Squareline Studio code. It shows the release version of both and tells if the 
  //  GPS module was found.  It is shown for a few seconds (see below) and, 
  //  while it is showing, it is also reading in NMEA data.
  lv_disp_load_scr(ui_scrSplash);  lv_task_handler(); // Load and show the Splash screen.
  int pctDone, prev_pctDone = -1;
  long unsigned endMillis = millis() + 10000;
  long unsigned nowMillis = millis();
  lv_bar_set_range(ui_Bar1, 0, 100); // Range set from 0 to 100 (for scaling)
  Serial.println("Start Splash screen and NMEA reading.");
  while (millis() < endMillis) { // Show Splash screen and read for 10 seconds.
    while (myI2CGPS.available()) {
      N = myI2CGPS.read();
      // if (N == NQ) Serial.println();
      // Serial.write(N);
      gps.encode(N);
    }
    pctDone = (millis() - nowMillis) / 100;
    if (prev_pctDone != pctDone) {
      prev_pctDone = pctDone;
      lv_bar_set_value(ui_Bar1, 100 - pctDone, LV_ANIM_OFF);
      lv_task_handler();  // Update the screen and handle touch events.
    }
  }
  Serial.println("End Splash");
  // I moved the setBrightness down here so the Splash screen will show 
  //  at full brightness because it looks really good that way.  
  //  It really "pops" at full brightness.
  amoled.setBrightness(brightness);

  // After a 5 second wait, show the main screen.
  lv_disp_load_scr(ui_screenMain); lv_task_handler();  

  // Finish initialization for the uninitialized TinyGPSCustom objects
  for (int i = 0; i < 4; ++i)  // A bit silly.  Should fix.
  {
    satNumberP[i].begin(gps, "GPGSV", 4 + 4 * i); // offsets 4,  8, 12, 16
    elevationP[i].begin(gps, "GPGSV", 5 + 4 * i); // offsets 5,  9, 13, 17
    azimuthP[i].begin  (gps, "GPGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
    snrP[i].begin      (gps, "GPGSV", 7 + 4 * i); // offsets 7, 11, 15, 19

    satNumberL[i].begin(gps, "GLGSV", 4 + 4 * i); // offsets 4,  8, 12, 16
    elevationL[i].begin(gps, "GLGSV", 5 + 4 * i); // offsets 5,  9, 13, 17
    azimuthL[i].begin  (gps, "GLGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
    snrL[i].begin      (gps, "GLGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
  }

  // This allows referencing the sats numbers by array reference.
  screenSats[0]  = ui_Sat00; screenSats[1]  = ui_Sat01; screenSats[2]  = ui_Sat02;
  screenSats[3]  = ui_Sat03; screenSats[4]  = ui_Sat04; screenSats[5]  = ui_Sat05;
  screenSats[6]  = ui_Sat06; screenSats[7]  = ui_Sat07; screenSats[8]  = ui_Sat08;
  screenSats[9]  = ui_Sat09; screenSats[10] = ui_Sat10; screenSats[11] = ui_Sat11;
  screenSats[12] = ui_Sat12; screenSats[13] = ui_Sat13; screenSats[14] = ui_Sat14;
  screenSats[15] = ui_Sat15; screenSats[16] = ui_Sat16; screenSats[17] = ui_Sat17;
  screenSats[18] = ui_Sat18; screenSats[19] = ui_Sat19; screenSats[20] = ui_Sat20;
  screenSats[21] = ui_Sat21; screenSats[22] = ui_Sat22; screenSats[23] = ui_Sat23;
  screenSats[24] = ui_Sat24; screenSats[25] = ui_Sat25; screenSats[26] = ui_Sat26;
  screenSats[27] = ui_Sat27; screenSats[28] = ui_Sat28; screenSats[29] = ui_Sat29;
  screenSats[30] = ui_Sat30; screenSats[31] = ui_Sat31; screenSats[32] = ui_Sat32;

  for (currScreenSat = 0; currScreenSat < SCREEN_SATS_INFO_ARRAY_SIZE_33; currScreenSat++)
    lv_obj_add_flag(screenSats[currScreenSat], LV_OBJ_FLAG_HIDDEN);

  // This allows referencing the moon phases by array reference.
  moonPhases[0] = ui_M000;   moonPhases[1]  = ui_M030; moonPhases[2]  = ui_M060;
  moonPhases[3] = ui_M090;   moonPhases[4]  = ui_M120; moonPhases[5]  = ui_M150;
  moonPhases[6] = ui_M180;   moonPhases[7]  = ui_M210; moonPhases[8]  = ui_M240;
  moonPhases[9] = ui_M270;   moonPhases[10] = ui_M300; moonPhases[11] = ui_M330;

  while (!timeUpdated || !locUpdated || !courseUpdated) {  // All initially fail
    while (myI2CGPS.available()) {
      N = myI2CGPS.read();
      if (N == NQ) Serial.println();
      Serial.write(N);
      gps.encode(N);
    }
    showBattLevel();
    showSatsInUseView();
    if (timeUpdated) {
      Serial.println();  // Just for making pretty.
      if (selectedTZ == tzPH)  // Philippines
        convertGPSTimeToLocalTime(PHtime_zone, true);
      else if (selectedTZ == tzCA)   // California
        convertGPSTimeToLocalTime(CAtime_zone, true);

      showTimeAndDate();
    }
    if (locUpdated) {
      showLatOnScreen();
      showLngOnScreen();
    }
    lv_task_handler();
    timeUpdated   = gps.time.isUpdated();    // These are "reset on read".
    locUpdated    = gps.location.isValid();  // Get them here then use the variable, later.
    courseUpdated = gps.course.isUpdated();  // That avoids incorrect, partial results.
  }

  lv_label_set_long_mode(ui_HeadingData, LV_LABEL_LONG_CLIP);  // NO JUMPING!

  Serial.println();
  Serial.println("**********************");
  Serial.println("**********************");
  Serial.println("*** Setup Finished ***");
  Serial.println("**********************");
  Serial.println("**********************");
}
/*********************************************************************************************/
void loop()
/*********************************************************************************************/
{
  lv_task_handler();  // Update the screen and handle touch events.

  //  readGPS(skipNMEA);  // Read NMEA data till exhausted.
  //  thisSec = gps.time.second();
  //  if (prevSec == thisSec) return;
  //  prevSec == thisSec;

  //  Serial.printf("Loop min:sec %02i:%02i\r\n", gps.time.minute(), gps.time.second());

  latitude  = gps.location.lat();
  longitude = gps.location.lng();

  if (selectedTZ == tzPH)
    convertGPSTimeToLocalTime(PHtime_zone, false);  // Philippines
  else if (selectedTZ == tzCA)
    convertGPSTimeToLocalTime(CAtime_zone, false); // California

  //  Serial.printf("Local Time: %02i:%02i:%02i %02i/%02i/%i",
  //                tmlocalTime->tm_hour, tmlocalTime->tm_min, tmlocalTime->tm_sec,
  //                tmlocalTime->tm_mon + 1, tmlocalTime->tm_mday,
  //                tmlocalTime->tm_year + 1900);
  //  strftime (work_char, sizeof(work_char), " %Z %z", tmlocalTime);
  //  Serial.println(work_char);

  if (Serial.available()) {
    work_int = toUpperCase(Serial.read());
    if (work_int == 'M' || work_int == '1')
      lv_disp_load_scr(ui_screenMain);  // Does all of the stuff to show a different screen.
    if (work_int == 'S' || work_int == '2')
      lv_disp_load_scr(ui_screenSats);  // Does all of the stuff to show a different screen.
    if (work_int == 'I' || work_int == '3')
      lv_disp_load_scr(ui_screenInfo);  // Does all of the stuff to show a different screen.
  }

  /* Testing for finding a no-rise day. If so, the time will be 0
     Same for set time for no-set day.
  */
  //  for (time_t fifteenDays = utcTime; fifteenDays < utcTime + 1296000;
  //       fifteenDays += 86400)
  //  {
  //    Serial.println(">>>>>>>>>>>>>>>> New Test Day <<<<<<<<<<<<<<<<");
  //    calculateMoon(fifteenDays);
  //  }
  /* End Testing */

  //  calculateMoon(utcTime);
  //  calculateSun(utcTime);

  //  if (localTime->tm_isdst > 0)
  //    Serial.println("DST is true");
  //  else
  //    Serial.println("DST is false");
  // Update GPS data periodically

  active_screen = lv_scr_act();

  if (active_screen == ui_screenMain) {
    //    Serial.println("Updating main screen");
    updateMainScreen();
    ltaLatitude = 0; ltaLongitude = 0; ltaCount = 0;
  }

  if (active_screen == ui_screenInfo) {
    //    Serial.println("Updating info screen");
    updateInfoScreen();
  }

  if (active_screen == ui_screenSats) {
    //    Serial.println("Updating sats screen");
    updateSatsScreen();
    ltaLatitude = 0; ltaLongitude = 0; ltaCount = 0;
  }
  if (gps.time.second() == 0 && gps.time.minute() % 10 == 0) {
    if (prefsCkNeeded) {
      Serial.printf("UTC Time %02i:%02i:%02i. Saving changed preferences.\r\n",
                    gps.time.hour(), gps.time.minute(), gps.time.second());
      savePrefs();
      prefsCkNeeded = false;
    }
  } else {
    prefsCkNeeded = true;
  }
}
/***************************************************************************/
void convertGPSTimeToLocalTime(const char * homeZone, bool forceIt)
/***************************************************************************/
{
  //  static int ctPrev = -1;

  //  Serial.printf("convertGPSTimeToLocalTime UTC %02i:%02i:%02i\r\n",
  //                gps.time.hour(), gps.time.minute(), gps.time.second(),
  //                forceIt);
  //  if (gps.time.minute() == 0 && gps.time.second() == 0 || forceIt) {

  //  if (ctPrev == gps.time.second()) return;
  if (!timeUpdated) return;

  //  ctPrev = gps.time.second();
  //  if (timeUpdated) {
  //  Serial.println("\r\nTop of the hour or init'ing, "
  //                 "setting the internal clock to UTC.");
  //  Serial.println("New second. Recomputing time.");

  setenv("TZ", UTCtime_zone, 1); tzset();
  // time(&utcTime); Serial.print(utcTime);
  // Serial.print(" - UTC: "); Serial.print(ctime(&utcTime));
  tm.tm_year = gps.date.year() - 1900;  // Year since 1900
  tm.tm_mon  = gps.date.month() - 1;    // Month (0-11)
  tm.tm_mday = gps.date.day();          // Day of the month (1-31)
  tm.tm_hour = gps.time.hour();         // Hours (0-23)
  tm.tm_min  = gps.time.minute();       // Minutes (0-59)
  tm.tm_sec  = gps.time.second();       // Seconds (0-59)
  tm.tm_isdst = -1;  // Daylight saving time flag - Set to unknown. Will be reset soon.
  utcTime = mktime(&tm);
  //  Serial.printf("UTC Epoch %lu", utcTime);
  //  Serial.print(" - UTC: "); Serial.print(ctime(&utcTime));
  localtime_r(&utcTime, &tm);
  strftime(work_char, sizeof(work_char), "%X %m/%d/%y", &tm);
  //    Serial.printf("UTC Time %s\r\n", work_char);

  //  setTime(utcTime);  // Not in this lifetime!
  // No, I have NO IDEA what this means or how this works!!!
  struct timeval now = {.tv_sec = utcTime};
  settimeofday(&now, NULL);
  setenv("TZ", homeZone, 1); tzset();
  strftime(work_char, sizeof(work_char), "%z", localtime(&utcTime));
  iOffset = atoi(work_char);  // Temporary holder
  // The format of the offset is a little strange.  Here's the decode of it.
  // Get the offset in seconds.
  iOffset = (iOffset / 100) * 3600 + iOffset % 100 * 60;
  //  Serial.printf("Offset %i\r\n", iOffset);

  //  time(&utcTime);
  //  Serial.printf("UTC Epoch %lu", utcTime);
  //  Serial.print(" - UTC: "); Serial.println(ctime(&utcTime));
  tmlocalTime = localtime(&utcTime);
  strftime (time_char, sizeof(time_char), "%T", tmlocalTime);
  //  Serial.print("Local Time: "); Serial.print(time_char);
  strftime (date_char, sizeof(date_char), "%m/%d/%Y", tmlocalTime);
  //  Serial.print(" - Local Date: "); Serial.println(date_char);
  //}
}
/*********************************************************************************************/
static void slider_event_cb(lv_event_t * e)
/*********************************************************************************************/
{
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  // Get the slider value
  brightness = (uint8_t)lv_slider_get_value(slider);
  Serial.printf("New brightness level: %i\r\n", brightness);
  // Set the brightness level
  amoled.setBrightness(brightness);
}
/*********************************************************************************************/
void Change2SatView(lv_event_t * e)
/*********************************************************************************************/
{
  // This is triggered by ui_event_Button1 which overlays the right arrow (ui_LabelRight)
  Serial.println("Change2SatView-Changing to ScreenSatView");
  showingScreen = ScrSats;
}
/*********************************************************************************************/
void Back2SatView(lv_event_t * e)  // This should go away after Joe fixes the SL.
/*********************************************************************************************/
{
  Serial.println("Back2SatView-Changing to ScreenSatView");
  showingScreen = ScrSats;
}
/*********************************************************************************************/
void Change2Main(lv_event_t * e)
/*********************************************************************************************/
{
  // This is triggered by ui_event_Button2 which overlays the left arrow (ui_LabelLeft)
  Serial.println("Change2Main-Changing to ScreenMain");
  showingScreen = ScrMain;
}
/*********************************************************************************************/
void Change2Statements(lv_event_t * e)
/*********************************************************************************************/
{
  Serial.println("Change2Statements-Changing to ScreenStats");
  showingScreen = ScrStats;
}
/*********************************************************************************************/
void BtnDist(lv_event_t * e)  // Click function
/*********************************************************************************************/
{
  distUnit++; if (distUnit > 1) distUnit = 0;
  Serial.println("Distance unit change by BtnDist.");
}
/*********************************************************************************************/
void BtnDist_Reset(lv_event_t * e)  // Long click
/*********************************************************************************************/
{
  totMovement = 0;
  Serial.println("Total distance traveled reset to 0.");
}
/*********************************************************************************************/
void BtnDate(lv_event_t * e)
/*********************************************************************************************/
{
  dateFormat++; if (dateFormat > dtFormatDMY) dateFormat = 0; // Reset to start.
  Serial.println("Date format changed");

  //  const  BoardsConfigure_t *boards = amoled.getBoarsdConfigure();
  //  uint8_t id = amoled.getBoardID();
  //  Serial.print("Board ID: "); Serial.print(id);
  //  Serial.print(" ");
  //  Serial.print(boards->pmu);
  //  Serial.print("="); Serial.print(LILYGO_AMOLED_147); Serial.println("?");
  //  if (boards->pmu && id == LILYGO_AMOLED_147) {
  //    Toggle CHG led
  //    amoled.setChargingLedMode(
  //      amoled.getChargingLedMode() != XPOWERS_CHG_LED_OFF ?
  //      XPOWERS_CHG_LED_OFF : XPOWERS_CHG_LED_ON);
  //  }
  //  amoled.setChargingLedMode(XPOWERS_CHG_LED_OFF); delay(5000);
  //  amoled.setChargingLedMode(XPOWERS_CHG_LED_ON);  delay(5000);
  //  amoled.setChargingLedMode(XPOWERS_CHG_LED_OFF); delay(5000);
  //  amoled.setChargingLedMode(XPOWERS_CHG_LED_ON);  delay(5000);
  //  amoled.setChargingLedMode(XPOWERS_CHG_LED_OFF); delay(5000);
  //  amoled.setChargingLedMode(XPOWERS_CHG_LED_ON);  delay(5000);
  //  amoled.setChargingLedMode(XPOWERS_CHG_LED_OFF); delay(5000);
  //  amoled.setChargingLedMode(XPOWERS_CHG_LED_ON);  delay(5000);
  //  amoled.setChargingLedMode(XPOWERS_CHG_LED_OFF); delay(5000);
}
/*********************************************************************************************/
void BtnTime(lv_event_t * e)
/*********************************************************************************************/
{
  // Change the time zone to the last defined one to reset it after reaching max+1.
  selectedTZ++; if (selectedTZ > tzCA) selectedTZ = 0;
  if (selectedTZ == tzPH)  // Philippines
    convertGPSTimeToLocalTime(PHtime_zone, true);
  else if (selectedTZ == tzCA)   // California
    convertGPSTimeToLocalTime(CAtime_zone, true);

  //  computeLocalTime();
  if (selectedTZ == tzPH)  // Philippines
    convertGPSTimeToLocalTime(PHtime_zone, true);
  else if (selectedTZ == tzCA)   // California
    convertGPSTimeToLocalTime(CAtime_zone, true);
}
