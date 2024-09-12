// This was working with Espressif Systems version 2.0.14. Recently upgraded to 3.0.4.
// 3.0.4 fails miserably.  Had to go back to 2.0.14.

/* Notes

   Press (click) directly on displayed items to change their presentation.
   Presentation options are saved each 10 minutes, if changed,
   to save on wear on the flash memory.

   The internal speed and distance are kept in meters and adjusted upon display only.
   The altitude is kept in meters and adjusted to feet upon display only.

   Only 2 presentations are supported, currently, for latitude and longitude.
   I would like to add UTM and Maidenhead.  UTM has always been a pain.
   Maidenhead is not so bad -- used by ham radio operators.

   Only 2 timezones are implemented, currently, but more can easily be added.
    They will rotate through as the time button is pressed.
    See the BtnTime routine to change it to add more. If you add them at the
    top of the current list, you won't have to change the routine at all.
    Look for timeZone label
    and adjust as needed.
*/
#include <Arduino.h>
#include <LilyGo_AMOLED.h>
#include <LV_Helper.h>
#include "ui.h"           // Path to squareline studio export
#include "lvgl.h"
#include <ESP32Time.h>
#include <time.h>       /* time_t, struct tm, time, localtime */

ESP32Time rtc(0);
#include "Preferences.h"
Preferences preferences;

#include <MoonRise.h>
MoonRise theMoon;

#include <moonPhase.h>
moonPhase moonPhase; // include a MoonPhase instance

#include <SunRise.h>
SunRise theSun;

LilyGo_Class amoled;

// Use Library Manager or download here:`
//  https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library
#include <SparkFun_I2C_GPS_Arduino_Library.h>
I2CGPS myI2CGPS;                             // Instantiate and hook object to the library
int sda_pin = 6;                             // GPIO6 as I2C SDA
int scl_pin = 7;                             // GPIO7 as I2C SCL

//From: https://github.com/mikalhart/TinyGPSPlus
#include <TinyGPS++.h>
TinyGPSPlus gps;                             // Declare and instantiate the gps object
TinyGPSCustom satsInViewP(gps, "GPGSV", 3);  // $GPGSV sentence, third element
TinyGPSCustom satsInViewL(gps, "GLGSV", 3);  // $GPGSV sentence, third element
static const int SATS_INFO_ARRAY_SIZE_100 = 100;

bool   moonHasRise, moonHasSet, moonVisible;
float  moonRiseAz, moonSetAz;
time_t moonRiseTime, moonSetTime, moonQueryTime;

bool   sunVisible, sunHasRise, sunHasSet;
float  sunRiseAz, sunSetAz;
time_t sunQueryTime, sunRiseTime, sunSetTime;

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
//       %GNRMC time value.           First element
TinyGPSCustom GNRMC_Time   (gps, "GNRMC", 1);

TinyGPSCustom fixQuality   (gps, "GNGGA", 6); // $GPGGA sentence, 6th element, Fix Quality
#define fixQualityInvalid  0   // No fix yet or it was lost due to no sats visible.
#define fixQualitySPS      1   // 3 sat fix. Position, time and date valid.
#define fixQuality3D       2   // 3D fix.  All fields should be valid.
#define fixQualityDR       6   // Dead Reckoning Fix.  Don't know how to use this.

// Capture GPS info
TinyGPSCustom satNumberP[4]; // to be initialized later
TinyGPSCustom elevationP[4];
TinyGPSCustom azimuthP[4];
TinyGPSCustom snrP[4];

// Capture GNSS info
TinyGPSCustom satNumberL[4]; // to be initialized later
TinyGPSCustom elevationL[4];
TinyGPSCustom azimuthL[4];
TinyGPSCustom snrL[4];
int totSatsInView;
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

// This is trying to set the range of all sats to fit in the outer circle.  If any are falling
//  outside of the outermost ring, then make this a bit smaller until they all fit.  Those on
//  the outermost ring are actually on the horizon.  So it should show that way.  Then, all of
//  the rest should be at their approximate positions in the rings. That's the plan, anyway...
#define satRanger 200.

/* The calcFullDate variables made available here */
boolean myIsPM;
int     cFD_Sec, cFD_Min, cFD_Hour, cFD_Day, cFD_Year, cFD_Days, cFD_Month;
int     cFD_DOM, cFD_DOW;
char    cFD_FullDate[100];
const boolean showAMPM = false;   // If you like AM and PM. False gives 24 hour clock format.
unsigned int  todayJulian;
unsigned int  newJulian;
unsigned long epochTime;
int   moonepoch = 614100, moonShowing = 0;  // Must be a valid moonphase number.
int   sunSetH, sunSetM;  // Hour and minute of sunset.
int   currentHour;
char  builtDate[100];
char  saveBuiltDate[100];
char  workDate[100];
char  justTime[100];  // Just the time part of the date-time string.
const unsigned char month_length[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

int   myMonth, myDOM, myYear, myHour, myMinute, mySecond, lastSecond = 100;

char  daysOfTheWeek[7][5] = {"Sun ", "Mon ", "Tue ", "Wed ",
                             "Thu ", "Fri ", "Sat "
                            };
//Month names
char  months[12][5] = {"Jan ", "Feb ", "Mar ", "Apr ", "May ", "Jun ",
                       "Jul ", "Aug ", "Sep ", "Oct ", "Nov ", "Dec "
                      };

int iOffset, iTempOffset;
//const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
time_t   UTC, workTime;
int      localTimeOffset = 0;
struct   tm timeinfo;
int      isItDST;  // +1 means DST in effect. 0 means no DST. -1 means invalid, I change to 0.
char     cCharWork[500];
char     TZid[10];
int      showHeadingTime = 1000, yPos;
const int ySpacing = 70;  // Line spacing for display
float    fWork1, fWork2, battVoltage;
uint16_t iWork1;
double   gps_course_deg;
double   GPS_Movement, totMovement = 0.;
char     localTime[10];
char     localDate[10];
String   myResult;
bool     isNegative, rslt = false;
// Initially waiting on a GSV sentence 1 to come along. No plotting of sats.
bool     waitforGSV1 = true;
bool     timeUpdated = false, locUpdated = false, courseUpdated = false;
int      actSat, currPRN, satX, satY, moonPhaseShowing = 0;

double   latitude;           // Declare varibles for GPS data
char     latitude_char[20];  // Declare varibles to convert data to String for SL screen
double   ltaLatitude, ltaLongitude;  // Long term averaging of latitude and longitude
long int ltaCount;

double   longitude;
char     longitude_char[20];
double   prevLat = 1000, prevLng = 1000, llAvg;
double   dAngle, satEl;

#define  LL_TRAIL_LEN 10  // Running average, length 10.
double   latArray[LL_TRAIL_LEN], lngArray[LL_TRAIL_LEN];
byte     latPtr = 0, lngPtr = 0, llCt;  // Work counter
byte     latCtr = 0, lngCtr = 0;  // These should be the same all the time, actually...

char     altitude_char[20];
float    hdop;
char     date_char[10];

char     time_char[7];        // Buffers to hold the formatted strings (really overkill!)
char     hdop_char[5];        //    .     .   .   .      . .
char     bat_char[10];        //    .     .   .   .      . .
char     sats_char[20];       //    .     .   .   .      . .
char     heading_char[100];   //    .     .   .   .      . .
char     dist_char[100];      //    .     .   .   .      . .
char     speed_char[100];     //    .     .   .   .      . .
char     work_char[200];      // Utility infielder
char     comma_fmt_char[50];  // More overkill!  But, we have lots of memory, so it's OK.
String   work_str;
int      work_int;
byte     N;
byte     NQ = '$';  // Start of next NMEA sentence.  Time for a CR/LF for beauty's sake.

int      whichGSV;
#define  GPGSV_UPDATED 0
#define  GLGSV_UPDATED 1

//double   heading;
int      prevSec = -1;  // 1 second gate on screen updates and other stuff.
int      angle, sats;
unsigned long endMillis;
uint8_t  brightness, level;

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
  readPreferences();
  Wire.setPins(sda_pin, scl_pin);  // Set the I2C pins before begin
  Wire.begin();                    // join i2c bus (address optional for master)
  if (myI2CGPS.begin() == false) {
    Serial.println("Module failed to respond. Please check wiring.");
    while (1);  //Freeze!
  }
  Serial.println("GPS module found!");
  Serial.println("Running from:");
  Serial.println(__FILE__);

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
  amoled.setBrightness(brightness);     // 0 to 255 is the range.
  amoled.XPowersPPM::disableStatLed();  // Turn off the flashing charge LED.

  ui_init();  // Start lvgl ui.c

  // Initialize all the uninitialized TinyGPSCustom objects
  for (int i = 0; i < 4; ++i)  // A bit silly.  Should fix.
  {
    satNumberP[i].begin(gps, "GPGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
    elevationP[i].begin(gps, "GPGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
    azimuthP[i].begin  (gps, "GPGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
    snrP[i].begin      (gps, "GPGSV", 7 + 4 * i); // offsets 7, 11, 15, 19

    satNumberL[i].begin(gps, "GLGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
    elevationL[i].begin(gps, "GLGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
    azimuthL[i].begin  (gps, "GLGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
    snrL[i].begin      (gps, "GLGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
  }

  // Put the sat number widgets into an array for easy indexing and changing data.
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

  // Put the moon phase pix addresses into an array for easy setting shown and hidden.
  moonPhases[0] = ui_M000;   moonPhases[1]  = ui_M030; moonPhases[2]  = ui_M060;
  moonPhases[3] = ui_M090;   moonPhases[4]  = ui_M120; moonPhases[5]  = ui_M150;
  moonPhases[6] = ui_M180;   moonPhases[7]  = ui_M210; moonPhases[8]  = ui_M240;
  moonPhases[9] = ui_M270;   moonPhases[10] = ui_M300; moonPhases[11] = ui_M330;

  lv_obj_add_event_cb(ui_SliderBrightness, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
  lv_slider_set_value(ui_SliderBrightness, brightness, LV_ANIM_OFF);
  snprintf(work_char, sizeof(work_char), "%i", brightness);
  // lv_label_set_text(ui_Brightness, work_char);

  timeUpdated   = gps.time.isUpdated();    // These are "reset on read".
  locUpdated    = gps.location.isValid();  // Get them here then use the variable, later.
  courseUpdated = gps.course.isUpdated();

  while (!timeUpdated || !locUpdated || !courseUpdated) {
    //  || !fixQuality.value()) {
    endMillis = millis() + 500;
    // The next one does not actually need the { but, just for nice alignment, there it is.
    while (endMillis > millis() && N != NQ) {
      while (myI2CGPS.available()) {
        N = myI2CGPS.read();
        if (N == NQ) {
          Serial.println();
          showBattLevel();
        }
        Serial.write(N);
        gps.encode(N);
      }
    }
    // The other end of the redundant { } pair for pretty alignment.
    //    Serial.printf("\r\n2 Time Updated? %i, Location Valid? %i, Course Updated? %i.",
    //                  timeUpdated, locUpdated, courseUpdated);
    showSatsInUseView();
    if (timeUpdated) {
      computeLocalTime();
      showTimeAndDate();
    }
    if (locUpdated) {
      showLatOnScreen();
      showLngOnScreen();
    }
    lv_task_handler();
    timeUpdated   = gps.time.isUpdated();    // These are "reset on read".
    locUpdated    = gps.location.isValid();  // Get them here then use the variable, later.
    courseUpdated = gps.course.isUpdated();
  }
  //  Serial.printf("\r\n3 Time Updated? %i, Location Valid? %i, Course Updated? %i.",
  //                timeUpdated, locUpdated, courseUpdated);
  // This is initially set to LV_LABEL_LONG_SCROLL so it will scroll back and forth
  //  during setup to show we are acquiring sats.  That was a good idea so the user
  //  will not get impatient too quickly. When that is done, I don't want scrolling
  //  any longer so reset it for visual stablility.
  lv_label_set_long_mode(ui_HeadingData, LV_LABEL_LONG_CLIP);  // NO JUMPING!

  // Hide all of the sat PRN's.  Comment this to see them in all their glory.
  for (currScreenSat = 0; currScreenSat < SCREEN_SATS_INFO_ARRAY_SIZE_33; currScreenSat++)
    lv_obj_add_flag(screenSats[currScreenSat], LV_OBJ_FLAG_HIDDEN);

  // Prime the pump. Location is now valid so prime the pump.
  prevLat = gps.location.lat();
  prevLng = gps.location.lng();

  computeLocalTime();

  Serial.println("\r\nSetup Finished.\r\n");
}
/*********************************************************************************************/
void loop()
/*********************************************************************************************/
{
  lv_task_handler();  // Update the screen and handle touch events.

  if (Serial.available()) {
    work_int = toUpperCase(Serial.read());
    if (work_int == 'M' || work_int == '1')
      lv_disp_load_scr(ui_screenMain);  // Does all of the stuff to show a different screen.
    if (work_int == 'S' || work_int == '2')
      lv_disp_load_scr(ui_screenSats);  // Does all of the stuff to show a different screen.
    if (work_int == 'I' || work_int == '3')
      lv_disp_load_scr(ui_screenInfo);  // Does all of the stuff to show a different screen.
  }
  // Update GPS data periodically
  active_screen = lv_scr_act();
  if (active_screen == ui_screenMain) {
    updateMainScreen();
    ltaLatitude = 0; ltaLongitude = 0; ltaCount = 0;
  }

  if (active_screen == ui_screenInfo) {
    updateInfoScreen();
  }

  if (active_screen == ui_screenSats) {
    updateSatsScreen();
    ltaLatitude = 0; ltaLongitude = 0; ltaCount = 0;
  }
}
/*********************************************************************************************/
void updateMainScreen()  //Display main screen (GPS info)
/*********************************************************************************************/
{
  //available() returns the number of new bytes available from the GPS module
  while (myI2CGPS.available()) gps.encode(myI2CGPS.read());  //Feed the GPS parser
  if (!gps.time.isUpdated()) return;
  if (prevSec == gps.time.second()) return;
  prevSec = gps.time.second();
  if (gps.time.second() == 0 && gps.time.minute() % 10 == 0) savePrefs();

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
  if (GPS_Movement < 1.) GPS_Movement = 0;  // Ignore movement less than .5 meters/second
  if (prevLat > 500 || prevLng > 500) GPS_Movement = 0;
  prevLat = latitude;  prevLng = longitude;
  totMovement += GPS_Movement;  // Kept in total in meters moved.
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

  /**********************/
  /* Heading            */  // Replaced with code, later
  /**********************/
  //  heading = gps.course.deg();  // Note that this is a double

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

  /**********************/
  /* Heading            */
  /**********************/
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
    if (N == NQ) Serial.println();
    Serial.write(N); gps.encode(N);
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
  }
  if (!gps.time.isUpdated()) return;
  if (prevSec == gps.time.second()) return;  // Probably redundant.
  prevSec = gps.time.second();

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

  getSunMoonInfo();

  lv_task_handler();  // And show on screen.
}
/*********************************************************************************************/
void updateSatsScreen()
/*********************************************************************************************/
{
  while (myI2CGPS.available()) {
    N = myI2CGPS.read();
    if (N == NQ) Serial.println();
    Serial.write(N); gps.encode(N);

    if (GPGSV_MessageNumber.isUpdated()) {
      //      Serial.printf("\r\n%i GPGSV_MessageNumber.value() %i\r\n",
      //                    millis(), atoi(GPGSV_MessageNumber.value()));
      fetchGSx_Data(GPGSV_UPDATED);
    }
    if (GLGSV_MessageNumber.isUpdated()) {
      //      Serial.printf("\r\n%i GLGSV_MessageNumber.value() %i\r\n",
      //                    millis(), atoi(GLGSV_MessageNumber.value()));
      fetchGSx_Data(GLGSV_UPDATED);
    }
    if (GNRMC_Time.isUpdated()) {
      if (waitforGSV1) return;
      Serial.println("Now fill the screen with sats, please. PLEASE!!!");
      printsatsArray100();
      showSatsPositions();
      waitforGSV1 = true;
    }
  }
  if (!gps.time.isUpdated()) return;
  if (prevSec == gps.time.second()) return;
  prevSec = gps.time.second();
  //  Serial.printf("\r\nNew Second %i, Fix Quality: %i\r\n", prevSec, atoi(fixQuality.value()));
}
/*********************************************************************************************/
void fetchGSx_Data(int iWhich)
/*********************************************************************************************/
{
  Serial.println();
  if (currentMessageP == 1) {
    // Show what we have collected lately then clear it out for a new collection cycle.
    //    Serial.println("Now fill the screen with sats, please. PLEASE!!!");
    //    printsatsArray100();
    //    showSatsPositions();
    // The updating of the sats screen is now triggered by reception of a GNRMC statement.
    waitforGSV1 = false;  // I'm here!  No more waiting!!!

    // Now, we start all over and collect position information.
    totSatsInView = 0;
    for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i) satsArray100[i].active = false;
  }
  if (iWhich == GPGSV_UPDATED) {
    /* Testing */
    currentMessageP = atoi(GPGSV_MessageNumber.value());
    Serial.printf("Now in fetchGSx_Data, currentMessageP is %i\r\n", currentMessageP);
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
    Serial.printf("Now in fetchGSx_Data, currentMessageL is %i\r\n", currentMessageL);
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

  // Hide all sat numbers.
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

    lv_obj_clear_flag(screenSats[currScreenSat], LV_OBJ_FLAG_HIDDEN);   // Show it.

    dAngle = satsArray100[currArray100Sat].azimuth; // In degrees.
    dAngle *= 0.0174533; // Radians now

    // Range the distance from Elevation to screen pixels
    satEl = satRanger - (satsArray100[currArray100Sat].elevation / 90. * satRanger);

    satX = int(sin(dAngle) * satEl);
    satY = 0 - int(cos(dAngle) * satEl);

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
  for (work_int = currArray100Sat; work_int < SATS_INFO_ARRAY_SIZE_100; work_int++)
    if (satsArray100[work_int].active) return work_int;
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

  //  Serial.print("Array Slot ");
  for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i)
    if (satsArray100[i].active) Serial.printf("%3i|", i);

  //  Serial.print("\r\nPRNs       ");
  for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i)
    if (satsArray100[i].active) Serial.printf("%3i|", i + 1);

  //  Serial.print("\r\nAzimuths   ");
  for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i)
    if (satsArray100[i].active)
      Serial.printf("%3i|", satsArray100[i].azimuth);

  //  Serial.print("\r\nElevations ");
  for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i)
    if (satsArray100[i].active)
      Serial.printf("%3i|", satsArray100[i].elevation);

  //  Serial.print("\r\nSNRs       ");
  for (int i = 0; i < SATS_INFO_ARRAY_SIZE_100; ++i)
    if (satsArray100[i].active)
      Serial.printf("%3i|", satsArray100[i].snr);

  //  Serial.println();
}
/*********************************************************************************************/
void showLatOnScreen()
/*********************************************************************************************/
{
  latitude = gps.location.lat();

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
  computeLocalTime();
  lv_label_set_text(ui_TimeData, localTime);
  lv_label_set_text(ui_DateData, localDate);
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
}
/*********************************************************************************************/
void showBattLevel()
/*********************************************************************************************/
{
  if (amoled.isCharging())
    lv_obj_clear_flag(ui_Flash, LV_OBJ_FLAG_HIDDEN);
  else
    lv_obj_add_flag(ui_Flash, LV_OBJ_FLAG_HIDDEN);

  // Do not work.
  //  Serial.printf("USB plugged, %.2fV @ %.0mA", amoled.getVbusVoltage() / 1000.,
  //                amoled.getVbusCurrent());
  //  Serial.print("Vbus Removed: "); Serial.println(amoled.isVbusRemoveIrq());

  battVoltage = amoled.getBattVoltage();
  if (battVoltage > 3300.) {  // 3.3 volts
    lv_label_set_text_fmt(ui_LabelBatteryLevel, "%.2f", battVoltage / 1000.);
  } else {
    lv_obj_add_flag(ui_Flash, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(ui_LabelBatteryLevel, "Lo Batt");
  }
}
/*********************************************************************************************/
void computeLocalTime()
/*********************************************************************************************/
{
  char cWork[100];

  myYear   = gps.date.year();
  myMonth  = gps.date.month();
  myDOM    = gps.date.day();
  myHour   = gps.time.hour();
  myMinute = gps.time.minute();
  mySecond = gps.time.second();

  if (gps.time.minute() == 0 && gps.time.second() == 0) {
    Serial.printf("\r\nSetting the internal clock to "
                  "%02i/%02i/%02i %02i:%02i:%02i UTC (from the GPS)\r\n",
                  myMonth, myDOM, myYear, myHour, myMinute, mySecond);
  }

  rtc.setTime(mySecond, myMinute, myHour, myDOM, myMonth, myYear);

  timeinfo = rtc.getTimeStruct();

  strftime(localTime, sizeof(localTime), "%X", &timeinfo);

  if (dateFormat == dtFormatMDY)
    strftime(localDate, sizeof(localDate), "%m/%d/%y", &timeinfo);
  else if (dateFormat == dtFormatDMY)
    strftime(localDate, sizeof(localDate), "%d/%m/%y", &timeinfo);

  strftime(cCharWork, sizeof(cCharWork), "%z", localtime(&workTime));
  iTempOffset = atoi(cCharWork);
  // The format of the offset is a little strange.  Here's the decode of it.
  iOffset = (iTempOffset / 100) * 3600 + iTempOffset % 100 * 60;
  localTimeOffset = iOffset;  // It was just easier and safer this way!
  rtc.offset = iOffset;  // change offset value

  strftime(TZid, sizeof(TZid), "%Z", &timeinfo);
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
  return myResult;
}
/*********************************************************************************************/
static void slider_event_cb(lv_event_t * e)
/*********************************************************************************************/
{
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  brightness = (uint8_t)lv_slider_get_value(slider);
  amoled.setBrightness(brightness);  // Set the brightness level
}
///*********************************************************************************************/
//void Change2SatView(lv_event_t * e)
///*********************************************************************************************/
//{
//  // This is triggered by ui_event_Button1 which overlays the right arrow (ui_LabelRight)
//  Serial.println("Change2SatView-Changing to ScreenSatView");
//  showingScreen = ScrSats;
//}
///*********************************************************************************************/
//void Back2SatView(lv_event_t * e)  // This should go away after Joe fixes the SL.
///*********************************************************************************************/
//{
//  Serial.println("Back2SatView-Changing to ScreenSatView");
//  showingScreen = ScrSats;
//}
///*********************************************************************************************/
//void Change2Main(lv_event_t * e)
///*********************************************************************************************/
//{
//  // This is triggered by ui_event_Button2 which overlays the left arrow (ui_LabelLeft)
//  Serial.println("Change2Main-Changing to ScreenMain");
//  showingScreen = ScrMain;
//}
///*********************************************************************************************/
//void Change2Statements(lv_event_t * e)
///*********************************************************************************************/
//{
//  Serial.println("Change2Statements-Changing to ScreenStats");
//  showingScreen = ScrStats;
//}
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
}
/*********************************************************************************************/
void BtnTime(lv_event_t * e)
/*********************************************************************************************/
{
  // Change the time zone to the last defined one to reset it after reaching max+1.
  selectedTZ++; if (selectedTZ > tzCA) selectedTZ = 0;
  setTimeZone();
  computeLocalTime();
}
/*********************************************************************************************/
void setTimeZone()
/*********************************************************************************************/
{
  if (selectedTZ == tzPH) {  // Philippines
    Serial.println("You selected Philipinne Time");
    //    selectedTZ = PHtime;
    setenv("TZ", PHtime_zone, 1); tzset();
    strftime(cCharWork, sizeof(cCharWork), "%z", localtime(&workTime));
    iTempOffset = atoi(cCharWork);
    // The format of the offset is a little strange.  Here's the decode of it.
    iOffset = (iTempOffset / 100) * 3600 + iTempOffset % 100 * 60;
    localTimeOffset = iOffset;  // It was just easier and safer this way!
    Serial.printf("Offset = %+i, ", iOffset);
    rtc.offset = iOffset;  // change offset value
  } else if (selectedTZ == tzCA) {  // California
    Serial.println("You selected California Time");
    //    selectedTZ = CAtime;
    setenv("TZ", CAtime_zone, 1); tzset();
    strftime(cCharWork, sizeof(cCharWork), "%z", localtime(&workTime));
    iTempOffset = atoi(cCharWork);
    // The format of the offset is a little strange.  Here's the decode of it.
    iOffset = (iTempOffset / 100) * 3600 + iTempOffset % 100 * 60;
    localTimeOffset = iOffset;  // It was just easier and safer this way!
    Serial.printf("Offset = %+i\r\n", iOffset);
    rtc.offset = iOffset;  // change offset value
  }
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
  spdUnit++; if (altUnit > METRIC) spdUnit = 0;
}
/*********************************************************************************************/
void getSunMoonInfo()
/*********************************************************************************************/
{
  Serial.printf("UTC Epoch %li, Local Epoch %li\r\n", rtc.getEpoch(), rtc.getLocalEpoch());

  time (&workTime);
  localtime_r(&workTime, &timeinfo);
  // +1 means DST in effect. 0 means no DST. -1 means invalid, I change to 0.
  isItDST = timeinfo.tm_isdst;

  theMoon.calculate(latitude, longitude, rtc.getLocalEpoch());

  //-----------------------------------

  // Returned values:
  moonVisible   = theMoon.isVisible;
  moonRiseAz    = theMoon.riseAz;      // Where the moon will rise/set in degrees from
  moonSetAz     = theMoon.setAz;       //  North.
  moonRiseTime  = theMoon.riseTime + localTimeOffset + (isItDST * 3600);
  moonSetTime   = theMoon.setTime + localTimeOffset + (isItDST * 3600);
  moonQueryTime = theMoon.queryTime + localTimeOffset + (isItDST * 3600);
  moonHasRise   = theMoon.hasRise;
  moonHasSet    = theMoon.hasSet;
  Serial.print("\nAs of ");  // Serial.print(moonQueryTime);
  calcFullDate(moonQueryTime);
  Serial.print(cFD_FullDate);
  if (moonVisible)
    Serial.print(", the moon is visible.");
  else
    Serial.print(", the moon is not visible.");

  Serial.print("\nNearest Moon Set:   \t"); Serial.print(moonSetTime);
  calcFullDate(moonSetTime);
  printRelativeDay(newJulian - todayJulian);
  Serial.print(justTime);
  Serial.print(" at Az ");
  if (moonSetAz < 100.) Serial.print(" ");
  Serial.print(lroundf(moonSetAz)); Serial.print("°");

  snprintf(work_char, sizeof(work_char), "%02i:%02i", cFD_Hour, cFD_Min);
  lv_label_set_text(ui_MoonSetData, work_char);

  Serial.print("\nNearest Moon Rise:   \t");  // Serial.print(moonRiseTime);
  calcFullDate(moonRiseTime);
  printRelativeDay(newJulian - todayJulian);
  Serial.print(justTime);
  Serial.print(" at Az ");
  if (moonRiseAz < 100.) Serial.print(" ");
  Serial.print(lroundf(moonRiseAz)); Serial.print("°");

  snprintf(work_char, sizeof(work_char), "%02i:%02i", cFD_Hour, cFD_Min);
  lv_label_set_text(ui_MoonRiseData, work_char);

  // Next, the current phase of the moon.
  moonData_t moon; // variable to receive the data
  moon = moonPhase.getPhase(rtc.getEpoch());

  Serial.print("\nMoon phase angle:\t");
  Serial.print(moon.angle);  // angle is a integer between 0-359

  // moonShowing is its own previous value.  Hide prev one showing then update it and show new.
  lv_obj_add_flag(moonPhases[moonShowing], LV_OBJ_FLAG_HIDDEN);  // Hide previous phase
  switch ((int)(moon.angle + .5))
  {
    // (0 and 29 being hidden and 14 is full)
    case   0 ...  29: moonShowing =   0; break;
    case  30 ...  59: moonShowing =   1; break;
    case  60 ...  89: moonShowing =   2; break;
    case  90 ... 119: moonShowing =   3; break;
    case 120 ... 149: moonShowing =   4; break;
    case 150 ... 179: moonShowing =   5; break;
    case 180 ... 209: moonShowing =   6; break;
    case 210 ... 239: moonShowing =   7; break;
    case 240 ... 269: moonShowing =   8; break;
    case 270 ... 299: moonShowing =   9; break;
    case 300 ... 329: moonShowing =  10; break;
    case 330 ... 359: moonShowing =  11; break;
  }
  lv_obj_clear_flag(moonPhases[moonShowing], LV_OBJ_FLAG_HIDDEN);   // Show it.

  Serial.print("°");
  Serial.print(" / Moon surface lit: ");
  Serial.printf("%.2f%%\r\n", moon.percentLit * 100.); // percentLit is a real between 0-1

  //-----------------------------------

  // Find the last and next sun set and rise.
  theSun.calculate(latitude, longitude, rtc.getLocalEpoch());
  sunVisible = theSun.isVisible;
  sunHasRise = theSun.hasRise;
  sunHasSet = theSun.hasSet;
  sunRiseAz = theSun.riseAz;  // Where the sun will rise/set in degrees from
  sunSetAz = theSun.setAz;    // North.

  // Additional returned values requiring conversion from UTC to local time zone
  // on the Arduino.
  sunQueryTime = theSun.queryTime + localTimeOffset + (isItDST * 3600);
  sunRiseTime = theSun.riseTime + localTimeOffset + (isItDST * 3600);
  sunSetTime = theSun.setTime + localTimeOffset + (isItDST * 3600);
  calcFullDate(sunSetTime);
  sunSetH = cFD_Hour; sunSetM = cFD_Min;
  //If you want to override this for testing,
  //  just uncomment and fixup these two statements.
  //  sunSetH = 12; sunSetM = 32;  // Set for testing.  TESTING OVERRIDE HERE!

  Serial.print("\nNearest Sun Rise: \t");
  calcFullDate(sunRiseTime);
  printRelativeDay(newJulian - todayJulian);
  Serial.print(justTime);

  snprintf(work_char, sizeof(work_char), "%02i:%02i", cFD_Hour, cFD_Min);
  lv_label_set_text(ui_SunRiseData, work_char);

  Serial.print(" at Az ");
  if (theSun.riseAz < 100.) Serial.print(" ");
  Serial.print(lroundf(theSun.riseAz)); Serial.print("°");
  Serial.print("\nNearest Sun Set:  \t");
  calcFullDate(sunSetTime);
  printRelativeDay(newJulian - todayJulian);
  Serial.print(justTime);

  snprintf(work_char, sizeof(work_char), "%02i:%02i", cFD_Hour, cFD_Min);
  lv_label_set_text(ui_SunSetData, work_char);

  Serial.print(" at Az ");
  if (theSun.setAz < 100.) Serial.print(" ");
  Serial.print(lroundf(theSun.setAz)); Serial.print("°");
  if (theSun.isVisible && theMoon.isVisible)  // If sun and moon are overhead, maybe trouble.
    Serial.print("\nThe moon is in the sky but may be washed out by the sun.");

  //-----------------------------------

  float fcurrMoonPhase = (float)(rtc.getEpoch() - moonepoch);
  fcurrMoonPhase = fmod(fcurrMoonPhase, 2551443.);

  float fcurrMoonAge = (fcurrMoonPhase / (24. * 3600.));  // + 1.;

  Serial.printf("\nMoon Age\t\t(Actual) %.2f, (Rounded) %i - ",
                fcurrMoonAge, int(fcurrMoonAge + .5));

  switch ((int)(fcurrMoonAge + .5))
  {
    // (0 and 29 being hidden and 14 is full)
    case  0:        Serial.printf("New Moon\r\n");        break;
    case  1 ... 6:  Serial.printf("Waxing Crescent\r\n"); break;
    case  7 ... 8:  Serial.printf("First Quarter\r\n");   break;
    case  9 ... 13: Serial.printf("Waxing Gibbus\r\n");   break;
    case 14 ... 16: Serial.printf("Full Moon\r\n");       break;
    case 17 ... 21: Serial.printf("Waning Gibbus\r\n");   break;
    case 22 ... 22: Serial.printf("Last Quarter\r\n");    break;
    case 23 ... 28: Serial.printf("Waning Crescent\r\n"); break;
    case 29 ... 30: Serial.printf("New Moon\r\n");        break;
  }
}
/*********************************************************************************************/
void savePrefs()
/*********************************************************************************************/
{
  Serial.println("Saving preferences");
  preferences.begin("lvglGPS", false);  // Open it, this time, in read/write mode.

  work_int = preferences.getInt("brightness", -1);  // -1 Says the key is not there yet.
  Serial.printf("Read Brightness value of %i\r\n", work_int);
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
  Serial.printf("Current saved TZ is %i, displayed is %i\r\n",
                work_int, selectedTZ);
  if (work_int != selectedTZ) {
    preferences.putInt("TZ", selectedTZ);
    Serial.printf("Saving changed timeZone value of %i\r\n", selectedTZ);
  }
  work_int = preferences.getInt("TZ",  PHtime);
  Serial.printf("Verifying saved selected TZ %i\r\n", work_int);

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

  Serial.println("Reading saved preferences.");
  preferences.begin("lvglGPS", true);  // Open it, this time, in read-only mode.

  brightness = preferences.getInt("brightness", 125);
  if (brightness < 50) brightness = 50;  // Not sure if I like this but trying it for now.
  Serial.printf("Brightness value read: %i\r\n", brightness);

  dateFormat = preferences.getInt("dateFormat", dtFormatMDY);
  Serial.printf("Date format read: %i\r\n",     dateFormat);

  selectedTZ = preferences.getInt("TZ",         PHtime);
  Serial.printf("Timezone preference: %i\r\n",  selectedTZ);
  setTimeZone();

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
void printRelativeDay(int jdDiff)
/*********************************************************************************************/
{
  switch (jdDiff)
  {
    case  0:
      Serial.print("Today"); break;
    case 1:
      Serial.print("Tomorrow"); break;
    case -1:
      Serial.print("Yesterday"); break;
    default:
      Serial.print(cFD_FullDate);
      break;
  }
  Serial.print(" at ");
}
/*********************************************************************************************/
static int isLeapYear(unsigned int year)
/*********************************************************************************************/
{
  // Must be divisible by 4 but not by 100, unless it is divisible by 400
  return ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0);
}
/*********************************************************************************************/
void calcFullDate(time_t timeToUse)
/*********************************************************************************************/
// This routine is REALLY OLD but it works for what I need
{
  // Just do it myself.  Avoid library errors!  So easy, anyway!!
  uint32_t lTime = (uint32_t) timeToUse;
  cFD_Sec = lTime % 60; /* It was seconds already. No conversion necessary */
  lTime /= 60; /* now it is minutes */ cFD_Min = lTime % 60;
  lTime /= 60; /* now it is hours */   cFD_Hour = lTime % 24;
  lTime /= 24; /* now it is days */    cFD_Day = (((lTime + 4) % 7) + 1);
  cFD_Year = 0;
  cFD_Days = 0;
  while ((unsigned) (cFD_Days += (isLeapYear(1970 + cFD_Year) ? 366 : 365)) <= lTime)
    cFD_Year++;
  cFD_Year += 1970;
  //  Serial.printf("\nMyYear: %i", cFD_Year);

  cFD_Days -= isLeapYear(1970 + cFD_Year) ? 366 : 365;
  lTime -= cFD_Days; // now it is days in this year, starting at 0

  //  cFD_Days = 0;
  cFD_Month = 0;
  int monthLength = 0;
  for (cFD_Month = 0; cFD_Month < 12; cFD_Month++) {
    // Check
    if (cFD_Month == 1) {
      if (isLeapYear(1970 + cFD_Year)) {
        monthLength = 29;
      } else {
        monthLength = 28;
      }
    } else {
      monthLength = month_length[cFD_Month];
    }
    if (lTime >= monthLength) {
      lTime -= monthLength;
    } else {
      break;
    }
  }

  // All of the numbers are obtained.  Now build the output string and julian.

  // At this point, cFD_Days is the DOW indicator & lTime is the day of the month - 1
  cFD_DOM = lTime + 1;  //int Day of the month here.
  //  Serial.printf("\nDOM %i", cFD_DOM);  // asdf
  cFD_DOW = cFD_Day - 1;  // Now Sunday = 0

  cFD_FullDate[0] = 0;   // Set to zero length string
  workDate[0] = 0;    // Set to zero length string
  justTime[0] = 0;     // Initialize the time part
  // For this version, cFD_Days Sun = 1
  sprintf(cFD_FullDate, "%s", daysOfTheWeek[cFD_DOW]);  // Sunday = 0
  currentHour = cFD_Hour;  // ptm->tm_hour;

  sprintf(workDate, "%s", months[cFD_Month]); strcat(cFD_FullDate, workDate);
  cFD_Month++;  // Fix the month to be "normal", based on 1.

  sprintf(workDate, "%i", cFD_DOM);
  strcat(cFD_FullDate, workDate); strcat(cFD_FullDate, ", ");

  sprintf(workDate, "%i", cFD_Year);
  strcat(cFD_FullDate, workDate); strcat(cFD_FullDate, " at ");

  // Now add in the time portion.

  if (showAMPM)
  {
    if (currentHour == 0) currentHour += 12;
    if (currentHour > 12) {
      currentHour -= 12;
      myIsPM = true;
    } else {
      myIsPM = false;
    }
    sprintf(workDate, "%2i", currentHour);
  } else {
    sprintf(workDate, "%02d", currentHour);
  }
  strcat(cFD_FullDate, workDate); strcat(cFD_FullDate, ":");
  strcat(justTime, workDate); strcat(justTime, ":");

  sprintf(workDate, "%02i", cFD_Min);
  strcat(cFD_FullDate, workDate); strcat(cFD_FullDate, ":");
  strcat(justTime, workDate); strcat(justTime, ":");

  sprintf(workDate, "%02i", cFD_Sec);
  strcat(cFD_FullDate, workDate);

  if (showAMPM) {
    if (myIsPM) {
      sprintf(workDate, "%s", " PM");
      strcat(cFD_FullDate, workDate);
      strcat(justTime, workDate);
    } else {
      sprintf(workDate, "%s", " AM");
      strcat(cFD_FullDate, workDate);
      strcat(justTime, workDate);
    }
  }
  newJulian = calcJulian(cFD_Year, cFD_Month, cFD_DOM);
}
unsigned int calcJulian(unsigned int myyear, unsigned int mymonth, unsigned int myday)
{
  if (!valiDATE(mymonth, myday, myyear)) return (0);

  myyear += 8000;
  if (mymonth < 3) {
    myyear--;
    mymonth += 12;
  }
  return (myyear * 365) + (myyear / 4) - (myyear / 100) + (myyear / 400) - 1200820
         + (mymonth * 153 + 3) / 5 - 92
         + myday - 1;
}
/*********************************************************************************************/
bool valiDATE(unsigned int month, unsigned int day, unsigned int year)
/*********************************************************************************************/
{
  unsigned int flag = 0;
  unsigned int days = 0;

  // Validate the month first
  if (month < 1 || month > 12) {
    Serial.printf ("\nValue for month %i is out of range.", month);
    return false;
  }

  // Validate the day of month
  days = days_in_month(month, year);

  if (day < 1 || day > days) {
    Serial.printf ("\nValue for day %i is out of range.", day);
    return false;
  }

  // Validate the year
  if (year < 1990 || year > 2030) {
    Serial.printf ("\nValue for year %i is out of range.", year);
    return false;
  }
  return true;  // If we got here, it is good!
}
/*********************************************************************************************/
unsigned int days_in_month(unsigned int month, unsigned int year)
/*********************************************************************************************/
{
  unsigned int days = 0;

  if (month == 2) {
    days = 28 + leap(year);  // function leap returns 1 for leap years
  }
  else if (month == 4 || month == 6 || month == 9 || month == 11) {
    days = 30;
  }
  else {
    days = 31;
  }
  return days;
}
/*********************************************************************************************/
unsigned int leap(unsigned int year)
/*********************************************************************************************/
{
  if (year % 400 == 0)
    return 1;
  else if (year % 100 != 0 && year % 4 == 0)
    return 1;
  else
    return 0;
}
