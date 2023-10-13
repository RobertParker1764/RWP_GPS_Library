/********************************************************************************
 RWP_GPS.h
 Version 1.0
 10/13/2023
 This library is derived from the  Adafruit GPS library - the ultimate GPS library
 for the ultimate GPS module! Changes have been made to reduce the code size /
 memory footprint / execution time. These changes have reduced the
 utility of the library such that it will only work with the Adafruit Ultimate GPS
 feather board in conjuction with  Adafruit M0 feather variants. Only serial1
 communications with the GPS module are supported. The LOCUS on-chip
 data logging is not supported.
 
 Original library written by Limor Fried/Ladyada  for Adafruit Industries.
 BSD license, check license.txt for more information
 *********************************************************************************/

#ifndef _RWP_GPS_H
#define _RWP_GPS_H

#define MAXLINELENGTH 120       // how long are max NMEA lines to parse?
#define NMEA_MAX_SENTENCE_ID 20 // maximum length of a sentence ID name, including terminating 0
#define NMEA_MAX_SOURCE_ID 3    // maximum length of a source ID name, including terminating 0

#include "Arduino.h"
#include <Adafruit_PMTK.h>
#include <NMEA_data.h>

// Type for resulting code from running check()
typedef enum {
    NMEA_BAD = 0,             //passed none of the checks
    NMEA_HAS_DOLLAR = 1,      //has a dollar sign or exclamation mark in the first position
    NMEA_HAS_CHECKSUM = 2,    //has a valid checksum at the end
    NMEA_HAS_NAME = 4,        //there is a token after the $ followed by a comma
    NMEA_HAS_SOURCE = 10,     //has a recognized source ID
    NMEA_HAS_SENTENCE = 20,   //has a recognized sentence ID
    NMEA_HAS_SENTENCE_P = 40  //has a recognized parseable sentence ID
} nmea_check_t;

/**************************************************************************/
/*!
 @brief  The GPS class
 */
class Adafruit_GPS : public Print {
public:
    // Adafruit_GPS.cpp
    bool begin(uint32_t baud);
    
    Adafruit_GPS(HardwareSerial *ser); // Constructor when using HardwareSerial
    void common_init(void);
    virtual ~Adafruit_GPS();
    
    size_t available(void);
    size_t write(uint8_t);
    char read(void);
    void sendCommand(const char *);
    bool newNMEAreceived();
    void pause(bool b);
    char *lastNMEA(void);
    bool waitForSentence(const char *wait, uint8_t max = MAXWAITSENTENCE,
                         bool usingInterrupts = false);
    
    bool standby(void);
    bool wakeup(void);
    nmea_float_t secondsSinceFix();
    nmea_float_t secondsSinceTime();
    nmea_float_t secondsSinceDate();
    void resetSentTime();
    
    // NMEA_parse.cpp
    bool parse(char *);
    bool check(char *nmea);
    bool onList(char *nmea, const char **list);
    uint8_t parseHex(char c);
    
    // NMEA_build.cpp
    
    void addChecksum(char *buff);
    
    // NMEA_data.cpp
    void newDataValue(nmea_index_t tag, nmea_float_t v);
    nmea_float_t boatAngle(nmea_float_t s, nmea_float_t c);
    nmea_float_t compassAngle(nmea_float_t s, nmea_float_t c);
    
    int thisCheck = 0;                                // The results of the check on the
    // current sentence
    char thisSource[NMEA_MAX_SOURCE_ID] = {0};        // The first two letters of the current
    // sentence, e.g. WI, GP
    char thisSentence[NMEA_MAX_SENTENCE_ID] = {0};    // The next three letters of the current
    // sentence, e.g. GLL, RMC
    char lastSource[NMEA_MAX_SOURCE_ID] = {0};        // The results of the check on the most
    // recent successfully parsed sentence
    char lastSentence[NMEA_MAX_SENTENCE_ID] = {0};    // The next three letters of the most recent
    // successfully parsed sentence, e.g. GLL, RMC
    
    uint8_t hour;          // GMT hours
    uint8_t minute;        // GMT minutes
    uint8_t seconds;       // GMT seconds
    uint16_t milliseconds; // GMT milliseconds
    uint8_t year;          // GMT year
    uint8_t month;         // GMT month
    uint8_t day;           // GMT day
    
    nmea_float_t latitude;  // Floating point latitude value in degrees/minutes
    // as received from the GPS (DDMM.MMMM)
    nmea_float_t longitude; // Floating point longitude value in degrees/minutes
    // as received from the GPS (DDDMM.MMMM)
    
    // Fixed point latitude and longitude value with degrees stored in units of
    // 1/10000000 of a degree. See pull #13 for more details:
    // https://github.com/adafruit/Adafruit-GPS-Library/pull/13
    int32_t latitude_fixed;     // Fixed point latitude in decimal degrees.
                                // Divide by 10000000.0 to get a double.
    int32_t longitude_fixed;    // Fixed point longitude in decimal degrees
                                // Divide by 10000000.0 to get a double.
    
    nmea_float_t latitudeDegrees;   // Latitude in decimal degrees
    nmea_float_t longitudeDegrees;  // Longitude in decimal degrees
    nmea_float_t geoidheight;       // Diff between geoid height and WGS84 height
    nmea_float_t altitude;          // Altitude in meters above MSL
    nmea_float_t speed;             // Current speed over ground in knots
    nmea_float_t angle;             // Course in degrees from true north
    nmea_float_t magvariation;      // Magnetic variation in degrees (vs. true north)
    nmea_float_t HDOP;              // Horizontal Dilution of Precision - relative accuracy
                                    // of horizontal position
    nmea_float_t VDOP;              // Vertical Dilution of Precision - relative accuracy
                                    // of vertical position
    nmea_float_t PDOP;              // Position Dilution of Precision - Complex maths derives
                                    // a simple, single number for each kind of DOP
    char lat = 'X';         // N/S
    char lon = 'X';         // E/W
    char mag = 'X';         // Magnetic variation direction
    bool fix;               // Have a fix?
    uint8_t fixquality;     // Fix quality (0, 1, 2 = Invalid, GPS, DGPS)
    uint8_t fixquality_3d;  // 3D fix quality (1, 3, 3 = Nofix, 2D fix, 3D fix)
    uint8_t satellites;     // Number of satellites in use
    uint8_t antenna;        // Antenna that is used (from PGTOP)
    
private:
    // NMEA_data.cpp
    void data_init();
    // NMEA_parse.cpp
    const char *tokenOnList(char *token, const char **list);
    bool parseCoord(char *p, nmea_float_t *angleDegrees = NULL,
                    nmea_float_t *angle = NULL, int32_t *angle_fixed = NULL,
                    char *dir = NULL);
    char *parseStr(char *buff, char *p, int n);
    bool parseTime(char *);
    bool parseFix(char *);
    bool parseAntenna(char *);
    bool isEmpty(char *pStart);
    
    // used by check() for validity tests, room for future expansion
    const char *sources[7] = {"II", "WI", "GP", "PG", "GN", "P",  "ZZZ"}; // valid source ids
    
    const char *sentences_parsed[5] = {"GGA", "RMC", "TOP", "CD", "ZZZ"}; // parseable sentence ids
    const char *sentences_known[6] = {"DBT", "HDM", "HDT", "GLL", "GSA", "ZZZ"}; // known, but not parseable
    
    // Make all of these times far in the past by setting them near the middle of
    // the millis() range. Timing assumes that sentences are parsed promptly.
    uint32_t lastUpdate = 2000000000L;  // millis() when last full sentence successfully parsed
    uint32_t lastFix = 2000000000L;     // millis() when last fix received
    uint32_t lastTime = 2000000000L;    // millis() when last time received
    uint32_t lastDate = 2000000000L;    // millis() when last date received
    uint32_t recvdTime = 2000000000L;   // millis() when last full sentence received
    uint32_t sentTime = 2000000000L;    // millis() when first character of last full sentence received
    bool paused;
    
    uint8_t parseResponse(char *response);
    bool noComms = false;
    HardwareSerial *gpsHwSerial;
    int8_t gpsSPI_cs = -1;
    
    int8_t _buff_max = -1;
    int8_t _buff_idx = 0;
    char last_char = 0;
    
    volatile char line1[MAXLINELENGTH]; // We double buffer: read one line in and leave one for the main program
    volatile char line2[MAXLINELENGTH]; // Second buffer
    volatile uint8_t lineidx = 0;       // our index into filling the current line
    volatile char *currentline;         // Pointer to current line buffer
    volatile char *lastline;            // Pointer to previous line buffer
    volatile bool recvdflag;            // Received flag
    volatile bool inStandbyMode;        // In standby flag
};


#endif
