// Project Horus, www.projecthorus.org
// Terry Baume, 2009
// terry@bogaurd.net

// RTTY code

// This code is based on Robert Harrison's (rharrison@hgf.com) RTTY code as used in the Icarus project
// http://pegasushabproject.org.uk/wiki/doku.php/ideas:notes?s=rtty

// We need a 'dummy' NewSoftSerial to unregister the interrupts
// registered for our software serial (GPS) port.
// Otherwise, incoming bits will upset our timing.
//NewSoftSerial dummySerial = NewSoftSerial(255, 255);

// Transmit a string, log it to SD & produce debug output
void txString(char *string) {
	noInterrupts(); // We need accurate timing!
	digitalWrite(RTTY_LED, HIGH);
	char txSum[6];
	
	// XOR sum
	//int checkSum = XORSum(string);
	//sprintf(txSum, "%02X", checkSum);
	
	// CRC16 checksum
	unsigned int checkSum = CRC16Sum(string);
	sprintf(txSum, "%04X", checkSum);
	
	// Transmit the telemetry
	Serial << F("RTTY: ") << string << "*" << txSum << endl;
	Logger << string << F("*") << txSum << endl;
	rtty_txstring(string);
	rtty_txstring("*");
	rtty_txstring(txSum);
	rtty_txstring("\r\n");
	txCount++;
	digitalWrite(RTTY_LED, LOW);
	interrupts();
}

// Transmit a string, one char at a time
void rtty_txstring (char *string) {
	//dummySerial.read();
	for (int i = 0; i < strlen(string); i++) {
		rtty_txbyte(string[i]);
	}
}

// Transmit a byte, bit by bit, LSB first
// ASCII_BIT can be either 7bit or 8bit
void rtty_txbyte (char c) {
	int i;
	// Start bit
	rtty_txbit (0);
	// Send bits for for char LSB first	
	for (i=0;i<RTTY_ASCII;i++) {
		if (c & 1) rtty_txbit(1); 
		else rtty_txbit(0);	
		c = c >> 1;
	}
	// Stop bit
	rtty_txbit (1);
}

// Transmit a bit as a mark or space
void rtty_txbit (int bit) {
	if (bit) {
		// High - mark
		digitalWrite(RTTY_PIN_1, HIGH);
		digitalWrite(RTTY_PIN_2, LOW);	
		digitalWrite(RTTY_LED, HIGH);
		
	} else {
		// Low - space
		digitalWrite(RTTY_PIN_2, HIGH);
		digitalWrite(RTTY_PIN_1, LOW);
		digitalWrite(RTTY_LED, LOW);
	}
	
	switch (bitRate) {
	
		case 200:
			delayMicroseconds(5050);
			break;
			
		case 300:
			delayMicroseconds(3400);
			break;
			
		case 150:
			delayMicroseconds(6830);
			break;
		
		case 100:
			delayMicroseconds(10300);
			break;
		
		default:
			delayMicroseconds(10000);
			delayMicroseconds(10600);
	}
}

void callback() {
	digitalWrite(9, digitalRead(9) ^ 1);
}

unsigned int CRC16Sum(char *string) {
	unsigned int i;
	unsigned int crc;
	
	crc = 0xFFFF;
	
	// Calculate the sum, ignore $ sign's
	for (i = 0; i < strlen(string); i++) {
		if (string[i] != '$') crc = _crc_xmodem_update(crc,(uint8_t)string[i]);
	}
	
	return crc;


}

// Project Horus, www.projecthorus.org
// Terry Baume, 2009
// terry@bogaurd.net

// Misc functions

// Returns a string with a textual representation of a float
void doubleToString(double val, int precision, char *string){

	// Print the int part
	sprintf(string, "%d", (int)(val));
	if(precision > 0) {
		// Print the decimal point
		strcat(string, ".");
		unsigned long frac;
		unsigned long mult = 1;
		int padding = precision -1;
		while (precision--) { mult *=10; }
		if (val >= 0)
			frac = (val - (int)(val)) * mult;
		else
			frac = ((int)(val)- val ) * mult;
		unsigned long frac1 = frac;
		while (frac1 /= 10) { padding--; }
		while (padding--) { strcat(string, "0"); }
		
		// Convert and print the fraction part
		sprintf(string+strlen(string), "%d", (int)(frac));
	}
}

// Converts a HEX string to an int
int atoh(char c) {
	if (c >= 'A' && c <= 'F') 
		return c - 55;
	else if (c >= 'a' && c <= 'f')
		return c - 87;
	else
		return c - 48;
}

// How much RAM do we have free?
extern int __bss_end;
extern void *__brkval;
int freeMem() {
 int free_memory;
 if((int)__brkval == 0)
   free_memory = ((int)&free_memory) - ((int)&__bss_end);
 else
   free_memory = ((int)&free_memory) - ((int)__brkval);
 return free_memory;
}

// Determine if a point is in a polygon
// nvert = number of verticies
// verticies = { x,y, x,y, x,y, x,y }, etc
// http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
int pnpoly(int nvert, float *verticies, float testx, float testy) {
	int i, j, c = 0;
	for (i = 0, j = nvert-1; i < nvert; j = i++) {
		if ( ((verticies[2*i+1]>testy) != (verticies[2*j+1]>testy)) &&
		(testx < (verticies[2*j]-verticies[2*i]) * (testy-verticies[2*i+1]) / (verticies[2*j+1]-verticies[2*i+1]) + verticies[2*i]) )
		c = !c;
	}
	return c;
}// Project Horus, www.projecthorus.org
// Terry Baume, 2009
// terry@bogaurd.net

// GPS Functions

// Reads a line from the GPS NMEA serial output
// Give up after 20 retries with no data read
int readLine(void) {
	char c;
	byte bufferIndex = 0;
	boolean startLine = 0;
	byte retries = 0;
	while (retries < 20) {
		if (GPS.available() > 0) {
			c = GPS.read();
			if (c == -1) { delay(2); continue; }
			if (c == '\n') continue;
			if (c == '$') startLine = 1;
			if ((bufferIndex == BUFFERSIZE-1) || (c == '\r')) {
				if (startLine) {
					buffer[bufferIndex] = 0;
					return 1;
				}
			}													 
			if (startLine) buffer[bufferIndex++] = c;
		} else {
			retries++;
			delay(50);
		}
	}
	return 0;
}

// Returns a specific field from the buffer
void getField(int getId, char *field, int maxLen) {
	byte bufferIndex = 0;
	byte fieldId = 0;
	byte i = 0;
	while (bufferIndex < sizeof(buffer)) {	
		if (fieldId == getId) {	
			// End of string, or string overflow
			if (buffer[bufferIndex] == ',' || i > (maxLen - 2)) {
				field[i] = 0;	// Null terminate
				return;
			}
			// Buffer chars to field
			field[i++] = buffer[bufferIndex++];
		} else {
			// Advance field on comma
			if (buffer[bufferIndex] == ',') {
				bufferIndex++;						// Advance in buffer
				fieldId++;							// Increase field position counter
			} else {
				bufferIndex++;						// Advance in buffer
			}
		}
	}
	// Null terminate incase we didn't already..
	field[i] = 0;
}

// Polls for an NMEA sentence of type requested
// Validates checksum, silently retries on failed checksums
int getNMEA(char *getType) {
	char type[7];
	byte retries = 0;
	while (retries < 2) {
		if (readLine() && validateChecksum()) {;
			getField(0, type, sizeof(type));
			if (strcmp(type, getType) == 0) {
				Serial << F("NMEA: ") << buffer << endl;
				return 1;
			}
		} else {
			retries++;
		}
	}
	Serial << F("NMEA: Failed to read from GPS!") << endl;
	return 0;
}

// Validates the checksum on an NMEA string
// Returns 1 on valid checksum, 0 otherwise
int validateChecksum(void) {
	char gotSum[2];
	gotSum[0] = buffer[strlen(buffer) - 2];
	gotSum[1] = buffer[strlen(buffer) - 1];
	// Check that the checksums match up
	if ((16 * atoh(gotSum[0])) + atoh(gotSum[1]) == XORSum(buffer)) return 1;
	else return 0;
}

// Calculates the checksum for a given string
// returns as integer
int XORSum(char *string) {
	int i; int XOR;	int c;
	// Calculate checksum ignoring any $'s in the string
	for (XOR = 0, i = 0; i < strlen(string); i++) {
		c = (unsigned char)string[i];
		if (c == '*') break;
		if (c != '$') XOR ^= c;
	}
	return XOR;
}

// Returns the groundspeed in km/h
int getSpeed(void) {
	char field[10];
	getField(7, field, sizeof(field));
	int speed = atoi(field);
	return speed;
}

// Return the fix type from a GGA string
int getFixType(void) {
	char field[5];
	getField(6, field, sizeof(field));
	int fixType = atoi(field);
	return fixType;
}

// Return the altitude in meters from a GGA string
long getAlt(void) {
	char field[10];
	getField(9, field, sizeof(field));
	long altitude = atol(field);
	return altitude;
}

// Returns the number of satellites being tracked from a GGA string
int getSats(void) {
	char field[3];
	getField(7, field, sizeof(field));
	int numSats = atoi(field);
	return numSats;
}

// Read the latitude in decimal format from a GGA string
double getLat(void) {
	char field[12];
	getField(2, field, sizeof(field));			// read the latitude
	double latitude = atof(field);					// convert to a double (precise)
	int deg = (int) latitude / 100;				// extract the number of degrees
	double min = latitude - (100 * deg);			// work out the number of minutes
	latitude = deg + (double) min/60.0;			// convert to decimal format
	getField(3, field, sizeof(field));			// get the hemisphere (N/S)
	if (strcmp(field, "S") == 0) latitude *= -1;	// sign the decimal latitude correctly
	return latitude;
}

// Read the longitude in decimal format from a GGA string
double getLong(void) {
	char field[12];
	getField(4, field, sizeof(field));			// read the longitude
	double longitude = atof(field);					// convert to a double
	int deg = (int) longitude / 100;				// extract the number of degrees
	double min = longitude - (100 * deg);			// work out the number of minutes
	longitude = deg + (double) min/60.00;			// convert to decimal format
	getField(5, field, sizeof(field));			// get the E/W status
	if (strcmp(field, "W") == 0) longitude *= -1; // sign decimal latitude correctly
	return longitude;
}

// Converts UTC time to the correct timezone
void convertTime(int *time) {
	// How many hours off GMT are we?
	float offset = 10.5;
	long sectime = ((long)(time[0]) * 3600) + (time[1] * 60) + time[2];
	sectime += (offset * 3600.0);
	// Did we wrap around?
	if (sectime < 0) sectime += 86400;
	if (sectime > 86400) sectime -= 86400;
	// Convert back to time
	time[0] = (int)(sectime / 3600);
	time[1] = (int)((sectime % 3600) / 60);
	time[2] = (int)((sectime % 3600) % 60);
}

// Parses a time field from a GGA string
void parseTime(char *field, int *time) {	
	char tmp[3]; tmp[2] = 0; // Init tmp and null terminate
	tmp[0] = field[0]; tmp[1] = field[1]; time[0] = atoi(tmp); // Hours	
	tmp[0] = field[2]; tmp[1] = field[3]; time[1] = atoi(tmp); // Minutes
	tmp[0] = field[4]; tmp[1] = field[5]; time[2] = atoi(tmp); // Seconds
}

// Gets the hours, minutes and seconds from a GGA string
void getTime(int *time) {
	char field[12];
	getField(1, field, sizeof(field));
	parseTime(field, time);
	convertTime(time);
}
// Project Horus flight code
// Terry Baume, 2009

// Configuration - pins, data rates, etc

// Digital mode pins
#define GPS_PIN_1		2	// GPS serial pin 1
#define GPS_PIN_2		3	// GPS serial pin 2
#define GPS_POWER		4	// Which pin turns the GPS on?
#define RTTY_LED		5
#define GPS_LED 		6	// Which LED to light when we have a GPS lock?
#define ONE_WIRE_BUS	7	// OneWire temp sensors are on pin 7
#define RTTY_PIN_1 		8	// RTTY space pin
#define RTTY_PIN_2 		9	// RTTY mark pin
#define KODAK_PIN 		16	// Pin to trigger the Kodak camera 
#define LUMIX_PIN		17

// Analog mode pins
#define HUMIDITY_PIN	0
#define PRESSURE_PIN	1

// Other stuff
#define RTTY_ASCII 		7	// ASCII set for RTTY (7/8bit)
#define GPSRATE 		4800// GPS baud rate
#define BUFFERSIZE 		90	// How many bytes of input to buffer from the GPS?

#include <Fat16.h>
#include <Streaming.h>
#include <NewSoftSerial.h>
#include <Flash.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <util/crc16.h>

// Initialize flight variables
int numSats = 0;
int fixType = 0;
int time[] = {0, 0, 0};
double latitude = 0.0;
double longitude = 0.0;
long altitude = 0;
long maxAlt = 0;
int speed = 0;
int txCount = 0;
int intTemp = 0;
int extTemp = 0; 
int bitRate = 50;
boolean descent = 0;
//int pressure = 0; // Pressure in hPa
//int humidity = 0; // Relative humidty, %

// Each temp sensor has a unique address
uint8_t internal[] = {40, 40, 218, 94, 2, 0, 0, 104};
uint8_t external[] = {40, 9, 246, 94, 2, 0, 0, 48}; 

SdCard card;
Fat16 Logger;
NewSoftSerial GPS = NewSoftSerial(GPS_PIN_1, GPS_PIN_2);
char buffer[BUFFERSIZE];
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//
// Initial setup/bootstrap
//
void setup() {
	Serial.begin(9600);
	Serial << F("Project Horus booting...") << endl;
	
	// Start the GPS, pull low to turn on
	pinMode(GPS_POWER, OUTPUT);
	digitalWrite(GPS_POWER, LOW);
	Serial << F("* GPS module siwtched on!") << endl;
	GPS.begin(GPSRATE);
	Serial << F("* GPS serial on pins ") << GPS_PIN_1 << F(" and ") << GPS_PIN_2 << F(" started at ") << GPSRATE << F(" baud") << endl;

	// LED's
	pinMode(RTTY_LED, OUTPUT);
	pinMode(GPS_LED, OUTPUT);
	
	// Setup for RTTY
	pinMode(RTTY_PIN_1, OUTPUT);
	pinMode(RTTY_PIN_2, OUTPUT);
	Serial << F("* RTTY on pins ") <<RTTY_PIN_1 << F(" and ") << RTTY_PIN_2 << F(" started at 50 baud") << endl;
	
	// Start temp sensors
	sensors.begin();
	sensors.requestTemperatures();
	Serial << F("* Temperature sensors started") << endl;
	
	// Ensure we can access the SD card
	if (!card.init()) Serial << F("* SD card init. failed!") << endl;
	if (!Fat16::init(card)) Serial << F("* FAT16 init. failed!") << endl;
	
	// Check for an available filename
	char fileName[12];
	Serial << F("* Trying to open logging file...") << endl;
	for (int i = 0; i < 1000; i++) {
		sprintf(fileName, "LOG%03d.TXT", i);
		if (Logger.open(fileName, O_CREAT | O_EXCL | O_WRITE)) break;
	}

	// Ensure we opened the file without error
	if (!Logger.isOpen()) {
		Serial << F("* Failed to open logging file!") << endl;
	} else {
		Logger.writeError = false;
		Serial << F("* Logging to ") << fileName << endl;
		Logger << F("Project Horus booted!") << endl;
		if (Logger.writeError || !Logger.sync()) Serial << F("* Error writing to SD card!") << endl;
	}
	Serial << endl;
}

//
// Main loop
//
void loop() {
	// We need to flush() because of errors introduced by delay() calls
	GPS.flush();

	// Get a GGA string from the GPS,
	// check if it's a valid fix, and extract the data
	getNMEA("$GPGGA");
	Logger << buffer << endl;
	numSats = getSats();
	fixType = getFixType();
	
	// Make sure we have a valid fix
	if (fixType != 0) { 
		getTime(time);	
		latitude = getLat();
		longitude = getLong();
		altitude = getAlt();
		digitalWrite(GPS_LED, HIGH);
		
		// Keep track of the maximum altitude
		if (altitude > maxAlt) { maxAlt = altitude; }
		
		// Check to see if we've fallen 300m, if so switch to descent mode
		if (altitude < (maxAlt - 300)) { descent = 1; }
		
		// Select baud rate based on altitude if we're ascending
		if (descent == 0) {
		
			// 5km - 7km = 100 baud
			if (altitude >= 5000 && altitude < 7000 && bitRate != 100) {
				txString("QRQ 100b");
				bitRate = 100;
				Logger << F("Alt = ") << altitude << F(", switching to ") << bitRate << F(" baud") << endl;
			}
			
			// 7km - 9km = 150 baud
			else if (altitude >= 7000 && altitude < 9000 && bitRate != 150) {
				txString("QRQ 150b");
				bitRate = 150;
				Logger << F("Alt = ") << altitude << F(", switching to ") << bitRate << F(" baud") << endl;
			}
			
			// 9km - 11km = 200 baud
			else if (altitude >= 9000 && altitude < 11000 && bitRate != 200) {
				txString("QRQ 200b");
				bitRate = 200;
				Logger << F("Alt = ") << altitude << F(", switching to ") << bitRate << F(" baud") << endl;
			}
			
			// 11km - 13km = 300 baud
			else if (altitude >= 11000 && altitude < 13000 && bitRate != 300) {
				txString("QRQ 300b");
				bitRate = 300;
				Logger << F("Alt = ") << altitude << F(", switching to ") << bitRate << F(" baud") << endl;
			}
			
			// < 5km, >13km = 50 baud
			else if (bitRate != 50) {
				txString("QRS 50b");
				bitRate = 50;
				Logger << F("Alt = ") << altitude << F(", switching to ") << bitRate << F(" baud") << endl;
			}

		} else {
			// If descending, set the bit rate to 50.
			bitRate = 50;
		}
	} else {
		digitalWrite(GPS_LED, LOW);
	}
	
	// Get the speed from a VTG message
	getNMEA("$GPVTG");
	Logger << buffer << endl;
	speed = getSpeed();	

	// Convert lat & long into strings
	char latString[12]; char longString[12];
	doubleToString(latitude, 4, latString);
	doubleToString(longitude, 4, longString);
	
	// Get the temperatures
	intTemp = sensors.getTempC(internal);
	extTemp = sensors.getTempC(external);
	
	// Request the temperatures, this is done before lengthy things
	sensors.requestTemperatures();
	
	// Get the pressure & humidity
	//pressure = (((analogRead(PRESSURE_PIN)/1024.0) + 0.095) / 0.009) * 10.0;
	//humidity = (((analogRead(HUMIDITY_PIN)/1024.0) - 0.16) / 0.0062) / (1.0546 - (0.00216 * (float)extTemp));
	
	// Form the data into a string
	//sprintf(buffer, "$$HORUS,%d,%02d:%02d:%02d,%s,%s,%ld,%d,%d;%d;%d;%d;%d", txCount, time[0], time[1], time[2], latString, longString, altitude, speed, numSats, intTemp, extTemp, pressure, humidity);
	sprintf(buffer, "$$HORUS,%d,%02d:%02d:%02d,%s,%s,%ld,%d,%d;%d;%d", txCount, time[0], time[1], time[2], latString, longString, altitude, speed, numSats, intTemp, extTemp);

	// Transmit and log to SD card
	txString(buffer);
	
	// Force an update on the SD card
	Logger.sync();	
	
	// Serial output
	Serial << F("RAM:  Free memory: ") << freeMem() << F(" bytes") << endl << endl;
	
	// Delay a moment before restarting loop
	delay(100);
	
	// Announce as VK5VZI every now and then
	if (txCount % 40 == 0) {
		sprintf(buffer, "DE VK5VZI, PROJECTHORUS.ORG");
		txString(buffer);
		Logger.sync();
		delay(100);
	}
	
}


