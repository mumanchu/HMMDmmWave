#pragma once

///////////////////////////////////////////////////////////////////////////////
// Arduino Library for WaveShare HMMD mmWave Sensor 
// 24GHz Human Micro-Motion detector
// Copyright (C) https://muman.ch + https://github.com/github, 2026.11.18
// If you re-use this software, please include the above copyright notice
// Released under the terms of the BSD-3-Clause-License
// https://github.com/mumanchu/HMMDmmWave


typedef unsigned long ulong;		// always 32 bits
typedef unsigned int uint;			// 16 or 32 bits
typedef unsigned short ushort;		// always 16 bits

// See FloatUtils.cpp
extern bool floatToString(float value, char* psz, unsigned int decimalPlaces);
extern bool stringToFloat(const char* psz, float* result, char** pszEnd);


// For debug output
#ifdef DEBUG
#ifndef LOGSERIAL
#define LOGSERIAL Serial
#endif
#ifndef LOGERROR
#define LOGERROR(s) { LOGSERIAL.println(s); LOGSERIAL.flush(); }
#endif
#else
#undef HMMD_SHOW_MESSAGES
#define LOGERROR(s)
#endif


class HMMDmmWave
{
protected:
	Stream* stream;

	// for receiver state machine
	uint rxState;
	uint rxMsgLength;
	uint rxMsgIndex;
	byte rxMsg[64];
	bool ackMessage;
	bool reportMessage;
	bool gotAck;

	// frame markers
	static const byte commandFrameStart[4];
	static const byte commandFrameEnd[4];
	static const byte reportFrameStart[4];
	static const byte reportFrameEnd[4];

public:
	// Data returned by radar report message
	struct RadarData
	{
		bool targetDetected;
		uint distanceToTarget;
		uint distanceGateEnergy[16];
	};

	// Operating Modes
	enum Mode : uint
	{
		Normal = 0x64,		// Normal mode, outputs text "Range <n>\r\nON|OFF\r\n"
		Debug = 0x00,		// Debug mode, outputs undocumented data
		Report = 0x04		// Report mode, outputs binary RadarData structures
	};

	bool begin(Stream* stream);
	bool setMode(Mode mode);
	bool enableConfigurationCommands();
	bool disableConfigurationCommands();
	bool getFirmwareVersion(char* fwVersion /*6 chars + NUL*/);
	bool getSerialNumber(char* serialNumber /*8 chars + NUL*/);
	bool setSerialNumber(const char* serialNumber /*8 chars*/);
	bool getDeviceId(ulong* deviceId);
	bool receive(RadarData* data = NULL);
	//bool receiveDebugFrame(Stream* stream, byte* buf, uint bufLength, uint* frameLength);

	#ifdef HMMD_ENABLE_CONFIG

	// Configuration Parameter Numbers, for read/writeParameter();
	enum Param : uint
	{
		// Integer values
		DetectionDistanceMin = 0,
		DetectionDistanceMax = 1,
		TargetActiveFrames = 2,
		TargetInactiveFrames = 3,
		UnoccupiedDelay = 4,
		// Float values are stored in integer log format,
		// see logToFloat() and floatToLog()
		TriggerThreshold = 0x10,	// 16 x 32-bit integer log values, 0x10 .. 0x1F
		HoldThreshold = 0x20		// 16 x 32-bit integer log values, 0x20 .. 0x3F
	};
	bool readParameter(Param param, ulong* data);
	bool writeParameter(Param param, ulong data);

	// The TriggerThreshold and HoldThreshold parameters are floating point 
	// values, but are stored in an "integer log" format, so they must be 
	// converted to/from this format with these methods
	float logToFloat(ulong data) { return data == 0 ? 0.0f : log10f((float)data) * 10.0f; }
	ulong floatToLog(float f)    { return (ulong)(powf(10.0f, f / 10.0f) + 0.5f); }

	#ifdef DEBUG
	bool printParameters();
	bool printIntParam(Param param, const char* name);
	bool printFloatParam(Param param, const char* name);
	void dump(const byte* pb, uint len);
	#endif

	#endif // #ifdef HMMD_ENABLE_CONFIG

protected:
	bool processRadarDataMessage(RadarData* radarData);
	bool sendCommandAndGetResponse(const byte* command, int length);
};

// Message frame markers
const byte HMMDmmWave::commandFrameStart[4] = { 0xFD, 0xFC, 0xFB, 0xFA };
const byte HMMDmmWave::commandFrameEnd[4] = { 0x04, 0x03, 0x02, 0x01 };
const byte HMMDmmWave::reportFrameStart[4]  = { 0xF4, 0xF3, 0xF2, 0xF1 };
const byte HMMDmmWave::reportFrameEnd[4]  = { 0xF8, 0xF7, 0xF6, 0xF5 };


// Call this from setup()
// sensor is put into Command mode
bool HMMDmmWave::begin(Stream* stream)
{
	this->stream = stream;
	rxState = 0;
	return enableConfigurationCommands();
}

// Set the sensor's operating mode
// this defines the data it will send repeatedly:
//   Normal : outputs text "Range <n>\r\nON|OFF\r\n"
//   Debug  : outputs undocumented data, see receiveDebugFrame()
//   Report : outputs binary RadarData structures
bool HMMDmmWave::setMode(Mode mode)
{
	static byte cmd[8] = { 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	cmd[4] = mode;
	return sendCommandAndGetResponse(cmd, 8);
}

// Enable configuration mode
// The sensor is usually in a mode where it continuously reports its status,
// called "radar data output protocol" in the manual. This stops the radar 
// data output reporting and puts it into a mode ready to receive commands.
bool HMMDmmWave::enableConfigurationCommands()
{
	static const byte cmd1[4] = { 0xFF, 0x00, 0x02, 0x00 };
	return sendCommandAndGetResponse(cmd1, 4);
}

// End configuration mode
// Back to "radar data output protocol" mode
bool HMMDmmWave::disableConfigurationCommands()
{
	static const byte cmd2[2] = { 0xFE, 0x00 };
	return sendCommandAndGetResponse(cmd2, 2);
}

// Return the Device ID as a 32-bit number
// on my board this is actually 4 ASCII characters "04PA", 0x41503430
bool HMMDmmWave::getDeviceId(ulong* deviceId)
{
	*deviceId = 0;
	static const byte cmd[2] = { 0x75, 0x00 };
	if (!sendCommandAndGetResponse(cmd, 2))
		return false;
	// 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17
	// F4 F3 F2 F1 08 00 75 01 00 00 30 34 50 41 04 03 02 01
	//            | len | cmd | ack | device id | frame end |
	*deviceId = *(ulong*)(rxMsg + 10);
	return true;
}

// Return the 8-character serial number of the chip as a string
// the default for my chip was "FFFFFFFF"
bool HMMDmmWave::getSerialNumber(char* serialNumber /*8 chars + NUL*/)
{
	*serialNumber = '\0';
	static const byte cmd[2] = { 0x11, 0x00 };
	if (!sendCommandAndGetResponse(cmd, 2))
		return false;
	// 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23
	// F4 F3 F2 F1 ll ll 11 01 00 00 08 00 46 46 46 46 46 46 46 46 04 03 02 01
	//            |     |     | ack | len |ss ss ss ss ss ss ss ss| frame end |
	uint len = rxMsg[10];
	if (len > 8)
		len = 8;
	strncpy(serialNumber, (char*)(rxMsg + 12), len);
	serialNumber[len] = '\0';
	return true;
}

// The 8-character Serial Number can be programmed into flash
bool HMMDmmWave::setSerialNumber(const char* serialNumber /*8 chars*/)
{
	if (serialNumber == NULL || strlen(serialNumber) != 8) {
		LOGERROR("bad param");
		return false;
	}
	byte cmd[12];
	memset(cmd, 0, sizeof(cmd));
	cmd[0] = 0x10;		// command
	cmd[2] = 0x08;		// length
	strncpy((char*)(cmd + 4), serialNumber, 8);
	return sendCommandAndGetResponse(cmd, 12);
}

// Read the firmware version as string, 6 chars + NUL.
// my module returns "v1.5.9"
bool HMMDmmWave::getFirmwareVersion(char* fwVersion /*6 chars + NUL*/)
{
	*fwVersion = '\0';
	static const byte cmd[2] = { 0x00, 0x00 };
	if (!sendCommandAndGetResponse(cmd, 2))
		return false;
	// 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21
	// FD FC FB FA 0C 00 00 01 00 00 06 00 76 31 2E 35 2E 39 04 03 02 01
	//            |     |     | ack | len |vv vv vv vv vv vv| frame end |
	uint len = rxMsg[10];
	if (len > 6)
		len = 6;
	strncpy(fwVersion, (char*)(rxMsg + 12), len);
	fwVersion[len] = '\0';
	return true;
}

#ifdef HMMD_ENABLE_CONFIG

// Read a single parameter, see Param enums
// NOTE! TriggerThreshold+n and HoldThreshold+n parameters are stored in 
// log10 format, and must be converted to float using logToFloat();
// For example:
//    ulong tt5;
//    sensor.readParameter(sensor.TriggerThreshold + 5, &tt5);
//    float ftt5 = sensor.logToFloat(tt5);
bool HMMDmmWave::readParameter(Param param, ulong* data)
{
	*data = 0;
	byte cmd[4] = { 0x08, 0x00, 0x00, 0x00 };
	*(ushort*)(cmd + 2) = param;
	if (!sendCommandAndGetResponse(cmd, 4))
		return false;
	// 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17
	// FD FC FB FA 08 00 08 01 00 00 0C 00 00 00 04 03 02 01
	//            | len | cmd | ack |   data    | frame end |
	*data = *(ulong*)(rxMsg + 10);
	return true;
}

// Write a single parameter, see Param enums
// NOTE! TriggerThreshold+n and HoldThreshold+n parameters are stored in 
// log10 format, and must be converted from float using floatToLog();
// For example:
//    ulong tt1 = floatToLog(ftt1);
//    sensor.writeParameter(sensor.TriggerThreshold + 1, &tt1);
bool HMMDmmWave::writeParameter(Param param, ulong data)
{
	byte cmd[8] = { 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	*(ushort*)(cmd + 2) = param;
	*(ulong*)(cmd + 4) = data;
	return sendCommandAndGetResponse(cmd, 8);
}

// Process a response when in binary radar data Report mode
bool HMMDmmWave::processRadarDataMessage(RadarData* radarData)
{
	if (radarData == NULL) {
		LOGERROR("bug");
		return false;
	}
	memset(radarData, 0, sizeof(RadarData));
	if (rxMsgLength != (35 + 4)) {
		LOGERROR("invalid data length");
		return false;
	}
	// 0  1  2  3  4  5  6  7  8  9  10 11 12  ...
	// F4 F3 F2 F1 23 00 01 69 00 39 3E 3D 10  ...  10 00 08 00 F8 F7 F6 F5
	//            | len |tt|dist |  0  |  1  | ... | 14  | 15  | frame end |
	radarData->targetDetected = rxMsg[6];
	radarData->distanceToTarget = *(ushort*)(rxMsg + 7);
	// is gate count fixed at 16?
	uint nGates = (*(ushort*)(rxMsg + 4) - 3) / 2;
	int ix = 9;
	for (uint gate = 0; gate < nGates; ++gate) {
		radarData->distanceGateEnergy[gate] = *(ushort*)(rxMsg + ix);
		ix += 2;
	}
	return true;
}

#endif // #ifdef HMMD_ENABLE_CONFIG

#ifdef HMMD_SHOW_MESSAGES
// Display bytes in hex
void HMMDmmWave::dump(const byte* pb, uint len)
{
	while (len--) {
		LOGSERIAL.printf("%02X ", *pb++);
	}
	LOGSERIAL.flush();
}
#endif

// Returns true if everything's ok, false if there's an error
bool HMMDmmWave::sendCommandAndGetResponse(const byte* command, int length)
{
	// return 0s if it fails
	memset(rxMsg, 0x00, sizeof(rxMsg));

	// remove old data from receive buffer
	while (stream->available())
		stream->read();

	// send command
	stream->write(commandFrameStart, 4);
	stream->write((byte)length);
	stream->write((byte)(length >> 8));
	stream->write(command, length);
	stream->write(commandFrameEnd, 4);

	#ifdef HMMD_SHOW_MESSAGES
	LOGSERIAL.print("\r\n>>> ");
	dump(commandFrameStart, 4);
	dump((const byte*)&length, 2);
	dump(command, length);
	dump(commandFrameEnd, 4);
	LOGSERIAL.println();
	#endif

	// wait for ack response, or timeout
	gotAck = false;
	rxState = 0;
	unsigned long startms = millis();
	while (1) {
		if (receive(NULL)) {
			if (gotAck)
				break;
		}
		if ((millis() - startms) > 1000) {
			LOGERROR("ack timeout");
			return false;
		}
	}

	// check for ack / nak response

	// 0  1  2  3  4  5  6  7  8  9  10   
	// FD FC FB FA ll ll cc 01 aa aa data 04 03 02 01
	// ll ll = data length
	// cc 01 = command byte | 0x0100
	// aa aa = ack status 0x0000 = success
	if (rxMsg[6] != command[0] || rxMsg[7] != 0x01) {
		LOGERROR("invalid response");
		return false;
	}
	// aa aa : 0x0000 = ack
	if (*(ushort*)(rxMsg + 8) != 0) {
		LOGERROR("nak response");
		return false;
	}
	return true;
}

// Simple state machine to receive radar data frames
// Works for radar Report mode, Normal mode and command response 
// frames.
// Call this from loop() to receive radar Report mode and Normal mode
// data frames. It returns 'true' when a full message is available.
// For NormalMode it's a test response "Range <n>\r\nON|OFF\r\n"
// NOTE! This does not handle Debug mode messages, for that use 
// receiveDebugFrame() below.
// (This code could probably be optimized, it's a bit messy.)
bool HMMDmmWave::receive(RadarData* radarData/*=NULL*/)
{
	while (stream->available()) {
		byte b = stream->read();

		#ifdef HMMD_SHOW_MESSAGES
		LOGSERIAL.printf("%02X ", b);
		LOGSERIAL.flush();
		#endif

		// if in Normal mode (text messages),
		// abort of this is a binary message character
		if (rxState == 6 && b > 'z')
			rxState = 0;

		switch (rxState) {

		// look for report frame or ack frame header
		case 0:
			reportMessage = false;
			ackMessage = false;
			gotAck = false;

			// radar report data when in Report mode
			// sent once a second
			if (b == reportFrameStart[0]) {
				reportMessage = true;
				rxMsg[0] = b;
				rxMsgIndex = 1;
				rxState = 1;
				break;
			}

			// ack response to command message
			if (b == commandFrameStart[0]) {
				ackMessage = true;
				rxMsg[0] = b;
				rxMsgIndex = 1;
				rxState = 2;
				break;
			}

			// text data when in Normal mode
			// sent every 100ms (10x per second)
			if (b == 'R') {
				rxMsg[0] = b;
				rxMsgIndex = 1;
				rxState = 6;
				break;
			}
			break;

		// receive report frame header
		case 1:
			if (b == reportFrameStart[rxMsgIndex]) {
				rxMsg[rxMsgIndex++] = b;
				if (rxMsgIndex == 4)
					rxState = 3;
				break;
			}
			rxState = 0;
			break;

		// receive ack frame header
		case 2:
			if (b == commandFrameStart[rxMsgIndex]) {
				rxMsg[rxMsgIndex++] = b;
				if (rxMsgIndex == 4)
					rxState = 3;
				break;
			}
			rxState = 0;
			break;

		// receive message length LS byte
		case 3:
			rxMsg[rxMsgIndex++] = b;
			rxMsgLength = b;
			rxState = 4;
			break;

		// receive message length MS byte
		case 4:
			rxMsg[rxMsgIndex++] = b;
			rxMsgLength += (uint)b << 8;
			// add 4 bytes for frame end
			rxMsgLength += 4;
			if ((rxMsgLength + 6) > sizeof(rxMsg)) {
				LOGERROR("msg too long");
				rxState = 0;
				break;
			}
			rxState = 5;
			break;

		// receive frame + 4 frame end bytes
		case 5:
			if (rxMsgIndex >= sizeof(rxMsg)) {
				LOGERROR("rxMsg overflow");
				rxState = 0;
				break;
			}
			rxMsg[rxMsgIndex++] = b;

			// full message + frame end received
			if (rxMsgIndex == (rxMsgLength + 6)) {
				rxState = 0;

				// check for valid frame end
				const byte* frameEnd = ackMessage ? commandFrameEnd : reportFrameEnd;
				if (memcmp(rxMsg + rxMsgLength + 2, frameEnd, 4) != 0) {
					LOGERROR("invalid frame end");
					rxState = 0;
					return false;
				}
				if (ackMessage) {
					// ack response is processed on return
					gotAck = true;
				}
				if (reportMessage && radarData != NULL) {
					// return radar data if required
					processRadarDataMessage(radarData);
				}
				return true;
			}
			break;

		case 6:
			// For Normal mode it's a text message 
			// "Range <n>\r\nON|OFF\r\n"
			// sent every 100ms (10x per second)

			// use the length to determine the end
			// shortest = 12 characters, longest = 16 characters
			if (rxMsgIndex >= 20) {
				LOGERROR("too long");
				rxState = 0;
				break;
			}
			rxMsg[rxMsgIndex++] = b;

			// second '\n' received, now we can parse it
			if (b == '\n' && rxMsgIndex >= 12) {
				rxMsg[rxMsgIndex] = '\0';
				rxState = 0;
				if (radarData == NULL)
					break;
				uint distanceToTarget;
				char onoff[4];
				if (sscanf((const char*)rxMsg, "Range %u\r\n%3s\r\n",
					&distanceToTarget, onoff) != 2) {
					LOGERROR("invalid fmt");
					break;
				}
				radarData->targetDetected = strcmp(onoff, "ON") == 0;
				radarData->distanceToTarget = distanceToTarget;
				// NOTE! the radarData->distanceGateEnergy[] array is
				// unchanged and may contain garbage
				return true;
			}
			break;

		default:
			LOGERROR("bug");
			break;
		}
	}
	return false;
}


#if 0
// A stand-alone method to receive the undocumented Debug mode frames.
// The frame contains 320 x 32-bit values, but I don't know what they are.
// The buffer length must be 1288 bytes (or larger) for the current firmware.
// All I know is this...
// "In debugging mode, the RDMAP data of all the range gates of a certain 
// chirp are sent out each time, i.e., the data of all the 16 range gates 
// of the first chirp will be sent out first, then goes the second chirp, 
// and so on."
//   AA BF 10 14
//   RDMAP : 20(Dopple) * 16(number of range gate) * 4(square of the amplitude)
//   FD FC FB FA
bool HMMDmmWave::receiveDebugFrame(Stream* stream, byte* buf, 
	uint bufLength, uint* frameLength)
{
	static const byte debugFrameStart[4] = { 0xAA, 0xBF, 0x10, 0x14 };
	static const byte debugFrameEnd[4] = { 0xFD, 0xFC, 0xFB, 0xFA };

	static uint frameEndCtr = 0;
	static uint bufIndex = 0;
	static uint state = 0;

	while (stream->available()) {
		byte b = stream->read();

		switch (state) {

		// look for start of debug frame header
		case 0:
			if (b == debugFrameStart[0]) {
				buf[0] = b;
				bufIndex = 1;
				frameEndCtr = 0;
				state = 1;
			}
			break;

		// receive debug frame header
		case 1:
			if (b == debugFrameStart[bufIndex]) {
				buf[bufIndex++] = b;
				if (bufIndex == 4)
					state = 2;
				break;
			}
			state = 0;
			break;

		// receiving message, look for end of frame marker
		// the message has no 'length'
		case 2:
			if (bufIndex >= bufLength) {
				state = 0;
				LOGERROR("debug data too long");
				break;
			}
			buf[bufIndex++] = b;

			// the data may contain frame end characters,
			// but hopefully not all four of them
			if (frameEndCtr > 0) {
				if (b == debugFrameEnd[frameEndCtr++]) {
					if (frameEndCtr == 4) {
						// end of debug frame
						state = 0;
						// frameLength includes the start and end frames
						*frameLength = bufIndex;
						return true;
					}
					break;
				}
				frameEndCtr = 0;
			}
			if (b == debugFrameEnd[0]) {
				frameEndCtr = 1;
			}
			break;

		default:
			LOGERROR("bug");
			break;
		}
	}
	return false;
}
#endif


#ifdef HMMD_ENABLE_CONFIG

#ifdef DEBUG
// Print all the parameters to LOGSERIAL
bool HMMDmmWave::printParameters()
{
	LOGSERIAL.println("SENSOR PARAMETERS\n\r");

	// 32-bit integer params
	if (printIntParam(Param::DetectionDistanceMin, "DetectionDistanceMin") &&
		printIntParam(Param::DetectionDistanceMax, "DetectionDistanceMax") &&
		printIntParam(Param::TargetActiveFrames, "TargetActiveFrames") &&
		printIntParam(Param::TargetInactiveFrames, "TargetInactiveFrames") &&
		printIntParam(Param::UnoccupiedDelay, "UnoccupiedDelay")) {

		// 32-bit float parameters, stored as '(ulong)(powf(10.0f, f / 10.0f) + 0.5f)'
		char name[32];
		for (uint i = 0; i < 16; ++i) {
			Param param = (Param)(Param::TriggerThreshold + i);
			sprintf(name, "TriggerThreshold_%u", i);
			if (!printFloatParam(param, name))
				return false;
		}
		for (uint i = 0; i < 16; ++i) {
			Param param = (Param)(Param::HoldThreshold + i);
			sprintf(name, "HoldThreshold_%u", i);
			if (!printFloatParam(param, name))
				return false;
		}
		LOGSERIAL.println("");
		return true;
	}
	return false;
}

bool HMMDmmWave::printIntParam(Param param, const char* name)
{
	ulong data;
	if (!readParameter(param, &data))
		return false;
	char buf[32];
	sprintf(buf, "%-2u %-20s %lu", (uint)param, name, data);
	LOGSERIAL.println(buf);
	LOGSERIAL.flush();
	return true;
}

bool HMMDmmWave::printFloatParam(Param param, const char* name)
{
	ulong data;
	if (!readParameter(param, &data))
		return false;
	float f = logToFloat(data);
	char fstr[20];
	floatToString(f, fstr, 2);
	char buf[32];
	sprintf(buf, "%-2u %-20s %s", param, name, fstr);
	LOGSERIAL.println(buf);
	LOGSERIAL.flush();
	return true;
}
#endif // #ifdef DEBUG

#endif // #ifdef HMMD_ENABLE_CONFIG

