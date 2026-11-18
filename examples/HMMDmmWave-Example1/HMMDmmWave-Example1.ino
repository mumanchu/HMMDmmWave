///////////////////////////////////////////////////////////////////////////////
// Example Sketch for Arduino Library for WaveShare HMMD mmWave Sensor 
// Copyright (C) https://muman.ch + https://github.com/github, 2026.11.18
// If you re-use this software, please include the above copyright notice
// Released under the terms of the BSD-3-Clause-License

// Comment this out to hide the debug output
#define DEBUG

// On the STM32F103 Nucleo board, 'Serial' is the USB Virtual port
// so we must use USART1, TX=D8 (PA9), RX=D2 (PA10)
// Put this with "Project Properties / Configuration Defines" for 
// automatic code generation: DEBUG;HAVE_HWSERIAL1
// OR use this definition:
//HardwareSerial Serial1(USART1);
Uart Serial1(USART1);

// Serial port for Serial Monitor ouput
#define LOGSERIAL Serial


// Uncomment this to the messages in hex
//#define HMMD_SHOW_MESSAGES
// Enables the configuration methods
#define HMMD_ENABLE_CONFIG
#include "HMMDmmWave.h"
HMMDmmWave sensor;


void setup() 
{
	// I use different pins for Serial logging on the STM32 Nucleo-64 boards,
	// else Serial is the USB port which is also used for download
	#ifdef ARDUINO_NUCLEO_64
	LOGSERIAL.setTx(PC_10);
	LOGSERIAL.setRx(PC_11);
	#endif

	LOGSERIAL.begin(115200);
	delay(1000);

	// PuTTY clear screen and scrollback
	// comment this out if not using PuTTY
	LOGSERIAL.print("\033[2J\033[H\033[3J");
	LOGSERIAL.flush();

	LOGSERIAL.println("\r\nHMMDmmWave-Example Started...\r\n");
	LOGSERIAL.flush();

	// initialize the senor
	Serial1.begin(115200);

	if (!sensor.begin(&Serial1)) {
		LOGSERIAL.println("sensor.begin() failed");
		LOGSERIAL.flush();
		while (1) yield();
	}

	// show the device information
	ulong deviceId;
	sensor.getDeviceId(&deviceId);
	char fwVersion[8];
	sensor.getFirmwareVersion(fwVersion);
	char serialNumber[16];
	sensor.getSerialNumber(serialNumber);

	LOGSERIAL.print("deviceId     0x");
	LOGSERIAL.println(deviceId, HEX);
	LOGSERIAL.print("fwVersion    ");
	LOGSERIAL.println(fwVersion);
	LOGSERIAL.print("serialNumber ");
	LOGSERIAL.println(serialNumber);
	LOGSERIAL.println();
	LOGSERIAL.flush();

	// show the current configuration
	#if defined(DEBUG) && defined(HMMD_ENABLE_CONFIG)
	sensor.printParameters();
	#endif

	// prepare for reporting
	sensor.setMode(sensor.Mode::Report);
	sensor.disableConfigurationCommands();

	LOGSERIAL.println();
	LOGSERIAL.println();
}


void loop() 
{
	// Poll the sensor and display the sensor readings
	HMMDmmWave::RadarData radarData;
	if (sensor.receive(&radarData)) {
		char buf[64];
		sprintf(buf, "targetDetected=%u  distanceToTarget=%u     \r", 
			radarData.targetDetected, radarData.distanceToTarget);
		LOGSERIAL.print(buf);
		LOGSERIAL.flush();
	}
}

