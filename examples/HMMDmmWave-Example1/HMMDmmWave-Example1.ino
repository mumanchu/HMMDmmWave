///////////////////////////////////////////////////////////////////////////////
// Example Sketch for Arduino Library for WaveShare HMMD mmWave Sensor 
// Copyright (C) https://muman.ch + https://github.com/github, 2026.11.18
// If you re-use this software, please include the above copyright notice
// Released under the terms of the BSD-3-Clause-License

// Comment this out to hide the debug output
#define DEBUG

// Serial port for Serial Monitor ouput
#define LOGSERIAL Serial


// The usual 'Serial' is for the Serial Monitor so you can see the data

// 'Serial1' is used for the mmWave sensor's UART, but the pins may be 
// different on each microcontroller board. Here are some examples...

#if defined(ARDUINO_SAM_DUE)
// The Arduino Due has Serial1 on TX1 (18) and RX1 (19), connect these to RX 
// and TX of the sensor
// !!!NOTE!!!  
// If you select "No Optimisation" for the Arduino Due, then the build may
// fail with this message:
//  "HardwareSerial.h:26: undefined reference to vtable for HardwareSerial"
// It's a problem with the board package code. Set it to "Default Optimisation",
// do a 'Clean solution' and re-build. Or use the fix described here:
// https://learn.adafruit.com/proper-step-debugging-atsamd21-arduino-zero-m0/paths-and-optimizations#fixing-some-core-files-2469468-17
#endif

#if defined(ARDUINO_ARCH_STM32)
// On the STM32F103 Nucleo board, 'Serial' is the USB port
// so we use USART1, TX=D8 (PA9), RX=D2 (PA10) for the mmWave sensor
//HardwareSerial Serial1(USART1);
Uart Serial1(USART1);
#endif

#if defined(ARDUINO_ARCH_AVR)
// SoftwareSerial will not work at 115200 baud on AVR processors
//TODO
#endif

#if defined(ARDUINO_SAMD_ZERO)
// Arduino Zero has 'Serial1' on pin 0 (RX) and pin 1 (TX), so you don't need 
// to do anything. 'Serial' is the USB port, so connect your Serial Monitor to 
// the port called 'Atmel Corp. EDBG CMSIS-DAP'
#endif

#if defined(ARDUINO_ARCH_ESP32)
// For the ESP32s, Serial1's pins can be defined
// see the code in setup() below
#endif


// Uncomment this to show the messages in hex
//#define HMMD_SHOW_MESSAGES

// This enables the configuration methods
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

