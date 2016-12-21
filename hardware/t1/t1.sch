EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L R R1
U 1 1 58200731
P 1900 2300
F 0 "R1" V 1980 2300 50  0000 C CNN
F 1 "R" V 1900 2300 50  0000 C CNN
F 2 "LEDs:LED-3MM" V 1830 2300 50  0001 C CNN
F 3 "" H 1900 2300 50  0000 C CNN
	1    1900 2300
	1    0    0    -1  
$EndComp
$Comp
L CP C1
U 1 1 58200815
P 2200 2300
F 0 "C1" H 2225 2400 50  0000 L CNN
F 1 "CP" H 2225 2200 50  0000 L CNN
F 2 "Connect:GS2" H 2238 2150 50  0001 C CNN
F 3 "" H 2200 2300 50  0000 C CNN
	1    2200 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 2150 2200 2150
Wire Wire Line
	1900 2450 2200 2450
$Comp
L GND #PWR01
U 1 1 58200B38
P 2050 2450
F 0 "#PWR01" H 2050 2200 50  0001 C CNN
F 1 "GND" H 2050 2300 50  0000 C CNN
F 2 "" H 2050 2450 50  0000 C CNN
F 3 "" H 2050 2450 50  0000 C CNN
	1    2050 2450
	1    0    0    -1  
$EndComp
$EndSCHEMATC
