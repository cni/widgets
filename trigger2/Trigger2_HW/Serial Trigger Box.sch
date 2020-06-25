EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title "CNI Serial Trigger - HW Ver 2"
Date "2020-06-21"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R R1
U 1 1 5EEFC70A
P 2350 2350
F 0 "R1" H 2420 2396 50  0000 L CNN
F 1 "100K" H 2420 2305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2280 2350 50  0001 C CNN
F 3 "~" H 2350 2350 50  0001 C CNN
	1    2350 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5EEFD0F0
P 2950 1950
F 0 "R2" V 2743 1950 50  0000 C CNN
F 1 "22K" V 2834 1950 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2880 1950 50  0001 C CNN
F 3 "~" H 2950 1950 50  0001 C CNN
	1    2950 1950
	0    1    1    0   
$EndComp
Wire Wire Line
	2350 1750 2350 1950
Wire Wire Line
	2800 1950 2350 1950
Connection ~ 2350 1950
Wire Wire Line
	2350 1950 2350 2200
Wire Wire Line
	3100 1950 3400 1950
$Comp
L power:+5V #PWR06
U 1 1 5EF092FF
P 3400 1100
F 0 "#PWR06" H 3400 950 50  0001 C CNN
F 1 "+5V" H 3415 1273 50  0000 C CNN
F 2 "" H 3400 1100 50  0001 C CNN
F 3 "" H 3400 1100 50  0001 C CNN
	1    3400 1100
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR03
U 1 1 5EF09888
P 2450 1100
F 0 "#PWR03" H 2450 950 50  0001 C CNN
F 1 "+5V" H 2465 1273 50  0000 C CNN
F 2 "" H 2450 1100 50  0001 C CNN
F 3 "" H 2450 1100 50  0001 C CNN
	1    2450 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 1350 2450 1100
Wire Wire Line
	2350 2500 2350 3000
$Comp
L power:GND #PWR02
U 1 1 5EF0A9F5
P 2350 3000
F 0 "#PWR02" H 2350 2750 50  0001 C CNN
F 1 "GND" H 2355 2827 50  0000 C CNN
F 2 "" H 2350 3000 50  0001 C CNN
F 3 "" H 2350 3000 50  0001 C CNN
	1    2350 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 1950 4650 1950
Connection ~ 3400 1950
$Comp
L Connector:Conn_Coaxial J1
U 1 1 5EF15E77
P 3150 2800
F 0 "J1" V 3032 2900 50  0000 L CNN
F 1 "Trigger In" V 3123 2900 50  0000 L CNN
F 2 "Connector_Coaxial:BNC_Amphenol_031-6575_Horizontal" H 3150 2800 50  0001 C CNN
F 3 " ~" H 3150 2800 50  0001 C CNN
	1    3150 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	2950 2800 2950 3000
$Comp
L power:GND #PWR04
U 1 1 5EF19AD6
P 2950 3000
F 0 "#PWR04" H 2950 2750 50  0001 C CNN
F 1 "GND" H 2955 2827 50  0000 C CNN
F 2 "" H 2950 3000 50  0001 C CNN
F 3 "" H 2950 3000 50  0001 C CNN
	1    2950 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 2600 3150 2050
Wire Wire Line
	3150 2050 3700 2050
$Comp
L power:GND #PWR011
U 1 1 5EF46890
P 7050 6050
F 0 "#PWR011" H 7050 5800 50  0001 C CNN
F 1 "GND" H 7055 5877 50  0000 C CNN
F 2 "" H 7050 6050 50  0001 C CNN
F 3 "" H 7050 6050 50  0001 C CNN
	1    7050 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 5900 7050 6050
Wire Wire Line
	5650 5900 7050 5900
Wire Wire Line
	4750 5900 4350 5900
Wire Wire Line
	5050 5900 5350 5900
$Comp
L Device:LED D2
U 1 1 5EF3F673
P 5500 5900
F 0 "D2" H 5493 5645 50  0000 C CNN
F 1 "Trigger Out" H 5493 5736 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm_Clear" H 5500 5900 50  0001 C CNN
F 3 "~" H 5500 5900 50  0001 C CNN
	1    5500 5900
	-1   0    0    1   
$EndComp
$Comp
L Device:R R5
U 1 1 5EF3E336
P 4900 5900
F 0 "R5" V 4693 5900 50  0000 C CNN
F 1 "220" V 4784 5900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4830 5900 50  0001 C CNN
F 3 "~" H 4900 5900 50  0001 C CNN
	1    4900 5900
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5EF5287D
P 6800 6400
F 0 "#PWR010" H 6800 6150 50  0001 C CNN
F 1 "GND" H 6805 6227 50  0000 C CNN
F 2 "" H 6800 6400 50  0001 C CNN
F 3 "" H 6800 6400 50  0001 C CNN
	1    6800 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 6250 6800 6400
Wire Wire Line
	5400 6250 6800 6250
Wire Wire Line
	4800 6250 5100 6250
$Comp
L Device:LED D1
U 1 1 5EF5288C
P 5250 6250
F 0 "D1" H 5243 5995 50  0000 C CNN
F 1 "Trigger Detect" H 5243 6086 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm_Clear" H 5250 6250 50  0001 C CNN
F 3 "~" H 5250 6250 50  0001 C CNN
	1    5250 6250
	-1   0    0    1   
$EndComp
$Comp
L Device:R R4
U 1 1 5EF52896
P 4650 6250
F 0 "R4" V 4443 6250 50  0000 C CNN
F 1 "220" V 4534 6250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4580 6250 50  0001 C CNN
F 3 "~" H 4650 6250 50  0001 C CNN
	1    4650 6250
	0    1    1    0   
$EndComp
Wire Wire Line
	4500 6250 4250 6250
Wire Wire Line
	3400 1950 3400 1750
Wire Wire Line
	3400 1450 3400 1100
$Comp
L Device:C C1
U 1 1 5EF61F44
P 3400 1600
F 0 "C1" H 3515 1646 50  0000 L CNN
F 1 "0.1uF" H 3515 1555 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D4.0mm_H7.0mm_P1.50mm" H 3438 1450 50  0001 C CNN
F 3 "~" H 3400 1600 50  0001 C CNN
	1    3400 1600
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push_SPDT SW1
U 1 1 5EF498E6
P 2350 1550
F 0 "SW1" V 2396 1362 50  0000 R CNN
F 1 "Trigger Out" V 2305 1362 50  0000 R CNN
F 2 "Button_Switch_SMD:SW_SPST_TL3305C" H 2350 1550 50  0001 C CNN
F 3 "~" H 2350 1550 50  0001 C CNN
	1    2350 1550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4650 2350 4250 2350
Wire Wire Line
	4250 2350 4250 6250
Wire Wire Line
	4650 2450 4350 2450
Wire Wire Line
	4350 2450 4350 5900
Wire Wire Line
	4650 2550 4450 2550
Wire Wire Line
	4450 2550 4450 5000
Wire Wire Line
	2250 1350 1950 1350
Wire Wire Line
	1950 1350 1950 1450
$Comp
L power:GND #PWR01
U 1 1 5EF9F6A1
P 1950 1450
F 0 "#PWR01" H 1950 1200 50  0001 C CNN
F 1 "GND" H 1955 1277 50  0000 C CNN
F 2 "" H 1950 1450 50  0001 C CNN
F 3 "" H 1950 1450 50  0001 C CNN
	1    1950 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5EFA56C8
P 4200 1400
F 0 "#PWR07" H 4200 1150 50  0001 C CNN
F 1 "GND" H 4205 1227 50  0000 C CNN
F 2 "" H 4200 1400 50  0001 C CNN
F 3 "" H 4200 1400 50  0001 C CNN
	1    4200 1400
	1    0    0    -1  
$EndComp
NoConn ~ 6050 1750
NoConn ~ 6050 1850
NoConn ~ 6050 2150
NoConn ~ 6050 2250
NoConn ~ 6050 2350
NoConn ~ 6050 2450
NoConn ~ 6050 2550
NoConn ~ 6050 2650
NoConn ~ 6050 2750
NoConn ~ 6050 2850
NoConn ~ 6050 2950
NoConn ~ 6050 3150
NoConn ~ 6050 3250
NoConn ~ 6050 3350
NoConn ~ 6050 3450
NoConn ~ 6050 3550
NoConn ~ 6050 3650
NoConn ~ 6050 3750
NoConn ~ 6050 3850
NoConn ~ 6050 3950
NoConn ~ 6050 4050
NoConn ~ 6050 4150
NoConn ~ 6050 4250
NoConn ~ 4650 1850
NoConn ~ 4650 2150
NoConn ~ 4650 2250
NoConn ~ 4650 2650
NoConn ~ 4650 2750
NoConn ~ 4650 2850
NoConn ~ 4650 2950
NoConn ~ 4650 3050
NoConn ~ 4650 3150
NoConn ~ 4650 3250
NoConn ~ 4650 3350
NoConn ~ 4650 3450
NoConn ~ 4650 3550
NoConn ~ 4650 3650
NoConn ~ 4650 3850
NoConn ~ 4650 3950
NoConn ~ 4650 4050
NoConn ~ 4650 4150
NoConn ~ 4650 4250
NoConn ~ 4650 3750
NoConn ~ 4650 4450
NoConn ~ 4650 4350
NoConn ~ 6050 4450
NoConn ~ 6050 4350
$Comp
L teensy:Teensy++2.0_(Arduino) U1
U 1 1 5EEDA480
P 5350 3100
F 0 "U1" H 5350 4737 60  0000 C CNN
F 1 "Teensy_++2.0" H 5350 4631 60  0000 C CNN
F 2 "" H 5450 2450 60  0001 C CNN
F 3 "" H 9150 2400 60  0001 C CNN
	1    5350 3100
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_Coaxial J2
U 1 1 5EF408B2
P 6700 5000
F 0 "J2" H 6800 4975 50  0000 L CNN
F 1 "Trigger Out" H 6800 4884 50  0000 L CNN
F 2 "Connector_Coaxial:BNC_Amphenol_031-6575_Horizontal" H 6700 5000 50  0001 C CNN
F 3 " ~" H 6700 5000 50  0001 C CNN
	1    6700 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 5000 5650 5000
Wire Wire Line
	6700 5200 6700 5400
$Comp
L power:GND #PWR09
U 1 1 5EF41519
P 6700 5400
F 0 "#PWR09" H 6700 5150 50  0001 C CNN
F 1 "GND" H 6705 5227 50  0000 C CNN
F 2 "" H 6700 5400 50  0001 C CNN
F 3 "" H 6700 5400 50  0001 C CNN
	1    6700 5400
	1    0    0    -1  
$EndComp
Text Label 2450 1900 0    50   ~ 0
Green
Text Label 3250 2150 0    50   ~ 0
Blue
Text Label 4600 4950 0    50   ~ 0
Yellow
Text Label 4450 5850 0    50   ~ 0
Red
Text Label 4300 6200 0    50   ~ 0
Blue
$Comp
L power:+5V #PWR013
U 1 1 5EF2EB6B
P 7950 4650
F 0 "#PWR013" H 7950 4500 50  0001 C CNN
F 1 "+5V" H 7965 4823 50  0000 C CNN
F 2 "" H 7950 4650 50  0001 C CNN
F 3 "" H 7950 4650 50  0001 C CNN
	1    7950 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5EF2F872
P 7950 5650
F 0 "#PWR014" H 7950 5400 50  0001 C CNN
F 1 "GND" H 7955 5477 50  0000 C CNN
F 2 "" H 7950 5650 50  0001 C CNN
F 3 "" H 7950 5650 50  0001 C CNN
	1    7950 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5EF37D31
P 3700 2350
F 0 "R3" H 3770 2396 50  0000 L CNN
F 1 "10K" H 3770 2305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3630 2350 50  0001 C CNN
F 3 "~" H 3700 2350 50  0001 C CNN
	1    3700 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 2050 3700 2200
Connection ~ 3700 2050
Wire Wire Line
	3700 2050 4650 2050
$Comp
L power:GND #PWR05
U 1 1 5EF37D3D
P 3700 3000
F 0 "#PWR05" H 3700 2750 50  0001 C CNN
F 1 "GND" H 3705 2827 50  0000 C CNN
F 2 "" H 3700 3000 50  0001 C CNN
F 3 "" H 3700 3000 50  0001 C CNN
	1    3700 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 2500 3700 3000
Wire Wire Line
	4450 5000 4800 5000
$Comp
L Device:C C4
U 1 1 5EF50B32
P 8650 5150
F 0 "C4" H 8765 5196 50  0000 L CNN
F 1 "100uF" H 8765 5105 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D4.0mm_H7.0mm_P1.50mm" H 8688 5000 50  0001 C CNN
F 3 "~" H 8650 5150 50  0001 C CNN
	1    8650 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5EF51493
P 9150 5150
F 0 "C5" H 9265 5196 50  0000 L CNN
F 1 "100nf" H 9265 5105 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 9188 5000 50  0001 C CNN
F 3 "~" H 9150 5150 50  0001 C CNN
	1    9150 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 4700 8650 5000
Wire Wire Line
	8650 4700 9150 4700
Wire Wire Line
	9150 4700 9150 5000
Connection ~ 8650 4700
Wire Wire Line
	7950 5600 8650 5600
Wire Wire Line
	8650 5600 8650 5300
Wire Wire Line
	8650 5600 9150 5600
Wire Wire Line
	9150 5600 9150 5300
Connection ~ 8650 5600
Wire Wire Line
	7950 4700 8650 4700
Wire Wire Line
	6750 1200 4550 1200
$Comp
L power:+5V #PWR012
U 1 1 5EEF8D86
P 7600 2900
F 0 "#PWR012" H 7600 2750 50  0001 C CNN
F 1 "+5V" H 7615 3073 50  0000 C CNN
F 2 "" H 7600 2900 50  0001 C CNN
F 3 "" H 7600 2900 50  0001 C CNN
	1    7600 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 3050 6500 3050
Wire Wire Line
	6750 2150 7000 2150
Connection ~ 6750 2150
Wire Wire Line
	6750 2150 6750 1200
Connection ~ 6500 3050
Wire Wire Line
	6500 3050 7000 3050
Wire Wire Line
	6500 3050 6500 2750
Wire Wire Line
	7000 2150 7000 2450
Wire Wire Line
	6500 2150 6750 2150
Wire Wire Line
	6500 2150 6500 2450
$Comp
L Device:C C3
U 1 1 5EF62E88
P 7000 2600
F 0 "C3" H 7115 2646 50  0000 L CNN
F 1 "100nf" H 7115 2555 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 7038 2450 50  0001 C CNN
F 3 "~" H 7000 2600 50  0001 C CNN
	1    7000 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5EF62E7E
P 6500 2600
F 0 "C2" H 6615 2646 50  0000 L CNN
F 1 "100uF" H 6615 2555 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D4.0mm_H7.0mm_P1.50mm" H 6538 2450 50  0001 C CNN
F 3 "~" H 6500 2600 50  0001 C CNN
	1    6500 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 3050 7000 2750
Wire Wire Line
	7000 3050 7600 3050
Wire Wire Line
	7600 3050 7600 2900
Connection ~ 7000 3050
Wire Wire Line
	4550 1200 4550 1750
Wire Wire Line
	4550 1200 4200 1200
Wire Wire Line
	4200 1200 4200 1400
Connection ~ 4550 1200
Wire Wire Line
	4550 1750 4650 1750
$Comp
L 74xx:74LS04 U2
U 1 1 5EF82ABC
P 5100 5000
F 0 "U2" H 5100 5317 50  0000 C CNN
F 1 "74LS04" H 5100 5226 50  0000 C CNN
F 2 "" H 5100 5000 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS04" H 5100 5000 50  0001 C CNN
	1    5100 5000
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74LS04 U2
U 2 1 5EF881B4
P 5950 5000
F 0 "U2" H 5950 5317 50  0000 C CNN
F 1 "74LS04" H 5950 5226 50  0000 C CNN
F 2 "" H 5950 5000 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS04" H 5950 5000 50  0001 C CNN
	2    5950 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 5000 6500 5000
$Comp
L 74xx:74LS04 U2
U 7 1 5EF8C800
P 7950 5150
F 0 "U2" H 8180 5196 50  0000 L CNN
F 1 "74LS04" H 8180 5105 50  0000 L CNN
F 2 "" H 7950 5150 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS04" H 7950 5150 50  0001 C CNN
	7    7950 5150
	1    0    0    -1  
$EndComp
$EndSCHEMATC
