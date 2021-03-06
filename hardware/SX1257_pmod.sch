EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "SX1257 PMOD"
Date "2019-05-07"
Rev "RevB"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	7450 3300 7450 3200
Wire Wire Line
	7550 3300 7550 3200
Wire Wire Line
	7650 3300 7650 3200
Wire Wire Line
	7750 3300 7750 3200
Wire Wire Line
	7850 3300 7850 3200
Wire Wire Line
	7950 3300 7950 3200
Wire Wire Line
	7450 3300 7550 3300
Connection ~ 7550 3300
Wire Wire Line
	7550 3300 7650 3300
Connection ~ 7650 3300
Wire Wire Line
	7650 3300 7750 3300
Connection ~ 7750 3300
Wire Wire Line
	7750 3300 7850 3300
Connection ~ 7850 3300
Wire Wire Line
	7850 3300 7950 3300
$Comp
L power:GND #PWR016
U 1 1 5C2A85FC
P 7450 3400
F 0 "#PWR016" H 7450 3150 50  0001 C CNN
F 1 "GND" H 7455 3227 50  0000 C CNN
F 2 "" H 7450 3400 50  0001 C CNN
F 3 "" H 7450 3400 50  0001 C CNN
	1    7450 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 3400 7450 3300
Connection ~ 7450 3300
$Comp
L SX1257:SX1257 U1
U 1 1 5C2A87F4
P 7800 2250
F 0 "U1" H 8300 1250 50  0000 C CNN
F 1 "SX1257" H 8400 1350 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-32-1EP_5x5mm_P0.5mm_EP3.1x3.1mm_ThermalVias" H 8350 3000 50  0001 C CNN
F 3 "$(KIPRJMOD)/../datasheet/DS_SX1257_V1.2.pdf" H 8350 3000 50  0001 C CNN
	1    7800 2250
	1    0    0    -1  
$EndComp
Text Label 1900 1550 0    50   ~ 0
PMOD_5
Wire Wire Line
	1700 1550 1900 1550
Text Label 1900 1450 0    50   ~ 0
PMOD_6
Wire Wire Line
	1700 1450 1900 1450
Text Label 1900 1350 0    50   ~ 0
PMOD_7
Wire Wire Line
	1700 1350 1900 1350
Text Label 1900 1250 0    50   ~ 0
PMOD_8
Wire Wire Line
	1700 1250 1900 1250
Text Label 1000 1550 2    50   ~ 0
PMOD_1
Wire Wire Line
	1200 1550 1000 1550
Text Label 1000 1450 2    50   ~ 0
PMOD_2
Wire Wire Line
	1200 1450 1000 1450
Text Label 1000 1350 2    50   ~ 0
PMOD_3
Wire Wire Line
	1200 1350 1000 1350
Text Label 1000 1250 2    50   ~ 0
PMOD_4
Wire Wire Line
	1200 1250 1000 1250
$Comp
L Connector_Generic:Conn_02x06_Odd_Even J1
U 1 1 5C2AB4BF
P 1400 1250
F 0 "J1" H 1450 1667 50  0000 C CNN
F 1 "Conn_02x06_Odd_Even" H 1450 1576 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x06_P2.54mm_Horizontal" H 1400 1250 50  0001 C CNN
F 3 "~" H 1400 1250 50  0001 C CNN
	1    1400 1250
	1    0    0    -1  
$EndComp
Text Label 1000 1850 2    50   ~ 0
PMOD_1
Text Label 1000 1950 2    50   ~ 0
PMOD_2
Text Label 1000 2050 2    50   ~ 0
PMOD_3
Wire Wire Line
	6750 2400 6550 2400
Wire Wire Line
	6750 2500 6550 2500
Wire Wire Line
	6750 2700 6550 2700
Wire Wire Line
	6750 2800 6550 2800
Text Label 6550 2100 2    50   ~ 0
CLK_OUT
Text Label 6550 2200 2    50   ~ 0
CLK_IN
Text Label 1000 2150 2    50   ~ 0
PMOD_4
Text Label 1900 2550 0    50   ~ 0
CLK_IN
Wire Wire Line
	6750 1800 6550 1800
Wire Wire Line
	6750 1900 6550 1900
Text Label 6550 1800 2    50   ~ 0
XTA
Text Label 6550 1900 2    50   ~ 0
XTB
Text Label 7000 4500 2    50   ~ 0
XTA
Text Label 7900 4500 0    50   ~ 0
XTB
$Comp
L Device:Crystal_GND24 Y1
U 1 1 5C2BAF22
P 7450 4500
F 0 "Y1" H 7500 4700 50  0000 L CNN
F 1 "36MHz 10ppm" H 7500 4800 50  0000 L CNN
F 2 "extra:Crystal_SMD_1612-4Pin_1.6x1.2mm" H 7450 4500 50  0001 C CNN
F 3 "https://eu.mouser.com/datasheet/2/3/ABM12W-1107622.pdf" H 7450 4500 50  0001 C CNN
F 4 "815-12W36-6B1UT" H 7450 4500 50  0001 C CNN "Mouser"
F 5 "https://eu.mouser.com/ProductDetail/ABRACON/ABM12W-360000MHZ-6-B1U-T3?qs=%2Fha2pyFaduiSskzRnk5YszYp%2FYQbyNEaqk3XaFpHyq5uanwnbHUz6heQId99%2FXtKk4c7I0zJw7I%3D" H 7450 4500 50  0001 C CNN "MouserURL"
F 6 "ABM12W-36.0000MHZ-6-B1U-T3" H 7450 4500 50  0001 C CNN "MfrNo"
	1    7450 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5C2ECB2D
P 7450 5050
F 0 "#PWR018" H 7450 4800 50  0001 C CNN
F 1 "GND" H 7455 4877 50  0000 C CNN
F 2 "" H 7450 5050 50  0001 C CNN
F 3 "" H 7450 5050 50  0001 C CNN
	1    7450 5050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5C2ECB42
P 7450 4100
F 0 "#PWR017" H 7450 3850 50  0001 C CNN
F 1 "GND" H 7455 3927 50  0000 C CNN
F 2 "" H 7450 4100 50  0001 C CNN
F 3 "" H 7450 4100 50  0001 C CNN
	1    7450 4100
	-1   0    0    1   
$EndComp
Wire Wire Line
	7450 4300 7450 4100
$Comp
L Device:C C18
U 1 1 5C2ED7C2
P 7100 4700
F 0 "C18" H 7215 4746 50  0000 L CNN
F 1 "16p" H 7215 4655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7138 4550 50  0001 C CNN
F 3 "~" H 7100 4700 50  0001 C CNN
	1    7100 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 4500 7100 4500
Wire Wire Line
	7100 4550 7100 4500
Connection ~ 7100 4500
Wire Wire Line
	7100 4500 7300 4500
$Comp
L Device:C C19
U 1 1 5C2F09E0
P 7800 4700
F 0 "C19" H 7915 4746 50  0000 L CNN
F 1 "16p" H 7915 4655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7838 4550 50  0001 C CNN
F 3 "~" H 7800 4700 50  0001 C CNN
	1    7800 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 4500 7800 4500
Wire Wire Line
	7800 4550 7800 4500
Connection ~ 7800 4500
Wire Wire Line
	7800 4500 7900 4500
$Comp
L power:+3V3 #PWR015
U 1 1 5C2FAC67
P 8000 1000
F 0 "#PWR015" H 8000 850 50  0001 C CNN
F 1 "+3V3" H 7900 1150 50  0000 L CNN
F 2 "" H 8000 1000 50  0001 C CNN
F 3 "" H 8000 1000 50  0001 C CNN
	1    8000 1000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7900 1300 7900 1200
Wire Wire Line
	8000 1300 8000 1200
Wire Wire Line
	8100 1300 8100 1200
Wire Wire Line
	8100 1200 8000 1200
Connection ~ 8000 1200
Wire Wire Line
	8000 1200 7900 1200
Text Label 1900 1050 0    50   ~ 0
PMOD_3V3
Wire Wire Line
	1700 1050 1900 1050
Text Label 1000 1050 2    50   ~ 0
PMOD_3V3
Wire Wire Line
	1000 1050 1200 1050
Text Label 1000 3150 2    50   ~ 0
PMOD_3V3
$Comp
L Device:Ferrite_Bead FB1
U 1 1 5C30A1D4
P 1450 3150
F 0 "FB1" V 1550 3250 50  0000 C CNN
F 1 "Ferrite_Bead" V 1250 3100 50  0001 C CNN
F 2 "Inductor_SMD:L_0603_1608Metric" V 1380 3150 50  0001 C CNN
F 3 "~" H 1450 3150 50  0001 C CNN
	1    1450 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	1000 3150 1300 3150
$Comp
L power:+3V3 #PWR03
U 1 1 5C30B964
P 2050 3100
F 0 "#PWR03" H 2050 2950 50  0001 C CNN
F 1 "+3V3" H 1950 3250 50  0000 L CNN
F 2 "" H 2050 3100 50  0001 C CNN
F 3 "" H 2050 3100 50  0001 C CNN
	1    2050 3100
	-1   0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5C3103FC
P 1900 3100
F 0 "#FLG01" H 1900 3175 50  0001 C CNN
F 1 "PWR_FLAG" H 1900 3274 50  0001 C CNN
F 2 "" H 1900 3100 50  0001 C CNN
F 3 "~" H 1900 3100 50  0001 C CNN
	1    1900 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 3100 1900 3150
Connection ~ 1900 3150
Wire Wire Line
	2050 3150 2050 3100
Wire Wire Line
	1900 3150 2050 3150
Text Label 8850 2100 0    50   ~ 0
DIO0
Text Label 8850 2200 0    50   ~ 0
DIO1
Text Label 8850 2300 0    50   ~ 0
DIO2
Text Label 8850 2400 0    50   ~ 0
DIO3
$Comp
L Connector:TestPoint TP5
U 1 1 5C32BD15
P 9200 2300
F 0 "TP5" V 9200 2500 50  0000 L CNN
F 1 "TestPoint" V 9200 2750 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 9400 2300 50  0001 C CNN
F 3 "~" H 9400 2300 50  0001 C CNN
	1    9200 2300
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP6
U 1 1 5C32DD47
P 9200 2400
F 0 "TP6" V 9200 2600 50  0000 L CNN
F 1 "TestPoint" V 9200 2850 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 9400 2400 50  0001 C CNN
F 3 "~" H 9400 2400 50  0001 C CNN
	1    9200 2400
	0    1    1    0   
$EndComp
$Comp
L Device:C C6
U 1 1 5C3391FA
P 3200 1350
F 0 "C6" H 3200 1450 50  0000 L CNN
F 1 "100n" H 3200 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3238 1200 50  0001 C CNN
F 3 "~" H 3200 1350 50  0001 C CNN
	1    3200 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5C3393B3
P 5350 1350
F 0 "C9" H 5350 1450 50  0000 L CNN
F 1 "100n" H 5350 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5388 1200 50  0001 C CNN
F 3 "~" H 5350 1350 50  0001 C CNN
	1    5350 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 5C33942E
P 5600 1350
F 0 "C10" H 5600 1450 50  0000 L CNN
F 1 "100n" H 5600 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5638 1200 50  0001 C CNN
F 3 "~" H 5600 1350 50  0001 C CNN
	1    5600 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 5C339464
P 5850 1350
F 0 "C11" H 5850 1450 50  0000 L CNN
F 1 "100n" H 5850 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5888 1200 50  0001 C CNN
F 3 "~" H 5850 1350 50  0001 C CNN
	1    5850 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 5C339591
P 3500 1350
F 0 "C14" H 3500 1450 50  0000 L CNN
F 1 "1u" H 3500 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3538 1200 50  0001 C CNN
F 3 "~" H 3500 1350 50  0001 C CNN
	1    3500 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1200 3200 1100
Wire Wire Line
	3500 1200 3500 1100
Connection ~ 3200 1100
Wire Wire Line
	3200 1100 3200 1000
$Comp
L power:+3V3 #PWR07
U 1 1 5C350038
P 3200 1000
F 0 "#PWR07" H 3200 850 50  0001 C CNN
F 1 "+3V3" H 3100 1150 50  0000 L CNN
F 2 "" H 3200 1000 50  0001 C CNN
F 3 "" H 3200 1000 50  0001 C CNN
	1    3200 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1500 3200 1600
Wire Wire Line
	3500 1500 3500 1600
Connection ~ 3200 1600
Wire Wire Line
	3200 1600 3200 1700
$Comp
L power:GND #PWR08
U 1 1 5C3661BF
P 3200 1700
F 0 "#PWR08" H 3200 1450 50  0001 C CNN
F 1 "GND" H 3205 1527 50  0000 C CNN
F 2 "" H 3200 1700 50  0001 C CNN
F 3 "" H 3200 1700 50  0001 C CNN
	1    3200 1700
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5C3953E6
P 1900 3450
F 0 "#FLG02" H 1900 3525 50  0001 C CNN
F 1 "PWR_FLAG" H 1900 3624 50  0001 C CNN
F 2 "" H 1900 3450 50  0001 C CNN
F 3 "~" H 1900 3450 50  0001 C CNN
	1    1900 3450
	-1   0    0    1   
$EndComp
Wire Wire Line
	8650 2600 8850 2600
Text Label 8850 2600 0    50   ~ 0
RF_IN
Text Label 8850 2800 0    50   ~ 0
RF_ON
Text Label 8850 2900 0    50   ~ 0
RF_OP
Wire Wire Line
	8850 2800 8650 2800
Wire Wire Line
	8850 2900 8650 2900
Text Label 1000 3950 2    50   ~ 0
RF_IN
Text Label 1050 5750 2    50   ~ 0
RF_ON
Text Label 1050 5300 2    50   ~ 0
RF_OP
$Comp
L Device:C C7
U 1 1 5C3CA991
P 3250 4200
F 0 "C7" H 3250 4300 50  0000 L CNN
F 1 "5.6p" H 3250 4100 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3288 4050 50  0001 C CNN
F 3 "~" H 3250 4200 50  0001 C CNN
	1    3250 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:L L3
U 1 1 5C3CAB4E
P 2850 4200
F 0 "L3" H 2903 4246 50  0000 L CNN
F 1 "12n" H 2903 4155 50  0000 L CNN
F 2 "Inductor_SMD:L_0402_1005Metric" H 2850 4200 50  0001 C CNN
F 3 "~" H 2850 4200 50  0001 C CNN
	1    2850 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 4050 3250 3950
Wire Wire Line
	3250 3950 2850 3950
$Comp
L power:GND #PWR09
U 1 1 5C3D12E0
P 3250 4450
F 0 "#PWR09" H 3250 4200 50  0001 C CNN
F 1 "GND" H 3255 4277 50  0000 C CNN
F 2 "" H 3250 4450 50  0001 C CNN
F 3 "" H 3250 4450 50  0001 C CNN
	1    3250 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 4450 3250 4350
Wire Wire Line
	2850 4050 2850 3950
$Comp
L power:GND #PWR05
U 1 1 5C3DB0F0
P 2850 4450
F 0 "#PWR05" H 2850 4200 50  0001 C CNN
F 1 "GND" H 2855 4277 50  0000 C CNN
F 2 "" H 2850 4450 50  0001 C CNN
F 3 "" H 2850 4450 50  0001 C CNN
	1    2850 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 4450 2850 4350
$Comp
L Device:C C4
U 1 1 5C3DE88B
P 2300 3950
F 0 "C4" V 2150 3950 50  0000 C CNN
F 1 "10p" V 2450 3950 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2338 3800 50  0001 C CNN
F 3 "~" H 2300 3950 50  0001 C CNN
	1    2300 3950
	0    1    1    0   
$EndComp
Wire Wire Line
	2450 3950 2850 3950
Connection ~ 2850 3950
$Comp
L Device:L L1
U 1 1 5C3E20E9
P 1800 3950
F 0 "L1" V 1990 3950 50  0000 C CNN
F 1 "15nH" V 1899 3950 50  0000 C CNN
F 2 "Inductor_SMD:L_0402_1005Metric" H 1800 3950 50  0001 C CNN
F 3 "~" H 1800 3950 50  0001 C CNN
	1    1800 3950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1950 3950 2150 3950
$Comp
L Device:C C1
U 1 1 5C3E5AA3
P 1450 4200
F 0 "C1" H 1450 4300 50  0000 L CNN
F 1 "2.7p" H 1450 4100 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1488 4050 50  0001 C CNN
F 3 "~" H 1450 4200 50  0001 C CNN
	1    1450 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 3950 1450 3950
Wire Wire Line
	1450 4050 1450 3950
Connection ~ 1450 3950
Wire Wire Line
	1450 3950 1650 3950
$Comp
L power:GND #PWR01
U 1 1 5C3ECF8F
P 1450 4450
F 0 "#PWR01" H 1450 4200 50  0001 C CNN
F 1 "GND" H 1455 4277 50  0000 C CNN
F 2 "" H 1450 4450 50  0001 C CNN
F 3 "" H 1450 4450 50  0001 C CNN
	1    1450 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 4450 1450 4350
$Comp
L power:GND #PWR011
U 1 1 5C3F0EB0
P 4400 5800
F 0 "#PWR011" H 4400 5550 50  0001 C CNN
F 1 "GND" H 4405 5627 50  0000 C CNN
F 2 "" H 4400 5800 50  0001 C CNN
F 3 "" H 4400 5800 50  0001 C CNN
	1    4400 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 5800 4400 5500
$Comp
L Device:C C8
U 1 1 5C3F0ED9
P 3300 5300
F 0 "C8" V 3150 5300 50  0000 C CNN
F 1 "10p" V 3450 5300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3338 5150 50  0001 C CNN
F 3 "~" H 3300 5300 50  0001 C CNN
	1    3300 5300
	0    1    1    0   
$EndComp
$Comp
L Device:L L2
U 1 1 5C3F0EE2
P 2350 5300
F 0 "L2" V 2540 5300 50  0000 C CNN
F 1 "10n" V 2449 5300 50  0000 C CNN
F 2 "Inductor_SMD:L_0402_1005Metric" H 2350 5300 50  0001 C CNN
F 3 "~" H 2350 5300 50  0001 C CNN
	1    2350 5300
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C3
U 1 1 5C3F0EEA
P 1450 5950
F 0 "C3" H 1450 6050 50  0000 L CNN
F 1 "33p" H 1450 5850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1488 5800 50  0001 C CNN
F 3 "~" H 1450 5950 50  0001 C CNN
	1    1450 5950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5C3F0EF5
P 1450 6200
F 0 "#PWR02" H 1450 5950 50  0001 C CNN
F 1 "GND" H 1455 6027 50  0000 C CNN
F 2 "" H 1450 6200 50  0001 C CNN
F 3 "" H 1450 6200 50  0001 C CNN
	1    1450 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 6200 1450 6100
$Comp
L Device:C C5
U 1 1 5C3FF2E6
P 2950 5550
F 0 "C5" H 2950 5650 50  0000 L CNN
F 1 "3.3p" H 2950 5450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2988 5400 50  0001 C CNN
F 3 "~" H 2950 5550 50  0001 C CNN
	1    2950 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 5400 2950 5300
Wire Wire Line
	2950 5300 3150 5300
Wire Wire Line
	2500 5300 2950 5300
Connection ~ 2950 5300
Text Notes 1900 5400 0    50   ~ 0
Check if 10mH or 10nH!
Wire Wire Line
	1050 5300 1450 5300
Wire Wire Line
	1450 5800 1450 5750
$Comp
L power:GND #PWR06
U 1 1 5C429A86
P 2950 5800
F 0 "#PWR06" H 2950 5550 50  0001 C CNN
F 1 "GND" H 2955 5627 50  0000 C CNN
F 2 "" H 2950 5800 50  0001 C CNN
F 3 "" H 2950 5800 50  0001 C CNN
	1    2950 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 5800 2950 5700
$Comp
L Device:C C2
U 1 1 5C43DA32
P 1450 5550
F 0 "C2" H 1450 5650 50  0000 L CNN
F 1 "3.3p" H 1450 5450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1488 5400 50  0001 C CNN
F 3 "~" H 1450 5550 50  0001 C CNN
	1    1450 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 5400 1450 5300
Wire Wire Line
	1450 5300 2200 5300
Connection ~ 1450 5300
Wire Wire Line
	1050 5750 1450 5750
Connection ~ 1450 5750
Wire Wire Line
	1450 5750 1450 5700
$Comp
L Device:C C15
U 1 1 5C478384
P 3700 1350
F 0 "C15" H 3700 1450 50  0000 L CNN
F 1 "1u" H 3700 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3738 1200 50  0001 C CNN
F 3 "~" H 3700 1350 50  0001 C CNN
	1    3700 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 1100 3700 1100
Wire Wire Line
	3700 1100 3700 1200
Connection ~ 3500 1100
Wire Wire Line
	3700 1500 3700 1600
Wire Wire Line
	3700 1600 3500 1600
Connection ~ 3500 1600
$Comp
L Device:C C17
U 1 1 5C4821AE
P 4200 1350
F 0 "C17" H 4200 1450 50  0000 L CNN
F 1 "10u" H 4200 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4238 1200 50  0001 C CNN
F 3 "~" H 4200 1350 50  0001 C CNN
	1    4200 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 1200 5100 1100
$Comp
L Device:C C16
U 1 1 5C4918C3
P 3900 1350
F 0 "C16" H 3900 1450 50  0000 L CNN
F 1 "1u" H 3900 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3938 1200 50  0001 C CNN
F 3 "~" H 3900 1350 50  0001 C CNN
	1    3900 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 1200 3900 1100
Wire Wire Line
	3900 1100 3700 1100
Connection ~ 3700 1100
Wire Wire Line
	3700 1600 3900 1600
Connection ~ 3700 1600
Wire Wire Line
	3900 1600 3900 1500
Wire Notes Line
	9300 2000 9600 2000
Wire Notes Line
	9600 2000 9600 2500
Wire Notes Line
	9600 2500 9300 2500
Text Notes 9700 2300 0    50   ~ 0
Registers are available\nthrough SPI.
$Comp
L Connector:Conn_Coaxial J2
U 1 1 5C4EEF69
P 4400 3950
F 0 "J2" H 4499 3926 50  0000 L CNN
F 1 "SMA_F_RX" H 4499 3835 50  0000 L CNN
F 2 "extra:CON-SMA-EDGE" H 4400 3950 50  0001 C CNN
F 3 "https://eu.mouser.com/datasheet/2/398/EMPCB.SMAFSTJ.B.HT-19975.pdf" H 4400 3950 50  0001 C CNN
F 4 "960-EMPCB.SMAFSTJBHT" H 4400 3950 50  0001 C CNN "Mouser"
F 5 "EMPCB.SMAFSTJ.B.HT" H 4400 3950 50  0001 C CNN "MfrNo"
F 6 "https://lcsc.com/product-detail/_RFsister-ANKX01-0093_C129604.html" H 4400 3950 50  0001 C CNN "LCSC"
	1    4400 3950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5C4EEF70
P 4400 4450
F 0 "#PWR010" H 4400 4200 50  0001 C CNN
F 1 "GND" H 4405 4277 50  0000 C CNN
F 2 "" H 4400 4450 50  0001 C CNN
F 3 "" H 4400 4450 50  0001 C CNN
	1    4400 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 4450 4400 4150
Connection ~ 3250 3950
Text Label 1000 1150 2    50   ~ 0
PMOD_GND
Wire Wire Line
	1000 1150 1200 1150
Text Label 1900 1150 0    50   ~ 0
PMOD_GND
Wire Wire Line
	1700 1150 1900 1150
Text Label 1000 3350 2    50   ~ 0
PMOD_GND
$Comp
L power:GND #PWR04
U 1 1 5C3092FD
P 2050 3450
F 0 "#PWR04" H 2050 3200 50  0001 C CNN
F 1 "GND" H 2055 3277 50  0000 C CNN
F 2 "" H 2050 3450 50  0001 C CNN
F 3 "" H 2050 3450 50  0001 C CNN
	1    2050 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 3350 2050 3450
Wire Wire Line
	1900 3450 1900 3350
Connection ~ 1900 3350
Wire Wire Line
	1900 3350 2050 3350
Wire Wire Line
	6550 1600 6750 1600
Wire Wire Line
	6550 2100 6750 2100
Wire Wire Line
	6550 2200 6750 2200
Text Label 8850 1600 0    50   ~ 0
SPI_SCK
Text Label 13250 2200 0    50   ~ 0
SPI_SCK
Text Label 8850 1700 0    50   ~ 0
SPI_MISO
Text Label 8850 1800 0    50   ~ 0
SPI_MOSI
Text Label 8850 1900 0    50   ~ 0
SPI_NSS
Text Label 13250 2000 0    50   ~ 0
SPI_MISO
Text Label 13250 2100 0    50   ~ 0
SPI_MOSI
Text Label 13250 1500 0    50   ~ 0
SPI_NSS
Text Label 1900 2650 0    50   ~ 0
CLK_OUT
Text Label 1900 2350 0    50   ~ 0
I2C_SCL
Text Label 1900 2450 0    50   ~ 0
I2C_SDA
$Comp
L SC18IS602B:SC18IS602B U2
U 1 1 5C309C14
P 12400 1500
F 0 "U2" H 12900 550 50  0000 C CNN
F 1 "SC18IS602B" H 12700 1650 50  0000 C CNN
F 2 "Package_SO:TSSOP-16_4.4x5mm_P0.65mm" H 12800 500 50  0001 C CNN
F 3 "" H 12800 500 50  0001 C CNN
	1    12400 1500
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP1
U 1 1 5C309F67
P 9200 1600
F 0 "TP1" V 9200 1800 50  0000 L CNN
F 1 "TestPoint" V 9200 2050 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 9400 1600 50  0001 C CNN
F 3 "~" H 9400 1600 50  0001 C CNN
	1    9200 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	8650 1600 9200 1600
$Comp
L Connector:TestPoint TP2
U 1 1 5C3239F0
P 9200 1700
F 0 "TP2" V 9200 1900 50  0000 L CNN
F 1 "TestPoint" V 9200 2150 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 9400 1700 50  0001 C CNN
F 3 "~" H 9400 1700 50  0001 C CNN
	1    9200 1700
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP3
U 1 1 5C323A5C
P 9200 1800
F 0 "TP3" V 9200 2000 50  0000 L CNN
F 1 "TestPoint" V 9200 2250 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 9400 1800 50  0001 C CNN
F 3 "~" H 9400 1800 50  0001 C CNN
	1    9200 1800
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP4
U 1 1 5C323ACA
P 9200 1900
F 0 "TP4" V 9200 2100 50  0000 L CNN
F 1 "TestPoint" V 9200 2350 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 9400 1900 50  0001 C CNN
F 3 "~" H 9400 1900 50  0001 C CNN
	1    9200 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	8650 1900 9200 1900
Wire Wire Line
	8650 1800 9200 1800
Wire Wire Line
	8650 1700 9200 1700
Wire Wire Line
	8650 2100 8850 2100
Wire Wire Line
	8650 2200 8850 2200
Wire Wire Line
	8650 2300 9200 2300
Wire Wire Line
	8650 2400 9200 2400
Wire Wire Line
	13050 2000 13250 2000
Wire Wire Line
	13050 2100 13250 2100
Wire Wire Line
	13050 2200 13250 2200
$Comp
L Connector:TestPoint TP7
U 1 1 5C34614B
P 13600 1600
F 0 "TP7" V 13600 1800 50  0000 L CNN
F 1 "TestPoint" V 13600 2050 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 13800 1600 50  0001 C CNN
F 3 "~" H 13800 1600 50  0001 C CNN
	1    13600 1600
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP8
U 1 1 5C346277
P 13600 1700
F 0 "TP8" V 13600 1900 50  0000 L CNN
F 1 "TestPoint" V 13600 2150 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 13800 1700 50  0001 C CNN
F 3 "~" H 13800 1700 50  0001 C CNN
	1    13600 1700
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP9
U 1 1 5C3462EB
P 13600 1800
F 0 "TP9" V 13600 2000 50  0000 L CNN
F 1 "TestPoint" V 13600 2250 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 13800 1800 50  0001 C CNN
F 3 "~" H 13800 1800 50  0001 C CNN
	1    13600 1800
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR022
U 1 1 5C34660D
P 12400 1200
F 0 "#PWR022" H 12400 1050 50  0001 C CNN
F 1 "+3V3" H 12300 1350 50  0000 L CNN
F 2 "" H 12400 1200 50  0001 C CNN
F 3 "" H 12400 1200 50  0001 C CNN
	1    12400 1200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	12400 1200 12400 1300
$Comp
L power:GND #PWR023
U 1 1 5C35839C
P 12400 2600
F 0 "#PWR023" H 12400 2350 50  0001 C CNN
F 1 "GND" H 12405 2427 50  0000 C CNN
F 2 "" H 12400 2600 50  0001 C CNN
F 3 "" H 12400 2600 50  0001 C CNN
	1    12400 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	12400 2500 12400 2600
$Comp
L Device:C C20
U 1 1 5C361716
P 14200 1600
F 0 "C20" H 14200 1700 50  0000 L CNN
F 1 "100n" H 14200 1500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 14238 1450 50  0001 C CNN
F 3 "~" H 14200 1600 50  0001 C CNN
	1    14200 1600
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR024
U 1 1 5C361728
P 14200 1250
F 0 "#PWR024" H 14200 1100 50  0001 C CNN
F 1 "+3V3" H 14100 1400 50  0000 L CNN
F 2 "" H 14200 1250 50  0001 C CNN
F 3 "" H 14200 1250 50  0001 C CNN
	1    14200 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 5C361732
P 14200 1950
F 0 "#PWR025" H 14200 1700 50  0001 C CNN
F 1 "GND" H 14205 1777 50  0000 C CNN
F 2 "" H 14200 1950 50  0001 C CNN
F 3 "" H 14200 1950 50  0001 C CNN
	1    14200 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	14200 1250 14200 1450
Wire Wire Line
	14200 1750 14200 1950
Wire Wire Line
	11750 2100 11650 2100
Wire Wire Line
	11750 2200 11650 2200
Wire Wire Line
	11750 2300 11650 2300
Wire Wire Line
	11650 2100 11650 2200
Connection ~ 11650 2200
Wire Wire Line
	11650 2200 11650 2300
Connection ~ 11650 2300
Wire Wire Line
	11650 2300 11650 2400
$Comp
L power:GND #PWR021
U 1 1 5C386252
P 11650 2400
F 0 "#PWR021" H 11650 2150 50  0001 C CNN
F 1 "GND" H 11655 2227 50  0000 C CNN
F 2 "" H 11650 2400 50  0001 C CNN
F 3 "" H 11650 2400 50  0001 C CNN
	1    11650 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	11750 1900 11550 1900
Text Label 11550 1800 2    50   ~ 0
I2C_SCL
Text Label 11550 1700 2    50   ~ 0
I2C_SDA
$Comp
L Device:R R9
U 1 1 5C3C96EE
P 11550 1250
F 0 "R9" H 11620 1296 50  0000 L CNN
F 1 "10k" H 11620 1205 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 11480 1250 50  0001 C CNN
F 3 "~" H 11550 1250 50  0001 C CNN
	1    11550 1250
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR020
U 1 1 5C3C9786
P 11550 1000
F 0 "#PWR020" H 11550 850 50  0001 C CNN
F 1 "+3V3" H 11450 1150 50  0000 L CNN
F 2 "" H 11550 1000 50  0001 C CNN
F 3 "" H 11550 1000 50  0001 C CNN
	1    11550 1000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	11550 1000 11550 1100
Wire Wire Line
	11550 1400 11550 1500
Wire Wire Line
	11550 1500 11750 1500
Text Label 11550 1900 2    50   ~ 0
I2C_INT
Text Notes 550  700  0    50   ~ 0
Icebreaker compatibility:\nPMOD_8 connects to P1A10 which is a global/clock input pin (IOB_3B_G6)
Text Label 3550 2350 0    50   ~ 0
DIO0
Text Label 3550 2550 0    50   ~ 0
DIO1
Wire Wire Line
	3550 2350 3900 2350
Wire Wire Line
	3550 2550 3900 2550
$Comp
L Device:R R7
U 1 1 5C416A75
P 4050 2350
F 0 "R7" V 4100 2500 50  0000 C CNN
F 1 "2k" V 4050 2350 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3980 2350 50  0001 C CNN
F 3 "~" H 4050 2350 50  0001 C CNN
	1    4050 2350
	0    1    1    0   
$EndComp
$Comp
L Device:LED D1
U 1 1 5C416E17
P 4450 2350
F 0 "D1" H 4350 2400 50  0000 C CNN
F 1 "LED" H 4300 2400 50  0001 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 4450 2350 50  0001 C CNN
F 3 "~" H 4450 2350 50  0001 C CNN
	1    4450 2350
	-1   0    0    1   
$EndComp
Wire Wire Line
	4300 2350 4200 2350
$Comp
L power:GND #PWR012
U 1 1 5C421A6E
P 4700 2350
F 0 "#PWR012" H 4700 2100 50  0001 C CNN
F 1 "GND" V 4705 2222 50  0000 R CNN
F 2 "" H 4700 2350 50  0001 C CNN
F 3 "" H 4700 2350 50  0001 C CNN
	1    4700 2350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4600 2350 4700 2350
$Comp
L Device:R R8
U 1 1 5C45728B
P 4050 2550
F 0 "R8" V 4100 2700 50  0000 C CNN
F 1 "2k" V 4050 2550 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3980 2550 50  0001 C CNN
F 3 "~" H 4050 2550 50  0001 C CNN
	1    4050 2550
	0    1    1    0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5C457292
P 4450 2550
F 0 "D2" H 4350 2600 50  0000 C CNN
F 1 "LED" H 4300 2600 50  0001 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 4450 2550 50  0001 C CNN
F 3 "~" H 4450 2550 50  0001 C CNN
	1    4450 2550
	-1   0    0    1   
$EndComp
Wire Wire Line
	4300 2550 4200 2550
$Comp
L power:GND #PWR013
U 1 1 5C45729A
P 4700 2550
F 0 "#PWR013" H 4700 2300 50  0001 C CNN
F 1 "GND" V 4705 2422 50  0000 R CNN
F 2 "" H 4700 2550 50  0001 C CNN
F 3 "" H 4700 2550 50  0001 C CNN
	1    4700 2550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4600 2550 4700 2550
Wire Wire Line
	7450 4700 7450 5050
$Comp
L power:GND #PWR019
U 1 1 5C4E97F4
P 7800 5050
F 0 "#PWR019" H 7800 4800 50  0001 C CNN
F 1 "GND" H 7805 4877 50  0000 C CNN
F 2 "" H 7800 5050 50  0001 C CNN
F 3 "" H 7800 5050 50  0001 C CNN
	1    7800 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 4850 7800 5050
$Comp
L power:GND #PWR014
U 1 1 5C4F3792
P 7100 5050
F 0 "#PWR014" H 7100 4800 50  0001 C CNN
F 1 "GND" H 7105 4877 50  0000 C CNN
F 2 "" H 7100 5050 50  0001 C CNN
F 3 "" H 7100 5050 50  0001 C CNN
	1    7100 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 4850 7100 5050
Text Notes 3100 2350 0    50   ~ 0
pll_lock_rx
Text Notes 3100 2550 0    50   ~ 0
pll_lock_tx
Text Label 13250 1700 0    50   ~ 0
GPIO2
Text Label 13250 1800 0    50   ~ 0
GPIO3
Wire Wire Line
	13050 1600 13600 1600
Wire Wire Line
	13050 1800 13600 1800
Wire Wire Line
	13050 1700 13600 1700
Text Label 6550 1600 2    50   ~ 0
SX_RESET
Text Label 13250 1600 0    50   ~ 0
SX_RESET
Wire Wire Line
	1000 1850 1900 1850
Wire Wire Line
	1000 1950 1900 1950
Wire Wire Line
	1000 2050 1900 2050
Wire Wire Line
	1000 2150 1900 2150
Wire Wire Line
	1600 3150 1900 3150
Wire Wire Line
	1000 3350 1900 3350
Wire Wire Line
	13050 1500 13250 1500
Wire Wire Line
	3250 3950 4200 3950
Wire Wire Line
	3450 5300 4200 5300
$Comp
L Graphic:Logo_Open_Hardware_Large LOGO1
U 1 1 5C32CA22
P 11250 10600
F 0 "LOGO1" H 11250 11100 50  0001 C CNN
F 1 "Logo_Open_Hardware_Large" H 11250 10200 50  0001 C CNN
F 2 "extra:oshw_logo" H 11250 10600 50  0001 C CNN
F 3 "~" H 11250 10600 50  0001 C CNN
	1    11250 10600
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_Coaxial J3
U 1 1 5C3ECE8D
P 4400 5300
F 0 "J3" H 4499 5276 50  0000 L CNN
F 1 "SMA_F_TX" H 4499 5185 50  0000 L CNN
F 2 "extra:CON-SMA-EDGE" H 4400 5300 50  0001 C CNN
F 3 "https://eu.mouser.com/datasheet/2/398/EMPCB.SMAFSTJ.B.HT-19975.pdf" H 4400 5300 50  0001 C CNN
F 4 "960-EMPCB.SMAFSTJBHT" H 4400 5300 50  0001 C CNN "Mouser"
F 5 "EMPCB.SMAFSTJ.B.HT" H 4400 5300 50  0001 C CNN "MfrNo"
F 6 "https://lcsc.com/product-detail/_RFsister-ANKX01-0093_C129604.html" H 4400 5300 50  0001 C CNN "LCSC"
	1    4400 5300
	1    0    0    -1  
$EndComp
Text Label 1000 2350 2    50   ~ 0
PMOD_5
Text Label 1000 2450 2    50   ~ 0
PMOD_6
Text Label 1000 2550 2    50   ~ 0
PMOD_7
Text Label 1000 2650 2    50   ~ 0
PMOD_8
Wire Wire Line
	1000 2350 1900 2350
Wire Wire Line
	1000 2450 1900 2450
Wire Wire Line
	1000 2550 1900 2550
Wire Wire Line
	1000 2650 1900 2650
Text Label 1900 1850 0    50   ~ 0
I_IN
Text Label 1900 1950 0    50   ~ 0
Q_IN
Text Label 1900 2050 0    50   ~ 0
Q_OUT
Text Label 1900 2150 0    50   ~ 0
I_OUT
Text Label 6550 2400 2    50   ~ 0
I_IN
Text Label 6550 2500 2    50   ~ 0
Q_IN
Text Label 6550 2700 2    50   ~ 0
Q_OUT
Text Label 6550 2800 2    50   ~ 0
I_OUT
$Comp
L Device:R R2
U 1 1 5C809682
P 11150 1250
F 0 "R2" H 11220 1296 50  0000 L CNN
F 1 "2k2" H 11220 1205 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 11080 1250 50  0001 C CNN
F 3 "~" H 11150 1250 50  0001 C CNN
	1    11150 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5C8096F4
P 10850 1250
F 0 "R1" H 10920 1296 50  0000 L CNN
F 1 "2k2" H 10920 1205 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 10780 1250 50  0001 C CNN
F 3 "~" H 10850 1250 50  0001 C CNN
	1    10850 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	11150 1400 11150 1700
Wire Wire Line
	11150 1700 11750 1700
Wire Wire Line
	10850 1800 10850 1400
Wire Wire Line
	10850 1800 11750 1800
$Comp
L power:+3V3 #PWR0101
U 1 1 5C84AE58
P 11150 1000
F 0 "#PWR0101" H 11150 850 50  0001 C CNN
F 1 "+3V3" H 11050 1150 50  0000 L CNN
F 2 "" H 11150 1000 50  0001 C CNN
F 3 "" H 11150 1000 50  0001 C CNN
	1    11150 1000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	11150 1000 11150 1100
$Comp
L power:+3V3 #PWR0102
U 1 1 5C854300
P 10850 1000
F 0 "#PWR0102" H 10850 850 50  0001 C CNN
F 1 "+3V3" H 10750 1150 50  0000 L CNN
F 2 "" H 10850 1000 50  0001 C CNN
F 3 "" H 10850 1000 50  0001 C CNN
	1    10850 1000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10850 1000 10850 1100
NoConn ~ 11550 1900
Wire Wire Line
	8000 1200 8000 1000
Text Label 7400 1100 1    50   ~ 0
VR_PA
Wire Wire Line
	7400 1100 7400 1300
Text Label 5100 1100 1    50   ~ 0
VR_PA
Text Label 7500 1100 1    50   ~ 0
VR_ANA1
Text Label 7600 1100 1    50   ~ 0
VR_ANA2
Text Label 7700 1100 1    50   ~ 0
VR_DIG
Wire Wire Line
	7700 1100 7700 1300
Wire Wire Line
	7600 1300 7600 1100
Wire Wire Line
	7500 1100 7500 1300
Text Label 5350 1200 1    50   ~ 0
VR_ANA1
Text Label 5600 1200 1    50   ~ 0
VR_ANA2
Text Label 5850 1200 1    50   ~ 0
VR_DIG
$Comp
L power:GND #PWR0103
U 1 1 5E1737C3
P 5100 1700
F 0 "#PWR0103" H 5100 1450 50  0001 C CNN
F 1 "GND" H 5105 1527 50  0000 C CNN
F 2 "" H 5100 1700 50  0001 C CNN
F 3 "" H 5100 1700 50  0001 C CNN
	1    5100 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 1500 5100 1600
Wire Wire Line
	5100 1600 5350 1600
Wire Wire Line
	5350 1600 5350 1500
Connection ~ 5100 1600
Wire Wire Line
	5100 1600 5100 1700
Wire Wire Line
	5350 1600 5600 1600
Wire Wire Line
	5600 1600 5600 1500
Connection ~ 5350 1600
Wire Wire Line
	5600 1600 5850 1600
Wire Wire Line
	5850 1600 5850 1500
Connection ~ 5600 1600
$Comp
L Device:C C21
U 1 1 5E1AD80C
P 5100 1350
F 0 "C21" H 5100 1450 50  0000 L CNN
F 1 "10n" H 5100 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5138 1200 50  0001 C CNN
F 3 "~" H 5100 1350 50  0001 C CNN
	1    5100 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 1100 4200 1100
Wire Wire Line
	4200 1100 4200 1200
Connection ~ 3900 1100
Wire Wire Line
	3900 1600 4200 1600
Wire Wire Line
	4200 1600 4200 1500
Connection ~ 3900 1600
Wire Wire Line
	3200 1100 3500 1100
Wire Wire Line
	3200 1600 3500 1600
$Comp
L Si514:Si514 U3
U 1 1 5E37915E
P 12450 4650
F 0 "U3" H 12750 4300 50  0000 C CNN
F 1 "Si514" H 12700 5100 50  0000 C CNN
F 2 "extra:Oscillator_SMD_Silicon_Labs_3.2x5.0mm_P1.27mm-6pin_5.0x3.2mm" H 12000 4750 50  0001 C CNN
F 3 "" H 12000 4750 50  0001 C CNN
	1    12450 4650
	1    0    0    -1  
$EndComp
Text Label 11800 4550 2    50   ~ 0
I2C_SDA
Text Label 11800 4650 2    50   ~ 0
I2C_SCL
Wire Wire Line
	11800 4650 12000 4650
Wire Wire Line
	11800 4550 12000 4550
$Comp
L power:+3V3 #PWR0104
U 1 1 5E392DB5
P 12450 4150
F 0 "#PWR0104" H 12450 4000 50  0001 C CNN
F 1 "+3V3" H 12350 4300 50  0000 L CNN
F 2 "" H 12450 4150 50  0001 C CNN
F 3 "" H 12450 4150 50  0001 C CNN
	1    12450 4150
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5E393209
P 12450 5050
F 0 "#PWR0105" H 12450 4800 50  0001 C CNN
F 1 "GND" H 12455 4877 50  0000 C CNN
F 2 "" H 12450 5050 50  0001 C CNN
F 3 "" H 12450 5050 50  0001 C CNN
	1    12450 5050
	1    0    0    -1  
$EndComp
NoConn ~ 12900 4550
$Comp
L Device:R R3
U 1 1 5E39DBA9
P 13150 4650
F 0 "R3" V 12943 4650 50  0000 C CNN
F 1 "R" V 13034 4650 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 13080 4650 50  0001 C CNN
F 3 "~" H 13150 4650 50  0001 C CNN
	1    13150 4650
	0    1    1    0   
$EndComp
Wire Wire Line
	12900 4650 13000 4650
Text Label 13450 4650 0    50   ~ 0
XTB
Wire Wire Line
	13300 4650 13450 4650
$EndSCHEMATC
