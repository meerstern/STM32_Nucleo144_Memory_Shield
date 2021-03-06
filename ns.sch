EESchema Schematic File Version 4
LIBS:ns-cache
EELAYER 29 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Nucleo 144 Memory Shield"
Date "2020-04-23"
Rev "v1.0"
Comp "Crescent"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L NUCLEO-144:NUCLEO-144 A1
U 1 1 5EA85283
P 3325 3225
F 0 "A1" H 3209 5492 50  0000 C CNN
F 1 "NUCLEO-144" H 3209 5401 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x36_P2.54mm_Vertical" H 3225 5375 50  0001 L BNN
F 3 "" H 4775 3325 50  0001 L BNN
F 4 "Manufacturer Recommendation" H 4075 1125 50  0001 L BNN "フィールド4"
F 5 "STMicroelectronics" H 3325 5475 50  0001 L BNN "フィールド5"
	1    3325 3225
	1    0    0    -1  
$EndComp
$Comp
L NUCLEO-144:NUCLEO-144 A6
U 2 1 5EA88AA6
P 13300 3050
F 0 "A6" H 13300 5317 50  0000 C CNN
F 1 "NUCLEO-144" H 13300 5226 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x36_P2.54mm_Vertical" H 13200 5200 50  0001 L BNN
F 3 "" H 14750 3150 50  0001 L BNN
F 4 "Manufacturer Recommendation" H 14050 950 50  0001 L BNN "フィールド4"
F 5 "STMicroelectronics" H 13300 5300 50  0001 L BNN "フィールド5"
	2    13300 3050
	1    0    0    -1  
$EndComp
$Comp
L NUCLEO-144:NUCLEO-144 A2
U 3 1 5EA8D4EA
P 6500 1825
F 0 "A2" H 6525 2700 50  0000 C CNN
F 1 "NUCLEO-144" H 6525 2600 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x08_P2.54mm_Vertical" H 6400 3975 50  0001 L BNN
F 3 "" H 7950 1925 50  0001 L BNN
F 4 "Manufacturer Recommendation" H 7250 -275 50  0001 L BNN "フィールド4"
F 5 "STMicroelectronics" H 6500 4075 50  0001 L BNN "フィールド5"
	3    6500 1825
	1    0    0    -1  
$EndComp
$Comp
L NUCLEO-144:NUCLEO-144 A3
U 4 1 5EA911C8
P 6525 4375
F 0 "A3" H 6525 5542 50  0000 C CNN
F 1 "NUCLEO-144" H 6525 5451 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x15_P2.54mm_Vertical" H 6425 6525 50  0001 L BNN
F 3 "" H 7975 4475 50  0001 L BNN
F 4 "Manufacturer Recommendation" H 7275 2275 50  0001 L BNN "フィールド4"
F 5 "STMicroelectronics" H 6525 6625 50  0001 L BNN "フィールド5"
	4    6525 4375
	1    0    0    -1  
$EndComp
$Comp
L NUCLEO-144:NUCLEO-144 A4
U 5 1 5EA9ED5A
P 10175 1850
F 0 "A4" H 10200 2850 50  0000 C CNN
F 1 "NUCLEO-144" H 10225 2750 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x10_P2.54mm_Vertical" H 10075 4000 50  0001 L BNN
F 3 "" H 11625 1950 50  0001 L BNN
F 4 "Manufacturer Recommendation" H 10925 -250 50  0001 L BNN "フィールド4"
F 5 "STMicroelectronics" H 10175 4100 50  0001 L BNN "フィールド5"
	5    10175 1850
	1    0    0    -1  
$EndComp
$Comp
L NUCLEO-144:NUCLEO-144 A5
U 6 1 5EAA4796
P 9925 4275
F 0 "A5" H 9925 5642 50  0000 C CNN
F 1 "NUCLEO-144" H 9925 5551 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x17_P2.54mm_Vertical" H 9825 6425 50  0001 L BNN
F 3 "" H 11375 4375 50  0001 L BNN
F 4 "Manufacturer Recommendation" H 10675 2175 50  0001 L BNN "フィールド4"
F 5 "STMicroelectronics" H 9925 6525 50  0001 L BNN "フィールド5"
	6    9925 4275
	1    0    0    -1  
$EndComp
$Comp
L NAND_Flash:TSOP-48 U6
U 1 1 5EBC0AD6
P 14675 6825
F 0 "U6" H 14975 6950 50  0000 C CNN
F 1 "NAND FLASH" H 14975 6859 50  0000 C CNN
F 2 "Package_SO:TSOP-I-48_18.4x12mm_P0.5mm" H 14675 6825 50  0001 C CNN
F 3 "" H 14675 6825 50  0001 C CNN
	1    14675 6825
	1    0    0    -1  
$EndComp
$Comp
L SDRAM:SDRAM U1
U 1 1 5EBD8ACC
P 1925 9100
F 0 "U1" H 2525 10350 50  0000 C CNN
F 1 "SDRAM" H 2500 10450 50  0000 C CNN
F 2 "Package_SO:TSOP-II-54_22.2x10.16mm_P0.8mm" H 1925 9100 50  0001 C CIN
F 3 "https://www.alliancememory.com/wp-content/uploads/pdf/dram/64M-AS4C4M16SA-CI_v3.0_March%202015.pdf" H 1925 8850 50  0001 C CNN
	1    1925 9100
	1    0    0    -1  
$EndComp
$Comp
L SDRAM:SDRAM U2
U 1 1 5EBD971E
P 4475 9100
F 0 "U2" H 5075 10375 50  0000 C CNN
F 1 "SDRAM" H 5050 10475 50  0000 C CNN
F 2 "Package_SO:TSOP-II-54_22.2x10.16mm_P0.8mm" H 4475 9100 50  0001 C CIN
F 3 "https://www.alliancememory.com/wp-content/uploads/pdf/dram/64M-AS4C4M16SA-CI_v3.0_March%202015.pdf" H 4475 8850 50  0001 C CNN
	1    4475 9100
	1    0    0    -1  
$EndComp
$Comp
L NOR_Flash:NOR_Flash U5
U 1 1 5EBF7200
P 11875 7875
F 0 "U5" H 12275 9225 50  0000 C CNN
F 1 "NOR_Flash" H 12250 9125 50  0000 C CNN
F 2 "Package_SO:TSOP-I-56_18.4x14mm_P0.5mm" H 11875 7875 50  0001 C CIN
F 3 "" H 11875 7625 50  0001 C CNN
	1    11875 7875
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR08
U 1 1 5EC42FB0
P 4675 1175
F 0 "#PWR08" H 4675 1025 50  0001 C CNN
F 1 "+3.3V" H 4690 1348 50  0000 C CNN
F 2 "" H 4675 1175 50  0001 C CNN
F 3 "" H 4675 1175 50  0001 C CNN
	1    4675 1175
	1    0    0    -1  
$EndComp
Text Label 1125 8050 2    50   ~ 0
FMC_A0
Text Label 1125 8150 2    50   ~ 0
FMC_A1
$Comp
L Device:C C2
U 1 1 5EC69579
P 1000 6975
F 0 "C2" H 1115 7021 50  0000 L CNN
F 1 "0.1u" H 1115 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1038 6825 50  0001 C CNN
F 3 "~" H 1000 6975 50  0001 C CNN
	1    1000 6975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5EC6957F
P 625 6975
F 0 "C1" H 740 7021 50  0000 L CNN
F 1 "0.1u" H 740 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 663 6825 50  0001 C CNN
F 3 "~" H 625 6975 50  0001 C CNN
	1    625  6975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5EC6B9C9
P 1750 6975
F 0 "C4" H 1865 7021 50  0000 L CNN
F 1 "0.1u" H 1865 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1788 6825 50  0001 C CNN
F 3 "~" H 1750 6975 50  0001 C CNN
	1    1750 6975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5EC6B9CF
P 1375 6975
F 0 "C3" H 1490 7021 50  0000 L CNN
F 1 "0.1u" H 1490 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1413 6825 50  0001 C CNN
F 3 "~" H 1375 6975 50  0001 C CNN
	1    1375 6975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5EC6C4B3
P 2500 6975
F 0 "C6" H 2615 7021 50  0000 L CNN
F 1 "0.1u" H 2615 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2538 6825 50  0001 C CNN
F 3 "~" H 2500 6975 50  0001 C CNN
	1    2500 6975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5EC6C4B9
P 2125 6975
F 0 "C5" H 2240 7021 50  0000 L CNN
F 1 "0.1u" H 2240 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2163 6825 50  0001 C CNN
F 3 "~" H 2125 6975 50  0001 C CNN
	1    2125 6975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5EC6CA4F
P 2875 6975
F 0 "C7" H 2990 7021 50  0000 L CNN
F 1 "0.1u" H 2990 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2913 6825 50  0001 C CNN
F 3 "~" H 2875 6975 50  0001 C CNN
	1    2875 6975
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR01
U 1 1 5EC81128
P 625 6375
F 0 "#PWR01" H 625 6225 50  0001 C CNN
F 1 "+3.3V" H 640 6548 50  0000 C CNN
F 2 "" H 625 6375 50  0001 C CNN
F 3 "" H 625 6375 50  0001 C CNN
	1    625  6375
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5EC84F15
P 625 7200
F 0 "#PWR02" H 625 6950 50  0001 C CNN
F 1 "GND" H 630 7027 50  0000 C CNN
F 2 "" H 625 7200 50  0001 C CNN
F 3 "" H 625 7200 50  0001 C CNN
	1    625  7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	625  6825 625  6775
Wire Wire Line
	625  7125 625  7175
Wire Wire Line
	1000 6825 1000 6775
Wire Wire Line
	1000 6775 625  6775
Connection ~ 625  6775
Wire Wire Line
	625  6775 625  6700
Wire Wire Line
	1000 6775 1375 6775
Wire Wire Line
	2875 6775 2875 6825
Connection ~ 1000 6775
Wire Wire Line
	2500 6825 2500 6775
Connection ~ 2500 6775
Wire Wire Line
	2500 6775 2875 6775
Wire Wire Line
	2125 6825 2125 6775
Connection ~ 2125 6775
Wire Wire Line
	2125 6775 2500 6775
Wire Wire Line
	1750 6825 1750 6775
Connection ~ 1750 6775
Wire Wire Line
	1750 6775 2125 6775
Wire Wire Line
	1375 6825 1375 6775
Connection ~ 1375 6775
Wire Wire Line
	1375 6775 1750 6775
Wire Wire Line
	1000 7125 1000 7175
Wire Wire Line
	1000 7175 625  7175
Connection ~ 625  7175
Wire Wire Line
	625  7175 625  7200
Wire Wire Line
	1000 7175 1375 7175
Wire Wire Line
	1375 7175 1375 7125
Connection ~ 1000 7175
Wire Wire Line
	1375 7175 1750 7175
Wire Wire Line
	1750 7175 1750 7125
Connection ~ 1375 7175
Wire Wire Line
	1750 7175 2125 7175
Wire Wire Line
	2125 7175 2125 7125
Connection ~ 1750 7175
Wire Wire Line
	2125 7175 2500 7175
Wire Wire Line
	2500 7175 2500 7125
Connection ~ 2125 7175
Wire Wire Line
	2500 7175 2875 7175
Wire Wire Line
	2875 7175 2875 7125
Connection ~ 2500 7175
$Comp
L Device:L L1
U 1 1 5EC9B287
P 625 6550
F 0 "L1" H 678 6596 50  0000 L CNN
F 1 "FerriteBeads" H 678 6505 50  0000 L CNN
F 2 "Inductor_SMD:L_0603_1608Metric" H 625 6550 50  0001 C CNN
F 3 "~" H 625 6550 50  0001 C CNN
	1    625  6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	625  6400 625  6375
Wire Wire Line
	2875 6775 3000 6775
Connection ~ 2875 6775
Text Label 3000 6775 0    50   ~ 0
VDD_SDRAM1
Text Label 1625 7550 0    50   ~ 0
VDD_SDRAM1
Wire Wire Line
	1625 7550 1625 7675
Wire Wire Line
	1725 7800 1725 7675
Wire Wire Line
	1725 7675 1625 7675
Connection ~ 1625 7675
Wire Wire Line
	1625 7675 1625 7800
Wire Wire Line
	1825 7800 1825 7675
Wire Wire Line
	1825 7675 1725 7675
Connection ~ 1725 7675
Wire Wire Line
	1925 7800 1925 7675
Wire Wire Line
	1925 7675 1825 7675
Connection ~ 1825 7675
Wire Wire Line
	2025 7800 2025 7675
Wire Wire Line
	2025 7675 1925 7675
Connection ~ 1925 7675
Wire Wire Line
	2125 7675 2025 7675
Wire Wire Line
	2125 7675 2125 7800
Connection ~ 2025 7675
Wire Wire Line
	2225 7800 2225 7675
Wire Wire Line
	2225 7675 2125 7675
Connection ~ 2125 7675
$Comp
L power:GND #PWR03
U 1 1 5ECB4EE2
P 1625 10500
F 0 "#PWR03" H 1625 10250 50  0001 C CNN
F 1 "GND" H 1630 10327 50  0000 C CNN
F 2 "" H 1625 10500 50  0001 C CNN
F 3 "" H 1625 10500 50  0001 C CNN
	1    1625 10500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1625 10400 1625 10450
Wire Wire Line
	1625 10450 1725 10450
Wire Wire Line
	1725 10450 1725 10400
Connection ~ 1625 10450
Wire Wire Line
	1625 10450 1625 10500
Wire Wire Line
	1825 10450 1825 10400
Wire Wire Line
	1725 10450 1825 10450
Connection ~ 1725 10450
Wire Wire Line
	1825 10450 1925 10450
Wire Wire Line
	1925 10450 1925 10400
Connection ~ 1825 10450
Wire Wire Line
	1925 10450 2025 10450
Wire Wire Line
	2025 10450 2025 10400
Connection ~ 1925 10450
Wire Wire Line
	2025 10450 2125 10450
Wire Wire Line
	2125 10450 2125 10400
Connection ~ 2025 10450
Wire Wire Line
	2225 10450 2225 10400
Wire Wire Line
	2125 10450 2225 10450
Connection ~ 2125 10450
$Comp
L Device:C C9
U 1 1 5ED38D67
P 3950 6975
F 0 "C9" H 4065 7021 50  0000 L CNN
F 1 "0.1u" H 4065 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3988 6825 50  0001 C CNN
F 3 "~" H 3950 6975 50  0001 C CNN
	1    3950 6975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5ED38D6D
P 3575 6975
F 0 "C8" H 3690 7021 50  0000 L CNN
F 1 "0.1u" H 3690 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3613 6825 50  0001 C CNN
F 3 "~" H 3575 6975 50  0001 C CNN
	1    3575 6975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 5ED38D73
P 4700 6975
F 0 "C12" H 4815 7021 50  0000 L CNN
F 1 "0.1u" H 4815 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4738 6825 50  0001 C CNN
F 3 "~" H 4700 6975 50  0001 C CNN
	1    4700 6975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 5ED38D79
P 4325 6975
F 0 "C10" H 4440 7021 50  0000 L CNN
F 1 "0.1u" H 4440 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4363 6825 50  0001 C CNN
F 3 "~" H 4325 6975 50  0001 C CNN
	1    4325 6975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 5ED38D7F
P 5450 6975
F 0 "C14" H 5565 7021 50  0000 L CNN
F 1 "0.1u" H 5565 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5488 6825 50  0001 C CNN
F 3 "~" H 5450 6975 50  0001 C CNN
	1    5450 6975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C13
U 1 1 5ED38D85
P 5075 6975
F 0 "C13" H 5190 7021 50  0000 L CNN
F 1 "0.1u" H 5190 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5113 6825 50  0001 C CNN
F 3 "~" H 5075 6975 50  0001 C CNN
	1    5075 6975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C15
U 1 1 5ED38D8B
P 5825 6975
F 0 "C15" H 5940 7021 50  0000 L CNN
F 1 "0.1u" H 5940 6930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5863 6825 50  0001 C CNN
F 3 "~" H 5825 6975 50  0001 C CNN
	1    5825 6975
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR05
U 1 1 5ED38D91
P 3575 6375
F 0 "#PWR05" H 3575 6225 50  0001 C CNN
F 1 "+3.3V" H 3590 6548 50  0000 C CNN
F 2 "" H 3575 6375 50  0001 C CNN
F 3 "" H 3575 6375 50  0001 C CNN
	1    3575 6375
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5ED38D97
P 3575 7200
F 0 "#PWR06" H 3575 6950 50  0001 C CNN
F 1 "GND" H 3580 7027 50  0000 C CNN
F 2 "" H 3575 7200 50  0001 C CNN
F 3 "" H 3575 7200 50  0001 C CNN
	1    3575 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3575 6825 3575 6775
Wire Wire Line
	3575 7125 3575 7175
Wire Wire Line
	3950 6825 3950 6775
Wire Wire Line
	3950 6775 3575 6775
Connection ~ 3575 6775
Wire Wire Line
	3575 6775 3575 6700
Wire Wire Line
	3950 6775 4325 6775
Wire Wire Line
	5825 6775 5825 6825
Connection ~ 3950 6775
Wire Wire Line
	5450 6825 5450 6775
Connection ~ 5450 6775
Wire Wire Line
	5450 6775 5825 6775
Wire Wire Line
	5075 6825 5075 6775
Connection ~ 5075 6775
Wire Wire Line
	5075 6775 5450 6775
Wire Wire Line
	4700 6825 4700 6775
Connection ~ 4700 6775
Wire Wire Line
	4700 6775 5075 6775
Wire Wire Line
	4325 6825 4325 6775
Connection ~ 4325 6775
Wire Wire Line
	4325 6775 4700 6775
Wire Wire Line
	3950 7125 3950 7175
Wire Wire Line
	3950 7175 3575 7175
Connection ~ 3575 7175
Wire Wire Line
	3575 7175 3575 7200
Wire Wire Line
	3950 7175 4325 7175
Wire Wire Line
	4325 7175 4325 7125
Connection ~ 3950 7175
Wire Wire Line
	4325 7175 4700 7175
Wire Wire Line
	4700 7175 4700 7125
Connection ~ 4325 7175
Wire Wire Line
	4700 7175 5075 7175
Wire Wire Line
	5075 7175 5075 7125
Connection ~ 4700 7175
Wire Wire Line
	5075 7175 5450 7175
Wire Wire Line
	5450 7175 5450 7125
Connection ~ 5075 7175
Wire Wire Line
	5450 7175 5825 7175
Wire Wire Line
	5825 7175 5825 7125
Connection ~ 5450 7175
$Comp
L Device:L L2
U 1 1 5ED38DC5
P 3575 6550
F 0 "L2" H 3628 6596 50  0000 L CNN
F 1 "FerriteBeads" H 3628 6505 50  0000 L CNN
F 2 "Inductor_SMD:L_0603_1608Metric" H 3575 6550 50  0001 C CNN
F 3 "~" H 3575 6550 50  0001 C CNN
	1    3575 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3575 6400 3575 6375
Wire Wire Line
	5825 6775 5950 6775
Connection ~ 5825 6775
Text Label 5950 6775 0    50   ~ 0
VDD_SDRAM2
Text Label 4175 7550 0    50   ~ 0
VDD_SDRAM2
Wire Wire Line
	4175 7550 4175 7675
Wire Wire Line
	4275 7800 4275 7675
Wire Wire Line
	4275 7675 4175 7675
Connection ~ 4175 7675
Wire Wire Line
	4175 7675 4175 7800
Wire Wire Line
	4375 7800 4375 7675
Wire Wire Line
	4375 7675 4275 7675
Connection ~ 4275 7675
Wire Wire Line
	4475 7800 4475 7675
Wire Wire Line
	4475 7675 4375 7675
Connection ~ 4375 7675
Wire Wire Line
	4575 7800 4575 7675
Wire Wire Line
	4575 7675 4475 7675
Connection ~ 4475 7675
Wire Wire Line
	4675 7675 4575 7675
Wire Wire Line
	4675 7675 4675 7800
Connection ~ 4575 7675
Wire Wire Line
	4775 7800 4775 7675
Wire Wire Line
	4775 7675 4675 7675
Connection ~ 4675 7675
$Comp
L power:GND #PWR07
U 1 1 5ED48746
P 4175 10500
F 0 "#PWR07" H 4175 10250 50  0001 C CNN
F 1 "GND" H 4180 10327 50  0000 C CNN
F 2 "" H 4175 10500 50  0001 C CNN
F 3 "" H 4175 10500 50  0001 C CNN
	1    4175 10500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4175 10400 4175 10450
Wire Wire Line
	4175 10450 4275 10450
Wire Wire Line
	4275 10450 4275 10400
Connection ~ 4175 10450
Wire Wire Line
	4175 10450 4175 10500
Wire Wire Line
	4375 10450 4375 10400
Wire Wire Line
	4275 10450 4375 10450
Connection ~ 4275 10450
Wire Wire Line
	4375 10450 4475 10450
Wire Wire Line
	4475 10450 4475 10400
Connection ~ 4375 10450
Wire Wire Line
	4475 10450 4575 10450
Wire Wire Line
	4575 10450 4575 10400
Connection ~ 4475 10450
Wire Wire Line
	4575 10450 4675 10450
Wire Wire Line
	4675 10450 4675 10400
Connection ~ 4575 10450
Wire Wire Line
	4775 10450 4775 10400
Wire Wire Line
	4675 10450 4775 10450
Connection ~ 4675 10450
$Comp
L SRAM:SRAM U3
U 1 1 5EEDD294
P 7000 9075
F 0 "U3" H 7550 10300 50  0000 C CNN
F 1 "SRAM" H 7550 10400 50  0000 C CNN
F 2 "Package_SO:TSOP-II-44_10.16x18.41mm_P0.8mm" H 7000 9075 50  0001 C CIN
F 3 "" H 7000 8825 50  0001 C CNN
	1    7000 9075
	1    0    0    -1  
$EndComp
$Comp
L SRAM:SRAM U4
U 1 1 5EEE0110
P 9350 9100
F 0 "U4" H 9925 10325 50  0000 C CNN
F 1 "SRAM" H 9900 10400 50  0000 C CNN
F 2 "Package_SO:TSOP-II-44_10.16x18.41mm_P0.8mm" H 9350 9100 50  0001 C CIN
F 3 "" H 9350 8850 50  0001 C CNN
	1    9350 9100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C17
U 1 1 5EF7E730
P 7100 7075
F 0 "C17" H 7215 7121 50  0000 L CNN
F 1 "0.1u" H 7215 7030 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7138 6925 50  0001 C CNN
F 3 "~" H 7100 7075 50  0001 C CNN
	1    7100 7075
	1    0    0    -1  
$EndComp
$Comp
L Device:C C16
U 1 1 5EF7E736
P 6725 7075
F 0 "C16" H 6840 7121 50  0000 L CNN
F 1 "0.1u" H 6840 7030 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6763 6925 50  0001 C CNN
F 3 "~" H 6725 7075 50  0001 C CNN
	1    6725 7075
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR011
U 1 1 5EF7E73C
P 6725 6475
F 0 "#PWR011" H 6725 6325 50  0001 C CNN
F 1 "+3.3V" H 6740 6648 50  0000 C CNN
F 2 "" H 6725 6475 50  0001 C CNN
F 3 "" H 6725 6475 50  0001 C CNN
	1    6725 6475
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5EF7E742
P 6725 7300
F 0 "#PWR012" H 6725 7050 50  0001 C CNN
F 1 "GND" H 6730 7127 50  0000 C CNN
F 2 "" H 6725 7300 50  0001 C CNN
F 3 "" H 6725 7300 50  0001 C CNN
	1    6725 7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6725 6925 6725 6875
Wire Wire Line
	6725 7225 6725 7275
Wire Wire Line
	7100 6925 7100 6875
Wire Wire Line
	7100 6875 6725 6875
Connection ~ 6725 6875
Wire Wire Line
	6725 6875 6725 6800
Wire Wire Line
	7100 6875 7475 6875
Connection ~ 7100 6875
Wire Wire Line
	7100 7225 7100 7275
Wire Wire Line
	7100 7275 6725 7275
Connection ~ 6725 7275
Wire Wire Line
	6725 7275 6725 7300
$Comp
L Device:L L3
U 1 1 5EF7E756
P 6725 6650
F 0 "L3" H 6778 6696 50  0000 L CNN
F 1 "FerriteBeads" H 6778 6605 50  0000 L CNN
F 2 "Inductor_SMD:L_0603_1608Metric" H 6725 6650 50  0001 C CNN
F 3 "~" H 6725 6650 50  0001 C CNN
	1    6725 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6725 6500 6725 6475
Text Label 7475 6875 0    50   ~ 0
VDD_SRAM1
$Comp
L Device:C C19
U 1 1 5EF969D6
P 9000 7075
F 0 "C19" H 9115 7121 50  0000 L CNN
F 1 "0.1u" H 9115 7030 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9038 6925 50  0001 C CNN
F 3 "~" H 9000 7075 50  0001 C CNN
	1    9000 7075
	1    0    0    -1  
$EndComp
$Comp
L Device:C C18
U 1 1 5EF969DC
P 8625 7075
F 0 "C18" H 8740 7121 50  0000 L CNN
F 1 "0.1u" H 8740 7030 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8663 6925 50  0001 C CNN
F 3 "~" H 8625 7075 50  0001 C CNN
	1    8625 7075
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR016
U 1 1 5EF969E2
P 8625 6475
F 0 "#PWR016" H 8625 6325 50  0001 C CNN
F 1 "+3.3V" H 8640 6648 50  0000 C CNN
F 2 "" H 8625 6475 50  0001 C CNN
F 3 "" H 8625 6475 50  0001 C CNN
	1    8625 6475
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5EF969E8
P 8625 7300
F 0 "#PWR017" H 8625 7050 50  0001 C CNN
F 1 "GND" H 8630 7127 50  0000 C CNN
F 2 "" H 8625 7300 50  0001 C CNN
F 3 "" H 8625 7300 50  0001 C CNN
	1    8625 7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8625 6925 8625 6875
Wire Wire Line
	8625 7225 8625 7275
Wire Wire Line
	9000 6925 9000 6875
Wire Wire Line
	9000 6875 8625 6875
Connection ~ 8625 6875
Wire Wire Line
	8625 6875 8625 6800
Wire Wire Line
	9000 6875 9375 6875
Connection ~ 9000 6875
Wire Wire Line
	9000 7225 9000 7275
Wire Wire Line
	9000 7275 8625 7275
Connection ~ 8625 7275
Wire Wire Line
	8625 7275 8625 7300
$Comp
L Device:L L4
U 1 1 5EF969FA
P 8625 6650
F 0 "L4" H 8678 6696 50  0000 L CNN
F 1 "FerriteBeads" H 8678 6605 50  0000 L CNN
F 2 "Inductor_SMD:L_0603_1608Metric" H 8625 6650 50  0001 C CNN
F 3 "~" H 8625 6650 50  0001 C CNN
	1    8625 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8625 6500 8625 6475
Text Label 9375 6875 0    50   ~ 0
VDD_SRAM2
Wire Wire Line
	6950 7775 6950 7625
Wire Wire Line
	7050 7625 6950 7625
Wire Wire Line
	6950 7625 6950 7425
Wire Wire Line
	7050 7625 7050 7775
Text Label 6950 7425 0    50   ~ 0
VDD_SRAM1
Wire Wire Line
	9300 7800 9300 7650
Wire Wire Line
	9400 7650 9300 7650
Connection ~ 9300 7650
Wire Wire Line
	9300 7650 9300 7450
Wire Wire Line
	9400 7650 9400 7800
Text Label 9300 7450 0    50   ~ 0
VDD_SRAM2
$Comp
L power:GND #PWR013
U 1 1 5EFBF675
P 6950 10450
F 0 "#PWR013" H 6950 10200 50  0001 C CNN
F 1 "GND" H 6955 10277 50  0000 C CNN
F 2 "" H 6950 10450 50  0001 C CNN
F 3 "" H 6950 10450 50  0001 C CNN
	1    6950 10450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 10450 6950 10425
Wire Wire Line
	6950 10425 7050 10425
Wire Wire Line
	7050 10425 7050 10375
Connection ~ 6950 10425
Wire Wire Line
	6950 10425 6950 10375
$Comp
L power:GND #PWR018
U 1 1 5EFCE7AA
P 9300 10475
F 0 "#PWR018" H 9300 10225 50  0001 C CNN
F 1 "GND" H 9305 10302 50  0000 C CNN
F 2 "" H 9300 10475 50  0001 C CNN
F 3 "" H 9300 10475 50  0001 C CNN
	1    9300 10475
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 10475 9300 10450
Wire Wire Line
	9300 10450 9400 10450
Wire Wire Line
	9400 10450 9400 10400
Connection ~ 9300 10450
Wire Wire Line
	9300 10450 9300 10400
Text Notes 2550 10925 0    79   ~ 16
SDRAMx2
Text Notes 8100 10850 0    79   ~ 16
SRAMx2
Text Notes 12000 9875 0    79   ~ 16
NOR FLASH x1
Text Notes 14525 9850 0    79   ~ 16
NAND FLASH x1
Text Label 1125 8250 2    50   ~ 0
FMC_A2
Text Label 1125 8350 2    50   ~ 0
FMC_A3
Text Label 1125 8450 2    50   ~ 0
FMC_A4
Text Label 1125 8550 2    50   ~ 0
FMC_A5
Text Label 1125 8650 2    50   ~ 0
FMC_A6
Text Label 1125 8750 2    50   ~ 0
FMC_A7
Text Label 1125 8850 2    50   ~ 0
FMC_A8
Text Label 1125 8950 2    50   ~ 0
FMC_A9
Text Label 1125 9050 2    50   ~ 0
FMC_A10
Text Label 1125 9150 2    50   ~ 0
FMC_A11
Text Label 1125 9250 2    50   ~ 0
FMC_A12
Text Label 1125 9400 2    50   ~ 0
FMC_BA0
Text Label 1125 9500 2    50   ~ 0
FMC_BA1
Text Label 2725 8050 0    50   ~ 0
FMC_D0
Text Label 2725 8150 0    50   ~ 0
FMC_D1
Text Label 2725 8250 0    50   ~ 0
FMC_D2
Text Label 2725 8350 0    50   ~ 0
FMC_D3
Text Label 2725 8450 0    50   ~ 0
FMC_D4
Text Label 2725 8550 0    50   ~ 0
FMC_D5
Text Label 2725 8650 0    50   ~ 0
FMC_D6
Text Label 2725 8750 0    50   ~ 0
FMC_D7
Text Label 2725 8850 0    50   ~ 0
FMC_D8
Text Label 2725 8950 0    50   ~ 0
FMC_D9
Text Label 2725 9050 0    50   ~ 0
FMC_D10
Text Label 2725 9150 0    50   ~ 0
FMC_D11
Text Label 2725 9250 0    50   ~ 0
FMC_D12
Text Label 2725 9350 0    50   ~ 0
FMC_D13
Text Label 2725 9450 0    50   ~ 0
FMC_D14
Text Label 1125 9900 2    50   ~ 0
FMC_SDNRAS
Text Label 1125 9650 2    50   ~ 0
FMC_SDCLK
Text Label 1125 10000 2    50   ~ 0
FMC_SDNCAS
Text Label 1125 10200 2    50   ~ 0
FMC_SDNE0
Text Label 1125 9750 2    50   ~ 0
FMC_SDCKE0
Text Label 1125 10100 2    50   ~ 0
FMC_SDNWE
Text Label 2725 9550 0    50   ~ 0
FMC_D15
Text Label 2725 9800 0    50   ~ 0
FMC_NBL0
Text Label 2725 9900 0    50   ~ 0
FMC_NBL1
Text Label 5275 8050 0    50   ~ 0
FMC_D0
Text Label 5275 8150 0    50   ~ 0
FMC_D1
Text Label 5275 8250 0    50   ~ 0
FMC_D2
Text Label 5275 8350 0    50   ~ 0
FMC_D3
Text Label 5275 8450 0    50   ~ 0
FMC_D4
Text Label 5275 8550 0    50   ~ 0
FMC_D5
Text Label 5275 8650 0    50   ~ 0
FMC_D6
Text Label 5275 8750 0    50   ~ 0
FMC_D7
Text Label 5275 8850 0    50   ~ 0
FMC_D8
Text Label 5275 8950 0    50   ~ 0
FMC_D9
Text Label 5275 9050 0    50   ~ 0
FMC_D10
Text Label 5275 9150 0    50   ~ 0
FMC_D11
Text Label 5275 9250 0    50   ~ 0
FMC_D12
Text Label 5275 9350 0    50   ~ 0
FMC_D13
Text Label 5275 9450 0    50   ~ 0
FMC_D14
Text Label 5275 9550 0    50   ~ 0
FMC_D15
Text Label 5275 9800 0    50   ~ 0
FMC_NBL0
Text Label 5275 9900 0    50   ~ 0
FMC_NBL1
Text Label 3675 8050 2    50   ~ 0
FMC_A0
Text Label 3675 8150 2    50   ~ 0
FMC_A1
Text Label 3675 8250 2    50   ~ 0
FMC_A2
Text Label 3675 8350 2    50   ~ 0
FMC_A3
Text Label 3675 8450 2    50   ~ 0
FMC_A4
Text Label 3675 8550 2    50   ~ 0
FMC_A5
Text Label 3675 8650 2    50   ~ 0
FMC_A6
Text Label 3675 8750 2    50   ~ 0
FMC_A7
Text Label 3675 8850 2    50   ~ 0
FMC_A8
Text Label 3675 8950 2    50   ~ 0
FMC_A9
Text Label 3675 9050 2    50   ~ 0
FMC_A10
Text Label 3675 9150 2    50   ~ 0
FMC_A11
Text Label 3675 9250 2    50   ~ 0
FMC_A12
Text Label 3675 9400 2    50   ~ 0
FMC_BA0
Text Label 3675 9500 2    50   ~ 0
FMC_BA1
Text Label 3675 9900 2    50   ~ 0
FMC_SDNRAS
Text Label 3675 9650 2    50   ~ 0
FMC_SDCLK
Text Label 3675 10000 2    50   ~ 0
FMC_SDNCAS
Text Label 3675 10200 2    50   ~ 0
FMC_SDNE1
Text Label 3675 9750 2    50   ~ 0
FMC_SDCKE1
Text Label 3675 10100 2    50   ~ 0
FMC_SDNWE
$Comp
L Device:C C21
U 1 1 5F0D7406
P 11350 6300
F 0 "C21" H 11465 6346 50  0000 L CNN
F 1 "0.1u" H 11465 6255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 11388 6150 50  0001 C CNN
F 3 "~" H 11350 6300 50  0001 C CNN
	1    11350 6300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C20
U 1 1 5F0D740C
P 10975 6300
F 0 "C20" H 11090 6346 50  0000 L CNN
F 1 "0.1u" H 11090 6255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 11013 6150 50  0001 C CNN
F 3 "~" H 10975 6300 50  0001 C CNN
	1    10975 6300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR020
U 1 1 5F0D7412
P 10975 5700
F 0 "#PWR020" H 10975 5550 50  0001 C CNN
F 1 "+3.3V" H 10990 5873 50  0000 C CNN
F 2 "" H 10975 5700 50  0001 C CNN
F 3 "" H 10975 5700 50  0001 C CNN
	1    10975 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5F0D7418
P 10975 6525
F 0 "#PWR021" H 10975 6275 50  0001 C CNN
F 1 "GND" H 10980 6352 50  0000 C CNN
F 2 "" H 10975 6525 50  0001 C CNN
F 3 "" H 10975 6525 50  0001 C CNN
	1    10975 6525
	1    0    0    -1  
$EndComp
Wire Wire Line
	10975 6150 10975 6100
Wire Wire Line
	10975 6450 10975 6500
Wire Wire Line
	11350 6150 11350 6100
Wire Wire Line
	11350 6100 10975 6100
Connection ~ 10975 6100
Wire Wire Line
	10975 6100 10975 6025
Wire Wire Line
	11350 6100 11450 6100
Connection ~ 11350 6100
Wire Wire Line
	11350 6450 11350 6500
Wire Wire Line
	11350 6500 10975 6500
Connection ~ 10975 6500
Wire Wire Line
	10975 6500 10975 6525
$Comp
L Device:L L5
U 1 1 5F0D742A
P 10975 5875
F 0 "L5" H 11028 5921 50  0000 L CNN
F 1 "FerriteBeads" H 11028 5830 50  0000 L CNN
F 2 "Inductor_SMD:L_0603_1608Metric" H 10975 5875 50  0001 C CNN
F 3 "~" H 10975 5875 50  0001 C CNN
	1    10975 5875
	1    0    0    -1  
$EndComp
Wire Wire Line
	10975 5725 10975 5700
Text Label 11450 6100 0    50   ~ 0
VDD_NOR
Wire Wire Line
	11825 6575 11825 6425
Wire Wire Line
	11925 6425 11825 6425
Connection ~ 11825 6425
Wire Wire Line
	11825 6425 11825 6225
Wire Wire Line
	11925 6425 11925 6575
Text Label 11825 6225 0    50   ~ 0
VDD_NOR
$Comp
L Device:C C24
U 1 1 5F157294
P 14250 6325
F 0 "C24" H 14365 6371 50  0000 L CNN
F 1 "0.1u" H 14365 6280 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 14288 6175 50  0001 C CNN
F 3 "~" H 14250 6325 50  0001 C CNN
	1    14250 6325
	1    0    0    -1  
$EndComp
$Comp
L Device:C C23
U 1 1 5F15729A
P 13875 6325
F 0 "C23" H 13990 6371 50  0000 L CNN
F 1 "0.1u" H 13990 6280 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 13913 6175 50  0001 C CNN
F 3 "~" H 13875 6325 50  0001 C CNN
	1    13875 6325
	1    0    0    -1  
$EndComp
Wire Wire Line
	13875 6175 13875 6125
Wire Wire Line
	14250 6175 14250 6125
Wire Wire Line
	14250 6125 13875 6125
Connection ~ 13875 6125
Wire Wire Line
	13875 6125 13875 6050
Wire Wire Line
	14250 6125 14625 6125
Connection ~ 14250 6125
$Comp
L Device:L L6
U 1 1 5F1572A9
P 13875 5900
F 0 "L6" H 13928 5946 50  0000 L CNN
F 1 "FerriteBeads" H 13928 5855 50  0000 L CNN
F 2 "Inductor_SMD:L_0603_1608Metric" H 13875 5900 50  0001 C CNN
F 3 "~" H 13875 5900 50  0001 C CNN
	1    13875 5900
	1    0    0    -1  
$EndComp
Text Label 14625 6125 0    50   ~ 0
VDD_NAND
Text Label 15225 5775 0    50   ~ 0
VDD_NAND
$Comp
L power:GND #PWR029
U 1 1 5F16A65C
P 13875 6600
F 0 "#PWR029" H 13875 6350 50  0001 C CNN
F 1 "GND" H 13880 6427 50  0000 C CNN
F 2 "" H 13875 6600 50  0001 C CNN
F 3 "" H 13875 6600 50  0001 C CNN
	1    13875 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	14250 6575 13875 6575
Connection ~ 13875 6575
Wire Wire Line
	13875 6575 13875 6600
Wire Wire Line
	13875 6475 13875 6575
Wire Wire Line
	14250 6475 14250 6575
$Comp
L power:+3.3V #PWR028
U 1 1 5F18E2B0
P 13875 5700
F 0 "#PWR028" H 13875 5550 50  0001 C CNN
F 1 "+3.3V" H 13890 5873 50  0000 C CNN
F 2 "" H 13875 5700 50  0001 C CNN
F 3 "" H 13875 5700 50  0001 C CNN
	1    13875 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	13875 5700 13875 5750
$Comp
L power:GND #PWR023
U 1 1 5F1A32DF
P 11825 9700
F 0 "#PWR023" H 11825 9450 50  0001 C CNN
F 1 "GND" H 11830 9527 50  0000 C CNN
F 2 "" H 11825 9700 50  0001 C CNN
F 3 "" H 11825 9700 50  0001 C CNN
	1    11825 9700
	1    0    0    -1  
$EndComp
Wire Wire Line
	11825 9700 11825 9675
Wire Wire Line
	11975 9625 11975 9675
Wire Wire Line
	11975 9675 11825 9675
Connection ~ 11825 9675
Wire Wire Line
	11825 9675 11825 9625
Text Label 6200 8025 2    50   ~ 0
FMC_A0
Text Label 6200 8125 2    50   ~ 0
FMC_A1
Text Label 6200 8225 2    50   ~ 0
FMC_A2
Text Label 6200 8325 2    50   ~ 0
FMC_A3
Text Label 6200 8425 2    50   ~ 0
FMC_A4
Text Label 6200 8525 2    50   ~ 0
FMC_A5
Text Label 6200 8625 2    50   ~ 0
FMC_A6
Text Label 6200 8725 2    50   ~ 0
FMC_A7
Text Label 6200 8825 2    50   ~ 0
FMC_A8
Text Label 6200 8925 2    50   ~ 0
FMC_A9
Text Label 6200 9025 2    50   ~ 0
FMC_A10
Text Label 6200 9125 2    50   ~ 0
FMC_A11
Text Label 6200 9225 2    50   ~ 0
FMC_A12
Text Label 7800 8025 0    50   ~ 0
FMC_D0
Text Label 7800 8125 0    50   ~ 0
FMC_D1
Text Label 7800 8225 0    50   ~ 0
FMC_D2
Text Label 7800 8325 0    50   ~ 0
FMC_D3
Text Label 7800 8425 0    50   ~ 0
FMC_D4
Text Label 7800 8525 0    50   ~ 0
FMC_D5
Text Label 7800 8625 0    50   ~ 0
FMC_D6
Text Label 7800 8725 0    50   ~ 0
FMC_D7
Text Label 7800 8825 0    50   ~ 0
FMC_D8
Text Label 7800 8925 0    50   ~ 0
FMC_D9
Text Label 7800 9025 0    50   ~ 0
FMC_D10
Text Label 7800 9125 0    50   ~ 0
FMC_D11
Text Label 7800 9225 0    50   ~ 0
FMC_D12
Text Label 7800 9325 0    50   ~ 0
FMC_D13
Text Label 7800 9425 0    50   ~ 0
FMC_D14
Text Label 7800 9525 0    50   ~ 0
FMC_D15
Text Label 6200 9325 2    50   ~ 0
FMC_A13
Text Label 6200 9425 2    50   ~ 0
FMC_A14
Text Label 6200 9525 2    50   ~ 0
FMC_A15
Text Label 6200 9625 2    50   ~ 0
FMC_A16
Text Label 6200 9725 2    50   ~ 0
FMC_A17
Text Label 6200 9825 2    50   ~ 0
FMC_A18
Text Label 8550 8050 2    50   ~ 0
FMC_A0
Text Label 8550 8150 2    50   ~ 0
FMC_A1
Text Label 8550 8250 2    50   ~ 0
FMC_A2
Text Label 8550 8350 2    50   ~ 0
FMC_A3
Text Label 8550 8450 2    50   ~ 0
FMC_A4
Text Label 8550 8550 2    50   ~ 0
FMC_A5
Text Label 8550 8650 2    50   ~ 0
FMC_A6
Text Label 8550 8750 2    50   ~ 0
FMC_A7
Text Label 8550 8850 2    50   ~ 0
FMC_A8
Text Label 8550 8950 2    50   ~ 0
FMC_A9
Text Label 8550 9050 2    50   ~ 0
FMC_A10
Text Label 8550 9150 2    50   ~ 0
FMC_A11
Text Label 8550 9250 2    50   ~ 0
FMC_A12
Text Label 8550 9350 2    50   ~ 0
FMC_A13
Text Label 8550 9450 2    50   ~ 0
FMC_A14
Text Label 8550 9550 2    50   ~ 0
FMC_A15
Text Label 8550 9650 2    50   ~ 0
FMC_A16
Text Label 8550 9750 2    50   ~ 0
FMC_A17
Text Label 8550 9850 2    50   ~ 0
FMC_A18
Text Label 10150 8050 0    50   ~ 0
FMC_D0
Text Label 10150 8150 0    50   ~ 0
FMC_D1
Text Label 10150 8250 0    50   ~ 0
FMC_D2
Text Label 10150 8350 0    50   ~ 0
FMC_D3
Text Label 10150 8450 0    50   ~ 0
FMC_D4
Text Label 10150 8550 0    50   ~ 0
FMC_D5
Text Label 10150 8650 0    50   ~ 0
FMC_D6
Text Label 10150 8750 0    50   ~ 0
FMC_D7
Text Label 10150 8850 0    50   ~ 0
FMC_D8
Text Label 10150 8950 0    50   ~ 0
FMC_D9
Text Label 10150 9050 0    50   ~ 0
FMC_D10
Text Label 10150 9150 0    50   ~ 0
FMC_D11
Text Label 10150 9250 0    50   ~ 0
FMC_D12
Text Label 10150 9350 0    50   ~ 0
FMC_D13
Text Label 10150 9450 0    50   ~ 0
FMC_D14
Text Label 10150 9550 0    50   ~ 0
FMC_D15
Text Label 7800 9675 0    50   ~ 0
FMC_NE1
Text Label 7800 9775 0    50   ~ 0
FMC_NWE
Text Label 7800 9875 0    50   ~ 0
FMC_NBL0
Text Label 7800 9975 0    50   ~ 0
FMC_NBL1
Text Label 7800 10075 0    50   ~ 0
FMC_NOE
Text Label 10150 9700 0    50   ~ 0
FMC_NE3
Text Label 10150 9800 0    50   ~ 0
FMC_NWE
Text Label 10150 9900 0    50   ~ 0
FMC_NBL0
Text Label 10150 10000 0    50   ~ 0
FMC_NBL1
Text Label 10150 10100 0    50   ~ 0
FMC_NOE
Text Label 11075 6825 2    50   ~ 0
FMC_A0
Text Label 11075 6925 2    50   ~ 0
FMC_A1
Text Label 11075 7025 2    50   ~ 0
FMC_A2
Text Label 11075 7125 2    50   ~ 0
FMC_A3
Text Label 11075 7225 2    50   ~ 0
FMC_A4
Text Label 11075 7325 2    50   ~ 0
FMC_A5
Text Label 11075 7425 2    50   ~ 0
FMC_A6
Text Label 11075 7525 2    50   ~ 0
FMC_A7
Text Label 11075 7625 2    50   ~ 0
FMC_A8
Text Label 11075 7725 2    50   ~ 0
FMC_A9
Text Label 11075 7825 2    50   ~ 0
FMC_A10
Text Label 11075 7925 2    50   ~ 0
FMC_A11
Text Label 11075 8025 2    50   ~ 0
FMC_A12
Text Label 11075 8125 2    50   ~ 0
FMC_A13
Text Label 11075 8225 2    50   ~ 0
FMC_A14
Text Label 11075 8325 2    50   ~ 0
FMC_A15
Text Label 11075 8425 2    50   ~ 0
FMC_A16
Text Label 11075 8525 2    50   ~ 0
FMC_A17
Text Label 11075 8625 2    50   ~ 0
FMC_A18
Text Label 11075 8725 2    50   ~ 0
FMC_A19
Text Label 11075 8825 2    50   ~ 0
FMC_A20
Text Label 11075 8925 2    50   ~ 0
FMC_A21
Text Label 11075 9025 2    50   ~ 0
FMC_A22
Text Label 11075 9125 2    50   ~ 0
FMC_A23
Text Label 12675 8325 0    50   ~ 0
FMC_D15
Text Label 12675 8225 0    50   ~ 0
FMC_D14
Text Label 12675 8125 0    50   ~ 0
FMC_D13
Text Label 12675 8025 0    50   ~ 0
FMC_D12
Text Label 12675 7925 0    50   ~ 0
FMC_D11
Text Label 12675 7825 0    50   ~ 0
FMC_D10
Text Label 12675 7725 0    50   ~ 0
FMC_D9
Text Label 12675 7625 0    50   ~ 0
FMC_D8
Text Label 12675 7525 0    50   ~ 0
FMC_D7
Text Label 12675 7425 0    50   ~ 0
FMC_D6
Text Label 12675 7325 0    50   ~ 0
FMC_D5
Text Label 12675 7225 0    50   ~ 0
FMC_D4
Text Label 12675 7125 0    50   ~ 0
FMC_D3
Text Label 12675 7025 0    50   ~ 0
FMC_D2
Text Label 12675 6925 0    50   ~ 0
FMC_D1
Text Label 12675 6825 0    50   ~ 0
FMC_D0
$Comp
L Device:R R2
U 1 1 5F2EBC35
P 12900 9475
F 0 "R2" V 12800 9350 50  0000 C CNN
F 1 "20k" V 12900 9475 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12830 9475 50  0001 C CNN
F 3 "~" H 12900 9475 50  0001 C CNN
	1    12900 9475
	0    1    1    0   
$EndComp
$Comp
L Device:R RX6
U 1 1 5F2F4FAD
P 12900 9625
F 0 "RX6" V 12825 9525 50  0000 C CNN
F 1 "0" V 12900 9625 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12830 9625 50  0001 C CNN
F 3 "~" H 12900 9625 50  0001 C CNN
	1    12900 9625
	0    1    1    0   
$EndComp
Wire Wire Line
	12675 9225 12700 9225
Wire Wire Line
	12700 9225 12700 9475
Wire Wire Line
	12700 9475 12750 9475
Wire Wire Line
	12700 9475 12700 9625
Wire Wire Line
	12700 9625 12750 9625
Connection ~ 12700 9475
$Comp
L power:GND #PWR025
U 1 1 5F306582
P 13250 9650
F 0 "#PWR025" H 13250 9400 50  0001 C CNN
F 1 "GND" H 13255 9477 50  0000 C CNN
F 2 "" H 13250 9650 50  0001 C CNN
F 3 "" H 13250 9650 50  0001 C CNN
	1    13250 9650
	1    0    0    -1  
$EndComp
Text Label 13075 9475 0    50   ~ 0
VDD_NOR
Wire Wire Line
	13050 9475 13075 9475
Wire Wire Line
	13250 9650 13250 9625
Wire Wire Line
	13250 9625 13050 9625
Text Label 12675 8625 0    50   ~ 0
FMC_NWE
$Comp
L Device:R R3
U 1 1 5F3231FD
P 13550 8725
F 0 "R3" V 13500 8875 50  0000 C CNN
F 1 "20k" V 13550 8725 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 13480 8725 50  0001 C CNN
F 3 "~" H 13550 8725 50  0001 C CNN
	1    13550 8725
	0    1    1    0   
$EndComp
$Comp
L Device:C C22
U 1 1 5F326EAB
P 13375 8925
F 0 "C22" H 13490 8971 50  0000 L CNN
F 1 "0.1u" H 13490 8880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 13413 8775 50  0001 C CNN
F 3 "~" H 13375 8925 50  0001 C CNN
	1    13375 8925
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR026
U 1 1 5F335123
P 13375 9125
F 0 "#PWR026" H 13375 8875 50  0001 C CNN
F 1 "GND" H 13380 8952 50  0000 C CNN
F 2 "" H 13375 9125 50  0001 C CNN
F 3 "" H 13375 9125 50  0001 C CNN
	1    13375 9125
	1    0    0    -1  
$EndComp
Text Label 13750 8725 0    50   ~ 0
VDD_NOR
Wire Wire Line
	12675 8725 13375 8725
Wire Wire Line
	13375 9075 13375 9125
Wire Wire Line
	13375 8775 13375 8725
Connection ~ 13375 8725
Wire Wire Line
	13375 8725 13400 8725
Wire Wire Line
	13700 8725 13750 8725
$Comp
L Device:R R4
U 1 1 5F37C052
P 13625 9475
F 0 "R4" V 13575 9625 50  0000 C CNN
F 1 "20k" V 13625 9475 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 13555 9475 50  0001 C CNN
F 3 "~" H 13625 9475 50  0001 C CNN
	1    13625 9475
	0    1    1    0   
$EndComp
$Comp
L Device:R RX8
U 1 1 5F37C058
P 13625 9625
F 0 "RX8" V 13575 9800 50  0000 C CNN
F 1 "0" V 13625 9625 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 13555 9625 50  0001 C CNN
F 3 "~" H 13625 9625 50  0001 C CNN
	1    13625 9625
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5F37C05E
P 13975 9650
F 0 "#PWR030" H 13975 9400 50  0001 C CNN
F 1 "GND" H 13980 9477 50  0000 C CNN
F 2 "" H 13975 9650 50  0001 C CNN
F 3 "" H 13975 9650 50  0001 C CNN
	1    13975 9650
	1    0    0    -1  
$EndComp
Text Label 13975 9475 0    50   ~ 0
VDD_NOR
Wire Wire Line
	13775 9475 13975 9475
Wire Wire Line
	13975 9650 13975 9625
Wire Wire Line
	13975 9625 13775 9625
Wire Wire Line
	12675 8825 13225 8825
Wire Wire Line
	13225 8825 13225 9375
Wire Wire Line
	13225 9375 13450 9375
Wire Wire Line
	13450 9375 13450 9475
Wire Wire Line
	13450 9475 13475 9475
Wire Wire Line
	13450 9475 13450 9625
Wire Wire Line
	13450 9625 13475 9625
Connection ~ 13450 9475
Text Label 12675 8925 0    50   ~ 0
FMC_NWAIT
Text Label 12675 9025 0    50   ~ 0
FMC_NE4
Text Label 12675 9125 0    50   ~ 0
FMC_NOE
Connection ~ 6950 7625
$Comp
L Device:R R5
U 1 1 5F551F06
P 13900 8075
F 0 "R5" V 13850 8225 50  0000 C CNN
F 1 "20k" V 13900 8075 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 13830 8075 50  0001 C CNN
F 3 "~" H 13900 8075 50  0001 C CNN
	1    13900 8075
	0    -1   1    0   
$EndComp
$Comp
L Device:R RX9
U 1 1 5F551F0C
P 13900 8225
F 0 "RX9" V 13850 8400 50  0000 C CNN
F 1 "0" V 13900 8225 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 13830 8225 50  0001 C CNN
F 3 "~" H 13900 8225 50  0001 C CNN
	1    13900 8225
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR027
U 1 1 5F551F12
P 13550 8250
F 0 "#PWR027" H 13550 8000 50  0001 C CNN
F 1 "GND" H 13555 8077 50  0000 C CNN
F 2 "" H 13550 8250 50  0001 C CNN
F 3 "" H 13550 8250 50  0001 C CNN
	1    13550 8250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	13750 8075 13550 8075
Wire Wire Line
	13550 8250 13550 8225
Wire Wire Line
	13550 8225 13750 8225
Wire Wire Line
	14075 8075 14075 8225
Wire Wire Line
	14075 8225 14050 8225
Text Label 13550 8075 2    50   ~ 0
VDD_NAND
Wire Wire Line
	14575 8725 14200 8725
Wire Wire Line
	14200 8725 14200 8075
Wire Wire Line
	14050 8075 14075 8075
Connection ~ 14075 8075
Wire Wire Line
	14075 8075 14200 8075
$Comp
L power:GND #PWR032
U 1 1 5F5B3981
P 15575 9250
F 0 "#PWR032" H 15575 9000 50  0001 C CNN
F 1 "GND" H 15580 9077 50  0000 C CNN
F 2 "" H 15575 9250 50  0001 C CNN
F 3 "" H 15575 9250 50  0001 C CNN
	1    15575 9250
	1    0    0    -1  
$EndComp
Wire Wire Line
	15375 9225 15575 9225
Wire Wire Line
	15575 9225 15575 9250
$Comp
L power:GND #PWR034
U 1 1 5F5BF28E
P 15875 8150
F 0 "#PWR034" H 15875 7900 50  0001 C CNN
F 1 "GND" H 15880 7977 50  0000 C CNN
F 2 "" H 15875 8150 50  0001 C CNN
F 3 "" H 15875 8150 50  0001 C CNN
	1    15875 8150
	1    0    0    -1  
$EndComp
Wire Wire Line
	15375 8125 15875 8125
Wire Wire Line
	15875 8125 15875 8150
$Comp
L power:GND #PWR033
U 1 1 5F5D60A0
P 15875 6950
F 0 "#PWR033" H 15875 6700 50  0001 C CNN
F 1 "GND" H 15880 6777 50  0000 C CNN
F 2 "" H 15875 6950 50  0001 C CNN
F 3 "" H 15875 6950 50  0001 C CNN
	1    15875 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	15375 6925 15875 6925
Wire Wire Line
	15875 6925 15875 6950
$Comp
L power:GND #PWR031
U 1 1 5F5E1CA3
P 14350 8150
F 0 "#PWR031" H 14350 7900 50  0001 C CNN
F 1 "GND" H 14355 7977 50  0000 C CNN
F 2 "" H 14350 8150 50  0001 C CNN
F 3 "" H 14350 8150 50  0001 C CNN
	1    14350 8150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	14575 8125 14350 8125
Wire Wire Line
	14350 8125 14350 8150
Text Label 14575 8025 2    50   ~ 0
VDD_NAND
Text Label 15375 7825 0    50   ~ 0
VDD_NAND
Text Label 15375 8025 0    50   ~ 0
VDD_NAND
Text Label 15375 7025 0    50   ~ 0
FMC_D15
Text Label 15375 7125 0    50   ~ 0
FMC_D14
Text Label 15375 7225 0    50   ~ 0
FMC_D13
Text Label 15375 7725 0    50   ~ 0
FMC_D12
Text Label 15375 8425 0    50   ~ 0
FMC_D11
Text Label 15375 8925 0    50   ~ 0
FMC_D10
Text Label 15375 9025 0    50   ~ 0
FMC_D9
Text Label 15375 9125 0    50   ~ 0
FMC_D8
Text Label 15375 7325 0    50   ~ 0
FMC_D7
Text Label 15375 7425 0    50   ~ 0
FMC_D6
Text Label 15375 7525 0    50   ~ 0
FMC_D5
Text Label 15375 7625 0    50   ~ 0
FMC_D4
Text Label 15375 8525 0    50   ~ 0
FMC_D3
Text Label 15375 8625 0    50   ~ 0
FMC_D2
Text Label 15375 8725 0    50   ~ 0
FMC_D1
Text Label 15375 8825 0    50   ~ 0
FMC_D0
Text Label 15375 8325 0    50   ~ 0
VDD_NAND
Text Label 14575 7525 2    50   ~ 0
FMC_NWAIT
Text Label 14575 7625 2    50   ~ 0
FMC_NOE
Text Label 14575 7725 2    50   ~ 0
FMC_NCE
Text Label 14575 8425 2    50   ~ 0
FMC_A16
Text Label 14575 8525 2    50   ~ 0
FMC_A17
Text Label 14575 8625 2    50   ~ 0
FMC_NWE
Text Label 4025 2325 0    39   ~ 0
FMC_A23
Text Label 4025 2425 0    39   ~ 0
FMC_A19
Text Label 4025 2525 0    39   ~ 0
FMC_A20
Text Label 4025 2625 0    39   ~ 0
FMC_A21
Text Label 4025 2725 0    39   ~ 0
FMC_A22
Text Label 4025 2925 0    39   ~ 0
FMC_A0
Text Label 4025 3025 0    39   ~ 0
FMC_A1
Text Label 4025 3125 0    39   ~ 0
FMC_A2
Text Label 14200 3250 0    39   ~ 0
FMC_A3
Text Label 14200 3350 0    39   ~ 0
FMC_A4
Text Label 14200 3450 0    39   ~ 0
FMC_A5
Text Label 2625 3225 2    39   ~ 0
FMC_SDNWE
Text Label 2625 3425 2    39   ~ 0
FMC_SDNE0
Text Label 2625 3525 2    39   ~ 0
FMC_SDCKE0
Text Label 14200 3650 0    39   ~ 0
FMC_SDNRAS
Text Label 14200 3750 0    39   ~ 0
FMC_A6
Text Label 14200 3850 0    39   ~ 0
FMC_A7
Text Label 14200 3950 0    39   ~ 0
FMC_A8
Text Label 14200 4050 0    39   ~ 0
FMC_A9
Text Label 4025 3825 0    39   ~ 0
FMC_A11
Text Label 4025 3725 0    39   ~ 0
FMC_A10
Text Label 14200 2250 0    39   ~ 0
FMC_D4
Text Label 14200 2350 0    39   ~ 0
FMC_D5
Text Label 14200 2450 0    39   ~ 0
FMC_D6
Text Label 14200 2550 0    39   ~ 0
FMC_D7
Text Label 14200 2650 0    39   ~ 0
FMC_D8
Text Label 14200 2750 0    39   ~ 0
FMC_D9
Text Label 14200 2850 0    39   ~ 0
FMC_D10
Text Label 14200 2950 0    39   ~ 0
FMC_D11
Text Label 14200 3050 0    39   ~ 0
FMC_D12
Text Label 14200 1350 0    39   ~ 0
FMC_D13
Text Label 2625 5125 2    39   ~ 0
FMC_D14
Text Label 14200 1450 0    39   ~ 0
FMC_D15
Text Label 14625 1550 0    39   ~ 0
FMC_A16
Wire Wire Line
	14575 1550 14575 1600
Connection ~ 14575 1550
Wire Wire Line
	14575 1550 14625 1550
Wire Wire Line
	14575 1600 14625 1600
Text Label 14625 1600 0    39   ~ 0
FMC_CLE
Text Label 14625 1650 0    39   ~ 0
FMC_A17
Wire Wire Line
	14575 1650 14575 1700
Connection ~ 14575 1650
Wire Wire Line
	14575 1650 14625 1650
Text Label 14625 1700 0    39   ~ 0
FMC_ALE
Text Label 14200 1750 0    39   ~ 0
FMC_A18
Text Label 14200 1850 0    39   ~ 0
FMC_D0
Text Label 14200 1950 0    39   ~ 0
FMC_D1
Text Label 4025 3925 0    39   ~ 0
FMC_A12
Text Label 4025 4025 0    39   ~ 0
FMC_A13
Text Label 14625 4250 0    39   ~ 0
FMC_BA0
Wire Wire Line
	14200 4250 14575 4250
Wire Wire Line
	14575 4250 14575 4300
Wire Wire Line
	14575 4300 14625 4300
Connection ~ 14575 4250
Wire Wire Line
	14575 4250 14625 4250
Text Label 14625 4300 0    39   ~ 0
FMC_A14
Text Label 14625 4350 0    39   ~ 0
FMC_BA1
Wire Wire Line
	14200 4350 14575 4350
Wire Wire Line
	14575 4350 14575 4400
Wire Wire Line
	14575 4400 14625 4400
Connection ~ 14575 4350
Wire Wire Line
	14575 4350 14625 4350
Text Label 14625 4400 0    39   ~ 0
FMC_A15
Text Label 4025 4225 0    39   ~ 0
FMC_NE3
Wire Wire Line
	14575 1700 14625 1700
Wire Wire Line
	14200 1650 14575 1650
Wire Wire Line
	14200 1550 14575 1550
Text Label 14200 4650 0    39   ~ 0
FMC_SDCLK
Text Label 2625 4925 2    39   ~ 0
FMC_NWAIT
Text Label 14925 4550 0    39   ~ 0
FMC_NWAIT
Text Notes 14650 4675 0    39   ~ 0
*FMC_INT_MODE
$Comp
L Device:R RX10
U 1 1 5F8B52DF
P 14725 4550
F 0 "RX10" V 14675 4725 50  0000 C CNN
F 1 "0" V 14725 4550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 14655 4550 50  0001 C CNN
F 3 "~" H 14725 4550 50  0001 C CNN
	1    14725 4550
	0    -1   1    0   
$EndComp
Wire Wire Line
	14575 4550 14200 4550
Wire Wire Line
	14875 4550 14925 4550
Text Label 2625 5025 2    39   ~ 0
FMC_NE1
Text Label 4025 4125 0    39   ~ 0
FMC_NCE
Text Label 2625 4325 2    39   ~ 0
FMC_D2
Text Label 2625 4425 2    39   ~ 0
FMC_D3
Text Label 2625 4725 2    39   ~ 0
FMC_NOE
Text Label 2625 4825 2    39   ~ 0
FMC_NWE
Text Label 4025 4425 0    39   ~ 0
FMC_NE4
Text Label 4025 4625 0    39   ~ 0
FMC_SDNCAS
Text Label 12400 2850 2    39   ~ 0
FMC_SDCKE1
Text Label 12400 2950 2    39   ~ 0
FMC_SDNE1
Text Label 14200 2150 0    39   ~ 0
FMC_NBL0
Text Label 4025 2225 0    39   ~ 0
FMC_NBL1
$Comp
L power:GND #PWR04
U 1 1 5F95B376
P 3075 5700
F 0 "#PWR04" H 3075 5450 50  0001 C CNN
F 1 "GND" H 3080 5527 50  0000 C CNN
F 2 "" H 3075 5700 50  0001 C CNN
F 3 "" H 3075 5700 50  0001 C CNN
	1    3075 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3075 5625 3075 5675
Wire Wire Line
	3075 5675 3125 5675
Wire Wire Line
	3125 5675 3125 5625
Connection ~ 3075 5675
Wire Wire Line
	3075 5675 3075 5700
Wire Wire Line
	3125 5675 3175 5675
Wire Wire Line
	3175 5675 3175 5625
Connection ~ 3125 5675
Wire Wire Line
	3175 5675 3225 5675
Wire Wire Line
	3225 5675 3225 5625
Connection ~ 3175 5675
Wire Wire Line
	3225 5675 3275 5675
Wire Wire Line
	3275 5675 3275 5625
Connection ~ 3225 5675
Wire Wire Line
	3275 5675 3325 5675
Wire Wire Line
	3325 5675 3325 5625
Connection ~ 3275 5675
Wire Wire Line
	3325 5675 3375 5675
Wire Wire Line
	3375 5675 3375 5625
Connection ~ 3325 5675
Wire Wire Line
	3375 5675 3425 5675
Wire Wire Line
	3425 5675 3425 5625
Connection ~ 3375 5675
$Comp
L power:GND #PWR010
U 1 1 5F9E09FC
P 6275 5725
F 0 "#PWR010" H 6275 5475 50  0001 C CNN
F 1 "GND" H 6280 5552 50  0000 C CNN
F 2 "" H 6275 5725 50  0001 C CNN
F 3 "" H 6275 5725 50  0001 C CNN
	1    6275 5725
	1    0    0    -1  
$EndComp
Wire Wire Line
	6275 5575 6275 5650
Wire Wire Line
	6275 5650 6475 5650
Wire Wire Line
	6475 5650 6475 5575
Connection ~ 6275 5650
Wire Wire Line
	6275 5650 6275 5725
$Comp
L power:GND #PWR019
U 1 1 5FA02DE8
P 9725 5775
F 0 "#PWR019" H 9725 5525 50  0001 C CNN
F 1 "GND" H 9730 5602 50  0000 C CNN
F 2 "" H 9725 5775 50  0001 C CNN
F 3 "" H 9725 5775 50  0001 C CNN
	1    9725 5775
	1    0    0    -1  
$EndComp
Wire Wire Line
	9725 5675 9725 5750
Wire Wire Line
	9725 5750 9825 5750
Wire Wire Line
	9825 5750 9825 5675
Connection ~ 9725 5750
Wire Wire Line
	9725 5750 9725 5775
Wire Wire Line
	9825 5750 9925 5750
Wire Wire Line
	9925 5750 9925 5675
Connection ~ 9825 5750
Wire Wire Line
	9925 5750 10025 5750
Wire Wire Line
	10025 5750 10025 5675
Connection ~ 9925 5750
$Comp
L power:GND #PWR024
U 1 1 5FA4BF7E
P 12950 5500
F 0 "#PWR024" H 12950 5250 50  0001 C CNN
F 1 "GND" H 12955 5327 50  0000 C CNN
F 2 "" H 12950 5500 50  0001 C CNN
F 3 "" H 12950 5500 50  0001 C CNN
	1    12950 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	12950 5350 12950 5450
Wire Wire Line
	12950 5450 13050 5450
Wire Wire Line
	13050 5450 13050 5350
Connection ~ 12950 5450
Wire Wire Line
	12950 5450 12950 5500
Wire Wire Line
	13050 5450 13250 5450
Wire Wire Line
	13250 5450 13250 5350
Connection ~ 13050 5450
Wire Wire Line
	13250 5450 13350 5450
Wire Wire Line
	13350 5450 13350 5350
Connection ~ 13250 5450
Wire Wire Line
	13350 5450 13450 5450
Wire Wire Line
	13450 5450 13450 5350
Connection ~ 13350 5450
Wire Wire Line
	13450 5450 13550 5450
Wire Wire Line
	13550 5450 13550 5350
Connection ~ 13450 5450
Wire Wire Line
	13550 5450 13650 5450
Wire Wire Line
	13650 5450 13650 5350
Connection ~ 13550 5450
$Comp
L power:GND #PWR022
U 1 1 5FAD28C2
P 11825 2625
F 0 "#PWR022" H 11825 2375 50  0001 C CNN
F 1 "GND" H 11830 2452 50  0000 C CNN
F 2 "" H 11825 2625 50  0001 C CNN
F 3 "" H 11825 2625 50  0001 C CNN
	1    11825 2625
	1    0    0    -1  
$EndComp
Wire Wire Line
	11675 2550 11825 2550
Wire Wire Line
	11825 2550 11825 2625
$Comp
L power:GND #PWR014
U 1 1 5FAE9572
P 7925 2475
F 0 "#PWR014" H 7925 2225 50  0001 C CNN
F 1 "GND" H 7930 2302 50  0000 C CNN
F 2 "" H 7925 2475 50  0001 C CNN
F 3 "" H 7925 2475 50  0001 C CNN
	1    7925 2475
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 2425 7925 2425
Wire Wire Line
	7925 2425 7925 2475
Wire Wire Line
	7925 2425 7925 2275
Wire Wire Line
	7925 2275 7800 2275
Connection ~ 7925 2425
$Comp
L Device:R R1
U 1 1 5FB2C049
P 4350 1225
F 0 "R1" V 4143 1225 50  0000 C CNN
F 1 "0" V 4234 1225 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4280 1225 50  0001 C CNN
F 3 "~" H 4350 1225 50  0001 C CNN
	1    4350 1225
	0    1    1    0   
$EndComp
Wire Wire Line
	4025 1225 4200 1225
Wire Wire Line
	4500 1225 4550 1225
Wire Wire Line
	4675 1175 4675 1225
$Comp
L Device:C C11
U 1 1 5FC78204
P 4675 1400
F 0 "C11" H 4790 1446 50  0000 L CNN
F 1 "10u" H 4790 1355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4713 1250 50  0001 C CNN
F 3 "~" H 4675 1400 50  0001 C CNN
	1    4675 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4675 1250 4675 1225
Connection ~ 4675 1225
$Comp
L power:GND #PWR09
U 1 1 5FC97369
P 4675 1600
F 0 "#PWR09" H 4675 1350 50  0001 C CNN
F 1 "GND" H 4680 1427 50  0000 C CNN
F 2 "" H 4675 1600 50  0001 C CNN
F 3 "" H 4675 1600 50  0001 C CNN
	1    4675 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4675 1550 4675 1600
$Comp
L Device:R RX1
U 1 1 5FCBEF8A
P 4350 1625
F 0 "RX1" V 4143 1625 50  0000 C CNN
F 1 "0" V 4234 1625 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4280 1625 50  0001 C CNN
F 3 "~" H 4350 1625 50  0001 C CNN
	1    4350 1625
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 1625 4025 1625
Wire Wire Line
	4500 1625 4550 1625
Wire Wire Line
	4550 1625 4550 1225
Connection ~ 4550 1225
Wire Wire Line
	4550 1225 4675 1225
Text Label 7675 6425 0    50   ~ 0
FMC_NE1
Text Label 9450 6575 0    50   ~ 0
FMC_NE3
Text Label 12500 6400 0    50   ~ 0
FMC_NWAIT
Text Label 13025 6400 0    50   ~ 0
FMC_NE4
Text Label 7675 6025 0    50   ~ 0
VDD_SRAM1
$Comp
L Device:R RX2
U 1 1 5FD41767
P 7675 6225
F 0 "RX2" V 7575 6225 50  0000 C CNN
F 1 "20k" V 7675 6225 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7605 6225 50  0001 C CNN
F 3 "~" H 7675 6225 50  0001 C CNN
	1    7675 6225
	-1   0    0    1   
$EndComp
Wire Wire Line
	7675 6075 7675 6025
Wire Wire Line
	7675 6375 7675 6425
$Comp
L Device:R RX4
U 1 1 5FDBAD7D
P 9450 6350
F 0 "RX4" V 9350 6350 50  0000 C CNN
F 1 "20k" V 9450 6350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9380 6350 50  0001 C CNN
F 3 "~" H 9450 6350 50  0001 C CNN
	1    9450 6350
	-1   0    0    1   
$EndComp
Text Label 9450 6150 0    50   ~ 0
VDD_SRAM2
Wire Wire Line
	9450 6500 9450 6575
Wire Wire Line
	9450 6200 9450 6150
$Comp
L Device:R RX5
U 1 1 5FE3551C
P 12500 6175
F 0 "RX5" V 12400 6175 50  0000 C CNN
F 1 "20k" V 12500 6175 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12430 6175 50  0001 C CNN
F 3 "~" H 12500 6175 50  0001 C CNN
	1    12500 6175
	-1   0    0    1   
$EndComp
$Comp
L Device:R RX7
U 1 1 5FE72F45
P 13025 6175
F 0 "RX7" V 12925 6175 50  0000 C CNN
F 1 "20k" V 13025 6175 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 12955 6175 50  0001 C CNN
F 3 "~" H 13025 6175 50  0001 C CNN
	1    13025 6175
	-1   0    0    1   
$EndComp
Text Label 12500 5900 0    50   ~ 0
VDD_NOR
Wire Wire Line
	12500 5900 12500 5950
Wire Wire Line
	12500 6325 12500 6400
Wire Wire Line
	12500 5950 13025 5950
Wire Wire Line
	13025 5950 13025 6025
Connection ~ 12500 5950
Wire Wire Line
	12500 5950 12500 6025
Wire Wire Line
	13025 6325 13025 6400
$Comp
L Device:R RX11
U 1 1 5FED5B92
P 15225 6000
F 0 "RX11" V 15125 5975 50  0000 C CNN
F 1 "20k" V 15225 6000 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 15155 6000 50  0001 C CNN
F 3 "~" H 15225 6000 50  0001 C CNN
	1    15225 6000
	-1   0    0    1   
$EndComp
Text Label 15225 6300 0    50   ~ 0
FMC_NWAIT
Wire Wire Line
	15225 6150 15225 6300
Wire Wire Line
	15225 5850 15225 5775
NoConn ~ 2625 2025
NoConn ~ 2625 2225
NoConn ~ 2625 2325
NoConn ~ 2625 2425
NoConn ~ 2625 2525
NoConn ~ 2625 2625
NoConn ~ 2625 2725
NoConn ~ 2625 2925
NoConn ~ 2625 3025
NoConn ~ 2625 3325
NoConn ~ 2625 3625
NoConn ~ 2625 3725
NoConn ~ 2625 3825
NoConn ~ 2625 3925
NoConn ~ 2625 4025
NoConn ~ 2625 4125
NoConn ~ 2625 4525
NoConn ~ 2625 4625
NoConn ~ 4025 4925
NoConn ~ 4025 4825
NoConn ~ 4025 4525
NoConn ~ 4025 4325
NoConn ~ 4025 3525
NoConn ~ 4025 3425
NoConn ~ 4025 3325
NoConn ~ 4025 3225
NoConn ~ 4025 1925
NoConn ~ 4025 1725
NoConn ~ 4025 1525
NoConn ~ 4025 1425
NoConn ~ 4025 1325
NoConn ~ 5200 1625
NoConn ~ 5200 1825
NoConn ~ 5200 1925
NoConn ~ 5200 2025
NoConn ~ 5200 2125
NoConn ~ 5200 2225
NoConn ~ 5200 2325
NoConn ~ 7800 1625
NoConn ~ 7800 1325
NoConn ~ 7800 1225
NoConn ~ 7800 1925
NoConn ~ 7800 2025
NoConn ~ 5325 3475
NoConn ~ 5325 3575
NoConn ~ 5325 3675
NoConn ~ 5325 3775
NoConn ~ 5325 3875
NoConn ~ 5325 3975
NoConn ~ 5325 4175
NoConn ~ 5325 4275
NoConn ~ 5325 4375
NoConn ~ 5325 4475
NoConn ~ 5325 4575
NoConn ~ 5325 4775
NoConn ~ 5325 4875
NoConn ~ 5325 4975
NoConn ~ 5325 5075
NoConn ~ 7725 5075
NoConn ~ 7725 4975
NoConn ~ 7725 4875
NoConn ~ 7725 4775
NoConn ~ 7725 4575
NoConn ~ 7725 4375
NoConn ~ 7725 4275
NoConn ~ 7725 4175
NoConn ~ 7725 3975
NoConn ~ 7725 3875
NoConn ~ 7725 3675
NoConn ~ 7725 3575
NoConn ~ 7725 3475
NoConn ~ 8725 4975
NoConn ~ 8725 4875
NoConn ~ 8725 4775
NoConn ~ 8725 4675
NoConn ~ 8725 4575
NoConn ~ 8725 4475
NoConn ~ 8725 4275
NoConn ~ 8725 4175
NoConn ~ 8725 4075
NoConn ~ 8725 3875
NoConn ~ 8725 3775
NoConn ~ 8725 3575
NoConn ~ 8725 3475
NoConn ~ 8725 3375
NoConn ~ 11125 3175
NoConn ~ 11125 3375
NoConn ~ 11125 3475
NoConn ~ 8675 1350
NoConn ~ 8675 1450
NoConn ~ 8675 1750
NoConn ~ 8675 1850
NoConn ~ 8675 1950
NoConn ~ 8675 2050
NoConn ~ 8675 2150
NoConn ~ 8675 2250
NoConn ~ 8675 2350
NoConn ~ 11675 1150
NoConn ~ 11675 1350
NoConn ~ 11675 1450
NoConn ~ 11675 1550
NoConn ~ 11675 1650
NoConn ~ 11675 1850
NoConn ~ 11675 1950
NoConn ~ 11675 2150
NoConn ~ 11675 2250
NoConn ~ 11675 2350
NoConn ~ 11125 3575
NoConn ~ 11125 3675
NoConn ~ 11125 3775
NoConn ~ 11125 3975
NoConn ~ 11125 4175
NoConn ~ 11125 4275
NoConn ~ 11125 4375
NoConn ~ 11125 4575
NoConn ~ 11125 4775
NoConn ~ 11125 4875
NoConn ~ 11125 4975
NoConn ~ 11125 5075
NoConn ~ 9625 5675
NoConn ~ 13150 5350
NoConn ~ 12400 1350
NoConn ~ 12400 1450
NoConn ~ 12400 1550
NoConn ~ 12400 1650
NoConn ~ 12400 1750
NoConn ~ 12400 1850
NoConn ~ 12400 1950
NoConn ~ 12400 2050
NoConn ~ 12400 2150
NoConn ~ 12400 2250
NoConn ~ 12400 2450
NoConn ~ 12400 2550
NoConn ~ 12400 2650
NoConn ~ 12400 2750
NoConn ~ 14200 1150
NoConn ~ 14200 1050
NoConn ~ 14200 3550
NoConn ~ 14200 4750
NoConn ~ 12400 3050
NoConn ~ 12400 3150
NoConn ~ 12400 3250
NoConn ~ 12400 3350
NoConn ~ 12400 3450
NoConn ~ 12400 3550
NoConn ~ 12400 3650
NoConn ~ 12400 3750
NoConn ~ 12400 3950
NoConn ~ 12400 4050
NoConn ~ 12400 4450
$Comp
L Device:R RX12
U 1 1 60DFF8ED
P 11125 9600
F 0 "RX12" V 11050 9500 50  0000 C CNN
F 1 "20k" V 11125 9600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 11055 9600 50  0001 C CNN
F 3 "~" H 11125 9600 50  0001 C CNN
	1    11125 9600
	0    1    1    0   
$EndComp
$Comp
L Device:R RX13
U 1 1 60E01A6A
P 11125 9775
F 0 "RX13" V 11050 9675 50  0000 C CNN
F 1 "20k" V 11125 9775 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 11055 9775 50  0001 C CNN
F 3 "~" H 11125 9775 50  0001 C CNN
	1    11125 9775
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 60E2E723
P 11425 9800
F 0 "#PWR0101" H 11425 9550 50  0001 C CNN
F 1 "GND" H 11430 9627 50  0000 C CNN
F 2 "" H 11425 9800 50  0001 C CNN
F 3 "" H 11425 9800 50  0001 C CNN
	1    11425 9800
	1    0    0    -1  
$EndComp
Wire Wire Line
	11075 9325 10875 9325
Wire Wire Line
	10875 9325 10875 9600
Wire Wire Line
	10875 9600 10975 9600
Wire Wire Line
	11075 9225 10750 9225
Wire Wire Line
	10750 9225 10750 9775
Wire Wire Line
	10975 9775 10750 9775
Wire Wire Line
	11275 9775 11425 9775
Wire Wire Line
	11425 9775 11425 9800
Wire Wire Line
	11425 9775 11425 9600
Wire Wire Line
	11425 9600 11275 9600
Connection ~ 11425 9775
NoConn ~ 7800 1425
NoConn ~ 14200 4450
NoConn ~ 12400 4150
NoConn ~ 12400 4250
NoConn ~ 12400 4350
Text Notes 7875 11000 0    50   ~ 0
ex)CY62157EV30LL-45ZSXI
Text Notes 11950 9950 0    50   ~ 0
ex)MT28EW512ABA1LJS-0SIT
Text Notes 14500 9925 0    50   ~ 0
ex)MT29F8G08ADAFAWP-AIT
Text Notes 2325 11000 0    50   ~ 0
ex)MT48LC16M16A2P-6A AIT
$EndSCHEMATC
