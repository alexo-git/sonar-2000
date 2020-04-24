# sonar-2000
Simple and cheap underwater sonar.

This is my very old project (1999-2000) but some schematic art and tricks may be still actual.
Some components is out of production now and should be replaced.

#### Short description

This sonar was developed as part of underwater pool cleaner robot project.
The sonar had rotated transducer head mounted on shaft of stepper motor to get 2D map.
The internal MCU (PIC16C76) receive and digitize analog signal from logarithmic amplifier, 
control boost convertor and stepper motor.
After initialization MCU going to wait for a command from host CPU (over UART).
The host CPU can control the sonar with some set of command (see "sonar-2000.c" file)

Hardware design have three main part:
- Analog logarithmic amplifier
- Boost converter and transducer amplifier
- MCU and stepper-motor driver

#### Analog logarithmic amplifier
The signals received by transducer have huge dynamic range - up to 90-100db.
So big range is need to accept echo from objects located on very different distance - from 1 meter up to 1 km.
Usually, to match such strong dynamic range requirements logarithmic amplifiers is used.
Also, logarithmic amplifier compress signal and low-cost ADC with 10-12 bit can be used.
But good logarithmic amplifier quite expensive (for example AD8310 price ~ $10 / 10 pcs).
At the time this device was developed, a good logarithmic amplifier cost even more - up to $ 25 in 1999!
Therefore, to comply with budgetary restrictions, I went for a trick - I used an inexpensive RF receiver chip (MC3371) with RSSI output.
The RSSI (Received Signal Strength Indicator) actually have logarithmic gain characteristics!
All another blocks of RF-receiver (like mixer and demodulator) is not used.
To improve selectivity and noise resistance LNA based on tuned amplifier is used (transistor Q1, MMBT5089L).
Diodes D1 and D2 protect LNA input from high voltage signals during transducer exitate.
Output of the amplifier connected to ADC of the microcontroller. 
C18 capacitor together with output impedance of MC3371 used as RC low-pass filter (3KHz) limited bandwith for ADC (10 KHz sample rate).
Note: Today the MC3371 chip is out of production but it can be replaced with many other alternatives, for example SA614A (~ $3).
Also possible to use cheap RF power meter chip (e.g LT5537, AD8310).

#### Boost converter
Transducer exitate required high voltage - up to 100V.
The dedicated step-up switch coverter can be used to get such high voltage, but need take into account that noise produced by step-up converter can degrade performance of logarithmic amplifier. 
The gain if logarithmic amplifier about 100 db, so even small noise can limit sensetivity!
Also, the low-noise high-voltage step-up converter is expensive (for example LT3482 ~ $5)
I met both requirements with a hybrid software / hardware solution - the microcontroller software has become part of the boost converter!
The hardware part of converter is Q5, D3, L3, C22. It is regular step-up converer topology. The microcontroller produce pulses for
Q5  and 'pumping' voltage on C22. The microcontroller measure voltage on voltage divider R16/R22 and stop pumping when this value reach some threshold. This threshold can be programmed from host CPU and define voltage for exitation pulses. 
The microcontroller DOES NOT pump while receiving an echo and have 'zero noise' during measurement!
This “pumping” process is very fast - less than 1ms and performed between pulses.

#### Transducer amplifier
Transducer amplifier should operate with high voltage (100V) and high capacitance loading on frequency ~200 KHz.
The amplifier designed on transistors Q2 and Q3 which is push-poll stage and Q4 which is current-protection circuit. 
This very simple and great solution described in detail in Horowitz and Hill, “Art of Electronics” book.
The exitation pulses (TX_PULSE) is produced by MCU. 
The filter L2/C21 block 200 KHz path during receive phase.
Important things - after transmit pulses keep Q3 closed (0V gate) - this will prevent current flow thru R12 and discharge C22.


#### MCU and stepper-motor driver
The microcontroller perform all tasks for initialization, host communication and manage boost converter, rx/tx and stepper motor.
The chip (PIC16C76) is out of production for long time, but it can be replaced with PIC16F76 or similar.
Please check "sonar-2000.c" source code for detail.

