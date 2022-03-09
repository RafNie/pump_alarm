## Pump station alarm

#### Motivation
I have sewage pumping stations. Unfortunately, it does not have an additional protection system against overfilling the tank in the event of the floats getting stuck. I cannot modify the automatics of the device, so I designed a current monitoring device

#### Idea
Ihe idea is based on measuring the current of the device. I know from observations that the device turns on on average once a day, at least once every two days.  
The device has a timer after which an acoustic alarm will be triggered. If the device registers the current, the counter is reset.

Additionally, in order not to manually switch off the monitoring device in the case of family trips, an additional activation input is designed. It is an analog input to which a photoresistor is connected. The level of light intensity in the hall is monitored. If there are people in the house, the light is turned on at least once a day.  
A photoresistor can be bridged if the alarm device should be always active.

#### Hardware
Digispark clone with ATTinny85

The device was mounted on a universal board attached to the digispark pins. I will try to draw a diagram soon.