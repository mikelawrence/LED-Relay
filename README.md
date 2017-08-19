# LED Relay Replacement PCB
This board was designed to act as a relay for multi-color strip LED lighting on my wife's Harley Tri Glide. The Infineon BTS7008 Dual High-Side Switch provides up to 7.5A per channel. The software allows a quick on-off-on sequence that keeps the LEDs lit after the trike is turned off for a programmable delay time in minutes.
There are 4 inputs to the LED Relay board:
* +12V - This should be connected through a fuse to the battery and should remain hot even when the trike is off. All power to the outputs comes from this wire.
* ACC1 - This is +12V when the bike is turned on (ignition).
* ACC2 - This is +12V when the LED lights should be on.
* GND - Chassis and negative terminal of the battery.
The current software turns both outputs on when both ACC1 and ACC2 are at +12V but this can easily be changed to two independent outputs each controlled by its own ACC input. When either ACC1 off the micro-controller enters a low power state waiting for ACC1 to go back to +12V. Note with my current setup ACC2 can never go to +12V without ACC1 also going to +12V.

## Status and Testing
* Rev 2.2 PCB is still in progress.
  * Added ACC1 connection to PD2 on micro-controller. Only PIN2 has full asynchronous edge detection needed for ACC1.
  * You can order parts from Mouser using this [shared BOM](http://www.mouser.com/ProjectManager/ProjectDetail.aspx?AccessID=7477630473). Note LT3014 is not available at Mouser you must get it somewhere else like Digikey.
  * You can order the PCB from OSH Park using this [link](https://oshpark.com/shared_projects/Q2hSPIYz) to OSH Park.
* Rev 2.1 PCB has been ordered from OSH Park, assembled, and testing is in progress.
  * Added R12 and R13.
  * Moved ACC connections on micro-controller.
* Rev 2.0 PCB.
  * Converted older design to KiCad.
  * Switched from low-side switch to dual high-side switch.
* Rev 1.0 Existed as another design that was close.

## Board Preview
<img src="meta/LED Relay-brd.png" style="width:100%">

## Notes
* The current usage is only one channel with a typical 4A LED load and maximum of 6A. I have not done any load testing to see how well the board operates with both channels going full bore. The High-Side Switch will shutdown to protect itself if it get too hot.
* When ACC1 is off the XMega8E5 enters an ultra low power mode drawing about 1µA. Add that to the LT3014 linear regulator's quiescent current and this board has standby current of 8µA. In bench testing my first board used 7.8µA at room temperature. Keep in mind this current is likely to double or even triple at higher temperatures but for a motorcycle or car battery this standby current is excellent.
* Assembling the this PCB will require a reflow oven. If you don't have one [Whizoo](http://www.whizoo.com/) sells a nice kit to make your own.
* PCB is designed in [KiCad](http://kicad-pcb.org) which is a great free EDA toolset.

## Software
The microcontroller is a Atmel/Microchip XMega8E5. Software is written in C using the free [Atmel Studio 7](https://www.google.com/search?q=atmel+studio+7). I recommend the Atmel ICE to both debug and program the XMega. Especially since it already has a 50mil PDI connector to plug directly onto the LED Relay board.

## Protection Against the Elements
When mounting on a motorcycle protection against the elements is crucial to longevity so the LED Relay board is thin enough to get 1" adhesive heat shrink on it. Once shrunk the board is well protected and the wires also get some strain relief.

Note I used several different sizes of adhesive heat shrink to get a secure seal around the wires.

## Final Result
Here is the obligatory end result picture!
<img src="meta/Trike LEDs.jpg" style="width:100%">
