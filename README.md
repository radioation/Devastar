# Devastar
This project is an attempt to create a flatscreen lightgun to work with my Sega Menacer game (and any other console and games)

The project is still in its early stages and it will be a while before I release the circuitry.  

It's now usable, but needs some refinement.  
[![IR Menacer](https://img.youtube.com/vi/hTlZYFlo4qI/0.jpg)](https://www.youtube.com/watch?v=hTlZYFlo4qI)
[![IR Atari XG-1](https://img.youtube.com/vi/AgynEI8CHoo/0.jpg)](https://www.youtube.com/watch?v=AgynEI8CHoo)


# License 
Most of the files here are MIT licensed.  The arduino files are LGPL.

# LM1881
<coming soon> 

# Faking a Menacer
## Circuit
<coming soon~ish> 

## Menacer X/Y Positioning
Setting the menacer position means pulling the Genesis TH pin low at the appropriate time.  I can use
the output from a LM1881 sync separator to determine the current scanline (Y) and wait (X) to
set the horizontal position on the scanline.   A few experiments with a Genesis 
Model 2 and Arduino Leonardo gave me the following values:

* On-screen Y ranges from 30 to 250
* On-screen X ranges from 19 to 68 with delayMicrosecond().

With 68 - 19 = 49 steps, waiting X microseconds to set the horizontal position
seems insufficient.  I will try a faster microcontroller, but right now I've only
got 16MHz Arduinos here.


Sub-microsecond delay increments can be obtained with a custom delay function:
~~~
void delayX4Cycles(unsigned int c)
{
        if (--c == 0)
                return;

        // busy wait
        __asm__ __volatile__ (
                "1: sbiw %0,1" "\n\t" // 2 cycles
                "brne 1b" : "=w" (c) : "0" (c) // 2 cycles
        );
}
~~~

This gave me an on-screen X range of about 73 to 269 or 196 steps.  Which
is probably OK considering the resoultion of the games. 




