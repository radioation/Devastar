# Devastar
This project is my attempt to create a flatscreen lightgun for work with my Sega Menacer game (and any other Menacer game if I get it to work)

It is currently in its early stages and will be a while before it becomes usable.

# License 
Most of the files here are MIT licensed.  The arduino meancer.ino file is LGPL.

# LM1881


# Faking a Menacer
## Circuit

## Menacer X/Y Positioning
Setting the menacer position means setting TH at the appropriate time.  I can use
the output from an LM1881 to detertime the current scanline (Y) and wait X to
set the horizontal position on the scanline.   A few experiments with a Genesis 
Model 2 and Arduino Leonardo gave me the following values:

* On-screen Y ranges from 30 to 250
* On-screen X ranges from 19 to 68 with delayMicrosecond().

With 68 - 19 = 49 steps, waiting X microseconds to set the horizontal position
seems insufficient.  I will try a faster microcontroller, but right now I've only
got slow Arduinos here.


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
is probably good enough.




