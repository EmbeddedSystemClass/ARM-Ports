Command Line Interface
----------------------

This exercises each available port on the processor, aimed specifically at
the ET-ARM-STAMP. The ports are set to digital output and toggled in a pattern
as follows:

- Long high,
- short pulses counting the port number (A=1, B=2 etc),
- long low,
- short pulses counting pin number plus one (1 to 16).

Short pulses and gaps between are set to 0.5 seconds.

This allows an LED or meter to be used to identify the correct working of the
pin. The process can be operated at higher speed for use of a CRO.

(c) K. Sarkies 10/02/2018

