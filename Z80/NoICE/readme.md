<h1>Z80 debugger - NoICE.</h1>

NoICE is produced and maintained by John Hartman. Full details can be found on his web site at https://www.noicedebugger.com/index.html

<h2>How it Works:</h2>
<p>NoICE runs in two parts: a program on the host PC and a monitor program on the target microcontroller. The monitor program intercepts instructions from the host PC and allows for debugging actions like stepping, breakpoint insertion and register display.
This monitor program requires some customisation before it can be used, in particular, the memory map of the target system needs to be defined, and customised GETCH and PUTCH
routines need to be provided to control the ACIA hardware (RS-232).
 
Additionally, some format changes were required to allow the resulting monitor agent to assemble through the [Telemark Assembler](https://github.com/spotco/TI-asm/tree/master/tasm)
</p>

<h2>Monitor Agent Customisation</h2>

This folder demonstartes the process involved in customising the default NoICE agent.


Firstly the [Default NoIce Monitor Agent](https://github.com/oddwires/8-Bit-Systems/blob/9ceb676996fe873981be03973ea686bb2531277b/Z80/NoICE/Monz80.asm)
was downloaded from the NoICE web site.

Then [Minimum changes](https://github.com/oddwires/8-Bit-Systems/commit/961c3db7079678fbee4d80518fa17ca49584dca4)
are applied to create a working monitor agent compatable with my Z80 hardware and the Telemark assembler.

Finally [Cosmetic changes](https://github.com/oddwires/8-Bit-Systems/commit/a3de44857b3467403254bd7cdfddcb15b4b24da8)
 were applied to clean up the resulting code. *

\* Note: The cosmetic changes look great in my VS Code editor, with all columns and comments neatly aligned - unfortunately, they don't work so great in the Github listings.
