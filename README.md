# scaled_paraglider_testbed
Control software for Scaled Paraglider Testbed senior design project.

Type make in program directory to build.

Files:
- paraglider_config.cfg -- Configuration file, list of settings that can easily be changed without needing to recompile.
- controller.c -- Main control code.
- install.sh -- Installs dependency packages.
- Makefile -- compiles with correct dependencies and flags.

To run: 'sudo ./controller'

Command List: 
- display - Displays orientation data. 'display exit' stops display output.
- drive ## - Sets desired angle of ## and sends to weight shift motor. 
- exit - Quit control software.
- help - Displays list of commands.
