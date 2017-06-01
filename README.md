# scaled_paraglider_testbed
Control software for Scaled Paraglider Testbed senior design project.

To install:
- Make install.sh executable, then run it with './install.sh' while in program directory.
- Type make in program directory to build.

To run: 'sudo ./controller'

Files:
- paraglider_config.cfg -- Configuration file, list of settings that can easily be changed without needing to recompile.
- controller.c -- Main control code.
- install.sh -- Installs dependency packages.
- Makefile -- Compiles with correct dependencies and flags.

Command List: 
- display - Displays orientation data. 'display exit' stops display output.
- drive ## - Sets desired angle of ## and sends to weight shift motor. 
- brakel # - Pulls left brake line down # centimeters.
- braker # - Pulls right brake line down # centimeters.
- brakes # - Pulls both brake lines down # centimeters.
- wingover - Performs the wingover maneuver.
- exit - Quit control software.
- help - Displays list of commands.
