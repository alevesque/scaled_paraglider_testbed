# scaled_paraglider_testbed
Control software for Scaled Paraglider Testbed senior design project.

##To install:
- Make install.sh executable, then run it with './install.sh' while in program directory.
- Type make in program directory to build.

To run: 'sudo ./controller'

##Files:
- paraglider_config.cfg -- Configuration file, list of settings that can easily be changed without needing to recompile.
- controller.c -- Main control code.
- install.sh -- Installs dependency packages. Must change the wifi address and set up wifi profile according to connmanctl. See below for example.
- Makefile -- Compiles with correct dependencies and flags.

##Command List: 
- display - Displays orientation data. 'display exit' stops display output.
- drive ## - Sets desired angle of ## and sends to weight shift motor. 
- brakel # - Pulls left brake line down # centimeters.
- braker # - Pulls right brake line down # centimeters.
- brakes # - Pulls both brake lines down # centimeters.
- wingover - Performs the wingover maneuver.
- exit - Quit control software.
- help - Displays list of commands.


##Example connmanctl wifi profile for university-style internet with login credentials.  
###Place in text file called WIFI_NAME.config.  

[service_WIFI_NAME]  
 Type = wifi  
Name = WIFI_NAME  
EAP = peap  
Phase2 = MSCHAPV2  
Identity = username  
Passphrase = password  
