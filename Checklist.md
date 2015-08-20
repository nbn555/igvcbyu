# Introduction #

  1. Check motor batteries connections
  * _nominal first battery plugged into motor controller connection with RED into + and BLACK into -_
  1. Check auxiliary battery connections
  1. Check sensor cable connections according to the wiring diagram
  1. Check motor controller status light
  * **Connect all sensors to computer**
    * LIDAR(Serial/USB)
    * GPS(Serial/USB)
    * Compass(Serial/USB)
    * Camera(Firewire)
    * Motor Encoders(Parallel)
  1. Check GPS Power Status light
  1. Turn on Computer(Make sure she beeps)
  1. Connect wireless Network Yclops(or wired network)
  1. Connect to Y-Clops through SSH or VNC
    * Connect laptop to yclops machine using crossover cable while monitor is still connected
    * Find the ip address for yclops.  You can find this by typing ifconfig in the terminal
    * open the terminal on the laptop and ssh into it
    * change directories to workspace/wiiDriver _cd workspace/wiiDriver/_
    * This directory contains the overall configuration files and location files(.ini files)
    * to run the program type _debug/wiiDriver (name of ini file)_
    * when prompted, connect using the wiimote
    * start the appropriate challenge with the a or b button