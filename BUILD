Install required packages:
  sudo apt-get install git g++-4.7 make grep mawk ntp
Note that this code requires g++-4.7 which is not installed by default in
Raspbian!

Make sure you are using the latest kernel by updating your system. The latest
kernel includes fixes wich improve NTP ppm measurement accuracy:
  sudo apt-get update
  sudo apt-get dist-upgrade

Get code/ compile:
  rm -rf PiCW
  git clone https://github.com/JamesP6000/PiCW.git
  cd PiCW
  make
Note that compiling takes about 60 seconds on the Pi.

Install to /usr/local/bin:
  sudo make install

Uninstall:
  sudo make uninstall

