# Dependencies for building
sudo apt-get update
sudo apt-get install gcc lcov gcc-avr binutils-avr avr-libc -y

# Depdendencies for arduinokeywords
sudo apt-get install python3-pip -y
python3 -m pip install --upgrade pip setuptools wheel
pip3 install arduinokeywords
