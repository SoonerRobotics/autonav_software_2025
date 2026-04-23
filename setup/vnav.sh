# Download and unzip vectornav files
curl --output vectornav.zip --netrc-file vectorsecrets.txt https://files.dylanzeml.in/protected/vectornav.zip
sudo mkdir /usr/local/vectornav
sudo chown -R "$USER":"$USER" /usr/local/vectornav
unzip -o vectornav.zip -d /usr/local/vectornav
rm vectornav.zip

# Install vectornav
sudo apt install python3-pybind11
python3 -m pip config set global.break-system-packages true
cd /usr/local/vectornav/python
sudo python3 setup.py install -q --break-system-packages