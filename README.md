# rapidradio
A rapidradio library and tools. Check the website http://rapidradio.pl/ for installation details.

Try: 
./rapidradio --help 
for command-line params description

The code already contains register definitions for the RFM75, because the RFM73 is being deprecated by HopeRf.

Getting and building:

sudo apt-get install autoconf

git clone https://github.com/micas-pro/rapidradio

cd rapidradio/raspberrypi

chmod 744 rapidradio_install.sh

./rapidradio_install.sh

./configure

make

sudo make install


Now you can enjoy your rapidradio!
