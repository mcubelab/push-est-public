# for isam
sudo apt-get install cmake libsuitesparse-dev libeigen3-dev libsdl1.2-dev doxygen graphviz
cd ${CODE_BASE}/software/externals/isam_v1_7
make

# for adol-c
#~ cd ../ADOL-C-2.6.0
#~ ./configure --prefix=${CODE_BASE}/build
#~ make clean
#~ make
#~ make install 


#
cd ${CODE_BASE}/software/externals/cppad-20161231
./configure --prefix=${CODE_BASE}/software/build
make
make install

# for scipy
sudo apt-get install python-pip python-dev build-essential 
sudo pip install --upgrade pip 
sudo pip install --upgrade virtualenv 
sudo pip install pandas 

# for egm
sudo apt --yes install protobuf-compiler
sudo pip install protobuf
sudo apt-get install python-xlib

# for jsoncpp
cd ${CODE_BASE}/software/externals/jsoncpp-src-0.5.0
python scons.py platform=linux-gcc
cp -r include ${CODE_BASE}/software/build/
mkdir -p ${CODE_BASE}/software/build/lib/
cp -r libs/linux-gcc-5.4.0/* ${CODE_BASE}/software/build/lib/

