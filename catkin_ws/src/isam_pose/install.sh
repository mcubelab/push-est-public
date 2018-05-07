# for isam
sudo apt-get install cmake libsuitesparse-dev libeigen3-dev libsdl1.2-dev doxygen graphviz
cd ${CODE_BASE}/software/externals/isam_v1_7
make

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

# for jsoncpp
cd ${CODE_BASE}/software/externals/jsoncpp-src-0.5.0
python scons.py platform=linux-gcc
cp -r include ${CODE_BASE}/software/build/
mkdir -p ${CODE_BASE}/software/build/lib/
cp -r libs/linux-gcc-5.4.0/* ${CODE_BASE}/software/build/lib/

