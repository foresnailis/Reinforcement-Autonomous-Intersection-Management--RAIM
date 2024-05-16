cd /root

sudo apt-get install cmake python g++ libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev libgl2ps-dev swig

wget -O sumo-src-1.19.0.tar.gz https://sourceforge.net/projects/sumo/files/sumo/version%201.19.0/sumo-src-1.19.0.tar.gz/download

tar xzf sumo-src-1.19.0.tar.gz
cd sumo-1.19.0
mkdir build/cmake-build
cd build/cmake-build
cmake ../..
make -j 5

echo "export PATH=$PATH:/root/sumo-1.19.0/bin" >> ~/.bashrc
echo "export SUMO_HOME=/root/sumo-1.19.0" >> ~/.bashrc
source ~/.bashrc

cd /root/miniconda3/lib/python3.8/site-packages
echo "/root/sumo-1.19.0/tools" >> traci.pth
cd /root/RAIM
sumo -V