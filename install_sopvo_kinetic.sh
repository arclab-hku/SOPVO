mkdir bag

sudo apt-get install libsuitesparse-dev 
sudo apt-get install ros-kinetic-octomap

sudo apt-get install liblapack-dev liblapack3 libopenblas-base libopenblas-dev
sudo apt-get install libmumps-seq-dev
sudo apt-get install sdpa
sudo apt-get install libsdpa-dev

cd 3rdPartLib

cd yaml-cpp-0.6.2
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j4
sudo make install

cd ../..

cd g2o
mkdir build
cd build
cmake ..
make -j4
sudo make install

cd ../..

cd Sophus
mkdir build
cd build
cmake ..
make -j4
sudo make install

cd ../..

cd DBow3
mkdir build
cd build
cmake ..
make -j4
sudo make install
