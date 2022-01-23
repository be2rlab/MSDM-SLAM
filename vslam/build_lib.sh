echo "Start build lib ORB_SLAM3"
cd ORB_SLAM3
mkdir build  && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF ..
make -j2
cd ../Vocabulary
if [ ! -f ./ORBvoc.txt ] 
then
    echo "Vocabulary is not found!"
    tar -xf ORBvoc.txt.tar.gz
else
    echo "Vocabulary is found!"
fi