echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cd ..

echo "Converting vocabulary to binary"
./tools/bin_vocabulary
