#!/bin/bash

# save the current directory for later use
CURRENT_DIR=$(pwd)

# go into the jsbsim folder
cd external/jsbsim

# make build folder and cd into it
mkdir -p build
cd build

# build the jsbsim library with cmake + gcc
cmake -DBUILD_SHARED_LIBS=ON \
      -DCMAKE_C_COMPILER=gcc \
      -DCMAKE_CXX_COMPILER=g++ ..

make -j$(nproc)

cd ..

# set the lib and include folders for use in the script
INCLUDE_FOLDER=$CURRENT_DIR/include
LIB_FOLDER=$CURRENT_DIR/lib

echo "Copying JSBSim header files to include folder: $INCLUDE_FOLDER"
# make the include folder
rm -rf "$INCLUDE_FOLDER"
mkdir -p "$INCLUDE_FOLDER"

# copy the include files (.h,.hpp,.hxx) from src (and subdirectories)
# into include folder, keeping directory structure
rsync -avm --include='*.h' --include='*.hpp' --include='*.hxx' -f 'hide,! */' src/ "$INCLUDE_FOLDER/"

echo "Copying JSBSim library to lib folder: $LIB_FOLDER"
# make the lib folder
rm -rf "$LIB_FOLDER"
mkdir -p "$LIB_FOLDER"

# copy the jsbsim library (.so or .a) from the build folder
cp ./build/src/*.so "$LIB_FOLDER/" 2>/dev/null || true
cp ./build/src/*.a  "$LIB_FOLDER/" 2>/dev/null || true

# go back out to the original directory
cd "$CURRENT_DIR"