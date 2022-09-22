#!/bin/bash
set -e -o xtrace

rm -rf build
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=. ..
make -j 4
make install

# Use the latest release on GitHub
# https://github.com/bytedeco/javacpp/releases
JAVACPP_VERSION=1.5.7

# Copy all Java code from the root of slam-wrapper into the build directory
cp -r ../../java/ .

# Move into the java directory; javacpp.jar needs to reside here
cd java

# Download and unzip javacpp into the java source directory
curl -L https://github.com/bytedeco/javacpp/releases/download/$JAVACPP_VERSION/javacpp-platform-$JAVACPP_VERSION-bin.zip -o javacpp-platform-$JAVACPP_VERSION-bin.zip
unzip -j javacpp-platform-$JAVACPP_VERSION-bin.zip

java -jar javacpp.jar org/bytedeco/slamWrapper/presets/SlamWrapperInfoMapper.java
# This will generate the jni shared library and place it into the classpath resources dir
java -jar javacpp.jar org/bytedeco/slamWrapper/SlamWrapper.java -d ../../../resources

# Clean old generated Java code
rm -rf ../../../generated-java/*

# Copy newly generated Java into generated-java
mkdir -p ../../../generated-java/org/bytedeco/slamWrapper
cp -r org/bytedeco/slamWrapper/SlamWrapper.java ../../../generated-java/org/bytedeco/slamWrapper
