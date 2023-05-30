buildscript {
   dependencies {
      classpath("org.apache.commons:commons-lang3:3.12.0")
   }
}

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.7"
   id("us.ihmc.ihmc-cd") version "1.24"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   configureDependencyResolution()
   javaDirectory("main", "generated-java")
   javaDirectory("slam-wrapper", "generated-java")
   javaDirectory("mapsense-wrapper", "generated-java")
   configurePublications()
}

//val javaCPPVersion = "1.5.9-SNAPSHOT"
val javaCPPVersion = "1.5.9-20230530.103643-355"

mainDependencies {
   api(ihmc.sourceSetProject("slam-wrapper"))
   api(ihmc.sourceSetProject("mapsense-wrapper"))
   // For experimenting with local OpenCV:
   // api(files("/usr/local/share/OpenCV/java/opencv-310.jar"))

   api("org.georegression:georegression:0.22")
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("net.java.dev.jna:jna:4.1.0")
   api("org.boofcv:boofcv-geo:0.36")
   api("org.boofcv:boofcv-ip:0.36")
   api("org.boofcv:boofcv-swing:0.36")
   api("org.boofcv:boofcv-io:0.36")
   api("org.boofcv:boofcv-recognition:0.36")
   api("org.boofcv:boofcv-calibration:0.36")
   api("org.ddogleg:ddogleg:0.18")

   // https://oss.sonatype.org/content/repositories/snapshots/org/bytedeco/
   api("org.bytedeco:javacpp:$javaCPPVersion")
   val javacvVersion = "1.5.9-20230514.133937-17"
   api("org.bytedeco:javacv:$javacvVersion")
   val openblasVersion = "0.3.23-1.5.9-20230404.015544-39"
   api("org.bytedeco:openblas:$openblasVersion")
   api("org.bytedeco:openblas:$openblasVersion:linux-x86_64")
   api("org.bytedeco:openblas:$openblasVersion:windows-x86_64")
   val opencvVersion = "4.7.0-1.5.9-20230516.151940-290"
   api("org.bytedeco:opencv:$opencvVersion")
   api("org.bytedeco:opencv:$opencvVersion:linux-x86_64")
   api("org.bytedeco:opencv:$opencvVersion:windows-x86_64")
   val ffmpegVersion = "6.0-1.5.9-20230528.134919-131"
   api("org.bytedeco:ffmpeg:$ffmpegVersion")
   api("org.bytedeco:ffmpeg:$ffmpegVersion:linux-x86_64")
   api("org.bytedeco:ffmpeg:$ffmpegVersion:windows-x86_64")
   val openclVersion = "3.0-1.5.9-20230508.065821-15"
   api("org.bytedeco:opencl:$openclVersion")
   api("org.bytedeco:opencl:$openclVersion:linux-x86_64")
   api("org.bytedeco:opencl:$openclVersion:windows-x86_64")
   val librealsense2Version = "2.53.1-1.5.9-20230108.102552-8"
   api("org.bytedeco:librealsense2:$librealsense2Version")
   api("org.bytedeco:librealsense2:$librealsense2Version:linux-x86_64")
   api("org.bytedeco:librealsense2:$librealsense2Version:windows-x86_64")
   val spinnakerVersion = "3.0.0.118-1.5.9-20230218.091411-11"
   api("org.bytedeco:spinnaker:$spinnakerVersion")
   api("org.bytedeco:spinnaker:$spinnakerVersion:linux-x86_64")
   api("org.bytedeco:spinnaker:$spinnakerVersion:windows-x86_64")
   val hdf5Version = "1.14.0-1.5.9-20230320.102117-24"
   api("org.bytedeco:hdf5:$hdf5Version")
   api("org.bytedeco:hdf5:$hdf5Version:linux-x86_64")
   api("org.bytedeco:hdf5:$hdf5Version:windows-x86_64")

   api("us.ihmc:euclid:0.20.0")
   api("us.ihmc:simulation-construction-set:0.23.4")
   api("us.ihmc:ihmc-native-library-loader:2.0.2")
   api("us.ihmc:ihmc-humanoid-robotics:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-ros-tools:source")
   api("us.ihmc:ihmc-whole-body-controller:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:robot-environment-awareness:source")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.32.0")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:simulation-construction-set-tools:source")
   api("us.ihmc:simulation-construction-set-tools-test:source")
}

slamWrapperDependencies {
   api("org.bytedeco:javacpp:$javaCPPVersion")
   api("us.ihmc:ihmc-java-toolkit:source")
}

mapsenseWrapperDependencies {
   api("org.bytedeco:javacpp:$javaCPPVersion")
   api("us.ihmc:ihmc-java-toolkit:source")
}
