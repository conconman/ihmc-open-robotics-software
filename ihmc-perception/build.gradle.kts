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
   val javaCPPVersion = "1.5.9-SNAPSHOT"
   api("org.bytedeco:javacpp:$javaCPPVersion")
   api("org.bytedeco:javacv:$javaCPPVersion")
   // openblas-0.3.23-1.5.9-20230404.015544-39
   api("org.bytedeco:openblas:0.3.23-1.5.9-20230404.015544-39")
   api("org.bytedeco:openblas:0.3.23-1.5.9-20230404.015544-39:linux-x86_64")
   api("org.bytedeco:openblas:0.3.23-1.5.9-20230404.015544-39:windows-x86_64")
   // opencv-4.7.0-1.5.9-20230516.151940-290
   api("org.bytedeco:opencv:4.7.0-1.5.9-20230516.151940-290")
   api("org.bytedeco:opencv:4.7.0-1.5.9-20230516.151940-290:linux-x86_64")
   api("org.bytedeco:opencv:4.7.0-1.5.9-20230516.151940-290:windows-x86_64")
   // ffmpeg-6.0-1.5.9-20230506.025210-104
   api("org.bytedeco:ffmpeg:6.0-1.5.9-20230506.025210-104")
   api("org.bytedeco:ffmpeg:6.0-1.5.9-20230506.025210-104:linux-x86_64")
   api("org.bytedeco:ffmpeg:6.0-1.5.9-20230506.025210-104:windows-x86_64")
   // opencl-3.0-1.5.9-20230508.065821-15
   api("org.bytedeco:opencl:3.0-1.5.9-20230508.065821-15")
   api("org.bytedeco:opencl:3.0-1.5.9-20230508.065821-15:linux-x86_64")
   api("org.bytedeco:opencl:3.0-1.5.9-20230508.065821-15:windows-x86_64")
   // librealsense2-2.53.1-1.5.9-20230108.102552-8
   api("org.bytedeco:librealsense2:2.53.1-1.5.9-20230108.102552-8")
   api("org.bytedeco:librealsense2:2.53.1-1.5.9-20230108.102552-8:linux-x86_64")
   api("org.bytedeco:librealsense2:2.53.1-1.5.9-20230108.102552-8:windows-x86_64")
   // spinnaker-3.0.0.118-1.5.9-20230218.091411-11
   api("org.bytedeco:spinnaker:3.0.0.118-1.5.9-20230218.091411-11")
   api("org.bytedeco:spinnaker:3.0.0.118-1.5.9-20230218.091411-11:linux-x86_64")
   api("org.bytedeco:spinnaker:3.0.0.118-1.5.9-20230218.091411-11:windows-x86_64")
   // hdf5-1.14.0-1.5.9-20230320.102117-24
   api("org.bytedeco:hdf5:1.14.0-1.5.9-20230320.102117-24")
   api("org.bytedeco:hdf5:1.14.0-1.5.9-20230320.102117-24:linux-x86_64")
   api("org.bytedeco:hdf5:1.14.0-1.5.9-20230320.102117-24:windows-x86_64")

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
   api("org.bytedeco:javacpp:1.5.9-SNAPSHOT")
   api("us.ihmc:ihmc-java-toolkit:source")
}

mapsenseWrapperDependencies {
   api("org.bytedeco:javacpp:1.5.9-SNAPSHOT")
   api("us.ihmc:ihmc-java-toolkit:source")
}
