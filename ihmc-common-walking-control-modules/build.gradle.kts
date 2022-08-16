plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
      
   configurePublications()
}

mainDependencies {
   api("gov.nist.math:jama:1.0.3")
   api("com.google.guava:guava:18.0")
   api("org.ejml:ejml-ddense:0.39")
   api("org.ejml:ejml-core:0.39")
   api("net.sf.trove4j:trove4j:3.0.3")

   api("us.ihmc:ihmc-realtime:1.5.0")
   api("us.ihmc:ihmc-native-library-loader:1.3.1")
   api("us.ihmc:euclid:0.18.1")
   api("us.ihmc:euclid-geometry:0.18.1")
   api("us.ihmc:ihmc-robot-models:source")
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-trajectory-optimization:source")
   api("us.ihmc:ihmc-communication:source")
   api("us.ihmc:ihmc-convex-optimization:0.17.13")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-java-toolkit:source")
//   api("us.ihmc:ihmc-footstep-planning:source")
   api("us.ihmc:ihmc-humanoid-robotics:source")
}

testDependencies {
   api(ihmc.sourceSetProject("visualizers"))
   api("us.ihmc:ihmc-commons-testing:0.31.0")
   api("us.ihmc:simulation-construction-set-tools-test:source")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
   api("us.ihmc:ihmc-convex-optimization-test:0.17.13")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:simulation-construction-set:0.22.6")
   api("us.ihmc:scs2-simulation-construction-set:17-0.9.3")

   var javaFXVersion = "17.0.2"
   api(ihmc.javaFXModule("base", javaFXVersion))
   api(ihmc.javaFXModule("controls", javaFXVersion))
   api(ihmc.javaFXModule("graphics", javaFXVersion))
}
