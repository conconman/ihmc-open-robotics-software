plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.1"
   id("us.ihmc.ihmc-cd") version "1.24"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("javax.vecmath:vecmath:1.5.2")
   api("com.google.guava:guava:18.0")

   api("us.ihmc:ihmc-robot-description:0.21.10")
   api("us.ihmc:scs2-definition:17-0.14.7")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {

}
