package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.tools.property.*;

/**
 * The JSON file for this property set is located here:
 * robot-environment-awareness/src/main/resources/us/ihmc/robotEnvironmentAwareness/geometry/ConcaveHullFactoryParameters.json
 *
 * This class was auto generated. Property attributes must be edited in the JSON file,
 * after which this class should be regenerated by running the main. This class uses
 * the generator to assist in the addition, removal, and modification of property keys.
 * It is permissible to forgo these benefits and abandon the generator, in which case
 * you should also move it from the generated-java folder to the java folder.
 *
 * If the constant paths have changed, change them in this file and run the main to regenerate.
 */
public class ConcaveHullFactoryParameters extends StoredPropertySet implements ConcaveHullFactoryParametersBasics
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "robot-environment-awareness/src/main/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "robot-environment-awareness/src/main/generated-java";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey edgeLengthThreshold = keys.addDoubleKey("Edge length threshold");
   public static final BooleanStoredPropertyKey removeAllTrianglesWithTwoBorderEdges = keys.addBooleanKey("Remove all triangles with two border edges");
   public static final BooleanStoredPropertyKey allowSplittingConcaveHull = keys.addBooleanKey("Allow splitting concave hull");
   public static final IntegerStoredPropertyKey maxNumberOfIterations = keys.addIntegerKey("Max number of iterations");
   public static final DoubleStoredPropertyKey triangulationTolerance = keys.addDoubleKey("Triangulation tolerance");

   public ConcaveHullFactoryParameters()
   {
      this("");
   }

   public ConcaveHullFactoryParameters(String versionSpecifier)
   {
      super(keys, ConcaveHullFactoryParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, versionSpecifier);
      load();
   }

   public ConcaveHullFactoryParameters(StoredPropertySetReadOnly other)
   {
      super(keys, ConcaveHullFactoryParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           ConcaveHullFactoryParameters.class,
                                                           DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                           SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
   }
}
