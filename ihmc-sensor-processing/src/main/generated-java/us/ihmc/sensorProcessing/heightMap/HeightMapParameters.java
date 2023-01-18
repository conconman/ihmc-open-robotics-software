package us.ihmc.sensorProcessing.heightMap;

import us.ihmc.tools.property.*;

/**
 * The JSON file for this property set is located here:
 * ihmc-sensor-processing/src/main/resources/us/ihmc/sensorProcessing/heightMap/HeightMapParameters.json
 *
 * This class was auto generated. Property attributes must be edited in the JSON file,
 * after which this class should be regenerated by running the main. This class uses
 * the generator to assist in the addition, removal, and modification of property keys.
 * It is permissible to forgo these benefits and abandon the generator, in which case
 * you should also move it from the generated-java folder to the java folder.
 *
 * If the constant paths have changed, change them in this file and run the main to regenerate.
 */
public class HeightMapParameters extends StoredPropertySet implements HeightMapParametersBasics
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "ihmc-sensor-processing/src/main/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "ihmc-sensor-processing/src/main/generated-java";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   /**
    * Resolution of the height map grid
    */
   public static final DoubleStoredPropertyKey gridResolutionXY = keys.addDoubleKey("Grid resolution XY");
   /**
    * Length of the side of the square height map grid
    */
   public static final DoubleStoredPropertyKey gridSizeXY = keys.addDoubleKey("Grid size XY");
   /**
    * Max z relative to robot mid foot z. Points above this threshold are ignored.
    */
   public static final DoubleStoredPropertyKey maxZ = keys.addDoubleKey("Max Z");
   /**
    * When calibrated on flat ground, this is the average standard deviation observed
    * for a grid cell.
    */
   public static final DoubleStoredPropertyKey nominalStandardDeviation = keys.addDoubleKey("Nominal standard deviation");
   public static final IntegerStoredPropertyKey maxPointsPerCell = keys.addIntegerKey("Max points per cell");
   /**
    * If a grid cell is at height h, points below (h - s * m) are ignored, and points
    * above (h + s * m) will cause the cell to throw out old data and reset. where s
    * is getNominalStandardDeviation() and m is this value.
    */
   public static final DoubleStoredPropertyKey mahalanobisScale = keys.addDoubleKey("Mahalanobis scale");

   /**
    * Loads this property set.
    */
   public HeightMapParameters()
   {
      this("");
   }

   /**
    * Loads an alternate version of this property set in the same folder.
    */
   public HeightMapParameters(String versionSpecifier)
   {
      this(HeightMapParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, versionSpecifier);
   }

   /**
    * Loads an alternate version of this property set in other folders.
    */
   public HeightMapParameters(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder, String versionSuffix)
   {
      super(keys, classForLoading, HeightMapParameters.class, directoryNameToAssumePresent, subsequentPathToResourceFolder, versionSuffix);
      load();
   }

   public HeightMapParameters(StoredPropertySetReadOnly other)
   {
      super(keys, HeightMapParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           HeightMapParameters.class,
                                                           DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                           SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
   }
}
