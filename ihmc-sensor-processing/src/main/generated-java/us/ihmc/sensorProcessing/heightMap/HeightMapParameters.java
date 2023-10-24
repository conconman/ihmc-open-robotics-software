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
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey localWidthInMeters = keys.addDoubleKey("Local width in meters");
   public static final DoubleStoredPropertyKey localCellSizeInMeters = keys.addDoubleKey("Local cell size in meters");
   public static final DoubleStoredPropertyKey globalWidthInMeters = keys.addDoubleKey("Global width in meters");
   public static final DoubleStoredPropertyKey globalCellSizeInMeters = keys.addDoubleKey("Global cell size in meters");
   public static final DoubleStoredPropertyKey internalGlobalWidthInMeters = keys.addDoubleKey("Internal global width in meters");
   public static final DoubleStoredPropertyKey internalGlobalCellSizeInMeters = keys.addDoubleKey("Internal global cell size in meters");
   public static final DoubleStoredPropertyKey heightScaleFactor = keys.addDoubleKey("Height scale factor");
   public static final IntegerStoredPropertyKey cropWindowSize = keys.addIntegerKey("Crop window size");
   public static final IntegerStoredPropertyKey searchWindowHeight = keys.addIntegerKey("Search window height");
   public static final IntegerStoredPropertyKey searchWindowWidth = keys.addIntegerKey("Search window width");
   public static final DoubleStoredPropertyKey minHeightRegistration = keys.addDoubleKey("Min height registration");
   public static final DoubleStoredPropertyKey maxHeightRegistration = keys.addDoubleKey("Max height registration");
   public static final DoubleStoredPropertyKey minHeightDifference = keys.addDoubleKey("Min height difference");
   public static final DoubleStoredPropertyKey maxHeightDifference = keys.addDoubleKey("Max height difference");
   public static final DoubleStoredPropertyKey heightFilterAlpha = keys.addDoubleKey("Height filter alpha");
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
    * This is the variance added to all past measurements when a cell is translated
    */
   public static final DoubleStoredPropertyKey varianceAddedWhenTranslating = keys.addDoubleKey("Variance added when translating");
   /**
    * This is the measurement variance when the robot is standing
    */
   public static final DoubleStoredPropertyKey sensorVarianceWhenStanding = keys.addDoubleKey("Sensor variance when standing");
   /**
    * This is the measurement variance when the robot is moving
    */
   public static final DoubleStoredPropertyKey sensorVarianceWhenMoving = keys.addDoubleKey("Sensor variance when moving");
   public static final BooleanStoredPropertyKey estimateHeightWithKalmanFilter = keys.addBooleanKey("Estimate height with kalman filter");
   public static final BooleanStoredPropertyKey resetHeightMap = keys.addBooleanKey("Reset Height Map");

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
   public HeightMapParameters(String versionSuffix)
   {
      this(HeightMapParameters.class, versionSuffix);
   }

   /**
    * Loads an alternate version of this property set in other folders.
    */
   public HeightMapParameters(Class<?> classForLoading, String versionSuffix)
   {
      super(keys, classForLoading, HeightMapParameters.class, versionSuffix);
      load();
   }

   public HeightMapParameters(StoredPropertySetReadOnly other)
   {
      super(keys, HeightMapParameters.class, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, HeightMapParameters.class);
      parameters.generateJavaFiles();
   }
}
