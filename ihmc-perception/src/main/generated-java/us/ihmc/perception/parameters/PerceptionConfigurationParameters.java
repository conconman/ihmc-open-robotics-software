package us.ihmc.perception.parameters;

import us.ihmc.tools.property.*;

/**
 * The JSON file for this property set is located here:
 * ihmc-perception/src/main/resources/us/ihmc/perception/parameters/PerceptionConfigurationParameters.json
 *
 * This class was auto generated. Property attributes must be edited in the JSON file,
 * after which this class should be regenerated by running the main. This class uses
 * the generator to assist in the addition, removal, and modification of property keys.
 * It is permissible to forgo these benefits and abandon the generator, in which case
 * you should also move it from the generated-java folder to the java folder.
 *
 * If the constant paths have changed, change them in this file and run the main to regenerate.
 */
public class PerceptionConfigurationParameters extends StoredPropertySet implements PerceptionConfigurationParametersBasics
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final IntegerStoredPropertyKey l515ThrottlerFrequency = keys.addIntegerKey("L515 throttler frequency");
   public static final IntegerStoredPropertyKey ousterThrottlerFrequency = keys.addIntegerKey("Ouster throttler frequency");
   public static final BooleanStoredPropertyKey rapidRegionsEnabled = keys.addBooleanKey("Rapid regions enabled");
   public static final BooleanStoredPropertyKey loggingEnabled = keys.addBooleanKey("Logging enabled");
   public static final BooleanStoredPropertyKey publishColor = keys.addBooleanKey("Publish color");
   public static final BooleanStoredPropertyKey publishDepth = keys.addBooleanKey("Publish depth");
   public static final BooleanStoredPropertyKey logColor = keys.addBooleanKey("Log color");
   public static final BooleanStoredPropertyKey logDepth = keys.addBooleanKey("Log depth");
   public static final BooleanStoredPropertyKey slamEnabled = keys.addBooleanKey("SLAM enabled");
   public static final BooleanStoredPropertyKey slamReset = keys.addBooleanKey("SLAM reset");
   public static final BooleanStoredPropertyKey supportSquareEnabled = keys.addBooleanKey("Support square enabled");
   public static final BooleanStoredPropertyKey boundingBoxFilter = keys.addBooleanKey("Bounding box filter");
   public static final BooleanStoredPropertyKey activeMapping = keys.addBooleanKey("Active mapping");

   /**
    * Loads this property set.
    */
   public PerceptionConfigurationParameters()
   {
      this("");
   }

   /**
    * Loads an alternate version of this property set in the same folder.
    */
   public PerceptionConfigurationParameters(String versionSuffix)
   {
      this(PerceptionConfigurationParameters.class, versionSuffix);
   }

   /**
    * Loads an alternate version of this property set in other folders.
    */
   public PerceptionConfigurationParameters(Class<?> classForLoading, String versionSuffix)
   {
      super(keys, classForLoading, PerceptionConfigurationParameters.class, versionSuffix);
      load();
   }

   public PerceptionConfigurationParameters(StoredPropertySetReadOnly other)
   {
      super(keys, PerceptionConfigurationParameters.class, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, PerceptionConfigurationParameters.class);
      parameters.generateJavaFiles();
   }
}
