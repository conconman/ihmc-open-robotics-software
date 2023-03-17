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
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "ihmc-perception/src/main/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "ihmc-perception/src/main/generated-java";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final IntegerStoredPropertyKey l515ThrottlerFrequency = keys.addIntegerKey("L515 throttler frequency");
   public static final IntegerStoredPropertyKey ousterThrottlerFrequency = keys.addIntegerKey("Ouster throttler frequency");
   public static final BooleanStoredPropertyKey loggingEnabled = keys.addBooleanKey("Logging enabled");
   public static final BooleanStoredPropertyKey publishColor = keys.addBooleanKey("Publish color");
   public static final BooleanStoredPropertyKey publishDepth = keys.addBooleanKey("Publish depth");
   public static final BooleanStoredPropertyKey logColor = keys.addBooleanKey("Log color");
   public static final BooleanStoredPropertyKey logDepth = keys.addBooleanKey("Log depth");

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
   public PerceptionConfigurationParameters(String versionSpecifier)
   {
      this(PerceptionConfigurationParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, versionSpecifier);
   }

   /**
    * Loads an alternate version of this property set in other folders.
    */
   public PerceptionConfigurationParameters(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder, String versionSuffix)
   {
      super(keys, classForLoading, PerceptionConfigurationParameters.class, directoryNameToAssumePresent, subsequentPathToResourceFolder, versionSuffix);
      load();
   }

   public PerceptionConfigurationParameters(StoredPropertySetReadOnly other)
   {
      super(keys, PerceptionConfigurationParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           PerceptionConfigurationParameters.class,
                                                           DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                           SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
   }
}
