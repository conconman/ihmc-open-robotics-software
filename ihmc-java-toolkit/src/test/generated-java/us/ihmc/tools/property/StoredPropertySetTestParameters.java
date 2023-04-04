package us.ihmc.tools.property;

import us.ihmc.tools.property.*;

/**
 * The JSON file for this property set is located here:
 * ihmc-java-toolkit/src/test/resources/us/ihmc/tools/property/StoredPropertySetTestParameters.json
 *
 * This class was auto generated. Property attributes must be edited in the JSON file,
 * after which this class should be regenerated by running the main. This class uses
 * the generator to assist in the addition, removal, and modification of property keys.
 * It is permissible to forgo these benefits and abandon the generator, in which case
 * you should also move it from the generated-java folder to the java folder.
 *
 * If the constant paths have changed, change them in this file and run the main to regenerate.
 */
public class StoredPropertySetTestParameters extends StoredPropertySet implements StoredPropertySetTestParametersBasics
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final BooleanStoredPropertyKey theFirstBooleanProperty = keys.addBooleanKey("The first boolean property");
   public static final DoubleStoredPropertyKey theFirstDoubleProperty = keys.addDoubleKey("The first double property");
   public static final IntegerStoredPropertyKey theFirstIntegerProperty = keys.addIntegerKey("The first integer property");
   /**
    * The first boolean description.
    */
   public static final BooleanStoredPropertyKey booleanPropertyWithADescription = keys.addBooleanKey("Boolean property with a description");
   /**
    * The first integer description.
    */
   public static final DoubleStoredPropertyKey doublePropertyWithADescription = keys.addDoubleKey("Double property with a description");
   /**
    * The first integer description.
    */
   public static final IntegerStoredPropertyKey integerPropertyWithADescription = keys.addIntegerKey("Integer property with a description");
   /**
    * The double property with more stuff.
    */
   public static final DoubleStoredPropertyKey doublePropertyWithMoreStuff = keys.addDoubleKey("Double property with more stuff");
   /**
    * The integer property with more stuff.
    */
   public static final IntegerStoredPropertyKey integerPropertyWithMoreStuff = keys.addIntegerKey("Integer property with more stuff");
   /**
    * The integer property with discrete valid values.
    */
   public static final IntegerStoredPropertyKey integerPropertyWithDiscreteValidValues = keys.addIntegerKey("Integer property with discrete valid values");

   /**
    * Loads this property set.
    */
   public StoredPropertySetTestParameters()
   {
      this("");
   }

   /**
    * Loads an alternate version of this property set in the same folder.
    */
   public StoredPropertySetTestParameters(String versionSuffix)
   {
      this(StoredPropertySetTestParameters.class, versionSuffix);
   }

   /**
    * Loads an alternate version of this property set in other folders.
    */
   public StoredPropertySetTestParameters(Class<?> classForLoading, String versionSuffix)
   {
      super(keys, classForLoading, StoredPropertySetTestParameters.class, versionSuffix);
      load();
   }

   public StoredPropertySetTestParameters(StoredPropertySetReadOnly other)
   {
      super(keys, StoredPropertySetTestParameters.class, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, StoredPropertySetTestParameters.class);
      parameters.generateJavaFiles();
   }
}
