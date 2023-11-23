package us.ihmc.footstepPlanning;

import us.ihmc.tools.property.*;

/**
 * The JSON file for this property set is located here:
 * ihmc-footstep-planning/src/main/resources/us/ihmc/footstepPlanning/MonteCarloFootstepPlannerParameters.json
 *
 * This class was auto generated. Property attributes must be edited in the JSON file,
 * after which this class should be regenerated by running the main. This class uses
 * the generator to assist in the addition, removal, and modification of property keys.
 * It is permissible to forgo these benefits and abandon the generator, in which case
 * you should also move it from the generated-java folder to the java folder.
 *
 * If the constant paths have changed, change them in this file and run the main to regenerate.
 */
public class MonteCarloFootstepPlannerParameters extends StoredPropertySet implements MonteCarloFootstepPlannerParametersBasics
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final IntegerStoredPropertyKey numberOfIterations = keys.addIntegerKey("Number of iterations");
   public static final IntegerStoredPropertyKey numberOfSimulations = keys.addIntegerKey("Number of simulations");
   public static final DoubleStoredPropertyKey timeoutDuration = keys.addDoubleKey("Timeout duration");
   public static final DoubleStoredPropertyKey feasibleContactReward = keys.addDoubleKey("Feasible Contact Reward");
   public static final DoubleStoredPropertyKey goalReward = keys.addDoubleKey("Goal Reward");
   public static final IntegerStoredPropertyKey maxNumberOfVisitedNodes = keys.addIntegerKey("Max number of visited nodes");
   public static final IntegerStoredPropertyKey maxNumberOfChildNodes = keys.addIntegerKey("Max number of child nodes");
   public static final DoubleStoredPropertyKey maxTransferHeight = keys.addDoubleKey("Max transfer height");
   public static final DoubleStoredPropertyKey maxTransferDepth = keys.addDoubleKey("Max transfer depth");
   public static final DoubleStoredPropertyKey maxTransferDistance = keys.addDoubleKey("Max transfer distance");
   public static final DoubleStoredPropertyKey minTransferDistance = keys.addDoubleKey("Min transfer distance");
   public static final DoubleStoredPropertyKey maxTransferYaw = keys.addDoubleKey("Max transfer yaw");
   public static final IntegerStoredPropertyKey goalMargin = keys.addIntegerKey("Goal margin");
   public static final DoubleStoredPropertyKey feasibleContactCutoff = keys.addDoubleKey("Feasible contact cutoff");
   public static final DoubleStoredPropertyKey explorationConstant = keys.addDoubleKey("Exploration constant");
   public static final IntegerStoredPropertyKey initialValueCutoff = keys.addIntegerKey("Initial value cutoff");
   public static final IntegerStoredPropertyKey maxTreeDepth = keys.addIntegerKey("Max tree depth");
   public static final DoubleStoredPropertyKey sidedYawOffset = keys.addDoubleKey("Sided yaw offset");
   public static final DoubleStoredPropertyKey searchYawBand = keys.addDoubleKey("Search yaw band");
   public static final DoubleStoredPropertyKey searchInnerRadius = keys.addDoubleKey("Search inner radius");
   public static final DoubleStoredPropertyKey searchOuterRadius = keys.addDoubleKey("Search outer radius");

   /**
    * Loads this property set.
    */
   public MonteCarloFootstepPlannerParameters()
   {
      this("");
   }

   /**
    * Loads an alternate version of this property set in the same folder.
    */
   public MonteCarloFootstepPlannerParameters(String versionSuffix)
   {
      this(MonteCarloFootstepPlannerParameters.class, versionSuffix);
   }

   /**
    * Loads an alternate version of this property set in other folders.
    */
   public MonteCarloFootstepPlannerParameters(Class<?> classForLoading, String versionSuffix)
   {
      super(keys, classForLoading, MonteCarloFootstepPlannerParameters.class, versionSuffix);
      load();
   }

   public MonteCarloFootstepPlannerParameters(StoredPropertySetReadOnly other)
   {
      super(keys, MonteCarloFootstepPlannerParameters.class, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, MonteCarloFootstepPlannerParameters.class);
      parameters.generateJavaFiles();
   }
}
