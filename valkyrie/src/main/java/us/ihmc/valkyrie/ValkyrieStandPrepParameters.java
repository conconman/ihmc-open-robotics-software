package us.ihmc.valkyrie;

import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.PositionControlParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieOrderedJointMap;

public class ValkyrieStandPrepParameters implements WholeBodySetpointParameters, PositionControlParameters
{
   private final HashMap<String, Double> setPoints = new HashMap<>();
   private final HashMap<String, Double> proportionalGain = new HashMap<>();
   private final HashMap<String, Double> integralGain = new HashMap<>();
   private final HashMap<String, Double> derivativeGain = new HashMap<>();
   private final ValkyrieJointMap jointMap;

   public ValkyrieStandPrepParameters(ValkyrieJointMap jointMap)
   {
      this.jointMap = jointMap;
      useDefaultAngles();
      loadCustomSetPoints();
      useDefaultGains();
      loadCustomGains();
   }

   private void loadCustomSetPoints()
   {
      Yaml yaml = new Yaml();
      InputStream setpointsStream = getClass().getClassLoader().getResourceAsStream("standPrep/setpoints.yaml");
      @SuppressWarnings("unchecked")
      Map<String, Double> setPointMap = (Map<String, Double>) yaml.load(setpointsStream);

      for (String jointName : ValkyrieOrderedJointMap.jointNames)
      {
         if (setPointMap.containsKey(jointName))
            setSetpoint(jointName, setPointMap.get(jointName));
      }

      try
      {
         setpointsStream.close();
      }
      catch (IOException e)
      {
      }
   }

   private void useDefaultAngles()
   {
      setSetpoint(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), 0.0);
      setSetpoint(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_YAW), 0.0);
      setSetpoint(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH), 0.0);

      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 0.0);
      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 0.0);
      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 0.0);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 0.0);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.6);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 1.3);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.65);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), 0.0);

         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), -0.2);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfLeftSide(1.2));
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 0.0);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), robotSide.negateIfLeftSide(1.5));
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), 1.3);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0);
      }
   }

   @SuppressWarnings("unchecked")
   private void loadCustomGains()
   {
      Yaml yaml = new Yaml();
      InputStream gainStream = getClass().getClassLoader().getResourceAsStream("standPrep/gains.yaml");

      Map<String, Map<String, Double>> gainMap = (Map<String, Map<String, Double>>) yaml.load(gainStream);

      try
      {
         gainStream.close();
      }
      catch (IOException e)
      {
         return;
      }

      for (String jointName : ValkyrieOrderedJointMap.jointNames)
      {
         Map<String, Double> jointGains = gainMap.get(jointName);
         if (jointGains == null)
         {
            setJointGains(jointName, 0.0, 0.0, 0.0);
         }
         else
         {
            Double kp = jointGains.get("kp");
            Double ki = jointGains.get("ki");
            Double kd = jointGains.get("kd");
            setProportionalGain(jointName, kp == null ? 0.0 : kp);
            setIntegralGain(jointName, ki == null ? 0.0 : ki);
            setDerivativeGain(jointName, kd == null ? 0.0 : kd);
         }
      }
   }

   private void useDefaultGains()
   {
      setJointGains(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), 0.0, 0.0, 0.0);
      setJointGains(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_YAW), 0.0, 0.0, 0.0);
      setJointGains(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH), 0.0, 0.0, 0.0);

      setJointGains(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 100.0, 0.0, 6.2);
      setJointGains(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 600.0, 0.0, 40.0);
      setJointGains(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 600.0, 0.0, 40.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 50.0, 0.0, 5.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), 50.0, 0.0, 5.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 40.0, 0.0, 4.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), 40.0, 0.0, 10.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), 0.0, 0.0, 0.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0, 0.0, 0.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0, 0.0, 0.0);

         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 100.0, 0.0, 6.2);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 500.0, 0.0, 25.0);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), 550.0, 0.0, 21.0);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 259.0, 0.0, 9.4);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), 68.0, 0.0, 6.29);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), 56.0, 0.0, 3.14);
      }
   }

   private void setJointGains(String jointName, double kp, double ki, double kd)
   {
      setProportionalGain(jointName, kp);
      setIntegralGain(jointName, ki);
      setDerivativeGain(jointName, kd);
   }

   private void setProportionalGain(String jointName, double value)
   {
      proportionalGain.put(jointName, value);
   }

   private void setDerivativeGain(String jointName, double value)
   {
      derivativeGain.put(jointName, value);
   }

   private void setIntegralGain(String jointName, double value)
   {
      integralGain.put(jointName, value);
   }

   @Override
   public double getProportionalGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getProportionalGain(jointName);
   }

   @Override
   public double getProportionalGain(String jointName)
   {
      if (proportionalGain.containsKey(jointName))
         return proportionalGain.get(jointName);
      else
         return 0.0;
   }

   @Override
   public double getDerivativeGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getDerivativeGain(jointName);
   }

   @Override
   public double getDerivativeGain(String jointName)
   {
      if (derivativeGain.containsKey(jointName))
         return derivativeGain.get(jointName);
      else
         return 0.0;
   }

   @Override
   public double getIntegralGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getIntegralGain(jointName);
   }

   @Override
   public double getIntegralGain(String jointName)
   {
      if (integralGain.containsKey(jointName))
         return integralGain.get(jointName);
      else
         return 0.0;
   }

   @Override
   public double getPositionControlMasterGain()
   {
      return 1.0;
   }

   private void setSetpoint(String jointName, double value)
   {
      setPoints.put(jointName, value);
   }

   @Override
   public double getSetpoint(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getSetpoint(jointName);
   }

   @Override
   public double getSetpoint(String jointName)
   {
      if (setPoints.containsKey(jointName))
         return setPoints.get(jointName);
      else
         return 0.0;
   }
}
