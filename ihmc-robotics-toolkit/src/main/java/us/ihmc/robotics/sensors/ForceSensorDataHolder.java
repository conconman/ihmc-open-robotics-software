package us.ihmc.robotics.sensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.robotics.screwTheory.GenericCRC32;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

public class ForceSensorDataHolder implements ForceSensorDataHolderReadOnly, Settable<ForceSensorDataHolder>
{
   private final List<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<>();
   private final List<ForceSensorData> forceSensorDatas = new ArrayList<>();

   private final transient Map<String, ForceSensorPackage> sensorNameToDefintionMap = new HashMap<>();

   public ForceSensorDataHolder()
   {
   }

   public ForceSensorDataHolder(ForceSensorDefinition[] forceSensors)
   {
      for (ForceSensorDefinition forceSensorDefinition : forceSensors)
      {
         registerForceSensor(forceSensorDefinition);
      }
   }

   public ForceSensorDataHolder(List<ForceSensorDefinition> forceSensors)
   {
      for (ForceSensorDefinition forceSensorDefinition : forceSensors)
      {
         registerForceSensor(forceSensorDefinition);
      }
   }

   public ForceSensorData registerForceSensor(ForceSensorDefinition forceSensorDefinition)
   {
      String sensorName = forceSensorDefinition.getSensorName();
      ForceSensorPackage sensorPackage = sensorNameToDefintionMap.get(sensorName);

      if (sensorPackage != null)
         throw new IllegalOperationException("Force sensor (%s) has already been registered.".formatted(forceSensorDefinition.getSensorFrame()));

      sensorPackage = new ForceSensorPackage(forceSensorDefinition);
      sensorNameToDefintionMap.put(sensorName, sensorPackage);
      return sensorPackage.data;
   }

   public void registerForceSensor(ForceSensorDefinition forceSensorDefinition, ForceSensorDataReadOnly forceSensorData)
   {
      registerForceSensor(forceSensorDefinition).set(forceSensorData);
   }

   public void clear()
   {
      forceSensorDefinitions.clear();
      forceSensorDatas.clear();
      sensorNameToDefintionMap.clear();
   }

   @Override
   public List<ForceSensorDefinition> getForceSensorDefinitions()
   {
      return forceSensorDefinitions;
   }

   public List<ForceSensorData> getForceSensorDatas()
   {
      return forceSensorDatas;
   }

   @Override
   public ForceSensorData getData(ForceSensorDefinition sensorDefinition)
   {
      return getData(sensorDefinition.getSensorName());
   }

   @Override
   public ForceSensorData getData(String sensorName)
   {
      ForceSensorPackage sensorPackage = sensorNameToDefintionMap.get(sensorName);
      return sensorPackage == null ? null : sensorPackage.data;
   }

   @Override
   public ForceSensorDefinition getDefinition(String sensorName)
   {
      ForceSensorPackage sensorPackage = sensorNameToDefintionMap.get(sensorName);
      return sensorPackage == null ? null : sensorPackage.definition;
   }

   public int getNumberOfForceSensors()
   {
      return forceSensorDefinitions.size();
   }

   @Override
   public void set(ForceSensorDataHolder other)
   {
      set((ForceSensorDataHolderReadOnly) other);
   }

   public void set(ForceSensorDataHolderReadOnly other)
   {
      clear();
      for (int i = 0; i < other.getForceSensorDefinitions().size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = other.getForceSensorDefinitions().get(i);
         registerForceSensor(forceSensorDefinition, other.getData(forceSensorDefinition));
      }
   }

   @Deprecated // maintains compatibility with the thread data synchronizer
   public void setDataOnly(ForceSensorDataHolder other)
   {
      for (int i = 0; i < other.getForceSensorDefinitions().size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = other.getForceSensorDefinitions().get(i);
         ForceSensorData thisData = getData(forceSensorDefinition);
         ForceSensorData otherData = other.getData(forceSensorDefinition);
         thisData.setWrench(otherData.getWrenchMatrix());
      }
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof ForceSensorDataHolder other)
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   public void calculateChecksum(GenericCRC32 checksum)
   {
      for (int i = 0; i < forceSensorDatas.size(); i++)
      {
         forceSensorDatas.get(i).calculateChecksum(checksum);
      }
   }

   private static class ForceSensorPackage
   {
      private final ForceSensorDefinition definition;
      private final ForceSensorData data;

      public ForceSensorPackage(ForceSensorDefinition source)
      {
         definition = new ForceSensorDefinition(source);
         data = new ForceSensorData(definition);
      }

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
            return true;
         else if (object instanceof ForceSensorPackage other)
            return Objects.equals(definition, other.definition) && Objects.equals(data, other.data);
         else
            return false;
      }
   }
}
