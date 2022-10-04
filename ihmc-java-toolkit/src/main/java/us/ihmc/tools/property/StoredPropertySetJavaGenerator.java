package us.ihmc.tools.property;

import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

public class StoredPropertySetJavaGenerator
{
   private final String jsonFileName;
   private final Class<?> clazz;
   private final WorkspaceDirectory javaDirectory;
   private final WorkspaceFile primaryJavaFile;
   private final WorkspaceFile basicsJavaFile;
   private final WorkspaceFile readOnlyJavaFile;
   private String directoryNameToAssumePresent;
   private String subsequentPathToResourceFolder;

   public StoredPropertySetJavaGenerator(Class<?> clazz,
                                         String directoryNameToAssumePresent,
                                         String subsequentPathToResourceFolder,
                                         String subsequentPathToJavaFolder)
   {
      this.clazz = clazz;
      this.directoryNameToAssumePresent = directoryNameToAssumePresent;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;

//      String subsequentPathToJavaPackage = subsequentPathToJavaFolder + "/" + clazz.getPackage().getName().replaceAll("\\.", "/");
      javaDirectory = new WorkspaceDirectory(directoryNameToAssumePresent, subsequentPathToJavaFolder, clazz);
      jsonFileName = clazz.getSimpleName() + ".json";
      primaryJavaFile = new WorkspaceFile(javaDirectory, clazz.getSimpleName() + ".java");
      basicsJavaFile = new WorkspaceFile(javaDirectory, clazz.getSimpleName() + "Basics.java");
      readOnlyJavaFile = new WorkspaceFile(javaDirectory, clazz.getSimpleName() + "ReadOnly.java");
   }

   public void generate()
   {
      String primaryJavaFileContents =
      """
      package %s;
      
      import us.ihmc.tools.property.*;
      
      public class %2$s extends StoredPropertySet implements %2$sBasics
      {
         public static final String PROJECT_NAME = "ihmc-open-robotics-software";
         public static final String TO_RESOURCE_FOLDER = "ihmc-high-level-behaviors/src/libgdx/resources";
         
         public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();
         
         %3$s
         
         public %2$s()
         {
            super(keys, %2$s.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
            load();
         }
      
         public static void main(String[] args)
         {
            StoredPropertySetJavaGenerator generator = new StoredPropertySetJavaGenerator(StoredPropertySetGeneratorTest.class,
                                                                                          "ihmc-open-robotics-software",
                                                                                          "ihmc-java-toolkit/src/test/resources",
                                                                                          "ihmc-java-toolkit/src/test/java");
            generator.generate();
         }
      }
      """.formatted(clazz.getPackage().getName(), clazz.getSimpleName(), getParameterKeysStrings());

      FileTools.write(primaryJavaFile.getFilePath(), primaryJavaFileContents.getBytes(), WriteOption.TRUNCATE, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   private String getParameterKeysStrings()
   {
      StringBuilder propertyKeyDeclarations = new StringBuilder();
      JSONFileTools.loadFromClasspath(clazz, jsonFileName, node ->
      {
         if (node instanceof ObjectNode objectNode)
         {
            objectNode.fieldNames().forEachRemaining(fieldName ->
            {
               LogTools.info("Name: {} Value: {}", fieldName, objectNode.get(fieldName));
               propertyKeyDeclarations.append(
               """
               public static final DoubleStoredPropertyKey %1$s = keys.addDoubleKey("%1$s");
               """.formatted(fieldName));
            });
         }
      });
      return propertyKeyDeclarations.toString();
   }
}
