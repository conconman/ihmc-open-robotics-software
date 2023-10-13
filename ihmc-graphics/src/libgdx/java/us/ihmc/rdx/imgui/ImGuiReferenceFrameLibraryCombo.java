package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import org.apache.commons.lang3.ArrayUtils;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import java.util.Collection;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Used to select between the reference frames in a library by human readable names.
 * It also includes any values that the user already might have and keeps those around
 * even if deselected. Also, this is done immediate mode style.
 */
public class ImGuiReferenceFrameLibraryCombo
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String comboName;
   private final Supplier<String> currentFrameNameGetter;
   private final Consumer<String> currentFrameNameSetter;
   private final SortedSet<String> referenceFrameLibraryNames = new TreeSet<>();
   /** Stores a history of all frames that were ever here. */
   private final SortedSet<String> selectableReferenceFrameNames = new TreeSet<>();
   private transient String[] selectableReferenceFrameNameArray = new String[0];

   public ImGuiReferenceFrameLibraryCombo(String comboName,
                                          Supplier<String> currentFrameNameGetter,
                                          Consumer<String> currentFrameNameSetter)
   {
      this.comboName = comboName;
      this.currentFrameNameGetter = currentFrameNameGetter;
      this.currentFrameNameSetter = currentFrameNameSetter;
   }

   public void render(Collection<ReferenceFrame> referenceFrames)
   {
      referenceFrameLibraryNames.clear();
      for (ReferenceFrame referenceFrame : referenceFrames)
         referenceFrameLibraryNames.add(referenceFrame.getName());

      selectableReferenceFrameNames.add(currentFrameNameGetter.get());
      selectableReferenceFrameNames.addAll(referenceFrameLibraryNames);

      selectableReferenceFrameNameArray = selectableReferenceFrameNames.toArray(selectableReferenceFrameNameArray);

      // Make sure current frame is selected
      int selectedFrameIndex = ArrayUtils.indexOf(selectableReferenceFrameNameArray, currentFrameNameGetter.get());

      if (ImGui.beginCombo(labels.get(comboName), selectableReferenceFrameNameArray[selectedFrameIndex]))
      {
         for (int i = 0; i < selectableReferenceFrameNameArray.length; i++)
         {
            String referenceFrameName = selectableReferenceFrameNameArray[i];
            boolean libraryContainsFrame = referenceFrameLibraryNames.contains(referenceFrameName);

            if (!libraryContainsFrame)
               ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.RED);

            if (ImGui.selectable(referenceFrameName, selectedFrameIndex == i))
            {
               selectedFrameIndex = i;
               currentFrameNameSetter.accept(referenceFrameName); // Keep the user's copy up to date
            }

            if (!libraryContainsFrame)
               ImGui.popStyleColor();
         }

         ImGui.endCombo();
      }
   }
}
