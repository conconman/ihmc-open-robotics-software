package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.type.ImDouble;

/**
 * This class is just like ImGuiInputDouble except decouples the + and -
 * buttons from the input part, because rotations wrap around and we also
 * transform them by "nudges", instead of directly editing the current
 * value.
 */
public class ImGuiInputDoubleForRotations
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String label;
   private final String plusButtonLabel = labels.get(" + ");
   private final String minusButtonLabel = labels.get(" - ");
   private final String format;
   private final String prefixLabel;
   private float prefixTextWidth;
   private boolean textWidthCalculated = false;
   private final ImDouble imDouble;
   private boolean inputChanged = false;
   private boolean stepButtonClicked = false;
   private double steppedAmount = 0.0;

   public ImGuiInputDoubleForRotations(String label, String format)
   {
      this(label, format, 0.0);
   }

   public ImGuiInputDoubleForRotations(String label, String format, double initialValue)
   {
      this.prefixLabel = label;
      this.label = labels.getHidden(label);
      this.format = format;
      imDouble = new ImDouble(initialValue);
   }

   /**
    * Shows the + and - buttons.
    * @param step normal step
    * @param stepFast step when holding ctrl key and clicking + and - buttons
    */
   public void render(double step, double stepFast)
   {
      if (!textWidthCalculated)
      {
         textWidthCalculated = true;
         ImVec2 size = new ImVec2();
         ImGui.calcTextSize(size, prefixLabel);
         prefixTextWidth = size.x;
      }

      ImGui.text(prefixLabel);
      ImGui.sameLine();
      float spaceForButtons = 53.0f;
      ImGui.pushItemWidth(ImGuiTools.getUsableWindowWidth() - prefixTextWidth - spaceForButtons);
      inputChanged = ImGuiTools.volatileInputDouble(label, imDouble, 0.0, 0.0, format);
      ImGui.popItemWidth();

//      ImGui.pushS
      steppedAmount = 0.0;
      stepButtonClicked = false;
      boolean ctrlHeld = ImGui.getIO().getKeyCtrl();
      ImGui.sameLine();
      boolean minusClicked = ImGui.button(minusButtonLabel);
      ImGui.sameLine();
      boolean plusClicked = ImGui.button(plusButtonLabel);

      if (minusClicked)
         steppedAmount += ctrlHeld ? stepFast : step;
      if (plusClicked)
         steppedAmount += ctrlHeld ? stepFast : step;

      stepButtonClicked = minusClicked || plusClicked;
   }

   public boolean getInputChanged()
   {
      return inputChanged;
   }

   public boolean getStepButtonClicked()
   {
      return stepButtonClicked;
   }

   public double getSteppedAmount()
   {
      return steppedAmount;
   }

   public void setDoubleValue(double value)
   {
      imDouble.set(value);
   }

   public double getDoubleValue()
   {
      return imDouble.get();
   }

   public ImDouble getImDouble()
   {
      return imDouble;
   }
}
