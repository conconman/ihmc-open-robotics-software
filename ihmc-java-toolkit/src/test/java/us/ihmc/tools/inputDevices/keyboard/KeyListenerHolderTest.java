package us.ihmc.tools.inputDevices.keyboard;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.graphicsDescription.input.keyboard.KeyListener;
import us.ihmc.graphicsDescription.input.keyboard.KeyListenerHolder;
import us.ihmc.tools.inputDevices.keyboard.Key;

public class KeyListenerHolderTest
{
   @Test
   public void testKeyListenerHolder()
   {
      final List<Key> pressed1 = new ArrayList<>();
      final List<Key> pressed2 = new ArrayList<>();
      final List<Key> released1 = new ArrayList<>();
      final List<Key> released2 = new ArrayList<>();
      
      KeyListenerHolder keyListenerHolder = new KeyListenerHolder();
      keyListenerHolder.addKeyListener(new KeyListener()
      {
         @Override
         public void keyReleased(Key key)
         {
            released1.add(key);
         }
         
         @Override
         public void keyPressed(Key key)
         {
            pressed1.add(key);
         }
      });
      keyListenerHolder.addKeyListener(new KeyListener()
      {
         @Override
         public void keyReleased(Key key)
         {
            released2.add(key);
         }
         
         @Override
         public void keyPressed(Key key)
         {
            pressed2.add(key);
         }
      });
      
      keyListenerHolder.keyPressed(Key.A);
      keyListenerHolder.keyReleased(Key.A);

      keyListenerHolder.keyPressed(Key.B);
      keyListenerHolder.keyReleased(Key.B);
      
      assertEquals("Event not registered", pressed1.size(), 2);
      assertEquals("Event not registered", pressed2.size(), 2);
      assertEquals("Event not registered", released1.size(), 2);
      assertEquals("Event not registered", released2.size(), 2);
      
      assertEquals("Key wrong", pressed1.get(0), Key.A);
      assertEquals("Key wrong", released1.get(0), Key.A);
      assertEquals("Key wrong", pressed2.get(1), Key.B);
      assertEquals("Key wrong", released2.get(1), Key.B);
   }
}
