package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import org.lwjgl.opengl.GL41;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.input.GDXInputMode;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;

/**
 * TODO: Pause and resume?
 */
public class GDX3DSceneManager
{
   private final GDX3DSceneBasics sceneBasics = new GDX3DSceneBasics();

   private InputMultiplexer inputMultiplexer;
   private GDXFocusBasedCamera camera3D;
   private ScreenViewport viewport;

   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;
   private boolean firstRenderStarted = false;
   private boolean addFocusSphere = true;
   private Runnable onCreate;
   private GLProfiler glProfiler;

   public void create()
   {
      create(GDXInputMode.libGDX);
   }

   public void create(GDXInputMode inputMode)
   {
      if (GDXTools.ENABLE_OPENGL_DEBUGGER)
         glProfiler = GDXTools.createGLProfiler();

      GDXTools.syncLogLevelWithLogTools();

      camera3D = new GDXFocusBasedCamera();
      if (inputMode == GDXInputMode.libGDX)
      {
         inputMultiplexer = new InputMultiplexer();
         Gdx.input.setInputProcessor(inputMultiplexer);

         inputMultiplexer.addProcessor(camera3D.setInputForLibGDX());
      }

      sceneBasics.create();

      if (addFocusSphere)
         sceneBasics.addModelInstance(camera3D.getFocusPointSphere(), GDXSceneLevel.VIRTUAL);
      viewport = new ScreenViewport(camera3D);
      viewport.setUnitsPerPixel(1.0f); // TODO: Is this relevant for high DPI displays?

      sceneBasics.addDefaultLighting();
      if (onCreate != null)
         onCreate.run();
   }

   public void renderShadowMap()
   {
      renderShadowMap(width, height);
   }

   public void renderShadowMap(int x, int y)
   {
      sceneBasics.renderShadowMap(camera3D, x, y);
   }

   public void render()
   {
      preRender();
      sceneBasics.render();
      sceneBasics.postRender(camera3D, GDXSceneLevel.VIRTUAL);

      if (GDXTools.ENABLE_OPENGL_DEBUGGER)
         glProfiler.reset();
   }

   private void preRender()
   {
      if (!firstRenderStarted)
      {
         firstRenderStarted = true;
         LogTools.info("Starting first render.");
      }

      if (width < 0)
         width = getCurrentWindowWidth();
      if (height < 0)
         height = getCurrentWindowHeight();

      viewport.update(width, height);

      sceneBasics.preRender(camera3D);

      GL41.glViewport(x, y, width, height);
      GDX3DSceneTools.glClearGray();
   }

   public void dispose()
   {
      sceneBasics.dispose();
      ExceptionTools.handle(() -> camera3D.dispose(), DefaultExceptionHandler.PRINT_MESSAGE);
   }
   // End render public API

   public boolean closeRequested()
   {
      return true;
   }

   public void addModelInstance(ModelInstance modelInstance)
   {
      sceneBasics.addModelInstance(modelInstance);
   }

   public void addModelInstance(ModelInstance modelInstance, GDXSceneLevel sceneLevel)
   {
      sceneBasics.addModelInstance(modelInstance, sceneLevel);
   }

   public void addCoordinateFrame(double size)
   {
      sceneBasics.addCoordinateFrame(size);
   }

   public void addRenderableProvider(RenderableProvider renderableProvider)
   {
      sceneBasics.addRenderableProvider(renderableProvider);
   }

   public void addRenderableProvider(RenderableProvider renderableProvider, GDXSceneLevel sceneLevel)
   {
      sceneBasics.addRenderableProvider(renderableProvider, sceneLevel);
   }

   public void setViewportBoundsToWindow()
   {
      setViewportBounds(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
   }

   /**
    * Coordinates in xy bottom left
    */
   public void setViewportBounds(int x, int y, int width, int height)
   {
      this.x = x;
      this.y = y;
      this.width = width;
      this.height = height;

      sceneBasics.getShadowManager().setViewportBounds(x, y, width, height);
   }

   public int getCurrentWindowWidth()
   {
      return Gdx.graphics.getWidth();
   }

   public int getCurrentWindowHeight()
   {
      return Gdx.graphics.getHeight();
   }

   public GDXFocusBasedCamera getCamera3D()
   {
      return camera3D;
   }

   public void addLibGDXInputProcessor(InputProcessor inputProcessor)
   {
      if (inputMultiplexer != null)
      {
         inputMultiplexer.addProcessor(inputProcessor);
      }
      else
      {
         LogTools.error(1, "libGDX is not being used for input!");
      }
   }

   public void setAddFocusSphere(boolean addFocusSphere)
   {
      this.addFocusSphere = addFocusSphere;
   }

   public void setOnCreate(Runnable onCreate)
   {
      this.onCreate = onCreate;
   }

   public GDX3DSceneBasics getSceneBasics()
   {
      return sceneBasics;
   }
}