package us.ihmc.rdx.simulation.bullet.libgdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.physics.bullet.collision.btManifoldPoint;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.simulation.bullet.RDXBulletTools;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.environment.object.objects.RDXLabFloorObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXMediumCinderBlockRoughed;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.ArrayList;

/**
 * https://web.archive.org/web/20170706235814/http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Main_Page
 * https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=2568
 * https://github.com/kripken/bullet/blob/master/Demos/CollisionInterfaceDemo/CollisionInterfaceDemo.cpp
 */
public class RDXBulletPhysicsInteractionForcesDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXEnvironmentBuilder environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
   private final ImFloat blockTransparency = new ImFloat(0.2f);

   public RDXBulletPhysicsInteractionForcesDemo()
   {
      RDXBulletTools.ensureBulletInitialized();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         private final ArrayList<btManifoldPoint> contactPoints = new ArrayList<>();
         private final RecyclingArrayList<ModelInstance> pointsOnA = new RecyclingArrayList<>(this::createSphere);
         private final RecyclingArrayList<ModelInstance> arrowsOnB = new RecyclingArrayList<>(this::createArrow);

         private RDXLabFloorObject labFloorObject;
         private RDXMediumCinderBlockRoughed fallingBlock;
         private RDXMediumCinderBlockRoughed sittingBlock;

         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder);

            labFloorObject = new RDXLabFloorObject();
            environmentBuilder.addObject(labFloorObject);
            labFloorObject.copyThisTransformToBulletMultiBody();

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

            recreateAndPlace();

            RDXPanel experimentPanel = new RDXPanel("Demo", () ->
            {
               if (ImGui.button("Replace Block"))
               {
                  recreateAndPlace();
               }
               ImGui.sliderFloat("Block transparency", blockTransparency.getData(), 0.0f, 1.0f);
               LibGDXTools.setOpacity(fallingBlock.getRealisticModelInstance(), blockTransparency.get());
            });
            baseUI.getImGuiPanelManager().addPanel(experimentPanel);

            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(2.0, 1.0, 1.0);
         }

         public void recreateAndPlace()
         {
            if (fallingBlock != null)
            {
               environmentBuilder.removeObject(fallingBlock);
               environmentBuilder.removeObject(sittingBlock);
            }

            fallingBlock = new RDXMediumCinderBlockRoughed();
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            transformToWorld.getTranslation().set(0.0, 0.0, 0.5);
            transformToWorld.getRotation().setYawPitchRoll(Math.toRadians(45.0), Math.toRadians(45.0), Math.toRadians(15.0));
            fallingBlock.setTransformToWorld(transformToWorld);
            environmentBuilder.addObject(fallingBlock);
            fallingBlock.copyThisTransformToBulletMultiBody();

            sittingBlock = new RDXMediumCinderBlockRoughed();
            transformToWorld = new RigidBodyTransform();
            transformToWorld.getTranslation().set(0.0, 0.0, 0.1);
            transformToWorld.getRotation().setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0));
            sittingBlock.setTransformToWorld(transformToWorld);
            environmentBuilder.addObject(sittingBlock);
            sittingBlock.copyThisTransformToBulletMultiBody();
         }

         private ModelInstance createSphere()
         {
            ModelInstance modelInstance = RDXModelBuilder.createSphere(0.01f, Color.RED);
//            LibGDXTools.setTransparency(modelInstance, 0.8f);
            return modelInstance;
         }

         private ModelInstance createArrow()
         {
//            ModelInstance modelInstance = RDXModelBuilder.createSphere(0.01f, Color.PINK);
            float length = 0.2f;
            Color color = Color.PINK;
            ModelInstance modelInstance = RDXModelBuilder.buildModelInstance(meshBuilder ->
            {
               double coneHeight = 0.10 * length;
               double cylinderLength = length - coneHeight;
               double cylinderRadius = cylinderLength / 20.0;
               double coneRadius = 1.5 * cylinderRadius;
               meshBuilder.addCylinder(cylinderLength, cylinderRadius, new Point3D(), color);
               meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, 0.0, cylinderLength), color);
            });
//            LibGDXTools.setTransparency(modelInstance, 0.8f);
            return modelInstance;
         }

         @Override
         public void render()
         {
            environmentBuilder.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            for (ModelInstance modelInstance : pointsOnA)
            {
               modelInstance.getRenderables(renderables, pool);
            }
            for (ModelInstance modelInstance : arrowsOnB)
            {
               modelInstance.getRenderables(renderables, pool);
            }
         }

         @Override
         public void dispose()
         {
            environmentBuilder.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXBulletPhysicsInteractionForcesDemo();
   }
}
