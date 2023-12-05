package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;

/**
 * A performance optimization to help avoid rebuilding meshes unecessarily
 * and also to do this minimum amount required when necessary.
 * Best if using an extending class like {@link RDXMutableLineModel}.
 */
public class RDXMutableMeshModel
{
   private Color color = null;
   private RDXModelInstance modelInstance;

   public boolean isColorOutOfDate(Color color)
   {
      boolean outOfDate = this.color != color;
      this.color = color;
      return outOfDate;
   }

   protected void updateMesh(MeshDataHolder meshDataHolder)
   {
      if (modelInstance == null)
      {
         modelInstance = new RDXModelInstance(RDXModelBuilder.buildModelInstance(meshBuilder -> meshBuilder.addMesh(meshDataHolder, color)));
      }
      else
      {
         Mesh mesh = modelInstance.model.nodes.get(0).parts.get(0).meshPart.mesh;
         RDXMeshDataInterpreter.repositionMeshVertices(meshDataHolder, mesh, color);
      }
   }

   public RDXModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);
   }
}
