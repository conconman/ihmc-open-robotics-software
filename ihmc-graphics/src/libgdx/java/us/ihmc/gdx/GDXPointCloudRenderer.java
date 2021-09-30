package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.gdx.shader.GDXShader;
import us.ihmc.gdx.shader.GDXUniform;

public class GDXPointCloudRenderer implements RenderableProvider
{
   private Renderable renderable;
   private float[] vertices;

   private final VertexAttributes vertexAttributes = new VertexAttributes(new VertexAttribute(VertexAttributes.Usage.Position,
                                                                                              3,
                                                                                              ShaderProgram.POSITION_ATTRIBUTE),
                                                                          new VertexAttribute(VertexAttributes.Usage.ColorUnpacked,
                                                                                              4,
                                                                                              ShaderProgram.COLOR_ATTRIBUTE),
                                                                          new VertexAttribute(VertexAttributes.Usage.Generic,
                                                                                              1,
                                                                                              GL41.GL_FLOAT,
                                                                                              false,
                                                                                              "a_size"));
   private final int floatsPerVertex = vertexAttributes.vertexSize / 4;
   private final GDXUniform screenWidthUniform = GDXUniform.createGlobalUniform("u_screenWidth", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, (float) Gdx.graphics.getWidth());
   });

   private RecyclingArrayList<Point3D32> pointsToRender;
   private ColorProvider colorProvider;
   private float pointScale = 0.01f;

   public interface ColorProvider
   {
      public float getNextR();

      public float getNextG();

      public float getNextB();
   }

   public void create(int size)
   {
      GL41.glEnable(GL41.GL_VERTEX_PROGRAM_POINT_SIZE);

      renderable = new Renderable();
      renderable.meshPart.primitiveType = GL41.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(ColorAttribute.createDiffuse(Color.WHITE));

      vertices = new float[size * floatsPerVertex];
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
      boolean isStatic = false;
      int maxVertices = size;
      int maxIndices = 0;
      renderable.meshPart.mesh = new Mesh(isStatic, maxVertices, maxIndices, vertexAttributes);

      GDXShader shader = new GDXShader(getClass());
      shader.create();
      shader.getBaseShader().register(DefaultShader.Inputs.viewTrans, DefaultShader.Setters.viewTrans);
      shader.getBaseShader().register(DefaultShader.Inputs.projTrans, DefaultShader.Setters.projTrans);
      shader.getBaseShader().register(screenWidthUniform.getUniform(), screenWidthUniform.getSetter());
      shader.init(renderable);
      renderable.shader = shader.getBaseShader();
   }

   public void updateMesh()
   {
      updateMesh(1.0f);
   }

   public void updateMesh(float alpha)
   {
      if (pointsToRender != null && !pointsToRender.isEmpty())
      {
         for (int i = 0; i < pointsToRender.size(); i++)
         {
            int offset = i * floatsPerVertex;

            Point3D32 point = pointsToRender.get(i);
            vertices[offset] = point.getX32();
            vertices[offset + 1] = point.getY32();
            vertices[offset + 2] = point.getZ32();

            // color [0.0f - 1.0f]
            vertices[offset + 3] = colorProvider.getNextR();
            vertices[offset + 4] = colorProvider.getNextG();
            vertices[offset + 5] = colorProvider.getNextB();
            vertices[offset + 6] = alpha; // alpha

            vertices[offset + 7] = pointScale; // size
//            vertices[offset + 8] = 1.0f; // cosine [0-1]
//            vertices[offset + 9] = 0.0f; // sine [0-1]
         }

         renderable.meshPart.size = pointsToRender.size();
         renderable.meshPart.mesh.setVertices(vertices, 0, pointsToRender.size() * floatsPerVertex);
         renderable.meshPart.update();
      }
   }

   public float[] getVerticesArray()
   {
      return vertices;
   }

   public void updateMeshFast(int numberOfPoints)
   {
      renderable.meshPart.size = numberOfPoints;
      renderable.meshPart.mesh.setVertices(vertices, 0, numberOfPoints * floatsPerVertex);
      renderable.meshPart.update();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      renderables.add(renderable);
   }

   public void dispose()
   {
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
   }

   public void setPointsToRender(RecyclingArrayList<Point3D32> pointsToRender)
   {
      setPointsToRender(pointsToRender, new ColorProvider()
      {
         @Override
         public float getNextR()
         {
            return Color.WHITE.r;
         }

         @Override
         public float getNextG()
         {
            return Color.WHITE.g;
         }

         @Override
         public float getNextB()
         {
            return Color.WHITE.b;
         }
      });
   }

   public void setPointsToRender(RecyclingArrayList<Point3D32> pointsToRender, Color color)
   {
      setPointsToRender(pointsToRender, new ColorProvider()
      {
         @Override
         public float getNextR()
         {
            return color.r;
         }

         @Override
         public float getNextG()
         {
            return color.g;
         }

         @Override
         public float getNextB()
         {
            return color.b;
         }
      });
   }

   public void setPointsToRender(RecyclingArrayList<Point3D32> pointsToRender, ColorProvider provider)
   {
      this.pointsToRender = pointsToRender;
      this.colorProvider = provider;
   }

   public void setPointScale(float size)
   {
      this.pointScale = size;
   }
}
