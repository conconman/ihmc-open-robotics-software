package us.ihmc.footstepPlanning.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

public class MonteCarloFootstepPlanner
{
   private MonteCarloFootstepPlannerParameters parameters;
   private MonteCarloFootstepPlanningDebugger debugger;

   private MonteCarloPlanningWorld world;
   private MonteCarloWaypointAgent agent;
   private MonteCarloFootstepNode root;

   private HashMap<MonteCarloFootstepNode, MonteCarloFootstepNode> visitedNodes = new HashMap<>();

   Random random = new Random();

   //private int searchIterations = 100;
   //private int simulationIterations = 8;
   //private int goalMargin = 5;

   private int uniqueNodeId = 0;
   private int worldHeight = 200;
   private int worldWidth = 200;

   private boolean planning = false;

   public MonteCarloFootstepPlanner(MonteCarloFootstepPlannerParameters parameters)
   {
      this.debugger = new MonteCarloFootstepPlanningDebugger(this);
      this.parameters = parameters;
      this.world = new MonteCarloPlanningWorld(parameters.getGoalMargin(), worldHeight, worldWidth);
   }

   public FootstepPlan generateFootstepPlan(MonteCarloFootstepPlannerRequest request)
   {
      if (root == null)
      {
         Point2D position = new Point2D(request.getStartFootPoses().get(RobotSide.LEFT).getPosition().getX() * 50,
                                        request.getStartFootPoses().get(RobotSide.LEFT).getPosition().getY() * 50);
         float yaw = (float) request.getStartFootPoses().get(RobotSide.LEFT).getYaw();
         Point3D state = new Point3D(position.getX(), position.getY(), yaw);
         root = new MonteCarloFootstepNode(state, null, RobotSide.LEFT, uniqueNodeId++);
      }

      planning = true;
      debugger.setRequest(request);
      debugger.refresh();

      for (int i = 0; i < parameters.getNumberOfIterations(); i++)
      {
         updateTree(root, request);
      }

      FootstepPlan plan = MonteCarloPlannerTools.getFootstepPlanFromTree(root, request);

      planning = false;
      return plan;
   }

   public void updateTree(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      if (node == null)
      {
         LogTools.debug("Node is null");
         return;
      }

      //node.sortChildren();
      //node.prune(plannerParameters.getMaxNumberOfChildNodes());

      if (node.getChildren().isEmpty())
      {
         MonteCarloFootstepNode childNode = expand(node, request);
         if (childNode != null)
         {
            double score = simulate(childNode, request);
            childNode.setValue((float) score);
            backPropagate(node, (float) score);
         }
      }
      else
      {
         float bestScore = 0;
         MonteCarloFootstepNode bestNode = null;
         for (MonteCarloTreeNode child : node.getChildren())
         {
            child.updateUpperConfidenceBound(parameters.getExplorationConstant());
            if (child.getUpperConfidenceBound() >= bestScore)
            {
               bestScore = child.getUpperConfidenceBound();
               bestNode = (MonteCarloFootstepNode) child;
            }
         }
         updateTree(bestNode, request);
      }
   }

   public MonteCarloFootstepNode expand(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      ArrayList<?> availableStates = node.getAvailableStates(request, parameters);

      //LogTools.info("Total Available States: {}", availableStates.size());

      for (Object newStateObj : availableStates)
      {
         MonteCarloFootstepNode newState = (MonteCarloFootstepNode) newStateObj;
         double score = MonteCarloPlannerTools.scoreFootstepNode(node, newState, request, parameters);

         if (node.getLevel() == 0)
         {
            debugger.plotNodes(availableStates);
            //LogTools.info(String.format("Previous: %d, %d, %.2f, Node: %d, %d, %.2f, Action: %d, %d, %.2f, Score: %.2f",
            //                            (int) node.getState().getX(),
            //                            (int) node.getState().getY(),
            //                            node.getState().getZ(),
            //                            (int) newState.getState().getX(),
            //                            (int) newState.getState().getY(),
            //                            newState.getState().getZ(),
            //                            (int) (newState.getState().getX() - node.getState().getX()),
            //                            (int) (newState.getState().getY() - node.getState().getY()),
            //                            newState.getState().getZ() - node.getState().getZ(),
            //                            score));
         }

         if (node.getLevel() < parameters.getMaxTreeDepth() && score > parameters.getInitialValueCutoff())
         {
            if (visitedNodes.getOrDefault(newState, null) != null)
            {
               MonteCarloFootstepNode existingNode = visitedNodes.get(newState);
               node.addChild(existingNode);
               existingNode.getParents().add(node);
            }
            else
            {
               MonteCarloFootstepNode postNode = new MonteCarloFootstepNode(newState.getState(), node, newState.getRobotSide(), uniqueNodeId++);
               postNode.setValue((float) score);
               visitedNodes.put(newState, postNode);
               node.addChild(postNode);

               // plot the newest node on debugger
               //debugger.plotNode(postNode);
               //debugger.display(30);
            }
         }
         //else
         //{
         //   LogTools.warn("Not Inserting Action: {}, Score: {}", newState.getState(), score);
         //}
      }

      if (node.getChildren().isEmpty())
      {
         LogTools.debug("No Children");
         return null;
      }

      return (MonteCarloFootstepNode) node.getMaxQueueNode();
   }

   public double simulate(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      double score = 0;

      MonteCarloFootstepNode simulationState = new MonteCarloFootstepNode(node.getState(), null, node.getRobotSide().getOppositeSide(), 0);

      for (int i = 0; i < parameters.getNumberOfSimulations(); i++)
      {
         ArrayList<MonteCarloFootstepNode> nextStates = simulationState.getAvailableStates(request, parameters);
         //LogTools.info("Number of next states: {}", nextStates.size());
         if (nextStates.isEmpty())
            break;

         int actionIndex = random.nextInt(0, nextStates.size() - 1);
         simulationState = nextStates.get(actionIndex);

         //LogTools.info(String.format("Simulation %d, Random State: %s, Actions: %d, Side:%s", i, randomState.getPosition(), nextStates.size(), randomState.getRobotSide()));
         score += MonteCarloPlannerTools.scoreFootstepNode(node, simulationState, request, parameters);
         //LogTools.info("Action Taken: {}, Score: {}", actionIndex, score);
      }

      return score;
   }

   public void backPropagate(MonteCarloFootstepNode node, float score)
   {
      node.setValue(Math.max(score, node.getValue()));
      node.incrementVisits();

      if (!node.getParents().isEmpty())
      {
         for (MonteCarloTreeNode parent : node.getParents())
         {
            backPropagate((MonteCarloFootstepNode) parent, score);
         }
      }
   }

   public void reset(MonteCarloFootstepPlannerRequest request)
   {
      random.setSeed(100);
      uniqueNodeId = 0;
      visitedNodes.clear();

      if (request == null)
         root = new MonteCarloFootstepNode(new Point3D(), null, RobotSide.LEFT, uniqueNodeId++);
      else
         root = new MonteCarloFootstepNode(new Point3D(request.getStartFootPoses().get(RobotSide.LEFT).getPosition().getX() * 50,
                                                       request.getStartFootPoses().get(RobotSide.LEFT).getPosition().getY() * 50,
                                                       request.getStartFootPoses().get(RobotSide.LEFT).getYaw()),
                                           null,
                                           RobotSide.LEFT,
                                           uniqueNodeId++);
   }

   public MonteCarloPlanningWorld getWorld()
   {
      return world;
   }

   public MonteCarloTreeNode getRoot()
   {
      return root;
   }

   public List<MonteCarloFootstepNode> getVisitedNodes()
   {
      return new ArrayList<>(visitedNodes.values());
   }

   public MonteCarloFootstepPlanningDebugger getDebugger()
   {
      return debugger;
   }

   public boolean isPlanning()
   {
      return planning;
   }
}
