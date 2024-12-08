// Useful to sort lists by a custom key
import java.util.Comparator;
import java.util.HashMap;
import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.*;

/// In this file you will implement your navmesh and pathfinding. 
/// This node representation is just a suggestion
class Node
{
   int id;    //each polygon is given a unique ID    
   ArrayList<Wall> polygon;    //each polygon will have convex walls
   PVector center;    //each polygon will have a centerpoint
   ArrayList<Node> neighbors;    //a list of all adjacent nodes
   HashMap<Node, Wall> neighborToWall;    //the map to quickly find a wall based on which neighbor is being analyzed
   ArrayList<Wall> connections;    //a seperate list of walls to make the design easier
   
   Node(int id, ArrayList<Wall> walls){
    this.id = id;  
    polygon = walls;
    center = findCenter(walls);
    neighbors = new ArrayList<Node>();
    neighborToWall = new HashMap<Node, Wall>();
    connections = new ArrayList<Wall>();   
  }
  
  //finds the center given the walls, simply an average of all the coordinates
  PVector findCenter (ArrayList<Wall> polygon){
    float xCoord = 0;
    float yCoord = 0;
    
    for(int i = 0; i < polygon.size(); i++){
      xCoord += polygon.get(i).start.x;
      yCoord += polygon.get(i).start.y;
    }
    xCoord /= polygon.size();
    yCoord /= polygon.size();
    
    return new PVector(xCoord, yCoord);
  }
  
  void addNeighbor(Node newNeighbor){
    neighbors.add(newNeighbor);
  }
  
  void addNeighborWall(Node newNeighbor, Wall wall){
    neighborToWall.put(newNeighbor, wall);
  }
  
  void addConnection(Wall newWall){
    connections.add(newWall);
  }
}



class NavMesh
{   
   public ArrayList<PVector> ReflexPoints;  //Reflex points that need to be adjusted
   public ArrayList<Wall> NewWalls;      //Walls that have been added to split polygon to be convex useful as a counter
   public ArrayList<Node> convexPolygons;    //The list of all convex polygons
   //public ArrayList<PVector> holyGrail;  //path to be followed
   HashMap<Integer, Node> NewWallsMap = new HashMap<Integer, Node>();   //Useful for providing neighbors into each node using the index number
      
   void bake(Map map)
   {
       ArrayList<Wall> Walls = map.walls;
       NewWalls = new ArrayList<Wall>();
       ReflexPoints = new ArrayList<PVector>(); 
       convexPolygons = new ArrayList<Node>();
       NewWallsMap = new HashMap<Integer, Node>(); 
       //if (ReflexPoints.size() > 0){
       Split(Walls);  //recursively splits the walls and will add everything to public memory
         
         //each wall should have a list of connections, and a varying number of walls. the map should contain a listing for each of these corresponding walls
       for(int i = 0; i < convexPolygons.size(); i++){
         if(convexPolygons.get(i).connections.size() != convexPolygons.get(i).neighbors.size()){ 
             
           for(int j = 0; j < convexPolygons.get(i).connections.size(); j++){  
               
             if(NewWallsMap.containsKey(convexPolygons.get(i).connections.get(j).getID())){
                 
               convexPolygons.get(i).addNeighbor(NewWallsMap.get(convexPolygons.get(i).connections.get(j).getID()));
               convexPolygons.get(i).addNeighborWall(NewWallsMap.get(convexPolygons.get(i).connections.get(j).getID()), convexPolygons.get(i).connections.get(j));              }
             }                     
           }
         }   
               
       //if(convexPolygons != null){
      for(int l = 0; l < convexPolygons.size(); l++){
        for(int f = 0; f < convexPolygons.get(l).polygon.size(); f++){
          stroke(020);            
        }
        
        for(int f = 0; f < convexPolygons.get(l).neighbors.size(); f++){
          stroke(150);          
        }
      }   
   }

   void Split(ArrayList<Wall> PolygonWalls)
   {
     //find first reflexive point to split off from
     int ReflexPointNumber = -1;
     boolean Convex = true;
        
     for(int i = 0; i<PolygonWalls.size(); ++i){
           int next = (i+1)%PolygonWalls.size();
           if(PolygonWalls.get(i).normal.dot(PolygonWalls.get(next).direction) >= 0)
           {
             ReflexPointNumber = next;
             Convex = false;
             break;
           }
         }          
     if(Convex){
       //Create new polygon, no reflex points found
       //creates them with IDs of the order they were created in
       Node convex = new Node(convexPolygons.size(), PolygonWalls);
             
       for(int e = 0; e < PolygonWalls.size(); e++){  
         if(PolygonWalls.get(e).getID() != 0){
           //found a wall that is a connection, so it will be guaranteed to be included in this convex node
           convex.addConnection(PolygonWalls.get(e));
           
           //Checking if the hashmap already contains the corresponding neighbor
           if(NewWallsMap.containsKey(PolygonWalls.get(e).getID())){
             //if it does it can add the neighboring node
             convex.addNeighbor(NewWallsMap.get(PolygonWalls.get(e).getID()));
             convex.addNeighborWall(NewWallsMap.get(PolygonWalls.get(e).getID()), PolygonWalls.get(e));
             
             NewWallsMap.put(-1 * PolygonWalls.get(e).getID(), convex);
           }else{  //if not the neighboring node has not been created yet so this node will create it
             NewWallsMap.put(-1 * PolygonWalls.get(e).getID(), convex);
             //the number is negative so it will correspond to the reverse wall
           }           
         }
       }          
       convexPolygons.add(convex);
       
     }  else { //Going to need to split it again
     
       PVector ReflexPoint = PolygonWalls.get(ReflexPointNumber).start;       
       int FixPointNumber = 0;    //initialize these variables, a valid fix point should always be found in the loop
       PVector FixPoint = ReflexPoint;
     
       int PointGap = 2;      //Points between start and fix, starts by skipping the point we know is reflex already
       boolean Reachable = false;
       boolean Reflexive = true;
       
       while(Reachable == false || Reflexive == false){
         FixPointNumber = (ReflexPointNumber+PointGap)%PolygonWalls.size();  //Uses the next available point so it is a very simple iteration through all the points, and there will always be at least one point that fulfills both requirements
         FixPoint = PolygonWalls.get(FixPointNumber).start;
         Wall TestWall = new Wall(ReflexPoint, FixPoint);
         
         //Checks if the new edge would actually interfere with any existing walls
         Reachable = IsPlaceable(PolygonWalls, TestWall);
         
         //get the previous wall even if it was the first wall in the list
         int Previous = (ReflexPointNumber-1);
         if(Previous <0){
           Previous = PolygonWalls.size() -1;
         }
         if(PolygonWalls.get(Previous).normal.dot(TestWall.direction) >= 0){    //Checks if the new wall will fix the initial reflexive point
           Reflexive = false;
         }else{
           Reflexive = true;
         }
         
         PointGap += 1;  //if the first point doesnt work, then it will go to the next point
       }
       
       //makes sure that in the order list of walls, the reflex point comes before so both new polygons are properly directioned and ordered
       if(FixPointNumber < ReflexPointNumber){
         int temp = FixPointNumber;
         FixPointNumber = ReflexPointNumber;
         ReflexPointNumber = temp;
         ReflexPoint = PolygonWalls.get(ReflexPointNumber).start;
         FixPoint = PolygonWalls.get(FixPointNumber).start;
       }
       
       //Two walls needed so each polygon has ordered walls going counter clockwise
       Wall Normal = new Wall(ReflexPoint, FixPoint);
       Normal.setID((NewWalls.size() + 1));
       Wall Reverse = new Wall(FixPoint, ReflexPoint);
       Reverse.setID(-1 * (NewWalls.size() + 1));
       
       //Adding the new wall to a list or later use/recursion
       NewWalls.add(Normal);   
                
       //Walls split between 2 polygons
       ArrayList<Wall> FrontWalls = new ArrayList<Wall>();
       ArrayList<Wall> BackWalls = new ArrayList<Wall>(); 
       BackWalls.add(Reverse);
                   
       for(int r = 0; r<PolygonWalls.size(); r++){
           if(r >= ReflexPointNumber && r < FixPointNumber){
             BackWalls.add(PolygonWalls.get(r));
           } else{
             FrontWalls.add(PolygonWalls.get(r));
           }
       }
       FrontWalls.add(ReflexPointNumber, Normal);      
       Split(FrontWalls);
       Split(BackWalls);
     }
   }  
   
   Boolean IsPlaceable (ArrayList<Wall> PolygonWalls, Wall TestWall){
     PVector From = PVector.add(TestWall.start, PVector.mult(TestWall.direction, 0.01));
     PVector To = PVector.add(TestWall.end, PVector.mult(TestWall.direction, -0.01));
     for(int n = 0; n<PolygonWalls.size(); n++){      //checks all walls to see if there is any crossing
           if(PolygonWalls.get(n).crosses(From, To)){
             return false;
           }
         }
     return true;
   }
   
   class frontierNode{
     float heuristic;
     float pathLength;
     float aValue;
     Node currentNode;
     frontierNode parentNode;
     
     frontierNode(Node currentNode, frontierNode parentNode, float pathLength, PVector target){
       this.currentNode = currentNode;
       this.parentNode = parentNode;
       this.pathLength = pathLength;
       this.heuristic = getPVectorDistance(currentNode.center, target);
       this.aValue = pathLength + heuristic;
     }
   }
   
   ArrayList<PVector> findPath(PVector boidPosition, PVector destinationPosition)
   {
     ArrayList<PVector> path = new ArrayList<PVector>();
    
     //find which nodes boid and destination are in
     Node startNode = null;
     Node endNode = null;

     for(Node testNode: convexPolygons){     
       if(pointContained(boidPosition, testNode.polygon)){
         startNode = testNode;
         break;
       }
     }
     
     for(Node testNode: convexPolygons){
       if(pointContained(destinationPosition, testNode.polygon)){
         endNode = testNode;
         break;
       }
     }
     
     //call find node path
     if (startNode != endNode){
       ArrayList<Node> nodePath = new ArrayList<Node>();
       nodePath = findNodePath(startNode, endNode);
       
       for(int i = 0; i < nodePath.size() - 1; i++){
         //path.add(nodePath.get(i).center);
         path.add(nodePath.get(i).neighborToWall.get(nodePath.get(i + 1)).center());    
       }
     }
     path.add(destinationPosition);   
     return path;  
   }
   
   boolean pointContained (PVector point, ArrayList<Wall> polygon){
     //Wall infinity = new Wall(point, new PVector(point.x + 2*width, point.y));
     int crosses = 0;
     for(Wall wall: polygon){
            if (wall.crosses(point, new PVector(point.x + 2*width, point.y))){
              crosses ++;
            } 
          }
     if((crosses % 2) == 0){
       return false;
     }
     return true;
   }
   
   ArrayList<Node> findNodePath(Node start, Node destination)
   {
     ArrayList<frontierNode> frontierList = new ArrayList<frontierNode>();
     ArrayList<Node> previouslyExpandedList = new ArrayList<Node>();
     
     frontierList.add(new frontierNode(start, null, 0, destination.center));
     while(frontierList.get(0).currentNode != destination){
        for(int i = 0; i < frontierList.get(0).currentNode.neighbors.size(); i++){
          float newPath = frontierList.get(0).pathLength + getPVectorDistance(frontierList.get(0).currentNode.center, frontierList.get(0).currentNode.neighbors.get(i).center);
          frontierList.add(new frontierNode(frontierList.get(0).currentNode.neighbors.get(i), frontierList.get(0), newPath, destination.center));
        }
        previouslyExpandedList.add(frontierList.get(0).currentNode);
        frontierList.remove(0);
        frontierList.sort(new FrontierCompare());
        while(previouslyExpandedList.contains(frontierList.get(0).currentNode)){
          frontierList.remove(0);
        }
     }
     
      ArrayList<Node> result = new ArrayList<Node>();
      result.add(frontierList.get(0).currentNode);
      
      frontierNode parentNode = frontierList.get(0).parentNode;
      
      while(result.get(0) != start){
        result.add(0, parentNode.currentNode);
        parentNode = parentNode.parentNode;
      }      
      return result;
   }
   
   class FrontierCompare implements Comparator<frontierNode>{
     int compare(frontierNode a, frontierNode b){
       if(a.aValue > b.aValue){
         return 1;
       } else if(a.aValue < b.aValue){
         return -1;
       } else
         return 0;
     }
   }
    
   TreeMap<Float, Node> expandFrontier(TreeMap<Float, Node>frontier, Node target){
     float shortestPath = frontier.firstKey();
     for(int i = 0; i<frontier.get(shortestPath).neighbors.size(); i++){
       float heuristic = shortestPath + getPVectorDistance(target.center, frontier.get(shortestPath).neighbors.get(i).center);
       frontier.put(heuristic, frontier.get(shortestPath).neighbors.get(i));
     }
     return frontier;
   }
   
   void update(float dt)
   {
      draw();    
   }
   
   void draw()
   {
      /// use this to draw the nav mesh graph
      for(int i = 0; i < ReflexPoints.size(); i++){
        stroke(0,255,0);
        circle(ReflexPoints.get(i).x, ReflexPoints.get(i).y, 20);
      }
      
      int blueColor = 255;
      int redColor = 0;
      if(NewWalls!=null){
      for(int j = 0; j < NewWalls.size(); j++){
        stroke(redColor,0,blueColor);
        blueColor -= 10;
        redColor += 10;
      }
     }
      
      if(convexPolygons != null){
        for(int l = 0; l < convexPolygons.size(); l++){
          stroke(150);
          circle(convexPolygons.get(l).center.x, convexPolygons.get(l).center.y, 20);
          for(int f = 0; f < convexPolygons.get(l).connections.size(); f++){
            stroke(150);
            line(convexPolygons.get(l).connections.get(f).start.x, convexPolygons.get(l).connections.get(f).start.y, convexPolygons.get(l).connections.get(f).end.x, convexPolygons.get(l).connections.get(f).end.y);
          }
          for(int f = 0; f < convexPolygons.get(l).polygon.size(); f++){
            stroke(020);          
          }
          for(int f = 0; f < convexPolygons.get(l).neighbors.size(); f++){
            stroke(150);          
          }
        }
      }
   }
}
