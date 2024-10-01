// Useful to sort lists by a custom key
import java.util.Comparator;


/// In this file you will implement your navmesh and pathfinding. 

/// This node representation is just a suggestion
class Node
{
   int id;
   ArrayList<Wall> polygon;
   PVector center;
   ArrayList<Node> neighbors;
   ArrayList<Wall> connections;
}



class NavMesh
{   
   ArrayList<Node> nodes;
   
   void bake(Map map)
   {
       /// generate the graph you need for pathfinding
       nodes = new ArrayList<>();
       // iterate over the outline walls and create nodes for each area
       for (Wall outlineWall : map.outline) {
         Node node = new Node();
         node.id = nodes.size(); //assign unique id
         node.polygon = new ArrayList<>(map.outline); //outlines walls as polygon boundary
         node.center = outlineWall.center(); // set the center as the midpoint of the wall for now
         node.neighbors = new ArrayList<>();
         node.connections = new ArrayList<>();
         
         nodes.add(node);
       }
       
       // now add connections based on shared walls between nodes
       for (Node node : nodes) {
         for (Node other : nodes) {
           if (node != other && shareEdge(node.polygon, other.polygon)) {
             node.neighbors.add(other);
             node.connections.add(sharedEdge(node.polygon, other.polygon));
           }
         }
       }
       
       // handle obstacles
       for (Obstacle obstacle : map.obstacles) {
         Node node = new Node();
         node.id = nodes.size();
         node.polygon = obstacle.walls;
         node.center = calculateCentroid(obstacle.walls);
         node.neighbors = new ArrayList<>();
         node.connections = new ArrayList<>();
       }
   }
   
   //helper methods
   //calculates centroid of a polygon
   PVector calculateCentroid(ArrayList<Wall> walls) {
     float x = 0, y = 0;
     for (Wall wall : walls) {
       x += wall.center().x;
       y += wall.center().y;
     }
     x /= walls.size();
     y /= walls.size();
     return new PVector(x, y);
   }
   
   // check if two sets of walls share an edge
   boolean shareEdge(ArrayList<Wall> polygon1, ArrayList<Wall> polygon2) {
     for (Wall wall1 : polygon1) {
       for (Wall wall2 : polygon2) {
         if (wall1.equals(wall2)) {
           return true;
         }
       }
     }
     return false;
   }
   
   //return shared edge between two sets of walls
   Wall sharedEdge(ArrayList<Wall> polygon1, ArrayList<Wall> polygon2) {
     for (Wall wall1 : polygon1) {
       for (Wall wall2 : polygon2) {
         if (wall1.equals(wall2)) {
           return wall1;
         }
       }
     }
     return null;
   }
   
   ArrayList<PVector> findPath(PVector start, PVector destination)
   {
      /// implement A* to find a path
      ArrayList<PVector> result = null;
      return result;
   }
   
   
   void update(float dt)
   {
      draw();
   }
   
   void draw()
   {
      if (!drawn) {
        ArrayList<String> drawnLines = new ArrayList<>();
      /// use this to draw the nav mesh graph
      // draw each node
      for (Node node : nodes) {
        // draw the polygon for the node(walls)
        for (Wall wall : node.polygon) {
          wall.draw();
          
        }
        
        //optional, draw center of the node
        fill(0, 255, 0);
        circle(node.center.x, node.center.y, 5);
        
        // collect unique vertices from polygons (start and end points of walls)
        ArrayList<PVector> uniqueVertices = new ArrayList<>();
        for (Wall wall : node.polygon) {
          if (!containsVertex(uniqueVertices, wall.start)) {
            uniqueVertices.add(wall.start); //add start point if it does not exist in list
          }
          if (!containsVertex(uniqueVertices, wall.end)) {
            uniqueVertices.add(wall.end); //add end point if it does not exist in list
          }
        }
        
        //draw lines from the center of the node to the unique vertices
        stroke(0, 255, 0);
        for (PVector vertex : uniqueVertices) {
          String lineKey = createLineKey(node.center, vertex);
          if (!drawnLines.contains(lineKey)) {
            line(node.center.x, node.center.y, vertex.x, vertex.y);
            drawnLines.add(lineKey);
          }
        }
        
        //draw the connection between nodes (neighboring nodes)
        stroke(0, 255, 255);
        for (Node neighbor : node.neighbors) {
          String neighborLineKey = createLineKey(node.center, neighbor.center);
          if (!drawnLines.contains(neighborLineKey)) {
            line(node.center.x, node.center.y, neighbor.center.x, neighbor.center.y);
            drawnLines.add(neighborLineKey);
          }
        }
      }
     }
   }
   
   //helper function to check is vertex is in the list
   boolean containsVertex(ArrayList<PVector> vertices, PVector vertex) {
     for (PVector v : vertices) {
       if (v.equals(vertex)) {
         return true;
       }
     }
     return false;
   }
   
   //helper function to make a unique key for a line between 2 points
   String createLineKey(PVector point1, PVector point2) {
     //simple concatenation of the coordinates of two points
     return point1.x + "," + point1.y + "-" + point2.x + "," + point2.y;
   }
}
