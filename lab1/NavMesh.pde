// Useful to sort lists by a custom key
import java.util.Comparator;
import java.util.Collections;
import java.util.PriorityQueue;
import java.util.HashMap;
import java.util.HashSet;

/// In this file you will implement your navmesh and pathfinding. 

/// This node representation is just a suggestion
class Node
{
   int id;
   ArrayList<Wall> polygon;
   PVector center;
   ArrayList<Node> neighbors;
   ArrayList<Wall> connections;
   
   //Attributes for A*
   float gCost; //distance from start node to this node
   float hCost; //heuristic from this node to destination node
   float fCost; //gCost + hCost used to prioritize nodes in the open list
   Node parent; //parent node for reconstructing the path
   
   Node() {
     this.gCost = Float.MAX_VALUE;
     this.fCost = Float.MAX_VALUE;
     this.parent = null;
   }
}



class NavMesh
{   
   ArrayList<Node> nodes;
   final int maxConnections = 1;
   HashMap<PVector, Integer> cornerConnectionCount = new HashMap<>();
   
   // Check if we can connect to a corner
   boolean canConnectToCorner(PVector corner) {
     return cornerConnectionCount.getOrDefault(corner, 0) < maxConnections;
   }

  // Increment the connection count for a corner
  void incrementConnectionCount(PVector corner) {
    int currentCount = cornerConnectionCount.getOrDefault(corner, 0);
    cornerConnectionCount.put(corner, currentCount + 1);
  }
   
   void bake(Map map)
   {
       /// generate the graph you need for pathfinding
       nodes = new ArrayList<>();
       
       HashMap<PVector, Node> midpointNodeMap = new HashMap<>();
       
       //start with the original outline
       ArrayList<Wall> outline = new ArrayList<>(map.outline);
       
       //Store edges that are created to help check intersections
       
       //split map outline into convex polygons
       ArrayList<ArrayList<Wall>> polygons = splitIntoConvexPolygons(outline, map);
       
       int nodeId = 0;
       for (ArrayList<Wall> polygon : polygons) {
         
         for (Wall wall : polygon) {
           
           if (outline.contains(wall)) {
             continue;
           }
           
           PVector midpoint = getMidpoint(wall);
           
           Node midpointNode = midpointNodeMap.get(midpoint);
           if (midpointNode == null) {
             midpointNode = new Node();
             midpointNode.id = nodeId++;
             midpointNode.polygon = polygon;
             midpointNode.center = midpoint;
             midpointNode.neighbors = new ArrayList<>();
             midpointNode.connections = new ArrayList<>();
             nodes.add(midpointNode);
             midpointNodeMap.put(midpoint, midpointNode);
           }
           
           
           //connect neighboring midpoint nodes
           
           for (Wall otherWall : polygon) {
             if (otherWall != wall) {
               PVector otherMidpoint = getMidpoint(otherWall);
               Node otherMidpointNode = midpointNodeMap.get(otherMidpoint);
               if (otherMidpointNode != null && !midpointNode.neighbors.contains(otherMidpointNode)) {
                 midpointNode.neighbors.add(otherMidpointNode);
                 otherMidpointNode.neighbors.add(midpointNode);
                 
                 midpointNode.connections.add(wall);
                 otherMidpointNode.connections.add(otherWall);
               }
             }
           }
         }
       }
       //add main node for the polygon nodes list
       System.out.println("Total nodes: " + nodes.size());
       int totalConnections = 0;
       for (Node node : nodes) {
         totalConnections += node.connections.size();
       }
       System.out.println("Total nodes connections: " + totalConnections);
   }
   
   //helper methods
   
   boolean polygonsAreAdjacent(ArrayList<Wall> polygonA, ArrayList<Wall> polygonB) {
     for (Wall wallA : polygonA) {
       for (Wall wallB : polygonB) {
         if (wallsAreEqual(wallA, wallB)) {
           return true;
         }
       }
     }
     return false;
   }
   
   Wall findSharedEdge(ArrayList<Wall> polygonA, ArrayList<Wall> polygonB) {
     for (Wall wallA : polygonA) {
       for (Wall wallB : polygonB) {
         if (wallsAreEqual(wallA, wallB)) {
           return wallA;
         }
       }
     }
     return null;
   }
   
   boolean wallsAreEqual(Wall wallA, Wall wallB) {
     return (wallA.start.equals(wallB.start) && wallA.end.equals(wallB.end)) || (wallA.start.equals(wallB.end) && wallA.end.equals(wallB.start));
   }

   //split map into polygons and return the list of polygons
   ArrayList<ArrayList<Wall>> splitIntoConvexPolygons(ArrayList<Wall> outline, Map map) {
     
     ArrayList<Wall> createdSplitEdges = new ArrayList<>();
     ArrayList<ArrayList<Wall>> convexPolygons = new ArrayList<>();
     ArrayList<ArrayList<Wall>> polygonQueue = new ArrayList<>();
     polygonQueue.add(new ArrayList<>(outline));
     
     final int maxIterations = 1000;
     int iterations = 0;
     final int MAX_POLYGONS = 1000;
    
     while (!polygonQueue.isEmpty() && iterations < maxIterations) {
       ArrayList<Wall> currentPolygon = polygonQueue.remove(0);
       if (hasReflexVertices(currentPolygon)) {
         //if current polygon has reflex vertices. split it
         ArrayList<PVector> reflexVertices = findReflexVertices(currentPolygon);
         for (PVector reflexVertex : reflexVertices) {
           Wall splitEdge = createSplitEdge(reflexVertex, currentPolygon);
           if (splitEdge != null && !doesEdgeIntersectPolygon(splitEdge, createdSplitEdges) && !doesEdgeIntersectPolygon(splitEdge, map.outline)) {
             PVector midpoint = getMidpoint(splitEdge);
             
             if (isPointInPolygon(midpoint, currentPolygon) && map.isReachable(midpoint)) {
               createdSplitEdges.add(splitEdge);
               incrementConnectionCount(reflexVertex); // Increment the connection count
               ArrayList<ArrayList<Wall>> splitPolygons = splitPolygonAlongEdge(currentPolygon, reflexVertex, splitEdge);
               
               //add both split polygons to the queue
               for (ArrayList<Wall> polygon : splitPolygons) {
                 if (!hasReflexVertices(polygon) && !splitPolygons.contains(polygon)) {
                   convexPolygons.add(polygon);
                   iterations++;
                 }
                 else {
                   polygonQueue.add(polygon);
                 }
               }
             }
             else {
               System.out.println("Midpoint outside polygon");
             }
           }
         }
       }
       else {
         convexPolygons.add(currentPolygon); //add to final list if already convex
       }
       if (convexPolygons.size() >= MAX_POLYGONS) {
         System.out.println("Polygon limit reached");
         break;
       }
     }
     System.out.println("Convex Polygon Count: " + convexPolygons.size());
     return convexPolygons;
  }
  
  boolean isConvexPolygon(ArrayList<Wall> polygon) {
     for (int i = 0; i < polygon.size(); i++) {
       Wall prev = polygon.get((i - 1 + polygon.size()) % polygon.size());
       Wall current = polygon.get(i);
       Wall next = polygon.get((i + 1) % polygon.size());
       
       if (isReflex(prev, current, next)) {
         return false;
       }
     }
     return true;
   }
   
   
   boolean hasReflexVertices(ArrayList<Wall> polygon) {
        int size = polygon.size();
        for (int i = 0; i < size; i++) {
          Wall prev = polygon.get((i - 1 + size) % size);
          Wall current = polygon.get(i);
          Wall next = polygon.get((i + 1) % size);
          
          if (isReflex(prev, current, next)) {
            System.out.println("Reflex at: " + current.end);
            return true;
          }
        }
        System.out.println("No reflex vertex found");
        return false;
    }

    ArrayList<PVector> findReflexVertices(ArrayList<Wall> polygon) {
        ArrayList<PVector> reflexVertices = new ArrayList<>();
        
        for (int i = 0; i < polygon.size(); i++) {
          Wall prev = polygon.get((i - 1 + polygon.size()) % polygon.size());  
          Wall current = polygon.get(i);
          Wall next = polygon.get((i + 1) % polygon.size());

            if (isReflex(prev, current, next)) {
                reflexVertices.add(current.end); // Assuming the reflex point is at the end of the current wall
            }
        }
        return reflexVertices;
    }

    boolean isReflex(Wall prev, Wall current, Wall next) {
        // Implement logic to determine if the current vertex is a reflex vertex
        PVector prevEnd = prev.end;
        PVector currentEnd = current.end;
        PVector nextEnd = next.end;

        // Check the orientation (you can use cross products or dot products)
        PVector v1 = PVector.sub(currentEnd, prevEnd);
        PVector v2 = PVector.sub(nextEnd, currentEnd);
        
        float crossProduct = (v1.x * v2.y) - (v1.y * v2.x);
        return crossProduct > 0; // Returns true if reflex
    }

    Wall createSplitEdge(PVector reflexVertex, ArrayList<Wall> polygon) {
      //try to connect valid corner
      for (int i = 0; i < polygon.size(); i++) {
        PVector candidate = polygon.get(i).start;
        if (!candidate.equals(reflexVertex) && !isOnSameLine(reflexVertex, candidate, polygon)) {
          
          Wall splitEdge = new Wall(reflexVertex, candidate);
          PVector midpoint = getMidpoint(splitEdge);
          if (!isPointInPolygon(midpoint, polygon)) continue;
          if (!doesEdgeIntersectPolygon(splitEdge, polygon)) {
            return splitEdge;
          }
        }
      }
      return null;
    }
    
    PVector getMidpoint(Wall wall) {
      float midpointX = (wall.start.x + wall.end.x) / 2;
      float midpointY = (wall.start.y + wall.end.y) / 2;
      PVector midpoint = new PVector(midpointX, midpointY);
      return midpoint;
    }
    
    boolean isOnSameLine(PVector p1, PVector p2, ArrayList<Wall> polygon) {
      for (Wall wall : polygon) {
        if (isPointOnLineSegment(p1, p2, wall)) {
          return true;
        }
      }
      return false;
    }
    
    boolean isPointOnLineSegment(PVector p1, PVector p2, Wall wall) {
      float crossProduct = (p2.y - p1.y) * (wall.end.x - wall.start.x) - (p2.x - p1.x) * (wall.end.y - wall.start.y);
      if (Math.abs(crossProduct) > 1e-6) return false;
      // Check if the points lie within the bounds of the line segment
      float minX = Math.min(wall.start.x, wall.end.x);
      float maxX = Math.max(wall.start.x, wall.end.x);
      float minY = Math.min(wall.start.y, wall.end.y);
      float maxY = Math.max(wall.start.y, wall.end.y);
      return (p1.x >= minX && p1.x <= maxX && p1.y >= minY && p1.y <= maxY) &&
             (p2.x >= minX && p2.x <= maxX && p2.y >= minY && p2.y <= maxY);
    }
    
    boolean doesEdgeIntersectPolygon(Wall newEdge, ArrayList<Wall> polygon) {
      for (Wall wall : polygon) {
        if (doLinesIntersect(newEdge.start, newEdge.end, wall.start, wall.end)) {
          return true;
        }
      }
      return false;
    }
    
    boolean doLinesIntersect(PVector p1, PVector p2, PVector q1, PVector q2) {
      float d1 = direction(q1, q2, p1);
      float d2 = direction(q1, q2, p2);
      float d3 = direction(p1, p2, q1);
      float d4 = direction(p1, p2, q2);
      
      if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        return true;
      }
    return false;
    }
    
    boolean isOnSegment(PVector p, PVector q, PVector r) {
      return r.x <= Math.max(p.x, q.x) && r.x >= Math.min(p.x, q.x) &&
             r.y <= Math.max(p.y, q.y) && r.y >= Math.min(p.y, q.y);
    }
    
    float direction(PVector pi, PVector pj, PVector pk) {
      return (pk.x - pi.x) * (pj.y - pi.y) - (pj.x - pi.x) * (pk.y - pi.y);
    }

    /*ArrayList<Wall> splitPolygon(ArrayList<Wall> polygon) {
      ArrayList<PVector> reflexVertices = findReflexVertices(polygon);
      
      for (PVector reflexVertex : reflexVertices) {
        Wall splitEdge = createSplitEdge(reflexVertex, polygon);
        
        if (splitEdge != null) {
          ArrayList<Wall> newPolygon = splitPolygonAlongEdge(polygon, reflexVertex, splitEdge);
          return newPolygon;
        }
      }
      
      return null;
    }*/
    
    ArrayList<ArrayList<Wall>> splitPolygonAlongEdge(ArrayList<Wall> polygon, PVector reflexVertex, Wall splitEdge) {
      ArrayList<Wall> firstPolygon = new ArrayList<>();
      ArrayList<Wall> secondPolygon = new ArrayList<>();
      
      boolean addingToFirstPolygon = true;
      boolean foundSplitStart = false;
      boolean foundSplitEnd = false;
      
      HashSet<ArrayList<Wall>> uniquePolygons = new HashSet<>();
      
      //walk through the polygon's walls and distribute them between two new polygons
      for (int i = 0; i < polygon.size(); i++) {
        Wall currentWall = polygon.get(i);
        
        //check if we found the start of the split edge
        if (!foundSplitStart && (currentWall.start.equals(splitEdge.start) || currentWall.end.equals(splitEdge.start))) {
          foundSplitStart = true;
          firstPolygon.add(currentWall); //start adding to first polygon
          addingToFirstPolygon = true;
        }
        
        //add walls to the current polygon (first or second)
        if (addingToFirstPolygon) {
          firstPolygon.add(currentWall);
        }
        else {
          secondPolygon.add(currentWall);
        }
        if (foundSplitStart && (currentWall.start.equals(splitEdge.end) || currentWall.end.equals(splitEdge.end))) {
          foundSplitEnd = true;
          addingToFirstPolygon = false; //switch to adding to the second polygon
        }
      }
      
      //add split edge to both polygons
      firstPolygon.add(splitEdge);
      secondPolygon.add(new Wall(splitEdge.end, splitEdge.start)); //reverse edge for second polygon
      
      if (uniquePolygons.add(firstPolygon)) {
        uniquePolygons.add(firstPolygon);
      }
      if (uniquePolygons.add(secondPolygon)) {
        uniquePolygons.add(secondPolygon);
      }
      
      //return both polygons as an array of polygons
      ArrayList<ArrayList<Wall>> splitPolygons = new ArrayList<>(uniquePolygons);
      return splitPolygons;
    }
   
   //calculates centroid of a polygon
   PVector calculateCentroid(ArrayList<Wall> polygon) {
     float cx = 0;
     float cy = 0;
     float area = 0;
     
     for (int i = 0; i < polygon.size(); i++) {
       PVector p1 = polygon.get(i).start;
       PVector p2 = polygon.get((i + 1) % polygon.size()).start; //get next vertex, wrap around
       
       float a = (p1.x * p2.y) - (p2.x * p1.y);
       area += a;
       cx += (p1.x + p2.x) * a;
       cy += (p1.y + p2.y) * a;
     }
     
     area *= 0.5;
     cx /= (6 * area);
     cy /= (6 * area);
     
     return new PVector(cx, cy);
   }
   
   //method for A* search
   ArrayList<PVector> findPath(PVector start, PVector destination)
   {
      /// implement A* to find a path
      PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingDouble(n -> n.fCost));
      ArrayList<Node> closedSet = new ArrayList<>();
      
      Node startNode = getClosestNode(start);
      Node endNode = getClosestNode(destination);
      
      if (startNode == null || endNode == null) {
        return new ArrayList<>();
      }
      
      startNode.gCost = 0;
      startNode.fCost = heuristic(startNode.center, endNode.center);
      openSet.add(startNode);
      
      //use loop and A* to traverse
      while (!openSet.isEmpty()) {
        Node current = openSet.poll();

        // If we reached the end node, reconstruct the path
        if (current == endNode) {
            return reconstructPath(endNode);
        }

        closedSet.add(current);

        // Check all neighbors
        for (Node neighbor : current.neighbors) {
            if (closedSet.contains(neighbor)) continue;

            float tentativeG = current.gCost + dist(current.center.x, current.center.y, neighbor.center.x, neighbor.center.y);

            if (!openSet.contains(neighbor) || tentativeG < neighbor.gCost) {
                neighbor.gCost = tentativeG;
                neighbor.fCost = tentativeG + heuristic(neighbor.center, endNode.center);
                neighbor.parent = current;

                if (!openSet.contains(neighbor)) {
                    openSet.add(neighbor);
                }
            }
        }
    }
    
    return null;  // No path found
   }
   
   //helper methods for the A*
   float heuristic(PVector location1, PVector location2) {
     return dist(location1.x, location1.y, location2.x, location2.y);
   }
   
   Node getClosestNode(PVector position) {
     Node closestNode = null;
     float minDistance = Float.MAX_VALUE;
     for (Node node: nodes) {
       float distance = dist(position.x, position.y, node.center.x, node.center.y);
       if (distance < minDistance) {
         minDistance = distance;
         closestNode = node;
       }
     }
     return closestNode;
   }
   
   ArrayList<PVector> reconstructPath(Node endNode) {
     ArrayList<PVector> path = new ArrayList<>();
     Node currentNode = endNode;
     while (currentNode != null) {
       path.add(currentNode.center);
       currentNode = currentNode.parent;
     }
     Collections.reverse(path);
     return path;
   }
   //end helper methods for A*
   
   
   void update(float dt)
   {
      draw();
   }
   
   void draw() {
     drawNavMesh();
   }
   
   void drawNavMesh() {
     stroke(0, 255, 0);
     
     //draw each polygon
     for (Node node : nodes) {
       ArrayList<Wall> polygon = node.polygon;
       
       for (Wall wall : polygon) {
         line(wall.start.x, wall.start.y, wall.end.x, wall.end.y);
       }
       
       fill(255, 0, 0);
       ellipse(node.center.x, node.center.y, 5, 5);
     }
     
     stroke(0, 0, 255);
     for (Node node : nodes) {
       for (Wall wall : node.connections) {
         line(wall.start.x, wall.start.y, wall.end.x, wall.end.y);
       }
     }
     
   }
}
