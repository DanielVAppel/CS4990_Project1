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
       // Start with the original outline walls
        ArrayList<Wall> outline = new ArrayList<>(map.outline);
        
        // Split polygons until no reflex vertices remain
        ArrayList<ArrayList<Wall>> polygons = splitIntoConvexPolygons(outline);
        
        // Create nodes from the resulting polygons
        for (ArrayList<Wall> polygon : polygons) {
            Node node = new Node();
            node.id = nodes.size();
            node.polygon = polygon;
            node.center = calculateCentroid(polygon);
            node.neighbors = new ArrayList<>();
            node.connections = new ArrayList<>();
            nodes.add(node);
        }
       
       //add connections based on reflex vertices
       connectReflexVertices(map);
       
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
   
   void connectReflexVertices(Map map) {
        for (Node node : nodes) {
            ArrayList<PVector> reflexVertices = findReflexVertices(node.polygon);
            for (int i = 0; i < reflexVertices.size(); i++) {
                for (int j = i + 1; j < reflexVertices.size(); j++) {
                    PVector startVertex = reflexVertices.get(i);
                    PVector endVertex = reflexVertices.get(j);
                    
                    // Check for collision with walls
                    if (!map.collides(startVertex, endVertex)) {
                        node.neighbors.add(createConnectionNode(startVertex, endVertex, node));
                        node.connections.add(new Wall(startVertex, endVertex)); // Save the new connection
                    }
                }
            }
        }
    }
   
   //split map into polygons and return the list of polygons
   ArrayList<ArrayList<Wall>> splitIntoConvexPolygons(ArrayList<Wall> outline) {
    ArrayList<ArrayList<Wall>> convexPolygons = new ArrayList<>();
    ArrayList<Wall> currentPolygon = new ArrayList<>(outline);
    
    while (!currentPolygon.isEmpty()) {
        if (!hasReflexVertices(currentPolygon)) {
            convexPolygons.add(currentPolygon);
            break; // if no reflex vertices, we can add the polygon as is
        }
        
        // Find all reflex vertices
        ArrayList<PVector> reflexVertices = findReflexVertices(currentPolygon);
        
        if (!reflexVertices.isEmpty()) {
            // Iterate through each reflex vertex and create split edges
            for (PVector reflexVertex : reflexVertices) {
                Wall newEdge = createSplitEdge(reflexVertex, currentPolygon, map);
                if (newEdge != null && !map.collides(newEdge.start, newEdge.end)) {
                    // Split the polygon using the new edge
                    ArrayList<Wall> newPolygon = splitPolygon(currentPolygon, reflexVertex, newEdge);
                    convexPolygons.add(newPolygon);
                    currentPolygon.remove(newEdge); // Remove the edge from currentPolygon

                    // Re-check for reflex vertices in the updated currentPolygon
                    if (!hasReflexVertices(currentPolygon)) {
                        convexPolygons.add(currentPolygon);
                        break; // No more reflex vertices, add the remaining polygon
                    }
                }
            }
        }
    }
    return convexPolygons;
}
   
   boolean hasReflexVertices(ArrayList<Wall> polygon) {
        for (int i = 0; i < polygon.size(); i++) {
            PVector p0 = polygon.get(i).start;
            PVector p1 = polygon.get(i).end;
            PVector p2 = polygon.get((i + 1) % polygon.size()).end;

            float angle = PVector.angleBetween(PVector.sub(p1, p0), PVector.sub(p2, p1));
            if (angle > Math.PI) {
                return true; // Reflex vertex found
            }
        }
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
        float crossProduct = (nextEnd.x - currentEnd.x) * (prevEnd.y - currentEnd.y) -
                             (nextEnd.y - currentEnd.y) * (prevEnd.x - currentEnd.x);
        return crossProduct > 0; // Returns true if reflex
    }

    Node createConnectionNode(PVector start, PVector end, Node node) {
        Node connectionNode = new Node();
        connectionNode.id = nodes.size(); // Unique ID
        connectionNode.polygon = new ArrayList<>(); // Not using walls for this node
        connectionNode.center = new PVector((start.x + end.x) / 2, (start.y + end.y) / 2);
        connectionNode.neighbors = new ArrayList<>();
        return connectionNode;
    }

    Wall createSplitEdge(PVector reflexVertex, ArrayList<Wall> polygon, Map map) {
        ArrayList<PVector> reflexVertices = findReflexVertices(polygon);
        for (PVector targetVertex : reflexVertices) {
          if (!reflexVertex.equals(targetVertex)
              && isPointInPolygon(reflexVertex, map.outline)
              && isPointInPolygon(targetVertex, map.outline)
              && !map.collides(reflexVertex, targetVertex)) {
            return new Wall(reflexVertex, targetVertex);
          }
        }
        return null;
    }

    ArrayList<Wall> splitPolygon(ArrayList<Wall> polygon, PVector reflexVertex, Wall newEdge) {
        ArrayList<Wall> newPolygon = new ArrayList<>();
        
        boolean splitting = false;
        for (Wall wall : polygon) {
          newPolygon.add(wall);
          if (wall.equals(newEdge)) {
            splitting = !splitting;
          }
        }
        
        if (splitting) {
          newPolygon.add(newEdge);
        }
        
        return newPolygon; // Return the newly formed polygon
    }
   
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
   
   void draw() {
     ArrayList<Wall> boundaryWalls = new ArrayList<>(map.outline);
      ArrayList<String> drawnLines = new ArrayList<>();
      ArrayList<PVector[]> existingLines = new ArrayList<>(); 
      
      HashMap<PVector, Integer> vertexConnectionCount = new HashMap<>();
      int maxConnections = 10;
      
      /// use this to draw the nav mesh graph
      // draw each node
      for (Node node : nodes) {
        // draw the polygon for the node(walls)
        for (Wall wall : node.polygon) {
          wall.draw();
          
        }
        
        //optional, draw center of the node
        if (!node.polygon.isEmpty()) {
          fill(0, 255, 0);
          circle(node.center.x, node.center.y, 5);
        }
        
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
        for (int i = 0; i < uniqueVertices.size(); i++) {
          PVector vertex1 = uniqueVertices.get(i);
          
          vertexConnectionCount.putIfAbsent(vertex1, 0);
          if (vertexConnectionCount.get(vertex1) >= maxConnections){
            continue;
          }
          
          for (int j = i + 1; j < uniqueVertices.size(); j++) {
            PVector vertex2 = uniqueVertices.get(j);
            
            vertexConnectionCount.putIfAbsent(vertex2, 0);
            
            if (vertexConnectionCount.get(vertex1) < maxConnections && vertexConnectionCount.get(vertex2) < maxConnections) {
              boolean allPointsInside = true;
              int numSamples = 50;
              float tolerance = 0.1;
              if (!isLineOnBoundary(vertex1, vertex2, boundaryWalls, tolerance)) {
                for (int k = 1; k < numSamples; k++) {
                  float t = k / (float)numSamples;
                  PVector samplePoint = PVector.lerp(vertex1, vertex2, t);
              
                  if (!isPointInPolygon(samplePoint, node.polygon)) {
                    allPointsInside = false;
                    break;
                  }
                }
                //draw line if all sampled points are inside
                if (allPointsInside  && !intersectsAnyLine(vertex1, vertex2, existingLines)) {
                  line(vertex1.x, vertex1.y, vertex2.x, vertex2.y);
                  drawnLines.add(createLineKey(vertex1, vertex2));
                  existingLines.add(new PVector[]{vertex1, vertex2});
                  
                  vertexConnectionCount.put(vertex1, vertexConnectionCount.get(vertex1) + 1);
                  vertexConnectionCount.put(vertex2, vertexConnectionCount.get(vertex2) + 1);
                }
              }
            }
          }
        }

        //draw the connection between nodes (neighboring nodes)
        stroke(0, 255, 255);
        for (Node neighbor : node.neighbors) {
          String neighborLineKey = createLineKey(node.center, neighbor.center);
          if (!drawnLines.contains(neighborLineKey) && !intersectsAnyLine(node.center, neighbor.center, existingLines)) {
            line(node.center.x, node.center.y, neighbor.center.x, neighbor.center.y);
            drawnLines.add(neighborLineKey);
            existingLines.add(new PVector[]{node.center, neighbor.center});
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
   
    // Check if the two lines intersect
    boolean intersects(PVector v1Start, PVector v1End, PVector v2Start, PVector v2End) {
      if (straddles(v1Start, v1End, v2Start, v2End) && straddles(v2Start, v2End, v1Start, v1End)) {
        return true;
      }
      // Check for collinear overlapping segments
      if (collinear(v1Start, v1End, v2Start, v2End)) {
        // If the lines are collinear, check if any of the points are on the other's segment
        if (pointOnSegment(v1Start, v2Start, v2End) || 
            pointOnSegment(v1End, v2Start, v2End) || 
            pointOnSegment(v2Start, v1Start, v1End) || 
            pointOnSegment(v2End, v1Start, v1End)) {
            return true;
        }
      }
      return false;
    }

    // Check if the two lines straddle each other
    boolean straddles(PVector p1, PVector p2, PVector q1, PVector q2) {
        float cross1 = crossProduct(p1, p2, q1);
        float cross2 = crossProduct(p1, p2, q2);
        return cross1 * cross2 < -0.00001;
    }

    // Cross product function
    float crossProduct(PVector a, PVector b, PVector c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    // Check if two lines are collinear
    boolean collinear(PVector p1, PVector p2, PVector q1, PVector q2) {
        return (crossProduct(p1, p2, q1) == 0) && (crossProduct(p1, p2, q2) == 0);
    }

    // Check if point is on the line segment
    boolean pointOnSegment(PVector p, PVector v, PVector w) {
        return (p.x <= Math.max(v.x, w.x) && p.x >= Math.min(v.x, w.x) &&
                p.y <= Math.max(v.y, w.y) && p.y >= Math.min(v.y, w.y));
    }
    
  // Helper method to check if a line intersects with any existing lines
  boolean intersectsAnyLine(PVector start, PVector end, ArrayList<PVector[]> existingLines) {
    for (PVector[] line : existingLines) {
        if (intersects(start, end, line[0], line[1])) {
            return true;
        }
    }
    return false;
  }
  
  boolean isLineOnBoundary(PVector vertex1, PVector vertex2, ArrayList<Wall> boundaryWalls, float tolerance) {
    for (Wall boundaryWall : boundaryWalls) {  // Assuming boundaryWalls is a list of boundary walls
        if (arePointsEqual(vertex1, boundaryWall.start, tolerance) && arePointsEqual(vertex2, boundaryWall.end, tolerance)) {
            return true;
        }
        if (arePointsEqual(vertex1, boundaryWall.end, tolerance) && arePointsEqual(vertex2, boundaryWall.start, tolerance)) {
            return true;
        }
    }
    return false;  // The line is not on the boundary
  }
  
  boolean arePointsEqual(PVector p1, PVector p2, float tolerance) {
    return PVector.dist(p1, p2) <= tolerance;
  }

}
