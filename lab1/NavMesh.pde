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
      ArrayList<String> drawnLines = new ArrayList<>();
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
          for (int j = i + 1; j < uniqueVertices.size(); j++) {
            PVector vertex2 = uniqueVertices.get(j);
            
            boolean allPointsInside = true;
            int numSamples = 15;
            for (int k = 1; k < numSamples; k++) {
              float t = k / (float)numSamples;
              PVector samplePoint = PVector.lerp(vertex1, vertex2, t);
              
              if (!isPointInPolygon(samplePoint, node.polygon)) {
                allPointsInside = false;
                break;
              }
            }
            if (allPointsInside) {
              line(vertex1.x, vertex1.y, vertex2.x, vertex2.y);
              drawnLines.add(createLineKey(vertex1, vertex2));
            }
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
   
   // Function to check if two line segments intersect
   boolean intersects(PVector p1, PVector p2, PVector p3, PVector p4) {
    // Calculate the four orientations needed for the general and special cases
    int o1 = orientation(p1, p2, p3);
    int o2 = orientation(p1, p2, p4);
    int o3 = orientation(p3, p4, p1);
    int o4 = orientation(p3, p4, p2);

    // General case
    if (o1 != o2 && o3 != o4) {
        return true;
    }

    // Special cases
    // p1, p2 and p3 are collinear and p3 lies on segment p1p2
    if (o1 == 0 && onSegment(p1, p3, p2)) return true;

    // p1, p2 and p4 are collinear and p4 lies on segment p1p2
    if (o2 == 0 && onSegment(p1, p4, p2)) return true;

    // p3, p4 and p1 are collinear and p1 lies on segment p3p4
    if (o3 == 0 && onSegment(p3, p1, p4)) return true;

    // p3, p4 and p2 are collinear and p2 lies on segment p3p4
    if (o4 == 0 && onSegment(p3, p2, p4)) return true;

    // Doesn't fall in any of the above cases
    return false;
  }

  // Function to find the orientation of the ordered triplet (p, q, r)
  // Returns:
  // 0 -> p, q and r are collinear
  // 1 -> Clockwise
  // 2 -> Counterclockwise
  int orientation(PVector p, PVector q, PVector r) {
    float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;  // collinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
  }

  // Function to check if point q lies on segment pr
  boolean onSegment(PVector p, PVector q, PVector r) {
    return (q.x <= Math.max(p.x, r.x) && q.x >= Math.min(p.x, r.x) &&
            q.y <= Math.max(p.y, r.y) && q.y >= Math.min(p.y, r.y));
  }

}
