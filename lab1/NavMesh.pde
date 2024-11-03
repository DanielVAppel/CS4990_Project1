// Useful to sort lists by a custom key
import java.util.Comparator;
import java.util.stream.Collectors;
import java.util.Collections;
import java.util.PriorityQueue;
import java.util.Arrays;
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
   ArrayList<Vertex> vertices;
   ArrayList<Wall> walls;
   
   void bake(Map map) {
    nodes = new ArrayList<>();
    ArrayList<Wall> walls = map.outline;

    // Step 1: Generate the navigation mesh walls.
    walls = generateNavMesh(walls);

    // Step 2: Generate vertices from the walls.
    ArrayList<Vertex> vertices = generateVerticesFromWalls(walls);

    // Step 3: Generate polygons using vertices.
    ArrayList<HashSet<PVector>> lineSegments = new ArrayList<>();
    for (Wall wall : walls) {
      if (map.outline.contains(wall)) continue;
        HashSet<PVector> segment = new HashSet<>(Arrays.asList(wall.start, wall.end));
        lineSegments.add(segment);
    }
    ArrayList<ArrayList<Wall>> polygons = generatePolygons(vertices, lineSegments);

    // Step 4: Process nodes and connections from polygons.
    HashMap<PVector, Node> midpointNodeMap = new HashMap<>();
    for (HashSet<PVector> lineSegment : lineSegments) {
      PVector[] points = lineSegment.toArray(new PVector[0]);
      PVector midpoint = getMidpoint(points[0], points[1]);
      if (!isPointInPolygon(midpoint, map.outline) && isOnSameLine(points[0], points[1], map.outline)) {
        continue;
      }
      Node midpointNode = midpointNodeMap.get(midpoint);
      if (midpointNode == null) {
        midpointNode = new Node();
        midpointNode.id = nodes.size();
        midpointNode.center = midpoint;
        midpointNode.neighbors = new ArrayList<>();
        midpointNode.connections = new ArrayList<>();
        nodes.add(midpointNode);
        midpointNodeMap.put(midpoint, midpointNode);
      }
      
      //connect neighboring nodes
      for (HashSet<PVector> otherSegment : lineSegments) {
        if (!otherSegment.equals(lineSegment)) {
          PVector[] otherPoints = otherSegment.toArray(new PVector[0]);
          PVector otherMidpoint = getMidpoint(otherPoints[0], otherPoints[1]);
          if (!isPointInPolygon(otherMidpoint, walls) || isOnSameLine(points[0], points[1], map.outline)) {
                    continue;
          }
          Node otherMidpointNode = midpointNodeMap.get(otherMidpoint);
          if (otherMidpointNode != null && !midpointNode.neighbors.contains(otherMidpointNode)) {
            Wall pathSegment = new Wall(midpointNode.center, otherMidpointNode.center);
            if (!doesEdgeIntersectPolygon(pathSegment, map.outline)) {
              midpointNode.neighbors.add(otherMidpointNode);
              otherMidpointNode.neighbors.add(midpointNode);
              midpointNode.connections.add(pathSegment);
              otherMidpointNode.connections.add(pathSegment);
            }
          }
        }
      }
    }

    // Debug Information.
    System.out.println("Total nodes: " + nodes.size());
    int totalConnections = 0;
    for (Node node : nodes) {
        totalConnections += node.connections.size();
    }
    System.out.println("Total nodes connections: " + totalConnections);
  }




   
   //helper methods
   boolean collides(PVector from, PVector to, ArrayList<Wall> navmesh)
   {
      for (Wall w : navmesh) {
         if (w.crosses(from, to)) return true;
      }
      return false;
   }
   
   // Get vertices based on walls
  ArrayList<Vertex> generateVerticesFromWalls(ArrayList<Wall> edges) {
    vertices = new ArrayList<Vertex>();
    // In the first iteration of edges we just create vertices
    for (int i = 0; i < edges.size(); i++) {
      Wall thisWall = edges.get(i);
      vertices.add(new Vertex(thisWall.start));
    }
    // In the second iteration of edges we associate vertices with their neighbors
    for (int i = 0; i < vertices.size(); i++) {
      Vertex prevVertex = vertices.get((i - 1 < 0 ? vertices.size() - 1 : i - 1) %
      vertices.size());
      Vertex thisVertex = vertices.get(i);
      Vertex nextVertex = vertices.get((i + 1) % vertices.size());
      thisVertex.setPrev(prevVertex);
      thisVertex.setNext(nextVertex);
    }
    return vertices;
  }
  
  // Generate the walls we use for navmesh
  ArrayList<Wall> generateNavMesh(ArrayList<Wall> edges) {
    ArrayList<Wall> navmesh = new ArrayList<>();
    for (Wall w : edges) {
      navmesh.add(w);
    }
    ArrayList<Vertex> vertices = generateVerticesFromWalls(edges);
    //Create new edges to break reflex angles
    List<Vertex> reflexPoints = vertices.stream().filter(v -> v.isReflexAngle()).collect(Collectors.toList());
    // While we still have reflex points
    for (Vertex reflexPoint : reflexPoints) {
      while (reflexPoint.isReflexAngle()) {
        // Identify all vertices in line of sight to the reflex angle
        ArrayList<Vertex> potentialPoints = new ArrayList<Vertex>();
        for (Vertex potentialPoint : vertices) {
          if (!potentialPoint.equals(reflexPoint) && (!reflexPoint.isConnectedTo(potentialPoint) || !potentialPoint.isConnectedTo(reflexPoint)) && 
            (!collides(reflexPoint.vector, potentialPoint.vector, navmesh) &&
              map.isReachable(new Wall(reflexPoint.vector, potentialPoint.vector).center())) && 
              map.isReachable(new
              Wall(PVector.sub(reflexPoint.vector, new PVector(.001, .001)), 
              potentialPoint.vector).center())) {
            potentialPoints.add(potentialPoint);
          }
        }
        if (potentialPoints.size() == 0) {
          break;
        }
        // Find the shortest of the new edges we can create
        List<Wall> potentialEdges = potentialPoints.stream().map(pp -> new
        Wall(reflexPoint.vector, pp.vector)).collect(Collectors.toList());
        potentialEdges.sort((w1, w2) ->
        Float.valueOf(w1.len).compareTo(Float.valueOf(w2.len)));
        // Sort potentialPoints by how much each point breaks up the reflex angle the more evenly it breaks it the better
        potentialPoints.sort((p1, p2) -> {
                double angleDiff1 = Math.abs(PVector.angleBetween(reflexPoint.vector, p1.vector));
                double angleDiff2 = Math.abs(PVector.angleBetween(reflexPoint.vector, p2.vector));
                return Double.compare(angleDiff1, angleDiff2);
            });
        Wall bestWall = new Wall(reflexPoint.vector, potentialPoints.get(0).vector); // Best wall by breaking angle
        bestWall = potentialEdges.get(0); // best wall by shortest distance
        Vertex bestVertex = vertices.get(vertices.lastIndexOf(new
        Vertex(bestWall.end)));
        Vertex rVertex = vertices.get(vertices.lastIndexOf(new
        Vertex(reflexPoint.vector)));
        rVertex.associate(bestVertex);
        bestVertex.associate(rVertex);
        navmesh.add(bestWall);
        stroke(255, 0, 0);
        line(bestWall.start.x, bestWall.start.y, bestWall.end.x, bestWall.end.y);
        }
      }
      return navmesh;
     }
 
   ArrayList<ArrayList<Wall>> generatePolygons(ArrayList<Vertex> vertices, ArrayList<HashSet<PVector>> lineSegments) {
      // The result to return
      ArrayList<ArrayList<Wall>> result = new ArrayList<ArrayList<Wall>>();
      // Used to track what sets of vertices we've found.
      ArrayList<HashSet<Vertex>> polygonSets = new ArrayList<HashSet<Vertex>>();
      // Maps a vertex to how many times we want to visit it. If we hit zero we remove it from the vertexMap (which makes it's key return null)
      HashMap<Vertex, Integer> vertexMap = new HashMap<Vertex, Integer>();
      for (Vertex v : vertices) {
        vertexMap.put(v, Integer.valueOf(v.neighbors.size() - 1));
      }
      // While there are still vertices we want to visit
      while (vertexMap.size() > 0) {
        // Create a polygon
        ArrayList<Wall> polygon = new ArrayList<Wall>(); // The walls that make up the polygon. What we actually want to return
        ArrayList<Vertex> polygonVertices = new ArrayList<Vertex>(); // The vertices of the polygon. Used to do the operations
        // The vertex we will start our polygon from
        Vertex start = vertexMap.keySet().iterator().next();
        // Add start to polygon
        polygonVertices.add(start);
        // This one is a bit of a doozy, find a Vertex neighbor of start that we can still visit, and that has an edge to it that we can still visit over
        // Find a Vertex neighbor of start that we can still visit
        List<Vertex> validNeighbors = start.neighbors.stream()
          .filter(neighbor -> vertexMap.get(neighbor) != null && lineSegments.contains(new HashSet<>(Arrays.asList(start.vector, neighbor.vector))))
          .collect(Collectors.toList());

        if (validNeighbors.isEmpty()) {
          break; // or handle the error appropriately
        }
        Vertex current = validNeighbors.get(0);
        // We want to track the last vertex we visited so we know how we reached our current Vertex
        Vertex last = start;
        // Actually add to the polygon while we haven't looped back to the start
        while (!current.equals(start)) {
          polygonVertices.add(current); // Add current to polygon
          polygon.add(new Wall(last.vector, current.vector));
          // Essentially a tuple, index 0 is the right neighbor from this perspective, left from vertex perspective
          ArrayList<Vertex> options = current.getClosestNeighbors(last, vertexMap.keySet());
          last = current; // Update last to be current
          // Just get the closest neighbor to the left of where we came from.
          if (options.get(1) != null) {
            current = options.get(1);
          } else {
            // This shouldn't be reached, but just in case I want to print something
            println("ERROR: Reached a dead end");
            continue;
          }
        }
        // Add the wall from the last vertex we visited to the first
        polygon.add(new Wall(last.vector, start.vector));
        if (polygonSets.stream().filter(vSet -> vSet.equals(new
            HashSet<Vertex>(polygonVertices))).collect(Collectors.toList()).size() != 0) {
            println("ERROR: We've seen this before");
            continue;
        }
        // Decrement vertexMap and also draw a circle in the center of each polygon
        PVector center = new PVector();
        for (Vertex v : polygonVertices) {
          vertexMap.put(v, vertexMap.get(v) - 1); // Decrement current visits
          if (vertexMap.get(v) <= 0) {
            vertexMap.remove(v);
          }
          center = PVector.add(center, v.vector);
        }
        center.div(polygonVertices.size());
        circle(center.x, center.y, 10);
        // Remove the line segments
        for (int i = 0; i < polygonVertices.size(); i++) {polygonVertices.add(start);
          // This one is a bit of a doozy, find a Vertex neighbor of start that we can still visit, and that has an edge to it that we can still visit over
          current = start.neighbors.stream()
          .filter(neighbor ->
            vertexMap.get(neighbor) != null && lineSegments.contains(new
            HashSet<PVector>(Arrays.asList(start.vector, neighbor.vector)))
            ).collect(Collectors.toList()).get(0);
          // We want to track the last vertex we visited so we know how we reached our current Vertex
          last = start;
          // Actually add to the polygon while we haven't looped back to the start
          while (!current.equals(start)) {
            polygonVertices.add(current); // Add current to polygon
            polygon.add(new Wall(last.vector, current.vector));
            // Essentially a tuple, index 0 is the right neighbor from this perspective, left from vertex perspective
            ArrayList<Vertex> options = current.getClosestNeighbors(last,
            vertexMap.keySet());
            last = current; // Update last to be current
            // Just get the closest neighbor to the left of where we came from.
            if (options.get(1) != null) {
              current = options.get(1);
            } else {
              // This shouldn't be reached, but just in case I want to print something
              println("ERROR: Reached a dead end");
              continue;
            }
        }
        // Add the wall from the last vertex we visited to the first
        polygon.add(new Wall(last.vector, start.vector));
        if (polygonSets.stream().filter(vSet -> vSet.equals(new
            HashSet<Vertex>(polygonVertices))).collect(Collectors.toList()).size() != 0) {
              println("ERROR: We've seen this before");
              continue;
        }
        // Decrement vertexMap and also draw a circle in the center of each polygon
        center = new PVector();
        for (Vertex v : polygonVertices) {
          vertexMap.put(v, vertexMap.get(v) - 1); // Decrement current visits
          if (vertexMap.get(v) <= 0) {
            vertexMap.remove(v);
          }
          center = PVector.add(center, v.vector);
        }
        center.div(polygonVertices.size());
        circle(center.x, center.y, 10);
        // Remove the line segments
        for (int j = 0; j < polygonVertices.size(); j++) {
          PVector thisVector = polygonVertices.get(j).vector;
          PVector nextVector = polygonVertices.get((j + 1) % polygonVertices.size()).vector;
          HashSet<PVector> temp = new HashSet<>();
          temp.add(thisVector);
          temp.add(nextVector);
          lineSegments.remove(temp);
        }
        result.add(polygon);
        polygonSets.add(new HashSet<Vertex>(polygonVertices));
      }
    }
    return result;
  }
    
    PVector getMidpoint(PVector p1, PVector p2) {
      return new PVector((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
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
   
   //method for A* search
   ArrayList<PVector> findPath(PVector start, PVector destination)
   {
     // Reset node parents to avoid stuck loop
    for (Node node : nodes) {
        node.parent = null;
      }
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
            Wall pathSegment = new Wall(current.center, neighbor.center);
            if (closedSet.contains(neighbor) || doesEdgeIntersectPolygon(pathSegment, map.outline)) continue;

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
    System.out.println("No path found.");
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
     ArrayList<Wall> walls = map.outline;

    // Step 1: Generate the navigation mesh walls.
    walls = generateNavMesh(walls);

    // Step 2: Generate vertices from the walls.
    ArrayList<Vertex> vertices = generateVerticesFromWalls(walls);

    // Step 3: Generate polygons using vertices.
    ArrayList<HashSet<PVector>> lineSegments = new ArrayList<>();
    for (Wall wall : walls) {
        HashSet<PVector> segment = new HashSet<>(Arrays.asList(wall.start, wall.end));
        lineSegments.add(segment);
    }
    ArrayList<ArrayList<Wall>> polygons = generatePolygons(vertices, lineSegments);
    
    for (Node node : nodes) {
       fill(0, 255, 0);
       ellipse(node.center.x, node.center.y, 5, 5);
     }
   }
   
   void drawNavMesh() {
     stroke(0, 255, 0);
     
     //draw each polygon
     for (Node node : nodes) {
       ArrayList<Wall> walls = node.polygon;
       
       for (Wall wall : walls) {
         line(wall.start.x, wall.start.y, wall.end.x, wall.end.y);
       }
       
       fill(255, 0, 0);
       ellipse(node.center.x, node.center.y, 5, 5);
     }
   }
}
