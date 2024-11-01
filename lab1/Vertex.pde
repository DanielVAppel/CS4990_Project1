import java.util.ArrayList;
import java.util.List;
import processing.core.PVector;

public class Vertex {
    PVector vector;
    Vertex prev;
    Vertex next;
    List<Vertex> neighbors;

    public Vertex(PVector vector) {
        this.vector = vector;
        this.neighbors = new ArrayList<>();
    }

    public void setPrev(Vertex prev) {
        this.prev = prev;
    }

    public void setNext(Vertex next) {
        this.next = next;
    }

    public boolean isReflexAngle() {
        // Calculate if the angle at this vertex is greater than 180 degrees
        PVector v1 = PVector.sub(prev.vector, vector);
        PVector v2 = PVector.sub(next.vector, vector);
        return v1.cross(v2).z < 0; // Cross product to check angle orientation
    }

    public void addNeighbor(Vertex neighbor) {
        if (!neighbors.contains(neighbor)) {
            neighbors.add(neighbor);
        }
    }

    public void associate(Vertex other) {
        addNeighbor(other);
    }
    
    public boolean isConnectedTo(Vertex other) {
      return neighbors.contains(other);
    }
    
    public List<Float> testNeighbor(Vertex other) {
      List<Float> distances = new ArrayList<>();
      for (Vertex neighbor : neighbors) {
        float distance = PVector.dist(this.vector, neighbor.vector);
        distances.add(distance);
      }
      return distances;
    }
    
    //get closest neighbors
    public ArrayList<Vertex> getClosestNeighbors(Vertex target, Iterable<Vertex> vertices) {
      ArrayList<Vertex> closestNeighbors = new ArrayList<>();
      float minDistance = Float.MAX_VALUE;
      
      for (Vertex neighbor : vertices) {
        if (!neighbor.equals(this)) {
          float distance = PVector.dist(target.vector, neighbor.vector);
          
          //check if current neighbor is closer than the known minimum distance
          if (distance < minDistance) {
            closestNeighbors.clear(); //clear previous if a new minimum is found
            closestNeighbors.add(neighbor);
            minDistance = distance; //update minimum distance
          }
          else if (distance == minDistance) {
            closestNeighbors.add(neighbor);
          }
        }
      }
      return closestNeighbors;
    }
    
    // Optional: Implement hashCode() and equals() for easier comparison
    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Vertex)) return false;
        Vertex other = (Vertex) obj;
        return this.vector.equals(other.vector);
    }

    @Override
    public int hashCode() {
        return vector.hashCode();
    }
}
