package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

/**
 * @author SP
 * 
 * A class which represents a node in a graph
 * A node here depicts a geographic location in a map.
 *
 */
class MapNode{
	GeographicPoint location;
	List<MapEdge> edges;
	Double distance;
	
	/** 
	 * Create a new empty MapNode
	 */
	public MapNode(){
		
	}
	
	/** 
	 * Create a MapNode with parameters
	 */ 
	public MapNode(GeographicPoint location){
		this.location = location;
		edges = new ArrayList<MapEdge>();
		
	}
	
	/** Get the location of node
	 * @return The geographic location corresponding to a node in the map
	 */
	public GeographicPoint getLocation(){
		return (GeographicPoint) location.clone();
	}
	
	/** Get the list of edges
	 * @return The list of all edges connected to the map node
	 */
	public List<MapEdge> getEdges(){
		List<MapEdge> edgeList = new ArrayList<>(this.edges);
		return edgeList;
	}
	/** Set the distance of a map node from the start node
	 * @return void
	 */
	public void setDistance(Double distance){
		this.distance = distance;
	}
	/** Set the distance of a map node from the start node to a default value of infinity
	 * @return void
	 */
	public void setDistance(){
		this.distance = Double.MAX_VALUE;
	}
	/** Get the distance of a map node from the start node
	 * @return distance
	 */
	public double getDistance(){
		return this.distance;
	}

//	public int compareTo(MapNode node2) {
//		// TODO Auto-generated method stub
//		double dist1 = this.getDistance();
//		double dist2 = node2.getDistance();
//		if (dist1==dist2){
//			return 0;
//		}
//		else if (dist1<dist2){
//			return 1;
//		}
//		else
//			return -1;
//		//return this.getDistance()<=node2.getDistance()? -1: 1;
//	}
}
