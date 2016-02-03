
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;


/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	List<MapNode> nodes;

	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		nodes = new ArrayList<MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return this.nodes.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> pointSet = new HashSet<GeographicPoint>();
		for (MapNode node : nodes){
			GeographicPoint point = node.getLocation();
			pointSet.add(point);
		}
		return pointSet;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		int numOfEdges = 0;
		for (MapNode node : nodes){
			int currEdges = node.edges.size();
			if (currEdges>0){
				numOfEdges+=currEdges;
			}
		}
		return numOfEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		boolean result = false;
		
		if (location!=null){
			MapNode newNode = new MapNode(location);
			if (!nodes.contains(newNode)){
				nodes.add(newNode);
				result = true;
			}
		}
		return result;
	}
	
	/** Find the MapNode in the graph given a geographic location
	 * 
	 * @param The geographic location 
	 * @return The node corresponding to the location if it exists in the graph,
	 * otherwise null 
	 */
	private MapNode getMapNode(GeographicPoint location){
		
		for (MapNode node : nodes){
			if (node.getLocation().equals(location)){
				return node;
			}
		}
		return null;
		
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		
		//TODO: Implement this method in WEEK 2
		if (from.equals(null) || to.equals(null) || 
				roadName.equals(null) || roadType.equals(null) || length <0 ){
			throw new IllegalArgumentException();
		}
		else {
			MapNode node1 = getMapNode(from);
			MapNode node2 = getMapNode(to);
			if (node1!=null || node2!=null){
				MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
				node1.edges.add(edge);
			}
			else{
				throw new  IllegalArgumentException();
			}
		}
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */

	public List<GeographicPoint> bfs(GeographicPoint start, 
		     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched){
		
		if (start == null || goal == null) {
			//System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}

		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		
		boolean found = bfsSearch(start,goal,parentMap);

		if (!found) {
			//System.out.println("No path exists");
			return null;
		}
		// reconstruct the path
		return constructPath(start, goal, parentMap);
		
	}
	

	/** Find the path from start to goal using breadth first search (re-factored)
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap A map where the value is the GeographicPoint explored and the key is the parent of the GeographicPoint
	 * @return true if a path is found, otherwise false.
	 */
	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal, 
			HashMap<GeographicPoint, GeographicPoint> parentMap){
		boolean found = false;
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Queue<GeographicPoint> toExplore = new LinkedList<GeographicPoint>();
		toExplore.add(start);
		while (!toExplore.isEmpty()) {
			GeographicPoint curr = toExplore.remove();
			if (curr.equals(goal)) {
				found = true;
				break;
			}
			List<MapEdge> edges = this.getMapNode(curr).getEdges();
			for (MapEdge edge : edges){
				GeographicPoint next = edge.getEndLocation();
				if (!visited.contains(next)){
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
			
		}
		return found;
		
	}
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap A map where the value is the GeographicPoint explored and the key is the parent of the GeographicPoint
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start, 
			GeographicPoint goal, HashMap<GeographicPoint, GeographicPoint> parentMap){
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint curr = goal;
		while (!curr.equals(start) && curr!=null) {
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location 
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		// Step 1: Initialization
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		HashMap<GeographicPoint,GeographicPoint> parentMap = new HashMap<>();
		//HashMap<GeographicPoint,Double> distances = new HashMap<>();
		
		// Initialize all distances to infinity
		//System.out.println("Step 1 reached");
		for (MapNode node : nodes){
			GeographicPoint currPoint = node.getLocation();
			if (currPoint.equals(start)){
				node.setDistance(0.0);
			}
			else{
				node.setDistance();
			}
		}
		
		// Step 2: Dijkstra's algorithm
		//System.out.println("Step 2 reached");
		boolean found  = dijkstraSearch(start,goal,parentMap, nodeSearched); 
		
		// Step 3: Construct Path
		if (!found) {
			//System.out.println("No path exists");
			return null;
		}
		// reconstruct the path
		return constructPath(start, goal, parentMap);
	}
	
	private boolean  dijkstraSearch(GeographicPoint start, GeographicPoint goal, 
			HashMap<GeographicPoint,GeographicPoint> parentMap,Consumer<GeographicPoint> nodeSearched){
		
		boolean found = false;
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		int capacity = this.nodes.size();
		DistComparator c = new DistComparator();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(capacity,c);
		toExplore.add(getMapNode(start));
		int count = 0;
		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
			count++;
			if (!visited.contains(curr.getLocation())){
				visited.add(curr.getLocation());
				if (curr.getLocation().equals(goal)) {
					found = true;
					break;
				}
				List<MapEdge> edges = curr.getEdges();
				for (MapEdge edge : edges){
					GeographicPoint next = edge.getEndLocation();
					if (!visited.contains(next)){
						//visited.add(next);
						MapNode n = getMapNode(next);
						double currDist = curr.getDistance()+edge.getLength();
						if (currDist<n.getDistance()){
							n.setDistance(currDist);
							parentMap.put(next,curr.location);
							toExplore.add(getMapNode(next));
						}
					}
				}
			}
		}
		//System.out.println("Parent map size : "+parentMap.size());
		System.out.println("Dijkstra count : "+count);
		return found;
		
	}
	
	

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		HashMap<GeographicPoint,GeographicPoint> parentMap = new HashMap<>();
		//HashMap<GeographicPoint,Double> distances = new HashMap<>();
		
		// Initialize all distances to infinity
		//System.out.println("Step 1 reached");
		for (MapNode node : nodes){
			GeographicPoint currPoint = node.getLocation();
			if (currPoint.equals(start)){
				node.setDistance(0.0);
			}
			else{
				node.setDistance();
			}
		}
		
		// Step 2: Dijkstra's algorithm
		//System.out.println("Step 2 reached");
		boolean found  = searchAStar(start,goal,parentMap, nodeSearched); 
		
		// Step 3: Construct Path
		if (!found) {
			//System.out.println("No path exists");
			return null;
		}
		// reconstruct the path
		return constructPath(start, goal, parentMap);
	}

	private boolean searchAStar(GeographicPoint start, GeographicPoint goal, 
	HashMap<GeographicPoint,GeographicPoint> parentMap,Consumer<GeographicPoint> nodeSearched){
		
		boolean found = false;
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		int capacity = this.nodes.size();
		DistComparator c = new DistComparator();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(capacity,c);
		toExplore.add(getMapNode(start));
		int count = 0;
		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
			count++;
			if (!visited.contains(curr.getLocation())){
				visited.add(curr.getLocation());
				if (curr.getLocation().equals(goal)) {
					found = true;
					break;
				}
				List<MapEdge> edges = curr.getEdges();
				for (MapEdge edge : edges){
					GeographicPoint next = edge.getEndLocation();
					if (!visited.contains(next)){
						//visited.add(next);
						MapNode n = getMapNode(next);
						double currDist = curr.getDistance()+edge.getLength();
						double hDist = next.distance(goal);
						double distFunction = currDist+hDist;
						if (distFunction<n.getDistance()){
							n.setDistance(distFunction);
							parentMap.put(next,curr.location);
							toExplore.add(getMapNode(next));
						}
					}
				}
			}
		}
		//System.out.println("Parent map size : "+parentMap.size());
		System.out.println("A star Count : "+count);
		return found;
		
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		 //Use this code in Week 3 End of Week Quiz
		//MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}

class DistComparator implements Comparator<MapNode>{

    

	@Override
	public int compare(MapNode arg0, MapNode arg1) {
		// TODO Auto-generated method stub
		return Double.compare(arg0.getDistance(), arg1.getDistance());
	}
}
