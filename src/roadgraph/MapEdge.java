package roadgraph;

import geography.GeographicPoint;

/**
 * @author SP
 * 
 * A class which represents a directed edge between two geographic locations
 */
public class MapEdge{
	GeographicPoint start;
	GeographicPoint end;
	String roadName;
	String roadType;
	double length;
	
	/** 
	 * Create a new empty MapEdge 
	 */
	public MapEdge(){
		
	}
	
	/** 
	 * Create a MapEdge with parameters
	 */
	public MapEdge(GeographicPoint start, GeographicPoint end,
			String roadName, String roadType, double length){
		this.start = start ; 
		this.end = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	
	/** Get the end geographic location
	 * @return The geographic location of the end point of the edge
	 */
	public GeographicPoint getEndLocation(){
		// returns a copy of the end geographic location
		return (GeographicPoint) this.end.clone();
	}
	


	/** Get the start geographic location
	 * @return The geographic location of the start point of the edge
	 */
	public GeographicPoint getStartLocation(){
		// returns a copy of the start geographic location
		return (GeographicPoint) this.start.clone();
	}
	

	/** Get the road name
	 * @return The road name corresponding to the edge
	 */
	public String getRoadName(){
		return new String(this.roadName);
	}
	
	/** Get the road type
	 * @return The road type corresponding to the edge
	 */
	public String getRoadType(){
		return new String(this.roadType);
	}
	
	/** Get the length of the edge
	 * @return The length of the edge in Kms
	 */
	public double getLength(){
		return this.length;
	}
	
	
}