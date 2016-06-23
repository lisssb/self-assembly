package trs;


import java.awt.geom.Point2D;


import sim.engine.SimState;
import sim.engine.Steppable;
import sim.field.continuous.Continuous2D;
import sim.util.Bag;
import sim.util.Double2D;
import sim.util.MutableDouble2D;

public class Robot  implements Steppable{
	
	
	
	public MutableDouble2D position = new MutableDouble2D(0.0, 0.0);
	double prev = Double.MAX_VALUE;
	double current = Double.MAX_VALUE;
	public MutableDouble2D localization;
	boolean isLocalized = false;
	State state = State.START;
	boolean isSeed;
	boolean validGradient = false;
	int gradientValue;
	
	double orientation;
	public int id;
	Bag neighborhood;
	double DESIRED_DISTANCE = TRS.robot_width + 0.2; //20mmm
	Continuous2D yard;
	boolean isStationary = true;
	double angle = 2;
	TRS swarm;
	Bag smallNeighborhood;
	
	public Robot(int id, boolean isSeed) {
		// TODO Auto-generated constructor stub
		this.id = id;
		this.isSeed = isSeed;
		if(isSeed){
			this.gradientValue = 0;				
		}
	}

	public int getGradientValue () {return gradientValue;}

	public void step(SimState state){
		swarm = (TRS) state;
		yard = swarm.yard;		
		neighborhood = yard.getNeighborsWithinDistance(yard.getObjectLocation(this), 10); // in each step we get the neighboprhood	
		smallNeighborhood = yard.getNeighborsExactlyWithinDistance(yard.getObjectLocation(this), (swarm.robot_width + 0.3));
		run();
//		followEdge();

	}
	
	
	private void rotate(boolean counterclockwise)
	{
		if(id == 5){
			if (!counterclockwise)
			{
				yard.setObjectLocation(this, 
						new Double2D(yard.getObjectLocation(this).getX() + Math.cos(angle )*0.1, yard.getObjectLocation(this).getY() + Math.sin(angle)*0.1));
			}
			else{
				yard.setObjectLocation(this, 
						new Double2D(yard.getObjectLocation(this).getX() + Math.cos(angle )*0.1, yard.getObjectLocation(this).getY() + Math.sin(angle)*0.1));

			}
		}
		
	}
//
	private void followEdge(){
		double measured_distance;
		for (int i = 0; i < neighborhood.size(); i++){
			if((neighborhood.get(i)!= this) && ((Robot)neighborhood.get(i)).isStationary){
				measured_distance = TRS.getDistance(yard.getObjectLocation(this), yard.getObjectLocation(neighborhood));
				if(measured_distance < current){
					current = measured_distance;
				}
			}
		}		
		if(current < DESIRED_DISTANCE){
			if(prev < current){
				move_straight_forward();
			}
			else{
				move_straight_forward();
				rotate(false);
			}
		}
		else{
			if(prev > current){
				move_straight_forward();
			}
			else{
				move_straight_forward();
				rotate(true);
			}
		}
		
	}
//	
	private void move_straight_forward(){
		
	}
	
	private void run (){
		if(state == State.START){
			if(isSeed){
				state = State.JOINED_SHAPE;
			}
			else {
				gradientFormation();
				localization();
				state = State.WAIT_TO_MOVE;
			}
		}
		else if(state == State.WAIT_TO_MOVE){
			boolean neighbours_moving = false;
			int max_neighbour_gradient = Integer.MIN_VALUE;
			int max_neighbour_id = 0;
			int h;
			Robot current_neighbour;
			
			for(int i = 0; i < neighborhood.size(); i++){
				current_neighbour = (Robot)neighborhood.get(i);
				if(max_neighbour_gradient <= current_neighbour.gradientValue){
					max_neighbour_gradient = current_neighbour.gradientValue;
					if(max_neighbour_id  < current_neighbour.id){
						max_neighbour_id = current_neighbour.id;
					}
				}
				if (((Robot)neighborhood.get(i)).isStationary){
					neighbours_moving = true;
				}
			}
			
			if(!neighbours_moving){
				h = max_neighbour_gradient;
				
				if(gradientValue > h || (gradientValue == h && id > max_neighbour_id)){
					state = State.MOVE_WHILE_OUTSIDE;
					isStationary = false;
				}				
			}
		}
		
		else if(state == State.MOVE_WHILE_OUTSIDE){
			if(swarm.isInsideShape(yard.getObjectLocation(this))){
				state = State.MOVE_WHILE_INSIDE;
				isStationary = false;
			}
			if(validMovement(yard.getObjectLocation(this))){//
				//followEdge()
				//!111111111111111111
			}
		}
		else if(state == State.MOVE_WHILE_INSIDE){/// edge-follow while inside desired shape
			if(!swarm.isInsideShape(yard.getObjectLocation(this))){
				state = State.JOINED_SHAPE;
			}
			else if(gradientValue <= getGradientClosestNeighbour()){
				state = State.JOINED_SHAPE;
			}
			else if(validMovement(yard.getObjectLocation(this))){
				//followEdge()
				//!111111111111111111
			}
		}
//		else if(state == State.JOINED_SHAPE){
//			
//		}
	}
	
	/**
	 * 
	 * @return the gradient of a Robot's closer neigbour
	 */
	private int getGradientClosestNeighbour(){
//		Robot closer_neighbour;
		double current_distance;
		double closer_distance = Integer.MAX_VALUE;
		int closer_gradient = 0;
		
		for(int i =0; i < neighborhood.size(); i++){
			current_distance = TRS.getDistance(yard.getObjectLocationAsDouble2D(this), yard.getObjectLocation(neighborhood.get(i)));
			if(current_distance < closer_distance){
				closer_distance = current_distance;
				closer_gradient = ((Robot)neighborhood.get(i)).gradientValue;
//				closer_neighbour = (Robot)neighborhood.get(i);
			}
		}
		return closer_gradient;
	}
	
	
	/**
	 * 
	 * @param first_position
	 * @param second_position
	 * @return says if the two position are in the same place
	 */
	static public boolean thereIsCollision(Double2D first_position, Double2D second_position)
	{
		double xDif = first_position.x - second_position.x;
		double yDif = first_position.y - second_position.y;
		double distanceSquared = xDif * xDif + yDif * yDif;
		return (Math.sqrt(distanceSquared) < TRS.robot_width);
	}
	
	
	private boolean validMovement (Double2D nextPosition)
	{	
		Bag neighbors = yard.getNeighborsWithinDistance(nextPosition, DESIRED_DISTANCE);
		
		for (int i = 0; i < neighbors.size(); i++)
		{
			if (neighbors.get(i) == this || ((Robot)neighbors.get(i)).isStationary ) // the algoritm check collision betwwen edge follow robots
				continue;
			if (thereIsCollision( yard.getObjectLocation(neighbors.get(i)) , yard.getObjectLocation(this)))
				return false;
		}
		return true;
	}
	
	
	/**
	 * 
	 * @param position1
	 * @param position2
	 * @param position3
	 * @return if the three position given are collinear
	 */
	private boolean isCollinear(MutableDouble2D position1, MutableDouble2D position2, MutableDouble2D position3){		
		return (position2.getY() - position1.getY()) / (position2.getX() - position1.getX()) == 
				(position3.getY() - position2.getY()) / (position3.getX() - position2.getX());
	}
	
	
	/**
	 * 
	 * @param neigbours
	 * @return say if the a neighbourhood exist 3 collinear points
	 */
	private boolean has3NoCollinearNeighbours (Bag neigbours){
		int length = neigbours.size();
		for (int i = 0; i < length - 2; i++){
			for (int j = i +1; j < length - 1; j++){
				for (int z = j +1; z < length; z++){
					if(!isCollinear(((Robot)neigbours.get(i)).localization, ((Robot)neigbours.get(j)).localization, ((Robot)neigbours.get(z)).localization)){
						return true;
					}
				}
			}			
		}
		return false;
	}
	
	
	/**
	 * 
	 */
	private void localization(){
		Bag localized = new Bag();
		MutableDouble2D position_me = new MutableDouble2D(0,0);
		
		for (int i = 0; i < neighborhood.size(); i++)
		{
			if (!((Robot)neighborhood.get(i)).isStationary ||  neighborhood.get(i) == this || ! (((Robot)neighborhood.get(i)).isLocalized))
				continue;
			else
				localized.add(neighborhood.get(i));
		}
		if(has3NoCollinearNeighbours(localized)){
			for(int l = 0; l < localized.size(); l ++){
				double measured_distance = TRS.getDistance(yard.getObjectLocation(this), yard.getObjectLocation(localized.get(l)));
				double c = TRS.getDistance(position_me, ((Robot)localized.get(l)).localization);
				Double2D v = new Double2D((position_me.x - ((Robot)localized.get(l)).localization.x)/c,
						(position_me.y - ((Robot)localized.get(l)).localization.y)/c);
				Double2D n = new Double2D ( ((Robot)localized.get(l)).localization.x + measured_distance * v.x, 
						((Robot)localized.get(l)).localization.y + measured_distance * v.y);
				position_me.x = position_me.x - (position_me.x - n.x)/4;
				position_me.y = position_me.y - (position_me.y - n.y)/4;
			}
		}
		localization = position_me;
		if (validGradient) isLocalized = true;		
	}
	
	
	/**
	 * Tiene siempre un valor de max a no ser que tengamos vecinos con gradientes ya definidos
	 */
	private void gradientFormation()
	{
		Bag currentNeighborhood = yard.getNeighborsExactlyWithinDistance(yard.getObjectLocation(this),  swarm.gradientDistance);
		int current_gradient_value = Integer.MAX_VALUE;
		int neighValue;
		for (int i = 0; i < smallNeighborhood.size(); i++)
		{
			if (((Robot)smallNeighborhood.get(i)).validGradient && smallNeighborhood.get(i) != this && ((Robot)smallNeighborhood.get(i)).state == State.WAIT_TO_MOVE)
			{
				neighValue = ((Robot)smallNeighborhood.get(i)).getGradientValue();
				if ( neighValue + 1 < current_gradient_value)
				{
					current_gradient_value = neighValue + 1;
					validGradient = true;
				}
					
			}
		}
		gradientValue = current_gradient_value;
		
	}

}
