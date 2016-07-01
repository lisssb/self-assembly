package trs;


import java.awt.geom.Point2D;

import sim.engine.SimState;
import sim.engine.Steppable;
import sim.field.continuous.Continuous2D;
import sim.util.Bag;
import sim.util.Double2D;
import sim.util.MutableDouble2D;

public class Robot  implements Steppable{

	public Double2D position = new Double2D(0.0, 0.0);
	double prev = Double.MAX_VALUE;
	double current = Double.MAX_VALUE;
	public MutableDouble2D localization;
	boolean isLocalized = false;
	State state = State.START;
	boolean isSeed;
	boolean validGradient = false;
	int gradientValue =Integer.MAX_VALUE;
	int last_gradient_value = Integer.MIN_VALUE;
	int steadyGradient = 0;
	
	double orientation;
	public int id;
	Bag neighborhood;
	double DESIRED_DISTANCE = TRS.robot_width + 0.2; //20mmm
	Continuous2D yard;
	boolean isStationary = true;
	TRS swarm;
	Bag smallNeighborhood;
	long startMovingTime;
	Double2D nextMovement = new Double2D(0,0);
	
	
	
	public Robot(int id, boolean isSeed) {
		// TODO Auto-generated constructor stub
//		this.id = id;
		this.isSeed = isSeed;
		if(isSeed){
			if(this.id == 0){
				this.gradientValue = 0;
			}
			else{
				this.gradientValue = 1;
			}			
		}
	}

	public int getGradientValue () {return gradientValue;}
	public boolean getIfValidGradient () {return validGradient;}
	public State getState () {return state;}
	public boolean getIsMoving() {return !isStationary;}
	public int getId () {return id;}
	public boolean getIfLocalized() {return isLocalized;}
	public void setOrientation(double o) {orientation = o;}
	public long getTime(){
		return startMovingTime;
	}

	
	public void step(SimState state){
		swarm = (TRS) state;
		yard = swarm.yard;
		neighborhood = yard.getNeighborsWithinDistance(yard.getObjectLocation(this), 10); // in each step we get the neighboprhood	
		smallNeighborhood = yard.getNeighborsExactlyWithinDistance(yard.getObjectLocation(this), swarm.gradientDistance );
				
		if(isSeed) return;
		position = yard.getObjectLocation(this);
		
		
		boolean moved;
		if(position != yard.getObjectLocation(this)) moved = true;
		else moved = false;		
		position = yard.getObjectLocation(this);
		if(moved){
			validGradient = false;
		}
		
		generateId();
		gradientFormation();
//		if(!isStationary){
//			localizate_robots();
//		}
		run();
	}
	/**
	 * 
	 * @return Main program used to change the state of the robot, update its position.
	 */
	private boolean run (){
		if (state == State.START) {
			// gradientFormation();
			// localizate_robots();

			// if(last_gradient_value != gradientValue){
			// steadyGradient = 0;
			// last_gradient_value = gradientValue;
			// }
			// else{
			// steadyGradient+=1;
			// }
			if (validGradient /* && steadyGradient > 10 */ /* && getIfLocalized() */) {
				state = State.WAIT_TO_MOVE;
			}
		}
		else if(state == State.WAIT_TO_MOVE){
			boolean neighbours_moving = false;
			int max_neighbour_gradient = Integer.MIN_VALUE;
			int max_neighbour_id;
			int h;
			Robot current_neighbour;
//			neighborhood = yard.getNeighborsWithinDistance(yard.getObjectLocation(this), TRS.robot_width * 2); // in each step we get the neighboprhood	

			/**
			 * Tomo el maximo gradiente de mis vecinos (cuyo gradiente sea válido)
			 */
			for(int i = 0; i < neighborhood.size(); i++){
				current_neighbour = (Robot)neighborhood.get(i);
				if(current_neighbour != this ){
					if(current_neighbour.validGradient && max_neighbour_gradient <= current_neighbour.getGradientValue()
							&& current_neighbour.state != State.JOINED_SHAPE){//oara no mirar los joined_shape
						max_neighbour_gradient = current_neighbour.getGradientValue();
					}
					if (!current_neighbour .isStationary || !current_neighbour.validGradient){
						neighbours_moving = true;
					}
				}
				
			}

			if(!neighbours_moving && validGradient){
				h = max_neighbour_gradient;
				max_neighbour_id = maxNeighbourIdWithGradientValue(h);
				if(gradientValue > h || (gradientValue == h && id > max_neighbour_id)){
					state = State.MOVE_WHILE_OUTSIDE;
					startMovingTime = System.currentTimeMillis();
					isStationary = false;
//					System.out.println(print);
				}				
			}
		}
		
		else if(state == State.MOVE_WHILE_OUTSIDE){
			followEdge();
			if(swarm.isInsideShape(nextMovement, id)){
				state = State.MOVE_WHILE_INSIDE;
				isStationary = false;
			}
			if(validMovement(nextMovement)){
				yard.setObjectLocation(this, nextMovement);
//				gradientFormation();
			}
			
		}
		else if(state == State.MOVE_WHILE_INSIDE){/// edge-follow while inside desired shape
			int closest_gradient_neighbour = getGradientClosestNeighbour();
			followEdge();
			
			if(!swarm.isInsideShape(nextMovement, id)){
				state = State.JOINED_SHAPE;
				isStationary = true;
			}
			
			if(gradientValue <= closest_gradient_neighbour){
				state = State.JOINED_SHAPE;
				isStationary = true;
			}
			
			if(validMovement(nextMovement)){
				yard.setObjectLocation(this, nextMovement);
//				gradientFormation();
			}
		}
	
		return true;
	}
	
	/**
	 * Generate locally unique ID if the robot is not a seed (this mantein its id only to paint it of different collor
	 */
	private void generateId ()
	{
		if(!isSeed){
			for (int i = 0; i < neighborhood.size(); i++)
			{
				if (neighborhood.get(i) == this)
					continue;
				if (((Robot)neighborhood.get(i)).id == id)
				{
					id = swarm.random.nextInt(swarm.numRobots*2);
					i=0;
				}
			}
		}
	}
	

	
	/**
	 * Tiene siempre un valor de max a no ser que tengamos vecinos con gradientes ya definidos
	 */
	private void gradientFormation()
	{
		Robot currentNeighbour;
		int neighValue;
//		validGradient = false; 
		
		
		for (int i = 0; i < smallNeighborhood.size(); i++)
		{
			currentNeighbour = (Robot)smallNeighborhood.get(i);
			if (currentNeighbour.validGradient && currentNeighbour != this && currentNeighbour.isStationary)
			{
				neighValue = currentNeighbour.getGradientValue();				
				if ( neighValue  < gradientValue)
				{
					gradientValue = neighValue;
					validGradient = true;
				}
			}
		}
		if(gradientValue < Integer.MAX_VALUE){			
			gradientValue = gradientValue +1 ;
		}
	}

	/**
	 * Edge follow nearest neighbor at DESIRED DISTANCE
	 */
	private void followEdge(){
		double measured_distance;
		
		current = Double.MAX_VALUE;
		for (int i = 0; i < neighborhood.size(); i++){			
			if((neighborhood.get(i)!= this) && ((Robot)neighborhood.get(i)).isStationary){
				measured_distance = TRS.getDistance(yard.getObjectLocation(this), yard.getObjectLocation(neighborhood.get(i)));
				if(measured_distance < current){
					current = measured_distance;
				}
			}
		}
		if(current < DESIRED_DISTANCE){
			if(prev < current){
				updateNextMovement();
			}
			else{
				updateNextMovement();
				turn_clockwise(false);
			}
		}
		else{
			if(prev > current){
				updateNextMovement();
			}
			else{
				updateNextMovement();
				turn_clockwise(true);
			}
		}
		prev = current;
		
	}
	
	/**
	 * 
	 * @param clockwise. add or substract rotation to the robot
	 */
	private void turn_clockwise(boolean clockwise)
	{
		if (clockwise)
		{
			orientation += 0.2;
		}
		else
			orientation -= 0.2;
	}
	
	/**
	 * This is used to know the next position of the robot
	 */
	private void updateNextMovement(){
		MutableDouble2D nextPosition = new MutableDouble2D(0.0, 0.0);
		nextPosition.addIn(Math.cos(orientation)*0.1, Math.sin(orientation)*0.1);
		nextPosition.addIn(yard.getObjectLocation(this));
		nextMovement = new Double2D(nextPosition);
	}

	/**
	 * 
	 * @param neighbourhood
	 * @param gradientValue
	 * @return the max_id of the a neighbourhood that:
	 * 		has the same gradient
	 * 		If it is not in the state JOINED_SHAPE
	 */
	private int maxNeighbourIdWithGradientValue (int gradientValue){
		int max_id = Integer.MIN_VALUE;
		for (int i = 0; i < neighborhood.size(); i++){
			if(((Robot)neighborhood.get(i)).id != id && ((Robot)neighborhood.get(i)).gradientValue == gradientValue
					&& ((Robot)neighborhood.get(i)).id > max_id 
					&& ((Robot)neighborhood.get(i)).state != State.JOINED_SHAPE
					&& ((Robot)neighborhood.get(i)).validGradient){
				max_id = ((Robot)neighborhood.get(i)).id;
			}
		}
		return max_id;
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
		Robot current_neighbour;
		for(int i =0; i < smallNeighborhood.size(); i++){
			current_neighbour = (Robot)smallNeighborhood.get(i);
			current_distance = TRS.getDistance(nextMovement, yard.getObjectLocation(current_neighbour));
			if(current_neighbour != this && !current_neighbour.isSeed && current_neighbour.isStationary  && (current_distance < closer_distance)){
				closer_distance = current_distance;
				closer_gradient = current_neighbour.gradientValue;
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
	static private boolean thereIsCollision(Double2D first_position, Double2D second_position)
	{
		double xDif = first_position.getX() - second_position.getX();
		double yDif = first_position.getY() - second_position.getY();
		double distanceSquared = xDif * xDif + yDif * yDif;
		return (Math.sqrt(distanceSquared) < TRS.robot_width);
	}
	

	
	private boolean validMovement (Double2D nextPosition)
	{	
		Bag neighbors = yard.getNeighborsWithinDistance(nextPosition, DESIRED_DISTANCE);
		
		for (int i = 0; i < neighbors.size(); i++)
		{
			if (neighbors.get(i) == this /*|| ((Robot)neighbors.get(i)).isStationary*/) // the algoritm check collision betwwen edge follow robots
				continue;
			// si mi vecino esta antes que mi y me voy a chocar entonces espero
			if ((  ((Robot)neighbors.get(i)).isStationary ||      ((Robot)neighbors.get(i)).startMovingTime < startMovingTime )&&
					thereIsCollision( yard.getObjectLocation(neighbors.get(i)) , new Double2D(nextPosition.getX(), nextPosition.getY()))){
				return false;
			}
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
	static private boolean isCollinear(MutableDouble2D position1, MutableDouble2D position2, MutableDouble2D position3){		
		return (position2.getY() - position1.getY()) / (position2.getX() - position1.getX()) == 
				(position3.getY() - position2.getY()) / (position3.getX() - position2.getX());
	}
	
	
	/**
	 * 
	 * @param neigbours
	 * @return say if the a neighbourhood exist 3 collinear points
	 */	
//	static private boolean has3NoCollinearNeighbours (Bag neigbours){
//		int length = neigbours.size();
//		for (int i = 0; i < length - 2; i++){
//			for (int j = i +1; j < length - 1; j++){
//				for (int z = j +1; z < length; z++){
//					if(!isCollinear(((Robot)neigbours.get(i)).localization, ((Robot)neigbours.get(j)).localization, ((Robot)neigbours.get(z)).localization)){
//						return true;
//					}
//				}
//			}			
//		}
//		return false;
//	}

	private boolean has3NoCollinearNeighbours (Bag neigbours){
		int length = neigbours.size();
		if(length < 3) return false;
		for (int i = 0; i < length - 2; i++){
			for (int j = i +1; j < length - 1; j++){
				for (int z = j +1; z < length; z++){
					if(i != j && i != z && j != z
							&& (Robot)neigbours.get(i) != this
							&& (Robot)neigbours.get(j) != this
							&& (Robot)neigbours.get(z) != this
							){
						if(!isCollinear(((Robot)neigbours.get(i)).localization,
								((Robot)neigbours.get(j)).localization,
								((Robot)neigbours.get(z)).localization)){
							return true;
						}
					}
				}
			}
		}
		return false;
	}

	private void localizate_robots(){
		Bag localized = new Bag();
		MutableDouble2D position_me = new MutableDouble2D(0,0);
		Double2D v;


		for (int i = 0; i < neighborhood.size(); i++)
		{
			if (!((Robot)neighborhood.get(i)).isStationary ||  neighborhood.get(i) == this || !(((Robot)neighborhood.get(i)).isLocalized))
				continue;
			else
				localized.add(neighborhood.get(i));
		}
		if(has3NoCollinearNeighbours(localized) ){

			for(int  l = 0; l < localized.size(); l++){
				double measured_distance = TRS.getDistance(yard.getObjectLocation(this), yard.getObjectLocation(localized.get(l)));
				double c = TRS.getDistance(position_me, ((Robot)localized.get(l)).localization);	
				if(c == 0){
					v = new Double2D(0,0);
				}
				else{
					v = new Double2D((position_me.getX() - ((Robot)localized.get(l)).localization.getX())/c,
							(position_me.getY() - ((Robot)localized.get(l)).localization.getY())/c);
				}
				Double2D n = new Double2D ( ((Robot)localized.get(l)).localization.getX() + measured_distance * v.getX(), 
						((Robot)localized.get(l)).localization.getY() + measured_distance * v.getY());
				position_me.setTo(position_me.getX() - (position_me.getX() - n.getX())/4, position_me.getY() - (position_me.getY() - n.getY())/4);
			}

			localization = position_me;
			if (validGradient) isLocalized = true;

		}
	}
	
	
}
