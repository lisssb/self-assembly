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
	boolean isLocalized = true; // esto se tiene que cambiar
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
	TRS swarm;
	Bag smallNeighborhood;
	long startMovingTime;
	Double2D nextMovement = new Double2D(0,0);
	
	
	
	public Robot(int id, boolean isSeed) {
		// TODO Auto-generated constructor stub
		this.id = id;
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
	public double orientation2D () {return orientation;}

//	public MutableDouble2D getLocalization() {return localization;}

	public Double2D getRobotPosition (){
		Double2D point = yard.getObjectLocation(this);
		int x = (int) (point.x - yard.getWidth()*0.5 );
		int y = (int) ( yard.getHeight()*0.5 - point.y);
//		y = swarm.map.getHeight() - y -1;
		return new Double2D(x, y);
	}
	
	public void step(SimState state){
		swarm = (TRS) state;
		yard = swarm.yard;		
		neighborhood = yard.getNeighborsWithinDistance(yard.getObjectLocation(this), swarm.robot_width * 3); // in each step we get the neighboprhood	
		smallNeighborhood = yard.getNeighborsExactlyWithinDistance(yard.getObjectLocation(this), (swarm.robot_width * 1.5 ));
		
		run();

	}

	
	private void rotate(boolean direction)
	{
		if (direction)
		{
			orientation += 0.2;
		}
		else
			orientation -= 0.2;
	}
	
	
	private void updateNextMovement(){
		MutableDouble2D nextPosition = new MutableDouble2D(0.0, 0.0);
		nextPosition.addIn(Math.cos(orientation)*0.1, Math.sin(orientation)*0.1);
		nextPosition.addIn(yard.getObjectLocation(this));
		nextMovement = new Double2D(nextPosition);
	}
	
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
				rotate(false);
			}
		}
		else{
			if(prev > current){
				updateNextMovement();
			}
			else{
				updateNextMovement();
				rotate(true);
			}
		}
		prev = current;
//		validGradient = false;
//		gradientFormation();
		
	}
	
	/**
	 * 
	 * @return Tell if the robot is in an outline position
	 */
	private boolean isOutline(){
		double x = position.getX();
		double y = position.getY();

		if( yard.getObjectsAtLocation(new Double2D(x, y+ swarm.robot_width))!=null && yard.getObjectsAtLocation(new Double2D(x, y-swarm.robot_width))!=null &&
				yard.getObjectsAtLocation(new Double2D(x+swarm.robot_width, y))!=null && yard.getObjectsAtLocation(new Double2D(x-swarm.robot_width, y))!=null){
			return false;
		}
		else{
			return true;
		}
	} 

	public boolean allGradientFormation (){
		Bag objects = yard.getAllObjects();
		for (int i = 0; i < objects.size(); i ++){
			if(!((Robot)objects.get(i)).validGradient){
				return false;
			}
		}
//		System.out.println(".....................................................");
//		for (int i = 0; i < objects.size(); i ++){
//			System.out.print(((Robot)objects.get(i)).gradientValue + " ");
//		}
//		System.out.println("---------------------------------------------------");
		return true;
	}
	/**
	 * 
	 * @param neighbourhood
	 * @param gradientValue
	 * @return the max_id of the a neighbourhood that:
	 * 		has the same gradient
	 * 		Is in the state WAIT_TO_MOVE
	 */
	private int maxNeighbourIdWithGradientValue (int gradientValue){
		int max_id = Integer.MIN_VALUE;
		for (int i = 0; i < neighborhood.size(); i++){
			if(((Robot)neighborhood.get(i)).id != id && ((Robot)neighborhood.get(i)).gradientValue == gradientValue
					&& ((Robot)neighborhood.get(i)).id > max_id 
					&& ((Robot)neighborhood.get(i)).state == State.WAIT_TO_MOVE){
				max_id = ((Robot)neighborhood.get(i)).id;
			}
		}
		return max_id;
	}

	private boolean run (){
		if(state == State.START){
			if(isSeed){
				state = State.JOINED_SHAPE;
			}
			else {
				gradientFormation();
//				localizate_robots();
//				if(getIfValidGradient() && getIfLocalized()){
//					state = State.WAIT_TO_MOVE;
//				}
				if(allGradientFormation()){
					state = State.WAIT_TO_MOVE;
				}
			}
		}
		else if(state == State.WAIT_TO_MOVE){
			boolean neighbours_moving = false;
			int max_neighbour_gradient = Integer.MIN_VALUE;
			int max_neighbour_id;
			int h;
			Robot current_neighbour;

			/**
			 * Tomo el maximo gradiente de mis vecinos (cuyo gradiente sea válido)
			 */
			for(int i = 0; i < neighborhood.size(); i++){
				current_neighbour = (Robot)neighborhood.get(i);
				if(current_neighbour != this ){
					if(current_neighbour.validGradient && max_neighbour_gradient <= current_neighbour.getGradientValue()
							&& current_neighbour.state == State.WAIT_TO_MOVE){
						max_neighbour_gradient = current_neighbour.getGradientValue();
					}
					if (!((Robot)neighborhood.get(i)).isStationary){
						neighbours_moving = true;
					}
				}
				
			}
			if(!neighbours_moving){
				h = max_neighbour_gradient;
				max_neighbour_id = maxNeighbourIdWithGradientValue( h);
				if(isOutline() && validGradient && (gradientValue > h || (gradientValue == h && id > max_neighbour_id))){
					state = State.MOVE_WHILE_OUTSIDE;
					startMovingTime = System.currentTimeMillis();
					isStationary = false;
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
				validGradient = false;
				gradientFormationL();
			}
			
		}
		else if(state == State.MOVE_WHILE_INSIDE){/// edge-follow while inside desired shape
			int closest_gradient_neighbour = getGradientClosestNeighbour();
			followEdge();
//			int x = (int) (nextMovement.x - yard.getWidth()*0.5);
			
			if(!swarm.isInsideShape(nextMovement, id) /*&& x >0*/){
				state = State.JOINED_SHAPE;
				isStationary = true;
			}
			
			if(gradientValue <= closest_gradient_neighbour){
				state = State.JOINED_SHAPE;
				isStationary = true;
			}
			
			if(validMovement(nextMovement)){
				yard.setObjectLocation(this, nextMovement);
				validGradient = false;
				gradientFormationL();
			}
		}
		return true;
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
	static public boolean thereIsCollision(Double2D first_position, Double2D second_position)
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
			if (neighbors.get(i) == this || ((Robot)neighbors.get(i)).isStationary) // the algoritm check collision betwwen edge follow robots
				continue;
			// si mi vecino esta antes que mi y me voy a chocar entonces espero
			if ( ((Robot)neighbors.get(i)).startMovingTime < startMovingTime &&
					thereIsCollision( yard.getObjectLocation(neighbors.get(i)) , yard.getObjectLocation(this)))
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

			for(int j = localized.size(), l = 0; j > 0; j --, l++){
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
	
	/**
	 * Tiene siempre un valor de max a no ser que tengamos vecinos con gradientes ya definidos
	 */
	private void gradientFormation()
	{
		Bag smallNeighborhood = yard.getNeighborsWithinDistance(yard.getObjectLocation(this), swarm.gradientDistance);
		int current_gradient_value = Integer.MAX_VALUE;
		Robot currentNeighbour;
		int neighValue;
		for (int i = 0; i < smallNeighborhood.size(); i++)
		{
			currentNeighbour = (Robot)smallNeighborhood.get(i);
			/** si el graidente de mi vecino es valido
			// si no soy yo
			// si mi vecino esta quieto**/
			if (((Robot)smallNeighborhood.get(i)).validGradient && smallNeighborhood.get(i) != this && ((Robot)smallNeighborhood.get(i)).isStationary)
			{

				neighValue = currentNeighbour.getGradientValue();
				
				if ( neighValue  < current_gradient_value)
				{
					current_gradient_value = neighValue;
					validGradient = true;
				}
					
			}
		}
		
		if(current_gradient_value < Integer.MAX_VALUE){
			gradientValue = current_gradient_value +1 ;
		}
	}
	
	
	
	private void gradientFormationL()
	{
//		Bag smallNeighborhood = yard.getNeighborsWithinDistance(yard.getObjectLocation(this), swarm.robot_width);
		int current_gradient_value = Integer.MAX_VALUE;
		Robot currentNeighbour;
		int neighValue;
		for (int i = 0; i < smallNeighborhood.size(); i++)
		{
			currentNeighbour = (Robot)smallNeighborhood.get(i);
			/** si el graidente de mi vecino es valido
			// si no soy yo
			// si mi vecino esta quieto**/
			if (((Robot)smallNeighborhood.get(i)).validGradient && smallNeighborhood.get(i) != this && ((Robot)smallNeighborhood.get(i)).isStationary)
			{

				neighValue = currentNeighbour.getGradientValue();
				
				if ( neighValue  < current_gradient_value)
				{
					current_gradient_value = neighValue;
					validGradient = true;
				}
					
			}
		}
		
		if(current_gradient_value < Integer.MAX_VALUE){
			gradientValue = current_gradient_value +1 ;
		}
	}

}
