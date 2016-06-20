package trs;



import sim.engine.SimState;
import sim.field.continuous.Continuous2D;
import sim.util.Double2D;
import sim.util.MutableDouble2D;

public class TRS extends SimState {
	
	
//	public Object space;
	public Continuous2D yard = new Continuous2D(1.0,100,100);
	public int numRobots = 52;
	public int robot_width = 2;


	

	public TRS(long seed) {
		super(seed);
		// TODO Auto-generated constructor stub
	}

	public static void main(String[] args)
	{
//		SimState state = new Students(System.currentTimeMillis());
//		state.start();
//		do
//			if (!state.schedule.step(state)) break;
//		while(state.schedule.getSteps() < 5000);
//		state.finish();
//		System.exit(0);
		doLoop(TRS.class, args);
		System.exit(0);
	}

	public void start()
	{
		super.start();
		
		yard.clear();
		
		
		createRobot(0, new Double2D(yard.getWidth() * 0.5 + 0, yard.getHeight() * 0.5 + 0));
		createRobot(1, new Double2D(yard.getWidth() * 0.5 + robot_width, yard.getHeight() * 0.5 + 0));
		createRobot(2, new Double2D(yard.getWidth() * 0.5 + 0, yard.getHeight() * 0.5 + robot_width));
		createRobot(3, new Double2D(yard.getWidth() * 0.5 + robot_width, yard.getHeight() * 0.5 + robot_width));
		
		
		int width = (int)Math.round(Math.sqrt(numRobots - 4));

		
		for(int i = 0, r = 2; i < numRobots - 4; r++)
		{
			if(numRobots - (i + 4) >= width){//here we fill columns with the not localized robots
				for (int j = 0; j < width; j++, i++){
					createRobot(i+4, new Double2D(yard.getWidth() * 0.5 + j*robot_width, yard.getHeight() * 0.5 + r*robot_width));
				}
			}
			else {// here we put the last robots in the last row.
				for (int j = 0; i+4 < numRobots; j++, i++){
					createRobot(i+4, new Double2D(yard.getWidth() * 0.5 + j*robot_width, yard.getHeight() * 0.5 + r*robot_width));
					
				}
				break;
			}

		}
	
		
		
	}
	
	
	private void createRobot(int id, Double2D position){
		Robot robot = new Robot(id);
		
		yard.setObjectLocation(robot, position);
		schedule.scheduleRepeating(robot);
	}
	
}
