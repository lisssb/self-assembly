package trs;

import sim.engine.SimState;
import sim.engine.Steppable;
import sim.field.continuous.Continuous2D;
import sim.util.MutableDouble2D;

public class Robot  implements Steppable{
	
	public MutableDouble2D position;
	public int id;
	
	public Robot(int id) {
		// TODO Auto-generated constructor stub
		this.id = id;
	}


	public void step(SimState state){
		TRS swarm = (TRS) state;
		Continuous2D yard = swarm.yard;

	}
	

}
