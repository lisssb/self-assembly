package trs;

import java.awt.Color;

import javax.swing.JFrame;


import sim.engine.*;
import sim.display.*;
import sim.portrayal.DrawInfo2D;
import sim.portrayal.Inspector;
import sim.portrayal.continuous.*;
import sim.portrayal.simple.*;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.color.*;

import javax.swing.JFrame;
public class TRSWithUI extends GUIState
{
	
	public Display2D display;
	public JFrame displayFrame;
	ContinuousPortrayal2D yardPortrayal = new ContinuousPortrayal2D();
	
	public static void main (String[] args)
	{
		TRSWithUI vid = new TRSWithUI();
		Console c = new Console(vid);
		c.setVisible(true);
	}
	
	
	public void start()
	{
		super.start();
		setupPortrayals();
	}

	public void load(SimState state)
	{
	super.load(state);
	setupPortrayals();
	}

	public void setupPortrayals()
	{
		TRS swarm = (TRS) state;
		// tell the portrayals what to portray and how to portray them
		yardPortrayal.setField( swarm.yard );
		yardPortrayal.setPortrayalForAll(
//				new MovablePortrayal2D(
						new OrientedPortrayal2D(
						new CircledPortrayal2D(
								new LabelledPortrayal2D(										
											new OvalPortrayal2D(Color.gray, swarm.robot_width, true){
												public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
												{
													Robot robot = (Robot)object;
													if (robot.isSeed && robot.id == 0)
														paint = Color.pink;
													else if(robot.isSeed && (robot.id == 1 || robot.id == 2 || robot.id ==3)){
														paint = Color.cyan;
													}
													else if(!robot.isStationary){
														paint = Color.orange;
													}
													else if(robot.state == State.JOINED_SHAPE){
														paint = Color.blue;
													}
													else if (robot.state == State.WAIT_TO_MOVE){
														paint = Color.green;
													}
//													else if (robot.isStationary)
//														paint = Color.black;
//													else if (!robot.validGradient)
//														paint = Color.orange;
													else
														paint = Color.LIGHT_GRAY;
													super.draw(object, graphics, info);
												}
									
											},											
										5.0, null, Color.black, true),
								0, 5.0, Color.blue, true),
						0, 5.0, Color.red));
		// reschedule the displayer
		
		display.reset();
		display.setBackdrop(Color.white);
		// redraw the display
		display.repaint();
	}
	
	public void init(Controller c)
	{
		super.init(c);
		display = new Display2D(600,600,this);
		display.setClipping(false);
		displayFrame = display.createFrame();
		displayFrame.setTitle("Thousand-robot swarm");
		c.registerFrame(displayFrame); // so the frame appears in the "Display" list
		displayFrame.setVisible(true);
		display.attach( yardPortrayal, "Space" );
	}

	
	public void quit()
	{
		super.quit();
		if (displayFrame!=null) displayFrame.dispose();
		displayFrame = null;
		display = null;
	}
	
	public Object getSimulationInspectedObject() { return state; }
	public Inspector getInspector()
	{
		Inspector i = super.getInspector();
		i.setVolatile(true);
		return i;
	}
	
	
	
	public TRSWithUI() { super(new TRS(System.currentTimeMillis())); }
	public TRSWithUI(SimState state) { super(state); }
	public static String getName() {return "Self assembling robots";}
}
