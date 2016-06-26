package trs;



import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;

import sim.engine.SimState;
import sim.field.continuous.Continuous2D;
import sim.util.Double2D;
import sim.util.MutableDouble2D;

public class TRS extends SimState {
	
	
//	public Object space;
	public Continuous2D yard = new Continuous2D(1.0,100,100);
	public int numRobots = 300;
	public static int robot_width = 2;
	public double gradientDistance = robot_width *  1.5;
	public BufferedImage map;
	public String imgFile = "shape1.png";
	boolean showSystem = false;
	
	public void setShowSystem (boolean value){
		showSystem = value;
	}

	public boolean getShowSystem (){
		return showSystem;
	}
	public TRS(long seed) {
		super(seed);
		// TODO Auto-generated constructor stub
	}

	public static void main(String[] args)
	{
		doLoop(TRS.class, args);
		System.exit(0);
	}
	
	private BufferedImage getImage(String filename)
	{
		try {
			// map = ImageIO.read(new File("prueba1.png"));
			 //map = new BufferedImage(im.getWidth(), im.getHeight(), BufferedImage.TYPE_BYTE_BINARY);
			InputStream in = getClass().getResourceAsStream(filename);
			return ImageIO.read(in);
		} catch (IOException e) {
			e.printStackTrace();
		}
		return null;
	}
	
	static public double getDistance(Double2D a, Double2D b)
	{
		double xDif = a.x - b.x;
		double yDif = a.y - b.y;
		return Math.sqrt(xDif * xDif + yDif * yDif);
	}
	
	static public double getDistance(MutableDouble2D a, MutableDouble2D b)
	{
		double xDif = a.x - b.x;
		double yDif = a.y - b.y;
		return Math.sqrt(xDif * xDif + yDif * yDif);
	}
	
	
	public void start()
	{
		super.start();		
		yard.clear();

		map = getImage("/resources/" + imgFile);
//		System.out.println("width " + map.getWidth() + "Height : " + map.getHeight() + " negro " + Color.black.getRGB());
//		for (int i = 0 ; i < map.getWidth(); i++){
//			String linea = "";
//			for (int j = map.getHeight()-1; j >0;  j--){
//				if( map.getRGB(i, j) == Color.black.getRGB()){
//					linea+="X";
//				}
//				else{
//					linea +=" ";
//				}
//			}
//			System.out.println(linea);
//		}

		createRobot(0, true, new Double2D(yard.getWidth() * 0.5 + 0, yard.getHeight() * 0.5 + 0));
		createRobot(1, true, new Double2D(yard.getWidth() * 0.5 + robot_width, yard.getHeight() * 0.5 ));
		createRobot(3, true, new Double2D(yard.getWidth() * 0.5 + robot_width/2, yard.getHeight() * 0.5 - robot_width));
		createRobot(2, true, new Double2D(yard.getWidth() * 0.5 + robot_width/2, yard.getHeight() * 0.5 + robot_width));

		int width = (int)Math.round(Math.sqrt(numRobots - 4));

		for(int i = 0, r = 2; i < numRobots - 4; r++)
		{
			if(numRobots - (i + 4) >= width){//here we fill columns with the not localized robots
				for (int j = 0; j < width; j++, i++){
					createRobot(i+4, false, new Double2D(yard.getWidth() * 0.5 + j*robot_width, yard.getHeight() * 0.5 + r*robot_width));
				}
			}
			else {// here we put the last robots in the last row.
				for (int j = 0; i+4 < numRobots; j++, i++){
					createRobot(i+4, false, new Double2D(yard.getWidth() * 0.5 + j*robot_width, yard.getHeight() * 0.5 + r*robot_width));
					
				}
				break;
			}
		}
		
	}
	
	
	
	/**
	 * 
	 * @param id
	 * @param isSeed
	 * @param position
	 */
	private void createRobot(int id, boolean isSeed, Double2D position){
		Robot robot = new Robot(id, isSeed);
		yard.setObjectLocation(robot, position);
//		robot.position = new MutableDouble2D(position);
		if(isSeed){
			robot.localization = new MutableDouble2D(position.getX()-yard.getWidth()*0.5, yard.getHeight()*0.5 - position.getY());
			robot.isLocalized = true;
			robot.validGradient = true;
		}
//		robot.setOrientation(0.5 * 4.1888);
		robot.setOrientation(0.5* (2 * robot_width * Math.PI/3 ));
		schedule.scheduleRepeating(robot);
	}
	
	/**
	 * 
	 * @param point
	 * @return is the given point is inside the coordinates of the shape.
	 * This mean if his position correspond to a black coordinate
	 */
	public boolean isInsideShape(Double2D point, int id)
	{
		int x = (int) (point.x - yard.getWidth()*0.5 );
		int y = (int) ( yard.getHeight()*0.5 - point.y );
//		x = x/TRS.robot_width;
//		y = map.getHeight() - (y /TRS.robot_width) -1;
		y = map.getHeight() - y -1;
//		System.out.println("id :: " + id + "x : "+ x + " y:: "+ y );
		
		if (x <= 0 || y < 0 || x >= map.getWidth() || y >= map.getHeight())
			return false;
		else{
//			System.out.println("Este es el color :: " + map.getRGB(x, y));
			if (map.getRGB(x, y) == Color.black.getRGB())
				return true;
			else 
				return false;
		}
	}
	
}
