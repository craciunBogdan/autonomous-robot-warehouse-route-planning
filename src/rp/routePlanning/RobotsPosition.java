package rp.routePlanning;

import java.awt.Point;

/**
 * Class that represents the positions of each robot at a given moment.
 * 
 */
public class RobotsPosition
{
	private Point R1, R2, R3;
	
	/**
	 * Constructor for the class. Each point represents the position of one of the robots (in x, y coordinates).
	 * @param R1 The position of the first robot.
	 * @param R2 The position of the second robot.
	 * @param R3 The position of the third robot.
	 */
	public RobotsPosition(Point R1, Point R2, Point R3)
	{
	    this.R1 = R1;
	    this.R2 = R2;
	    this.R3 = R3;
	}
	
	/**
	 * Creates an empty RobotsPosition object
	 */
	public RobotsPosition()
	{
	}

	/**
	 * Return the position of the robot with the given index (starts at 0).
	 * @param index The index of the robot whose position is requested.
	 * @return A Point which contains the position of the robot.
	 */
	public Point getR(int index)
	{
	    if(index == 0)
		return R1;
	    if(index == 1)
		return R2;
	    return R3;
	}
	
	/**
	 * Method that returns the position of the first robot.
	 * @return Point which contains the position of the first robot.
	 */
	public Point getR1()
	{
	    return R1;
	}
	
	/**
	 * Method that allows the modification of the position of a robot at the given index (starts from 0).
	 * @param index The index of the robot whose position we need to modify.
	 * @param p The new position of the robot given as a Point.
	 */
	public void set(int index, Point p)
	{
	    if(index == 0)
		R1 = p;
	    if(index == 1)
		R2 = p;
	    else 
		R3 = p;
	}

	/**
	 * Method that allows the modification of the position of the first robot.
	 * @param r1 The new position of the first robot given as a Point.
	 */
	public void setR1(Point r1)
	{
	    R1 = r1;
	}

	/**
	 * Method that returns the position of the second robot.
	 * @return Point which contains the position of the second robot.
	 */
	public Point getR2()
	{
	    return R2;
	}

	/**
	 * Method that allows the modification of the position of the second robot.
	 * @param r1 The new position of the second robot given as a Point.
	 */
	public void setR2(Point r2)
	{
	    R2 = r2;
	}

	/**
	 * Method that returns the position of the third robot.
	 * @return Point which contains the position of the third robot.
	 */
	public Point getR3()
	{
	    return R3;
	}

	/**
	 * Method that allows the modification of the position of the third robot.
	 * @param r1 The new position of the third robot given as a Point.
	 */
	public void setR3(Point r3)
	{
	    R3 = r3;
	}

	/**
	 * Method that checks whether every single robot has a given position.
	 * @return Whether or not every robot has an assigned position.
	 */
	public boolean isFull()
	{
		return (R1!=null && R2!=null && R3!=null);
	}
}
