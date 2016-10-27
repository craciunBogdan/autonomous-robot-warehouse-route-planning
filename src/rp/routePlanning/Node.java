package rp.routePlanning;

import java.awt.Point;

/**
 * Class that represents a node in the tree build by the AStar algorithm.
 * 
 */
public class Node implements Comparable<Node>
{

	private Node parent;
	private Point point;
	private int facing;
	private int h; // heuristic - Manhattan
	private int g; // cost
	private int f;

	/**
	 * Constructor for a node.
	 * @param parent The parent node (the node from which we have arrived to this one).
	 * @param point The position of the current node on the map (given in x, y coordinates).
	 * @param facing The facing of the robot in this node (given as an int, check AStar.java for headings).
	 * @param heuristic The heuristic cost of getting from this node to the goal.
	 * @param cost The cost of getting from the start node to the current one.
	 */
	public Node(Node parent, Point point, int facing, int heuristic, int cost)
	{
		this.parent = parent;
		this.facing = facing;
		this.point = point;
		this.h = heuristic;
		this.g = cost;
		f = h + g;
	}

	/**
	 * Method that returns the facing of the robot.
	 * @return The facing of the robot.
	 */
	public int getFacing()
	{
		return facing;
	}

	/**
	 * Method that returns the parent node of the current node.
	 * @return The parent node of the current node.
	 */
	public Node getParent()
	{
		return parent;
	}

	/**
	 * Method that changes the parent node of the current node.
	 * @param parent The new parent node of the current node.
	 */
	public void setParent(Node parent)
	{
		this.parent = parent;
	}

	/**
	 * Method that returns the position that the current node holds.
	 * @return The position of the current node given as a Point.
	 */
	public Point getPoint()
	{
		return point;
	}

	/**
	 * Method that changes the current position of the node.
	 * @param point The new position of the current node given as a Point.
	 */
	public void setPoint(Point point)
	{
		this.point = point;
	}

	/**
	 * Method that returns the heuristic value of the node.
	 * @return The heuristic value of the node.
	 */
	public int getH()
	{
		return h;
	}

	/**
	 * Method that changes the heuristic value of the current node.
	 * @param h The new heuristic value of the current node.
	 */
	public void setH(int h)
	{
		this.h = h;
		f = g + h;
	}

	/**
	 * Method that returns the cost of getting from the start node to the current node.
	 * @return The cost of getting from the start node to the current node.
	 */
	public int getG()
	{
		return g;
	}

	/**
	 * Method that changes the cost of getting from the start node to the current node.
	 * @param g The new cost of getting from the start node to the current node.
	 */
	public void setG(int g)
	{
		this.g = g;
		f = g + h;
	}

	/**
	 * Method that returns the sum of the heuristic value and of the cost.
	 * @return The sum of the heuristic value and of the cost.
	 */
	public int getF()
	{
		return f;
	}

	@Override
	public int compareTo(Node o)
	{
		return ((Integer) f).compareTo(o.getF());
	}

}
