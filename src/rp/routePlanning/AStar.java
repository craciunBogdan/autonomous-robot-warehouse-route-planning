package rp.routePlanning;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;

import MotionControlAndQueue.AdderConnection;
import MotionControlAndQueue.AdderConnection2;
import MotionControlAndQueue.RobotQueueTable;
import jobSelect.JobDistances;
import lejos.util.Delay;
import parsing.JobList;
import rp.robotics.mapping.GridMap;

/**
 * Route computing class
 */
public class AStar {

	private PriorityQueue<Node> frontier;
	private ArrayList<Point2D> explored;
	private Point start;
	private Point goal;
	private Node goalNode;
	private GridMap map;
	private int facing;
	private BlockingQueue<Byte[]> queue;

	private int robotIndex;

	public static final int PICKUP = 0;
	public static final int DROPOFF = 1;

	private int[] X = { -1, 0, 1, 0 };
	private int[] Y = { 0, -1, 0, 1 };

	// Facings
//	public static final int LEFT = 1;
//	public static final int DOWN = 0;
//	public static final int RIGHT = 3;
//	public static final int UP = 2;

	public static final int UP = 3;
	public static final int RIGHT = 2;
	public static final int LEFT = 0;
	public static final int DOWN = 1;
	
	// The algorithm moves
	private static final int FORWARD = 3;
	private static final int LEFT_TURN = 1;
	private static final int RIGHT_TURN = 0;
	private static final int BACKWARD = 2;

	static int[][] DIRECTIONS = /// [node][parent]
	{ { FORWARD, RIGHT_TURN, BACKWARD, LEFT_TURN }, { LEFT_TURN, FORWARD, RIGHT_TURN, BACKWARD },
			{ BACKWARD, LEFT_TURN, FORWARD, RIGHT_TURN }, { RIGHT_TURN, BACKWARD, LEFT_TURN, FORWARD } };

	// JUST FOR DEBUGGING
	int[][] mapDisplay;
	private int timeStep;
	private AdderConnection con;

	/**
	 * Initialises the AStar object (used for the one-robot configuration)
	 * @param map The map used for path-finding
	 */
	public AStar(GridMap map) {
		frontier = new PriorityQueue<>();
		explored = new ArrayList<>();
		mapDisplay = new int[map.getXSize()][map.getYSize()];
		this.queue = RobotQueueTable.get(0);
		this.map = map;
	}

	/**
	 * Initialises the AStar object  
	 * @param robotIndex The index of the robot (0-2)
	 * @param map The map used for path-finding
	 * @param queue The queue in which the instructions will be sent
	 * @param facing The initial facing of the robot
	 */
	public AStar(int robotIndex, GridMap map, BlockingQueue<Byte[]> queue, int facing) {
		frontier = new PriorityQueue<>();
		explored = new ArrayList<>();
		this.robotIndex = robotIndex;
		// mapDisplay = new int[map.getXSize()][map.getYSize()]; // DEBUG

		this.queue = queue;
		this.map = map;
		this.facing = facing;
	}
	
	/**
	 * Initialises the AStar object  
	 * @param robotIndex The index of the robot (0-2)
	 * @param map The map used for path-finding
	 * @param con The AdderConnection object used for sending instructions
	 * @param facing The initial facing of the robot
	 */
	public AStar(int robotIndex, GridMap map, AdderConnection con, int facing) {
		frontier = new PriorityQueue<>();
		explored = new ArrayList<>();
		this.robotIndex = robotIndex;
		// mapDisplay = new int[map.getXSize()][map.getYSize()]; // DEBUG

		this.con = con;
		this.map = map;
		this.facing = facing;
	}

	/**
	 * Calculates and sends a path to the BlockingQueue (used for the one-robot configuration)
	 * @param points The waypoints on the way
	 * @param facing The initial facing of the robot
	 * @param pointsName The list of the names of the items
	 * @param pointsItemNumber The list of the number of items in the job
	 * @throws UnreachableWaypointException Thrown if a point on the way is unreachable
	 */
	public void sendPath(Point[] points, int facing, ArrayList<String> pointsName, ArrayList<Integer> pointsItemNumber)
			throws UnreachableWaypointException {
		goal = points[0];
		int totalNumberOfItems = 0;

		for (int i = 0; i < points.length - 1; i++) {
			start = goal;
			goal = points[i + 1];

			// // DEBUG
			// System.out.println("Part " + (i + 1) + " of the path:");

			Byte[] bytes = new Byte[100];
			bytes[0] = new Byte((byte) 1);

			calculatePath();
			getPath(goalNode, bytes);

			// for (Byte b : bytes)
			// System.out.print(b + " ");
			// System.out.println();
			RobotQueueTable.put(robotIndex, bytes);

			// // DEBUG
			// System.out.println();

			facing = getGoalNode().getFacing();

			if (i != points.length - 2) {
				totalNumberOfItems += pointsItemNumber.get(i).intValue();
				Byte[] bytes1 = new Byte[] { new Byte((byte) -1), new Byte((byte) 1), new Byte((byte) 2),
						new Byte((byte) pointsName.get(i).charAt(0)), new Byte((byte) pointsName.get(i).charAt(1)),
						new Byte((byte) pointsItemNumber.get(i).intValue()) };
				// for (Byte b : bytes1)
				// System.out.print((char)(byte)b + " ");
				// System.out.println();
				RobotQueueTable.put(robotIndex, bytes1);
			}
		}
		Byte[] bytes = new Byte[] { new Byte((byte) -1), new Byte((byte) 2), new Byte((byte) 3), new Byte((byte) 'a'),
				new Byte((byte) 'l'), new Byte((byte) 'l'), new Byte((byte) totalNumberOfItems) };
		// for (Byte b : bytes)
		// System.out.print(b + " ");
		// System.out.println();
		RobotQueueTable.put(robotIndex, bytes);
	}

	/**
	 * Gets all the distances of a set of jobs
	 * @param jobMap The map of jobs
	 * @param start The start position
	 * @param dropOffs The list of drop-off points
	 * @return A map of JobDistances
	 * @throws UnreachableWaypointException Thrown if a points is unreachable
	 */
	public ConcurrentHashMap<Integer, JobDistances> calculateJobDistances(ConcurrentHashMap<Integer, JobList> jobMap,
			Point start, Point[] dropOffs) throws UnreachableWaypointException {
		this.start = start;
		this.facing = 0;

		ConcurrentHashMap<Integer, JobDistances> jobDistancesMap = new ConcurrentHashMap<Integer, JobDistances>();

		for (Entry<Integer, JobList> e : jobMap.entrySet()) {

			int[] startToPoint = new int[e.getValue().getList().size()];
			int[][] pointToPoint = new int[e.getValue().getList().size()][e.getValue().getList().size()];
			int[][] pointToDrop = new int[e.getValue().getList().size()][dropOffs.length];

			for (int j = 0; j < e.getValue().getList().size(); j++) {

				startToPoint[j] = calculateDistance(start, e.getValue().getJobItem(j).getCoordinates());

				for (int k = 0; k < e.getValue().getList().size(); k++)
					pointToPoint[j][k] = calculateDistance(e.getValue().getJobItem(j).getCoordinates(),
							e.getValue().getJobItem(k).getCoordinates());

				for (int k = 0; k < dropOffs.length; k++)
					pointToDrop[j][k] = calculateDistance(e.getValue().getJobItem(j).getCoordinates(), dropOffs[k]);
			}

			jobDistancesMap.put(e.getKey(), new JobDistances(startToPoint, pointToPoint, pointToDrop));

		}

		return jobDistancesMap;

	}

	/**
	 * Calculates the distance from the start point to the pick-up points
	 * @param start The start point
	 * @param jobList The job from which the pick-up points will be retrieved
	 * @return The array of distances
	 * @throws UnreachableWaypointException Thrown if a point is unreachable
	 */
	public int[] calculateDSP(Point start, JobList jobList) throws UnreachableWaypointException {
		this.start = start;
		this.facing = 0;
		int[] startToPoint = new int[jobList.getList().size()];

		for (int j = 0; j < jobList.getList().size(); j++)
			startToPoint[j] = calculateDistance(start, jobList.getJobItem(j).getCoordinates());

		return startToPoint;
	}

	/**
	 * Calculates a single JobDistance
	 * @param pickups The array of pick-up points 
	 * @param start The start point
	 * @param dropOffs The array of drop-off points
	 * @return The JobDistances object 
	 * @throws UnreachableWaypointException Thrown if a point is unreachable
	 */
	public JobDistances calculateJobDistance(Point[] pickups, Point start, Point[] dropOffs)
			throws UnreachableWaypointException {
		this.start = start;
		this.facing = 0;

		int[] startToPoint = new int[pickups.length];
		int[][] pointToPoint = new int[pickups.length][pickups.length];
		int[][] pointToDrop = new int[pickups.length][dropOffs.length];

		for (int j = 0; j < pickups.length; j++) {

			startToPoint[j] = calculateDistance(start, pickups[j]);

			for (int k = 0; k < pickups.length; k++)
				pointToPoint[j][k] = calculateDistance(pickups[j], pickups[k]);

			for (int k = 0; k < dropOffs.length; k++)
				pointToDrop[j][k] = calculateDistance(pickups[j], dropOffs[k]);
		}

		JobDistances jobDistances = new JobDistances(startToPoint, pointToPoint, pointToDrop);

		return jobDistances;
	}

	/**
	 * Calculates a single JobDistance
	 * @param jobList The job from which the pick-up points will be retrieved
	 * @param start The start point
	 * @param dropOffs The drop-off points
	 * @return The JobDistances object 
	 * @throws UnreachableWaypointException Thrown if a point is unreachable
	 */
	public JobDistances calculateJobDistance(JobList jobList, Point start, Point[] dropOffs)
			throws UnreachableWaypointException {
		this.start = start;
		this.facing = 0;

		int[] startToPoint = new int[jobList.getList().size()];
		int[][] pointToPoint = new int[jobList.getList().size()][jobList.getList().size()];
		int[][] pointToDrop = new int[jobList.getList().size()][dropOffs.length];

		for (int j = 0; j < jobList.getList().size(); j++) {

			startToPoint[j] = calculateDistance(start, jobList.getJobItem(j).getCoordinates());

			for (int k = 0; k < jobList.getList().size(); k++)
				pointToPoint[j][k] = calculateDistance(jobList.getJobItem(j).getCoordinates(),
						jobList.getJobItem(k).getCoordinates());

			for (int k = 0; k < dropOffs.length; k++)
				pointToDrop[j][k] = calculateDistance(jobList.getJobItem(j).getCoordinates(), dropOffs[k]);
		}

		JobDistances jobDistances = new JobDistances(startToPoint, pointToPoint, pointToDrop);

		return jobDistances;

	}

	/**
	 * 
	 * Recalculates all the distances of a set of jobs 
	 * @param jobMap The map of jobs
	 * @param jobDistancesMap The map of job distances
	 * @param start The start point
	 * @param dropOffs The drop-off points
	 * @return A HashMap of the distances
	 * @throws UnreachableWaypointException Thrown if a point is unreachable
	 */
	public ConcurrentHashMap<Integer, JobDistances> recalculateStartToPointDistances(
			ConcurrentHashMap<Integer, JobList> jobMap, ConcurrentHashMap<Integer, JobDistances> jobDistancesMap,
			Point start, Point[] dropOffs) throws UnreachableWaypointException {

		this.start = start;
		facing = 0;

		for (Entry<Integer, JobList> e : jobMap.entrySet()) {

			int[] startToPoint = new int[e.getValue().getList().size()];

			for (int j = 0; j < e.getValue().getList().size(); j++)
				startToPoint[j] = calculateDistance(start, e.getValue().getJobItem(j).getCoordinates());

			jobDistancesMap.get(e.getKey()).setDistanceToPoints(startToPoint);
		}

		return jobDistancesMap;
	}

	/**
	 * Calculates and sends a path to the robot (used for the multi-robot configuration)
	 * @param start The start point 
	 * @param goal The goal point
	 * @param timeStep The initial timestep of the robot
	 * @param pickOrDrop Whether the goal is a pick-up or a drop-off point
	 * @param itemName The name of the item to be picked-up (if applicable)
	 * @param itemNumber The number of items to be dropped-off
	 * @return The final facing of the robot
	 * @throws UnreachableWaypointException Thrown if the goal is unreachable
	 */
	public int sendPathV2_0(Point start, Point goal, int timeStep, int pickOrDrop, String itemName, int itemNumber)
			throws UnreachableWaypointException {
		if (start.equals(goal)) {
//			RoutePlanning.taskFinished(robotIndex);
			return facing;
		}

		System.out.println(start + " -> " + goal);
		if (pickOrDrop == DROPOFF) {
			System.out.println("Items dropped off.");
		}
		this.start = start;
		this.goal = goal;
		this.timeStep = timeStep;

		// System.out.println("From " + start + " to " + goal + " in time " +
		// timeStep + " for " + itemName + " " + itemNumber);

		assert (start != null);
		assert (goal != null);

		Byte[] bytes = new Byte[100];
		bytes[0] = new Byte((byte) 1);

		if (calculatePathV2_0()) {
			getPath(goalNode, bytes);

			//				 for (Byte b : bytes)
//				 System.out.print(b + " ");
//				 System.out.println();
//			RobotQueueTable.put(robotIndex, bytes);
			con.addToQueue(bytes);

			// // DEBUG
			// System.out.println();

			facing = getGoalNode().getFacing();

			// System.out.println(itemName);

			if (pickOrDrop == PICKUP) {
				bytes = new Byte[] { new Byte((byte) -1), new Byte((byte) 1), new Byte((byte) 2),
						new Byte((byte) itemName.charAt(0)), new Byte((byte) itemName.charAt(1)),
						new Byte((byte) itemNumber) };
				// Delay.msDelay(10 * goalNode.getG());
				// RoutePlanning.taskFinished(robotIndex);
			} else {
				bytes = new Byte[] { new Byte((byte) -1), new Byte((byte) 2), new Byte((byte) 3), new Byte((byte) 'a'),
						new Byte((byte) 'l'), new Byte((byte) 'l'), new Byte((byte) 2) };
				// Delay.msDelay(10 * goalNode.getG());
//				RoutePlanning.taskFinished(robotIndex);
			}
			// for (Byte b : bytes)
			// System.out.print(b + " ");
			// System.out.println();
//			RobotQueueTable.put(robotIndex, bytes);
			con.addToQueue(bytes);

			return facing;
		} else {
			System.out.println("unreachable");
			Delay.msDelay(2000);
			return sendPathV2_0(start, goal, timeStep + 1, pickOrDrop, itemName, itemNumber);
		}
	}

	//<--------------------------------------------------------->
	//<------------------------HELPERS-------------------------->
	//<--------------------------------------------------------->
	
	/**
	 * Goes through the path and puts the instructions in the Byte array
	 * @param node The current node
	 */
	private void getPath(Node node, Byte[] bytes) {

		if (RoutePlanning.positionMap.containsKey(timeStep + node.getG()))
			RoutePlanning.positionMap.get(timeStep + node.getG()).set(robotIndex, node.getPoint());
		else {
			RobotsPosition rp = new RobotsPosition();
			rp.set(robotIndex, node.getPoint());
			// System.out.println(robotIndex);
			RoutePlanning.positionMap.put(timeStep + node.getG(), rp);

			assert (RoutePlanning.positionMap.get(timeStep + node.getG()) != null);
		}

		if (timeStep > 7 && robotIndex == 0) {
			assert (RoutePlanning.positionMap.get(timeStep + node.getG()) != null);
			assert (RoutePlanning.positionMap.get(timeStep + node.getG()).getR1() != null);
		}

		if (node.getParent() != null) {
			getPath(node.getParent(), bytes);

			bytes[node.getG()] = new Byte(moveToByte(getMove(node.getParent(), node)));
			System.out.println(bytes[node.getG()]);
			// System.out.print(node.getPoint().getX() + "," +
			// node.getPoint().getY() + " -> ");
			// System.out.print(node.getFacing() + " ");
			// System.out.print(getMove(node.getParent(), node) + " ");
			// System.out.println(moveToString(getMove(node.getParent(), node))
			// + " ");
		}
		// else
		// System.out.print(node.getFacing() + " ");

		// DEBUG - for betterDisplay
		// mapDisplay[(int) node.getPoint().getX()][(int)
		// node.getPoint().getY()] = 1;

		// if(robotIndex == 1)
		// System.out.println(node.getPoint());
	}

	/**
	 * Gets the move of the robot (the instruction to be sent) between 2 nodes
	 * @param parentThe first node
	 * @param node The second node
	 * @return The move
	 */
	private Integer getMove(Node parent, Node node) {
		return DIRECTIONS[node.getFacing()][parent.getFacing()];
	}

	/**
	 * Calculates the distance between two points
	 * @param start The start point
	 * @param goal The goal point
	 * @return The distance in junctions
	 * @throws UnreachableWaypointException Thrown if the goal is unreachable
	 */
	private int calculateDistance(Point start, Point goal) throws UnreachableWaypointException {
		if (start.equals(goal))
			return 0;

		this.start = start;
		this.goal = goal;
		calculatePath();
		return getNumSteps();
	}

	/**
	 * Tells whether a point was explored or not
	 * @param point The point
	 * @return True if the point was explored, false if not
	 */
	private boolean isInExplored(Point2D point) {
		for (Point2D iPoint : explored) {
			if (point.equals(iPoint))
				return true;
		}
		return false;
	}

	/**
	 * Converts an int instruction to a Byte
	 * @param move The move
	 * @return The corresponding byte
	 */
	private Byte moveToByte(int move) {
		// if (move == FORWARD)
		// return "" + Q_FORWARD;
		// if (move == BACKWARD)
		// return "" + Q_BACKWARD;
		// if (move == LEFT_TURN)
		// return "" + Q_LEFT_TURN;
		// return "" + Q_RIGHT_TURN;
		return (byte) move;
	}

	/**
	 * Adds the adjacent points of a node's point to the frontier
	 * (used for the single-robot configuration)
	 * @param current The current node
	 */
	private void addAdjacent(Node current) {

		Point2D position = current.getPoint();
		int xCurrent = (int) position.getX();
		int yCurrent = (int) position.getY();

		for (int i = 0; i < 4; i++) {

			int xNext = xCurrent + X[i];
			int yNext = yCurrent + Y[i];
			Point nextPoint = new Point(xNext, yNext);
			if (map.isValidTransition(xCurrent, yCurrent, xNext, yNext) && !isInExplored(nextPoint)) {
				// System.out.println("is valid transition from " + xCurrent +
				// "," + yCurrent + " to " + xNext + " , " + yNext);
				// System.out.println("the Manhattan distance is :" +
				// getManhattan(xNext, yNext, goal.getX(), goal.getY()));
				frontier.add(new Node(current, new Point(xNext, yNext), i,
						getManhattan(xNext, yNext, goal.getX(), goal.getY()), current.getG() + 1));
			}
		}
	}

	/**
	 * Adds the adjacent points of a node's point to the frontier 
	 * (used for the multi-robot configuration)
	 * @param current The current node
	 */
	private void addAdjacentV2_0(Node current) {

		Point2D position = current.getPoint();
		int xCurrent = (int) position.getX();
		int yCurrent = (int) position.getY();

		for (int i = 0; i < 4; i++) {
			int xNext = xCurrent + X[i];
			int yNext = yCurrent + Y[i];
			Point nextPoint = new Point(xNext, yNext);

			if (map.isValidTransition(xCurrent, yCurrent, xNext, yNext) && !isInExplored(nextPoint)
					&& ((!RoutePlanning.dropPoints.contains(nextPoint)
							&& !RoutePlanning.isBusy(nextPoint, timeStep + current.getG())) 
					||
			(RoutePlanning.dropPoints.contains(nextPoint)
					&& !RoutePlanning.isBusyDropOff(nextPoint, timeStep + current.getG())))) {

				// System.out.println("is valid transition from " + xCurrent +
				// "," + yCurrent + " to " + xNext + " , " + yNext);
				// System.out.println("the Manhattan distance is :" +
				// getManhattan(xNext, yNext, goal.getX(), goal.getY()));
				frontier.add(new Node(current, new Point(xNext, yNext), i,
						getManhattan(xNext, yNext, goal.getX(), goal.getY()), current.getG() + 1));

			}
		}
	}

	/**
	 * Calculates the path between the start and the goal (uses local variables)
	 * (used for single-robot configuration)
	 * @throws UnreachableWaypointException The goal is unreachable
	 */
	public void calculatePath() throws UnreachableWaypointException {
		explored.clear();
		frontier.clear();
		frontier.add(
				new Node(null, start, facing, getManhattan(start.getX(), start.getY(), goal.getX(), goal.getY()), 0));

		while (!frontier.isEmpty()) {
			Node current = frontier.remove();
			if (current.getPoint().equals(goal)) {
				goalNode = current;
				return;
			} else {
				explored.add(current.getPoint());
				addAdjacent(current);
			}
		}

		System.out.println("Unreachable");
		System.out.println(start + " -> " + goal);
		throw new UnreachableWaypointException("A waypoint is unreachable!");
	}

	/**
	 * Calculates the path between the start and the goal (uses local variables)
	 * (used for multi-robot configuration)
	 * @return True if the goal is reachable
	 * @throws UnreachableWaypointException Used to be thrown if the goal was unreachable
	 */
	public boolean calculatePathV2_0() throws UnreachableWaypointException {
		explored.clear();
		frontier.clear();
		frontier.add(
				new Node(null, start, facing, getManhattan(start.getX(), start.getY(), goal.getX(), goal.getY()), 0));

		while (!frontier.isEmpty()) {
			Node current = frontier.remove();

			if (current.getPoint().equals(goal)) {
				goalNode = current;
				return true;
			} else {
				explored.add(current.getPoint());
				addAdjacentV2_0(current);
			}
		}
		// System.out.println(robotIndex);
		assert (RoutePlanning.queues.get(0).isEmpty());
		return false;

		// throw new UnreachableWaypointException("A waypoint is unreachable! "
		// + start.getLocation() + " -> " + goal + " " );
	}

	/**
	 * Gets the goal node of the last calculated path
	 * @return The goal node
	 */
	public Node getGoalNode() {
		return goalNode;
	}

	/**
	 * Gets the length of the last calculated path
	 * @return The length of the path
	 */
	private int getNumSteps() {
		return goalNode.getG();
	}

	/**
	 * Gets the Manhattan distance between two points
	 * @param a The x coordinate of the first point
	 * @param b The y coordinate of the first point
	 * @param x The x coordinate of the second point
	 * @param y The y coordinate of the second point
	 * @return The heuristic distance
	 */
	private int getManhattan(double a, double b, double x, double y) {
		return Math.abs((int) (y - b)) + Math.abs((int) (x - a));
	}

//	<-------------------------------------------------------->
//	<------------------------DEBUG--------------------------->
//	<-------------------------------------------------------->
	
	/**
	 * Initialises the A Star with a single drop-off point
	 * (used in JUnit tests)
	 * @param start The starting location
	 * @param goal The finish location
	 * @param map The GridMap object containing the obstacles
	 * @param facing The initial rotation of the robot
	 */
	public AStar(Point start, Point goal, GridMap map, int facing) {
		frontier = new PriorityQueue<>();
		explored = new ArrayList<>();
		mapDisplay = new int[map.getXSize()][map.getYSize()];
		this.facing = facing;
		this.goal = goal;
		this.start = start;
		this.map = map;
	}

	/**
	 * Puts the path in an ArrayList
	 * (used in JUnit tests)
	 * @param node The current node
	 * @param list The ArrayList
	 */
	public void getPath(Node node, ArrayList<Integer> list) {
		if (node.getParent() != null) {
			getPath(node.getParent(), list);
			// System.out.println(node.getPoint().getX() + "," +
			// node.getPoint().getY() + " -> ");
			list.add(getMove(node.getParent(), node));
		}
	}

	/**
	 * Displays the map like so: - X -> explored - 8 -> obstacle
	 */
	@SuppressWarnings("unused")
	private void display() {
		for (int i = 0; i < map.getXSize(); i++) {
			for (int j = 0; j < map.getYSize(); j++) {
				if (map.isObstructed(i, j) == false) {
					if (explored.contains(new Point(i, j)) || goal.equals(new Point(i, j))) {
						System.out.print("X");
					} else {
						System.out.print("_");
					}
				} else {
					System.out.print("8");
				}
			}
			System.out.println();
		}
	}

	/**
	 * Displays the map like so: - X -> on path - 8 -> obstacle
	 */
	@SuppressWarnings("unused")
	private void betterDisplay() {
		System.out.println();

		for (int i = 0; i < map.getXSize(); i++) {
			System.out.println();
			for (int j = 0; j < map.getYSize(); j++)
				if (mapDisplay[i][j] == 1)
					System.out.print("X");
				else if (map.isObstructed(i, j))
					System.out.print("8");
				else
					System.out.print("_");
		}
	}

	/**
	 * Clears the matrix mapDisplay
	 */
	@SuppressWarnings("unused")
	private void clearDisplay() {
		for (int i = 0; i < map.getXSize(); i++)
			for (int j = 0; j < map.getYSize(); j++)
				mapDisplay[i][j] = 0;
	}

}
