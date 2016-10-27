package rp.routePlanning;

import java.awt.Point;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Queue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.LinkedBlockingDeque;
import java.util.concurrent.LinkedBlockingQueue;

import MotionControlAndQueue.AdderConnection;
import MotionControlAndQueue.AdderConnection2;
import jobSelect.JobDistances;
import jobSelect.JobSelection;
import jobSelect.TSP;
import lejos.util.Delay;
import parsing.ItemData;
import parsing.JobList;
import parsing.ReadFiles;
import rp.robotics.mapping.GridMap;
import rp.robotics.mapping.MapUtils;

/**
 * Multi-robot system route planning class
 */
public class RoutePlanning
{

	// FOR DEBUGGING
	public static ArrayList<LinkedBlockingDeque<Point>> queues;

	private static AStar[] aStar;
	private BlockingQueue<Byte[]> R1;
	private BlockingQueue<Byte[]> R2;
	private BlockingQueue<Byte[]> R3;
	
	private static int[] id;

	private static boolean[] finished =
	{ false, false, false };

	private static int STEP_LENGTH = 2000; 	//configurable
	
	private static long whenItAllBegan;

	private static int firstTimeStep;

	private static int[] facing;

	public static HashMap<Integer, RobotsPosition> positionMap;

	public static ArrayList<Point> dropPoints;
	// private static Point[] pickPoints;
	private static Map<Integer, JobList> jobMap;

	/**
	 * Initialises the object
	 * @param map The map used for path-finding
	 * @param facing The initial facing array for the three robots 
	 * @param dropPoints The drop-off points 
	 * @param pickPoints The pick-up points
	 * @param jobMap The map of jobs
	 * @throws UnreachableWaypointException Thrown if a waypoint is unreachable
	 */
	public RoutePlanning(GridMap map, int[] facing, ArrayList<Point> dropPoints, Point[] pickPoints,
			Map<Integer, JobList> jobMap) throws UnreachableWaypointException
	{
		aStar[0] = new AStar(0, map, R1, facing[0]);
		aStar[1] = new AStar(1, map, R2, facing[1]);
		aStar[2] = new AStar(2, map, R3, facing[2]);
		whenItAllBegan = System.currentTimeMillis();
		RoutePlanning.facing = facing;
		firstTimeStep = 0;
		RoutePlanning.dropPoints = dropPoints;
		// RoutePlanning.pickPoints = pickPoints;
		RoutePlanning.jobMap = jobMap;
		RoutePlanning.positionMap = new LinkedHashMap<Integer, RobotsPosition>();
		id = new int[3];

		principala();
	}

	/**
	 * Initialises the object
	 * (used for testing)
	 * @param map The map used for path finding
	 * @param R1 The queue in which the movement of the first robot will be sent
	 * @param R2 The queue in which the movement of the second robot will be sent
	 * @param R3 The queue in which the movement of the third robot will be sent
	 * @param facing The initial facing array for the three robots 
	 * @param dropPoints The drop-off points 
	 * @param pickPoints The pick-up points
	 * @param jobMap The map of jobs
	 * @throws UnreachableWaypointException Thrown if a waypoint is unreachable
	 */
	public RoutePlanning(GridMap map, BlockingQueue<Byte[]> R1, BlockingQueue<Byte[]> R2, BlockingQueue<Byte[]> R3,
			int[] facing, ArrayList<Point> dropPoints, Point[] pickPoints, Map<Integer, JobList> jobMap)
					throws UnreachableWaypointException
	{
		aStar = new AStar[3];
		this.R1 = R1;
		this.R2 = R2;
		this.R3 = R3;
		aStar[0] = new AStar(0, map, R1, facing[0]);
		aStar[1] = new AStar(1, map, R2, facing[1]);
		aStar[2] = new AStar(2, map, R3, facing[2]);
		whenItAllBegan = System.currentTimeMillis();
		firstTimeStep = 0;
		RoutePlanning.facing = facing;
		RoutePlanning.dropPoints = dropPoints;
		// RoutePlanning.pickPoints = pickPoints;
		RoutePlanning.jobMap = jobMap;
		RoutePlanning.positionMap = new LinkedHashMap<Integer, RobotsPosition>();
		id = new int[3];

		principala();
	}
	
	/**
	 * Constructor for RoutePlanning that uses AdderConection objects to send the paths to the respective queues.
	 * @param map The GridMap object on which the robots are moving.
	 * @param con1 The AdderConnection for the first robot.
	 * @param con2 The AdderConnection for the second robot.
	 * @param con3 The AdderConnection for the third robot.
	 * @param facing The array of facings of each robot.
	 * @param dropPoints The ArrayList of drop-off Points.
	 * @param pickPoints The Array of pick-up Points.
	 * @param jobMap The map containing all the jobs.
	 * @throws UnreachableWaypointException Exception that notifies of an unreachable waypoint.
	 */
	public RoutePlanning(GridMap map, AdderConnection con1, AdderConnection con2, AdderConnection con3,
			int[] facing, ArrayList<Point> dropPoints, Point[] pickPoints, Map<Integer, JobList> jobMap)
					throws UnreachableWaypointException
	{
		aStar = new AStar[3];
		aStar[0] = new AStar(0, map, con1, facing[0]);
		aStar[1] = new AStar(1, map, con2, facing[1]);
		aStar[2] = new AStar(2, map, con3, facing[2]);
		whenItAllBegan = System.currentTimeMillis();
		firstTimeStep = 0;
		RoutePlanning.facing = facing;
		RoutePlanning.dropPoints = dropPoints;
		// RoutePlanning.pickPoints = pickPoints;
		RoutePlanning.jobMap = jobMap;
		RoutePlanning.positionMap = new LinkedHashMap<Integer, RobotsPosition>();
		id = new int[3];

		principala();
	}	

	/**
	 * Method that returns the current job of the robot at the given index.
	 * @param index The index of the robot.
	 * @return The id of the job the robot is executing.
	 */
	public static int getCurrentJob(int index)
	{
		return id[index];
	}

	/**
	 * Method that returns the queue of waypoints through which a robot (represented by the index) has to move. 
	 * @param index The index of the robot.
	 * @return The queue of Points representing the waypoints.
	 */
	private static Queue<Point> getPointsQueue(int index)
	{
		return queues.get(index);
	}

	/**
	 * Method that gets called during the constructor. Calculates the path from the first point of the queue to the second for each of the three robots.
	 * @throws UnreachableWaypointException Exception that notifies of an unreachable waypoint.
	 */
	private void principala() throws UnreachableWaypointException
	{
		// System.out.println(getPointsQueue(0));
		// System.out.println(getPointsQueue(1));
		// System.out.println(getPointsQueue(2));

		for (int i = 0; i < 3; i++)
		{
//			 if(i==1)
//			 assert(getPointsQueue(0).isEmpty());

			System.out.println("i=" + i);
			assert (!getPointsQueue(i).isEmpty()) : "Robotul " + (i + 1) + " nu a primit niciun job";
			Point firstPoint = getPointsQueue(i).remove();
			Point secondPoint = getPointsQueue(i).remove();

			int itemNumber = -1;
			String itemName = null;

			for (Map.Entry<Integer, JobList> entry : jobMap.entrySet())
			{

				for (int pp = 0; pp < entry.getValue().size(); pp++)
				{
//					 System.out.println(entry.getValue().getJobItem(pp).getCoordinates());
					if (secondPoint.equals(entry.getValue().getJobItem(pp).getCoordinates()))
					{
						id[i] = entry.getKey();

						itemNumber = entry.getValue().getJobNum(pp);
						itemName = entry.getValue().getJobItem(pp).getID();

						// System.out.println(pathPoints.get(p) + " " +
						// bestJob.getJobNum(pp));

						break;
					}
				}
			}
			facing[i] = aStar[i].sendPathV2_0(firstPoint, secondPoint, 0, AStar.PICKUP, itemName, itemNumber);
		}
	}

	/**
	 * Method that deletes entries from the positionMap which contain the positions in a timestep that is no longer needed.
	 */
	private static void deleteWhileYouCan() // pt timestep
	{
		while (positionMap.get(firstTimeStep + 1) != null && positionMap.get(firstTimeStep + 1).isFull())
			positionMap.remove(firstTimeStep++);
	}

	// to be called when robot says it finished pick-up/drop-off
	/**
	 * Method that should be called every time the robot is done picking-up/droping-off an item.
	 * @param index The index of the robot that has finished its task.
	 * @throws UnreachableWaypointException Exception that notifies that a waypoint is unreachable.
	 */
	public static synchronized void taskFinished(int index) throws UnreachableWaypointException
	{
		System.out.println("TASK FINISHED");
		try
		{
			Point nextPoint = getPointsQueue(index).remove();
			// System.out.println(nextPoint);

			// System.out.println("Now:" + getCurrentPos(index));
			// System.out.println("Next:" + nextPoint);

			int itemNumber = 0;
			String itemName = null;
			int pickOrDrop = AStar.PICKUP;
			if (dropPoints.contains(nextPoint))
			{
				pickOrDrop = AStar.DROPOFF;
				// System.out.println("DROP");
			}
			else
				for (Map.Entry<Integer, JobList> entry : jobMap.entrySet())
					for (int pp = 0; pp < entry.getValue().size(); pp++)
						if (nextPoint.equals(entry.getValue().getJobItem(pp).getCoordinates()))
						{

							id[index] = entry.getKey();

							itemNumber = entry.getValue().getJobNum(pp);
							itemName = entry.getValue().getJobItem(pp).getID();

							// System.out.println(pathPoints.get(p) + " " +
							// bestJob.getJobNum(pp));

							break;
						}

			deleteWhileYouCan();

			// while(positionMap.get(calculateStep(System.currentTimeMillis())).getR(index)
			// )

			Delay.msDelay(calculateMillis(calculateStep(System.currentTimeMillis())) - System.currentTimeMillis());
			// System.out.println("Time step" +
			// calculateStep(System.currentTimeMillis()));
			facing[index] = aStar[index].sendPathV2_0(getCurrentPos(index), nextPoint,
					calculateStep(System.currentTimeMillis()), pickOrDrop, itemName, itemNumber);

		}
		catch (NoSuchElementException e)
		{
			if (!finished[index])
			{
				finished[index] = true;
				getPointsQueue(index).add(dropPoints.get(0));
				taskFinished(index);
				// System.out.println("S-A TERMINAT!");
			}
		}

	}

	/**
	 * Method that returns the last known position of the robot.
	 * @param index The index of the robot.
	 * @return The last known position of the robot given as a Point.
	 */
	private static Point getCurrentPos(int index)
	{
		int posIndex = calculateStep(System.currentTimeMillis());
		while (positionMap.get(posIndex) == null || positionMap.get(posIndex).getR(index) == null)
			posIndex--;
		return positionMap.get(posIndex).getR(index);
	}

	/**
	 * Calculate the millisecond value of a certain timestep.
	 * @param timestep The timestep whose millisecond value we want.
	 * @return The millisecond value of the timestep.
	 */
	private static long calculateMillis(int timestep)
	{
		return timestep * STEP_LENGTH + whenItAllBegan;
	}

	/**
	 * The timestep of a given millisecond value.
	 * @param currentTimeMillis The millisecond value.
	 * @return The timestep of the given value.
	 */
	private static int calculateStep(long currentTimeMillis)
	{
		return (int) Math.ceil((double) (currentTimeMillis - whenItAllBegan) / STEP_LENGTH);
	}

	/**
	 * Method that returns whether or not a certain position is busy at a certain timestep. (SHOULD NOT BE USED FOR DROP-OFF LOCATIONS)
	 * @param x The position given as a Point.
	 * @param timeStep The timestep.
	 * @return Boolean value that indicates whether or not the given position is busy at the given timestep.
	 */
	public static boolean isBusy(Point x, int timeStep)
	{
		// if(x.equals(new Point(4,7)))
		// System.out.println("ajunseram la " + timeStep);// +
		// positionMap.get(8).getR1());

		int aux = timeStep;
		while (aux >= 0 && (positionMap.get(aux) == null || positionMap.get(aux).getR1() == null))
			aux--;
		if (aux >= 0 && positionMap.get(aux).getR1().equals(x))
			return true;

		// if(x.equals(new Point(4,7)))
		// System.out.println(aux);

		aux = timeStep;
		while (aux >= 0 && (positionMap.get(aux) == null || positionMap.get(aux).getR2() == null))
			aux--;
		if (aux >= 0 && positionMap.get(aux).getR2().equals(x))
			return true;

		// if(x.equals(new Point(4,7)))
		// System.out.println(aux);

		aux = timeStep;
		while (aux >= 0 && (positionMap.get(aux) == null || positionMap.get(aux).getR2() == null))
			aux--;
		if (aux >= 0 && positionMap.get(aux).getR2().equals(x))
			return true;

		return false;

	}

	/**
	 * Method that returns whether or not a certain position is busy at a certain timestep. (SHOULD ONLY BE USED FOR DROP-OFF LOCATIONS)
	 * @param x The position given as a Point.
	 * @param timeStep The timestep.
	 * @return Boolean value that indicates whether or not the given position is busy at the given timestep.
	 */
	public static boolean isBusyDropOff(Point x, int timeStep)
	{
		if ((positionMap.get(timeStep) == null || positionMap.get(timeStep).getR1() == null
				|| !positionMap.get(timeStep).getR1().equals(x))
				&& (positionMap.get(timeStep) == null || positionMap.get(timeStep).getR2() == null
						|| !positionMap.get(timeStep).getR2().equals(x))
				&& (positionMap.get(timeStep) == null || positionMap.get(timeStep).getR3() == null
						|| !positionMap.get(timeStep).getR3().equals(x)))
			return false;
		return true;
	}

	public static void main(String[] args) throws UnreachableWaypointException
	{

		// <-------------------------CONFIGURABLE TEST
		// DATA----------------------->
		Point[] startpoint =
		{ new Point(0, 7), new Point(0, 6), new Point(0, 5) };
		GridMap map = MapUtils.createMarkingWarehouseMap();

		BlockingQueue<Byte[]> R1 = new LinkedBlockingQueue<>();
		BlockingQueue<Byte[]> R2 = new LinkedBlockingQueue<>();
		BlockingQueue<Byte[]> R3 = new LinkedBlockingQueue<>();

		queues = new ArrayList<>();
		queues.add(new LinkedBlockingDeque<>());
		queues.add(new LinkedBlockingDeque<>());
		queues.add(new LinkedBlockingDeque<>());

		int[] facing =
		{ AStar.UP, AStar.UP, AStar.UP };

		// <-------------------------OBJECTS FROM LUKAS----------------------->
		ReadFiles readFiles = new ReadFiles();
		Map<Integer, JobList> jobMap = new ConcurrentHashMap<Integer, JobList>();
		Map<String, ItemData> itemMap = new ConcurrentHashMap<String, ItemData>();
		ArrayList<Point> dropPoints = new ArrayList<Point>();
		try
		{
			// drop points
			readFiles.readDPFile("file/drops.csv", dropPoints);

			// items and their location
			readFiles.readItemLocationFile("file/locations.csv", itemMap);
			readFiles.readItemRWFile("file/items.csv", itemMap);

			// jobs
			readFiles.readJobFile("file/jobs.csv", jobMap, itemMap);

			readFiles.readCancellationFile("file/cancellations.csv", jobMap);

		}
		catch (FileNotFoundException e)
		{
			e.printStackTrace();
		}

		// <-------------------------OBJECTS FROM ALEX----------------------->

		Map<Integer, JobList> jobMap1 = new HashMap<>();
		jobMap1.putAll(jobMap);

		Point[] dropPointsArray = dropPoints.toArray(new Point[dropPoints.size()]);

		ConcurrentHashMap<Integer, JobDistances> jobDistancesMap = new ConcurrentHashMap<Integer, JobDistances>();
		// BlockingQueue<Byte[]> queue = new LinkedBlockingQueue<>();
		AStar aStar = new AStar(map);

		jobDistancesMap = aStar.calculateJobDistances((ConcurrentHashMap<Integer, JobList>) jobMap, startpoint[0],
				dropPointsArray);

		JobSelection jobSelection = new JobSelection((ConcurrentHashMap<Integer, JobList>) jobMap, jobDistancesMap,
				startpoint[0], dropPointsArray);
		JobList bestJob = jobSelection.getJob();

		// JobAssignment assignment = new JobAssignment(bestJob, startpoint[0],
		// startpoint[1], startpoint[2], dropPointsArray);

		Point[] pickpoints = null;

		System.out.println("job map size is " + jobMap1.size());

		for (int j = 1; j < jobMap1.size(); j++)
		{
			pickpoints = new Point[bestJob.size()];

			double[] weights = new double[bestJob.size()];
			for (int i = 0; i < bestJob.size(); i++)
			{
				pickpoints[i] = new Point(bestJob.getJobItem(i).getX(), bestJob.getJobItem(i).getY());
				weights[i] = bestJob.getJobItem(i).getWeight();
			}

			TSP tsp;

			tsp = new TSP(jobSelection.getJobDistances(), startpoint[j % 3], pickpoints, dropPointsArray, weights);

			ArrayList<Point> pathPoints = tsp.getPathPoints();
			queues.get(j % 3).addAll(pathPoints);
			startpoint[j % 3] = queues.get(j % 3).peekLast();

			jobSelection = new JobSelection(jobSelection.getNewJobMap(), jobSelection.getNewJobDistancesMap(),
					pathPoints.get(pathPoints.size() - 1), dropPointsArray);
			bestJob = jobSelection.getJob();

			assert (bestJob != null) : j;

			// assignment = new JobAssignment(bestJob, startpoint[0],
			// startpoint[1],
			// startpoint[2], dropPointsArray);

			// System.out.println("iese din assignment");

			// assignment.getFirstRobotPath(RoutePlanning.queues.get(0));
			// assignment.getSecondRobotPath(RoutePlanning.queues.get(1));
			// assignment.getThirdRobotPath(RoutePlanning.queues.get(2));

			// System.out.println("roboti assignment");

			// if(!queues.get(0).isEmpty())
			// startpoint[0]=queues.get(0).peekLast();
			// if(!queues.get(1).isEmpty())
			// startpoint[1]=queues.get(1).peekLast();
			// if(!queues.get(2).isEmpty())
			// startpoint[2]=queues.get(2).peekLast();

		}

		// <-------------------------DEBUGGING----------------------->

		// System.out.println("q:");
		// System.out.println(queues.get(0));

		// <-------------------------OUR OBJECT----------------------->

		System.out.println(queues.get(0));

		@SuppressWarnings("unused")
		RoutePlanning routePlanning = new RoutePlanning(MapUtils.createRealWarehouse(), R1, R2, R3, facing, dropPoints,
				pickpoints, jobMap1);

	}

}
