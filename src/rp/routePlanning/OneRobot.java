//<<<<<<< HEAD
//package rp.routePlanning;
//
//import java.awt.Point;
//import java.io.FileNotFoundException;
//import java.util.ArrayList;
//import java.util.Map;
//import java.util.concurrent.ConcurrentHashMap;
//
//import jobSelect.JobDistances;
//import jobSelect.JobSelection;
//import jobSelect.TSP;
//import parsing.ItemData;
//import parsing.JobList;
//import parsing.ReadFiles;
//import rp.robotics.mapping.GridMap;
//import rp.robotics.mapping.MapUtils;
//
//public class OneRobot
//{
//
//    public static void main(String[] args) throws UnreachableWaypointException
//    {
//	// <-------------------------CONFIGURABLE TEST DATA----------------------->
//	Point startpoint = new Point(0, 0);
//	GridMap map = MapUtils.createRealWarehouse();
//	int facing = AStar.UP;
//
//	// <-------------------------LUKAS----------------------->
//	ReadFiles readFiles = new ReadFiles();
//	Map<Integer, JobList> jobMap = new ConcurrentHashMap<Integer, JobList>();
//	Map<String, ItemData> itemMap = new ConcurrentHashMap<String, ItemData>();
//	ArrayList<Point> dropPoints = new ArrayList<Point>();
//	try
//	{
//	    // drop points
//	    readFiles.readDPFile("file/drops.csv", dropPoints);
//
//	    // items and their location
//	    readFiles.readItemLocationFile("file/locations.csv", itemMap);
//	    readFiles.readItemRWFile("file/items.csv", itemMap);
//
//	    // jobs
//	    readFiles.readJobFile("file/jobs.csv", jobMap, itemMap);
//	    readFiles.readCancellationFile("file/cancellations.csv", jobMap);
//
//	} catch (FileNotFoundException e)
//	{
//	    e.printStackTrace();
//	}
//
//
//	// <-------------------------ALEX----------------------->
//
//	Point[] dropPointsArray = dropPoints.toArray(new Point[dropPoints.size()]);
//
//	ConcurrentHashMap<Integer, JobDistances> jobDistancesMap = new ConcurrentHashMap<Integer, JobDistances>();
//	//BlockingQueue<Byte[]> queue = new LinkedBlockingQueue<>();
//	AStar aStar = new AStar(map);
//
//	jobDistancesMap = aStar.calculateJobDistances((ConcurrentHashMap<Integer, JobList>) jobMap, startpoint,
//		dropPointsArray);
//
//	JobSelection jobSelection = new JobSelection((ConcurrentHashMap<Integer, JobList>) jobMap, jobDistancesMap,
//		startpoint, dropPointsArray);
//	JobList bestJob = jobSelection.getJob();
//
//	/////////////////////////////
//	
//	while (bestJob != null)
//	{
//	    
////	    for (int i = 0; i < bestJob.size(); i++)
////		{
////		    System.out.println(bestJob.getJobItem(i).getX() + " " + bestJob.getJobItem(i).getY());
////		}
//
////		System.out.println("Drop-offs:");
//
////		System.out.println(dropPoints);
//
//		Point[] pickpoints = new Point[bestJob.size()];
//
//		double[] weights = new double[bestJob.size()];
//		for (int i = 0; i < bestJob.size(); i++)
//		{
//		    pickpoints[i] = new Point(bestJob.getJobItem(i).getX(), bestJob.getJobItem(i).getY());
//		    weights[i] = bestJob.getJobItem(i).getWeight();
//		}
//
//		TSP tsp = new TSP(jobSelection.getJobDistances(), startpoint, pickpoints, dropPointsArray, weights);
//
//		ArrayList<Point> pathPoints = tsp.getPathPoints();
//		ArrayList<Integer> pointsItemNumber = new ArrayList<>();
//		
//		ArrayList<String> pointsName = new ArrayList<>();
//		
//		for(int p=1;p<pathPoints.size()-1;p++)
//		{
//		    for(int pp=0;pp<pickpoints.length;pp++)
//		    {
//			if(pathPoints.get(p).equals(pickpoints[pp]))
//			{
//			    pointsItemNumber.add(bestJob.getJobNum(pp));
//			    pointsName.add(bestJob.getJobItem(pp).getID());
//			    
//			   // System.out.println(pathPoints.get(p) + " " + bestJob.getJobNum(pp));
//			    
//			    break;
//			}
//		    }
//		}
//		
//		// <-------------------------ROUTE----------------------->
//		aStar.sendPath(pathPoints.toArray(new Point[pathPoints.size()]), facing, pointsName, pointsItemNumber);
//		
//		
//		
//		jobSelection = new JobSelection(jobSelection.getNewJobMap(), jobSelection.getNewJobDistancesMap(), pathPoints.get(pathPoints.size()-1), dropPointsArray);
//		bestJob = jobSelection.getJob();
//		
//		startpoint = pathPoints.get(pathPoints.size()-1);
//	}
//	
//    }
//
//}
//=======
//package rp.routePlanning;
//
//import java.awt.Point;
//import java.io.FileNotFoundException;
//import java.util.ArrayList;
//import java.util.Map;
//import java.util.concurrent.ConcurrentHashMap;
//
//import jobSelect.JobDistances;
//import jobSelect.JobSelection;
//import jobSelect.TSP;
//import parsing.ItemData;
//import parsing.JobList;
//import parsing.ReadFiles;
//import rp.robotics.mapping.GridMap;
//import rp.robotics.mapping.MapUtils;
//
//public class OneRobot extends Thread {
//
//	private Point startpoint;
//	private GridMap map;
//	private int facing;
//	private Point[] dropPointsArray;
//	private ConcurrentHashMap<Integer, JobList> jobMap;
//	private ConcurrentHashMap<Integer, JobDistances> jobDistancesMap = new ConcurrentHashMap<Integer, JobDistances>();
//	private ArrayList<JobList> jobsToDisplay = new ArrayList<JobList>();
//	private AStar aStar;
//	private JobSelection jobSelection;
//	private JobList bestJob;
//
//	public OneRobot(Point sP, int f, ArrayList<Point> dP, ConcurrentHashMap<Integer, JobList> jM) {
//		// <-------------------------CONFIGURABLE TEST
//		// DATA----------------------->
//		startpoint = sP;
//		map = MapUtils.createRealWarehouse();
//		facing = f;
//		jobMap = jM;
//		dropPointsArray = dP.toArray(new Point[dP.size()]);
//
//		
//
//		/////////////////////////////
//
//		
//	}
//	
//	public void run(){
//		
//		// <-------------------------ALEX----------------------->
//
//				// BlockingQueue<Byte[]> queue = new LinkedBlockingQueue<>();
//				aStar = new AStar(map);
//
//				jobDistancesMap = aStar.calculateJobDistances((ConcurrentHashMap<Integer, JobList>) jobMap, startpoint,
//						dropPointsArray);
//
//				jobSelection = new JobSelection((ConcurrentHashMap<Integer, JobList>) jobMap, jobDistancesMap,
//						startpoint, dropPointsArray);
//				bestJob = jobSelection.getJob();
//				jobsToDisplay.add(bestJob);
//				
//				
//		while (bestJob != null) {
//
//			// for (int i = 0; i < bestJob.size(); i++)
//			// {
//			// System.out.println(bestJob.getJobItem(i).getX() + " " +
//			// bestJob.getJobItem(i).getY());
//			// }
//
//			// System.out.println("Drop-offs:");
//
//			// System.out.println(dropPoints);
//
//			Point[] pickpoints = new Point[bestJob.size()];
//
//			double[] weights = new double[bestJob.size()];
//			for (int i = 0; i < bestJob.size(); i++) {
//				pickpoints[i] = new Point(bestJob.getJobItem(i).getX(), bestJob.getJobItem(i).getY());
//				weights[i] = bestJob.getJobItem(i).getWeight();
//			}
//
//			TSP tsp = new TSP(jobSelection.getJobDistances(), startpoint, pickpoints, dropPointsArray, weights);
//
//			ArrayList<Point> pathPoints = tsp.getPathPoints();
//			ArrayList<Integer> pointsItemNumber = new ArrayList<>();
//
//			ArrayList<String> pointsName = new ArrayList<>();
//
//			for (int p = 1; p < pathPoints.size() - 1; p++) {
//				for (int pp = 0; pp < pickpoints.length; pp++) {
//					if (pathPoints.get(p).equals(pickpoints[pp])) {
//						pointsItemNumber.add(bestJob.getJobNum(pp));
//						pointsName.add(bestJob.getJobItem(pp).getID());
//
//						// System.out.println(pathPoints.get(p) + " " +
//						// bestJob.getJobNum(pp));
//
//						break;
//					}
//				}
//			}
//
//			// <-------------------------ROUTE----------------------->
//			aStar.sendPath(pathPoints.toArray(new Point[pathPoints.size()]), facing, pointsName, pointsItemNumber);
//
//			jobSelection = new JobSelection(jobSelection.getNewJobMap(), jobSelection.getNewJobDistancesMap(),
//					pathPoints.get(pathPoints.size() - 1), dropPointsArray);
//			bestJob = jobSelection.getJob();
//			jobsToDisplay.add(bestJob);
//		}
//	}
//	
//	public ArrayList<JobList> getJobsToDisplay(){
//		return jobsToDisplay;
//	}
//	
//
//}
//>>>>>>> fcc0b992c749beabde1421cebb97a0001e7f566b
