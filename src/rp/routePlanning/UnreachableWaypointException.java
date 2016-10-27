package rp.routePlanning;

/**
 * Exception that notifies of the fact that a certain waypoint is out of reach.
 *
 */
@SuppressWarnings("serial")
public class UnreachableWaypointException extends Exception
{
	public UnreachableWaypointException(String message)
	{
		super(message);
	}
}
