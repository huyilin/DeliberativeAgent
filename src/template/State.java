package template;
/* import table */
import logist.plan.Plan;
import logist.topology.Topology.City;

import java.util.HashSet;

public class State {
	
	int capacity;
	long cost;
	public City currentCity;
	public	HashSet<Integer> carriedTasks = new HashSet<Integer>();
	public HashSet<Integer> deliveredTasks = new HashSet<Integer> ();
	public Plan plan;
}
