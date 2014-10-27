package template;
/* import table */
import logist.simulation.Vehicle;
import logist.agent.Agent;
import logist.behavior.DeliberativeBehavior;
import logist.plan.Plan;
import logist.task.Task;
import logist.task.TaskDistribution;
import logist.task.TaskSet;
import logist.topology.Topology;
import logist.topology.Topology.City;

import java.util.ArrayList;
import java.util.HashSet;

public class State {
	
	int capacity;
	long cost;
	public City currentCity;
	public HashSet<Integer>	carriedTasks = new HashSet<Integer>();
	public HashSet<Integer> deliveredTasks = new HashSet<Integer> ();
	public Plan plan;
}
