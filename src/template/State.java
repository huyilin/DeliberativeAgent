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

public class State {
	Plan plan;
	public boolean ifFull = false;
	public int currentCity;
	public	ArrayList<Integer>	carriedTasks = new ArrayList<Integer>();
	public ArrayList<Integer> deliveredTasks = new ArrayList<Integer> ();
	
}
