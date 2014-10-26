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
	
	private Vehicle vehicle;
	private ArrayList<City> packLocation = new ArrayList<City> ();
	private TaskSet tasks;
	
	public State(Vehicle vehicle, ArrayList<City> packLocation) {
		this.packLocation = packLocation;
	};
	
	
	/*overload, this is used to create a new initial state*/		
	public State(Vehicle vehicle, TaskSet tasks) {

		this.vehicle = vehicle;

		/*Initialize the arrayList to store the location*/
		for (int i=0; i < tasks.size(); i++) {
			packLocation.add(null);
		}
		
		for (Task task : tasks) {
			packLocation.add(task.pickupCity.id, task.pickupCity);
		}
		this.tasks = tasks;
	}
	
	public State move(City city) {
		
		State state = new State(vehicle, this.packLocation);
		 = city;
		
		return State
	}
	
	public State delivery(pdAction action) {
		
	}
	
	public State pickup(pdAction action) {}
			
			
	}
}
