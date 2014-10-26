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
	
	private City currentCity;
	private ArrayList<City> packLocation = new ArrayList<City> ();
	private TaskSet tasks;
	
	public State(ArrayList<City> packLocation) {
		this.packLocation = packLocation;
	};
	
	
	/*overload, this is used to create a new initial state*/		
	public State(City initialCity, TaskSet tasks) {

		currentCity = initialCity;

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
		State state = new State(this.packLocation);
		state.currentCity = city;
		
		
		return State
	}
	
	public State delivery(pdAction action) {
		
	}
	
	public State pickup(pdAction action) {}
			
			
	}
	
	public boolean identical(State s){
		boolean result = false;
		int count = 0;
		for (int i = 0; i < 6; i++){
			if (this.packLocation.get(i) == s.packLocation.get(i))
				count ++;
		}
		if (count == 6){
			result = true;
		}
		
		return result;
	}
}