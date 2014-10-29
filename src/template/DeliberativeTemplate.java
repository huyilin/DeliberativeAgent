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

import java.util.Iterator;
import java.util.List;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Queue;
import logist.plan.Action;

/**
 * An optimal planner for one vehicle.
 */
@SuppressWarnings("unused")
public class DeliberativeTemplate implements DeliberativeBehavior {
	
	
	enum Algorithm { BFS, ASTAR }
	
	/* Environment */
	Topology topology;
	TaskDistribution td;
	int cityNum;
	
	/* the properties of the agent */
	Agent agent;
	int capacity;
//	TaskSet carriedTasks;
	/* the planning class */
	Algorithm algorithm;
	HashMap<City, HashSet<Task>> pickupMap;
	HashMap<City, HashSet<Task>> deliveryMap;
	City InitialCity;
	Vehicle vehicle;
	
	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		
		this.topology = topology;
		this.td = td;
		this.agent = agent;
		this.cityNum = topology.cities().size();
	
		// initialize the planner
		int capacity = agent.vehicles().get(0).capacity();
		String algorithmName = agent.readProperty("algorithm", String.class, "BFS");
		
		// Throws IllegalArgumentException if algorithm is unknown
		algorithm = Algorithm.valueOf(algorithmName.toUpperCase());
		
		// ...
		
	}
	
	@Override
	public Plan plan(Vehicle vehicle, TaskSet tasks) {
		Plan plan;
		// Compute the plan with the selected algorithm.
		switch (algorithm) {
		case ASTAR:
			// ...
			plan = naivePlan(vehicle, tasks);
			break;
		case BFS:
			// ...
			plan = bfsPlan(vehicle, tasks);
			break;
		default:
			throw new AssertionError("Should not happen.");
		}		
		return plan;
	}
	
	private Plan naivePlan(Vehicle vehicle, TaskSet tasks) {
		return null;
	}
	
	private Plan bfsPlan(Vehicle vehicle, TaskSet tasks) {
		
		Queue<State> stateQueue = new LinkedList<State> ();
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);
		this.InitialCity = current;
		this.vehicle = vehicle;
		State initalState = initiateState(vehicle, tasks, plan);
		stateQueue.add(initalState);
		State optimalState = null;
		boolean firstHit = false;
		this.pickupMap = new HashMap<City, HashSet<Task>> ();
		this.deliveryMap = new HashMap<City, HashSet<Task>> ();
		
		for(City city : topology.cities()) {
			pickupMap.put(city, new HashSet<Task>());
			deliveryMap.put(city, new HashSet<Task> ());
		}
		
		for(Task task : tasks) {
			pickupMap.get(task.pickupCity).add(task);
			deliveryMap.get(task.deliveryCity).add(task);
		}
		
		for(Task task : vehicle.getCurrentTasks()) {
			deliveryMap.get(task.deliveryCity).add(task);
		}
		
		while (!stateQueue.isEmpty()) {
			State state = stateQueue.remove();
			
			if (state.deliveredTasks.size() == tasks.size() + vehicle.getCurrentTasks().size()) {
				if (!firstHit) {
					optimalState = state;
					firstHit = true;
				} else if (state.cost < optimalState.cost) {
					optimalState = state;
				}
			} else {
				for (State nextState: this.nextStates(state)) {
					if (!firstHit  || state.cost < optimalState.cost) {
//						if (!hasCircle(nextState)) {
							stateQueue.add(nextState);
//						}
					}
				}
			}
		}
		System.out.println("Return Optimal");
		System.out.println(optimalState.cost);
		System.out.println(optimalState.plan);
		return optimalState.plan;
	}
	
//	private boolean hasCircle(State state) {
//		ArrayList<Integer> path = state.pathControl;
//		if(path.size() < 2) {
//			return false;
//		}
//		int tail = path.get(path.size() -1);
//		if(tail ==  cityNum) return false;		
//		for(int i = path.size() -2; i>= 0; i--) {
//			if(path.get(i) == cityNum) {
//				return false;
//			} else{
//				if(path.get(i) == tail) return true;
//			} 	
//		}		
//		return false;
//	}
	
	private State initiateState (Vehicle vehicle, TaskSet tasks, Plan plan) {
		State state = new State();
		state.plan = plan;
		state.currentCity = vehicle.getCurrentCity();
		state.capacity = vehicle.capacity();
		state.cost = 0;
		if(vehicle.getCurrentTasks() != null) {
			for(Task task : vehicle.getCurrentTasks()) {
				state.carriedTasks.add(task.id);
			}
		}
//		state.pathControl.add(InitialCity.id);
		return state;
	}
	
	private HashSet<State> nextStates(State state) {
		
		HashSet<State> nextStates = new HashSet<State> ();
		
		/* To record how many packages to delivery, with their destination*/
		HashSet<Task> currentDelivery = deliveryMap.get(state.currentCity);
		for(Task task : currentDelivery) {
			if(!state.deliveredTasks.contains(task.id) && state.carriedTasks.contains(task.id)) {
			/* decide to deliver this package*/
				nextStates.add(newDeliver(task, state));
			}
		}
		
		if(nextStates.size() > 0) {
			return nextStates;
		}
		
		for(City city : topology.cities()) {
			HashSet<Task> cityTasks = pickupMap.get(city);
			HashSet<Task> cityDelivery = deliveryMap.get(city);
			
			if(state.currentCity.id != city.id) {
				for(Task task : cityDelivery) {
					if(!state.deliveredTasks.contains(task.id) && state.carriedTasks.contains(task.id)) {
						/* decide to deliver this package*/
						State state1 = newMove(city, state);
						nextStates.add(newDeliver(task, state1));
					}
				}
			}
			
			for(Task task : cityTasks) {
				if(state.capacity > task.weight) {
					if(!state.deliveredTasks.contains(task.id) && !state.carriedTasks.contains(task.id)) {
						State state2 = newMove(city, state);
						nextStates.add(newTake(task, state2));
					}
				}
			}
		}
		return nextStates;
	}
	
	State newTake(Task task, State state) {
		State returnState = copyState(state);
		returnState.plan.appendPickup(task);
		returnState.carriedTasks.add(task.id);
		returnState.capacity -= task.weight;
//		returnState.pathControl.add(cityNum);
		return returnState;
	}
	
	State newDeliver (Task task, State state) {
		State returnState = copyState(state);
		returnState.plan.appendDelivery(task);
		returnState.carriedTasks.remove(task.id);
		returnState.deliveredTasks.add(task.id);
		returnState.capacity += task.weight;
//		returnState.pathControl.add(cityNum);
		return returnState;
	}
	
	State newMove (City city, State state) {
		State returnState = copyState(state);
		
		for(City paseCity : returnState.currentCity.pathTo(city)) {
			returnState.plan.appendMove(paseCity);
		}
		
		returnState.cost += state.currentCity.distanceUnitsTo(city)*vehicle.costPerKm();
//		returnState.pathControl.add(city.id);
		returnState.currentCity = city;
		return returnState;
	}
	
	
	State copyState(State state) {
		
		State returnState = new State();
		returnState.capacity = state.capacity;
		returnState.cost = state.cost;
		returnState.currentCity = state.currentCity;
		
		Plan plan = new Plan(InitialCity);
		
//		for(int i : state.pathControl) {
//			returnState.pathControl.add(i);
//		}
		
		for(int taskID : state.carriedTasks) {
			returnState.carriedTasks.add(taskID);
		}
		
		for(int taskID : state.deliveredTasks) {
			returnState.deliveredTasks.add(taskID);
		}
		
		Iterator<Action> iter = state.plan.iterator();
		
		while(iter.hasNext()) {
			plan.append(iter.next());
		}
		returnState.plan = plan;
		return returnState;
	}
	
	@Override
	public void planCancelled(TaskSet carriedTasks) {
		if (!carriedTasks.isEmpty()) {
			// This cannot happen for this simple agent, but typically
			// you will need to consider the carriedTasks when the next
			// plan is computed.
		}
	}
}
