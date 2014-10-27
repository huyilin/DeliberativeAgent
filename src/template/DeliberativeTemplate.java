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
	
	/* the properties of the agent */
	Agent agent;
	int capacity;
	TaskSet carriedTasks;
	/* the planning class */
	Algorithm algorithm;
	HashMap<City, HashSet<Task>> pickupMap = new HashMap<City, HashSet<Task>> ();
	HashMap<City, HashSet<Task>> deliveryMap = new HashMap<City, HashSet<Task>> ();
	City InitialCity;
	Vehicle vehicle;
	
	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		
		this.topology = topology;
		this.td = td;
		this.agent = agent;
		this.carriedTasks = null;
	
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
		
		for(City city : topology.cities()) {
			pickupMap.put(city, new HashSet<Task>());
			deliveryMap.put(city, new HashSet<Task> ());
		}
		
		for(Task task : tasks) {
			pickupMap.get(task.pickupCity).add(task);
			pickupMap.get(task.deliveryCity).add(task);
		}
		
		while(!stateQueue.isEmpty()) {
			State state = stateQueue.remove();
			if(state.deliveredTasks.size() == tasks.size() && state.cost < optimalState.cost) {
				optimalState = state;
			}
			
			System.out.println("Here");
			System.out.println("Here");
			System.out.println("Here");
			System.out.println("Here");
			
			for(State nextState: this.nextStates(state)) {
				stateQueue.add(nextState);
			}
		}
		return optimalState.plan;
	}
	
	private State initiateState (Vehicle vehicle, TaskSet tasks, Plan plan) {
		State state = new State();
		state.plan = plan;
		state.currentCity = vehicle.getCurrentCity();
		state.capacity = vehicle.capacity();
		return state;
	}
	
	private int min(int i, int j) {
		if (i >= j) return j;
		else return i;
	}
	
	private HashSet<State> nextStates(State state) {
		
		HashSet<State> nextStates = new HashSet<State> ();
		
		HashSet<Task> currentTasks = pickupMap.get(state.currentCity);
		
		/* To record how many packages to delivery, with their destination*/
		HashSet<Task> currentDelivery = deliveryMap.get(state.currentCity);
		
//		Iterator<Task> iterator = currentTasks.iterator();
		
//		while (iterator.hasNext()) {
//			int id = iterator.next().id;
//			if(state.deliveredTasks.contains(id) || state.carriedTasks.contains(id))
//				iterator.remove();
//		}
		
		System.out.println(currentTasks);
		
		if(currentTasks!=null) {
//			int available = min(currentTasks.size(), capacity - state.carriedTasks.size());
			for(Task task : currentTasks) {
//				for(int toTake = 1; toTake <= available; toTake++) {
//					nextStates.addAll(newTake(currentTasks, toTake, state));
//				}
				/*the for loop runs only one time, which means only one decision was made at one time */
				if(state.capacity > task.weight) {
					if(!state.deliveredTasks.contains(task.id) && !state.carriedTasks.contains(task.id));
					nextStates.add(newTake(task, state));
					currentTasks.remove(task);
					break;
				}
			}
		}
		
		if(state.carriedTasks.size() > 0 && currentDelivery.size() > 0 ) {
			for(Task task : currentDelivery) {
				if(!state.deliveredTasks.contains(task.id) && state.carriedTasks.contains(task.id))
				/* decide to deliver this pakage*/
				nextStates.add(newDeliver(task, state));
				break;
			}
		}
		
		for(City neighbor : state.currentCity.neighbors()) {
			nextStates.add(newMove(neighbor, state));
		}
		
		return null;
	}
	
	
	State newTake(Task task, State state) {
		State returnState = copyState(state);
		returnState.plan.appendPickup(task);
		returnState.carriedTasks.add(task.id);
		returnState.capacity -= task.weight;
		return returnState;
	}
	
	
	State newDeliver (Task task, State state) {
		State returnState = copyState(state);
		returnState.plan.appendDelivery(task);
		returnState.carriedTasks.remove(task.id);
		returnState.deliveredTasks.add(task.id);
		returnState.capacity += task.weight;
		return returnState;
	}
	
	State newMove (City neighbor, State state) {
		State returnState = copyState(state);
		returnState.currentCity = neighbor;
		returnState.plan.appendMove(neighbor);
		returnState.cost += state.currentCity.distanceUnitsTo(neighbor)*vehicle.costPerKm();
		return returnState;
	}
	
	
	State copyState(State state) {
		
		State returnState = new State();
		returnState.capacity = state.capacity;
		returnState.cost = state.cost;
		returnState.currentCity = state.currentCity;
		
		Plan plan = new Plan(InitialCity);
		
		
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
	
//	HashSet<State> newTake(HashSet<Task> currentTasks, int toTake,
//			State prestate) {
//		HashSet<State> returnStates = new HashSet<State> (); 
//		if( toTake == 1) {
//			for(Task task : currentTasks) {
//				State state = new State();
//				state = prestate;
//				state.carriedTasks.add(task.id);
//				state.plan.appendPickup(task);
//				returnStates.add(state);
//			}
//		} else {
//			for(Task task : currentTasks) {
//				State state = new State();
//				state.carriedTasks.add(task.id);
//				state.plan.appendPickup(task);
//				HashSet<Task> deductTasks = new HashSet<Task> ();
//				deductTasks = currentTasks;
//				deductTasks.remove(task);
//				returnStates.addAll(newTake(deductTasks));
//				
//			}
//			state = new state(
//		}
//		
//		State returnState = new State();
//		
//		returnState = state;
//		for(Task task : currentTList) {
//			if(taked.size() == toTake) {
//				return taked;
//			}
//			
//			if(!taked.contains(task.id) && taked.size() < toTake) {
//				state.add(task.id);
//				toTake --;
//				return newTake(currentTasks, toTake, taked);
//			}
//		}
//		
//		return null;
//	}

	@Override
	public void planCancelled(TaskSet carriedTasks) {
		if (!carriedTasks.isEmpty()) {
			// This cannot happen for this simple agent, but typically
			// you will need to consider the carriedTasks when the next
			// plan is computed.
		}
	}
}
