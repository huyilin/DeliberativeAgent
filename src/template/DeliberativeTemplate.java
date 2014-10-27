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
	HashMap<Integer, HashSet<Task>> taskMap = new HashMap<Integer, HashSet<Task>> ();
	
	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		
		this.topology = topology;
		this.td = td;
		this.agent = agent;
		this.carriedTasks = null;
		
		// initialize the planner
		int capacity = agent.vehicles().get(0).capacity();
		String algorithmName = agent.readProperty("algorithm", String.class, "ASTAR");
		
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
		State initalState = initiateState(vehicle, tasks, plan);
		stateQueue.add(initalState);
		this.taskMap = null;
		for(City city : topology.cities()) {
			taskMap.put(city.id, new HashSet<Task>());
		}
		
		for(Task task : tasks) {
			taskMap.get(task.pickupCity.id).add(task);
		}
		
		while(!stateQueue.isEmpty()) {
			State state = stateQueue.remove();
			for(State nextState: this.nextStates(state)) {
				stateQueue.add(nextState);
			}
		}
		
		
		return plan;
	}
	
	private State initiateState (Vehicle vehicle, TaskSet tasks, Plan plan) {
		State state = new State();
		state.plan = plan;
		state.currentCity = vehicle.getCurrentCity().id;
		return state;
	}
	
	private int min(int i, int j) {
		if (i >= j) return j;
		else return i;
	}
	
	private HashSet<State> nextStates(State state) {
		
		HashSet<State> nextStates = new HashSet<State> ();
		HashSet<Task> currentTasks = taskMap.get(state.currentCity);
		
		Iterator<Task> iterator = currentTasks.iterator();
		while (iterator.hasNext()) {
			int id = iterator.next().id;
			if(state.deliveredTasks.contains(id) || state.carriedTasks.contains(id)) 
				iterator.remove();
		}
		
		if(!state.ifFull && currentTasks.size() > 0) {
			int available = min(currentTasks.size(), capacity - state.carriedTasks.size());
			for(Task task : currentTasks) {
				for(int toTake = 1; toTake <= available; toTake++) {
					nextStates.addAll(newTake(currentTasks, toTake, state));
				}
			}
		}
		return null;
	}
	
	HashSet<State> newTake(HashSet<Task> currentTasks, int toTake,
			State prestate) {
		HashSet<State> returnStates = new HashSet<State> (); 
		if( toTake == 1) {
			for(Task task : currentTasks) {
				State state = new State();
				state = prestate;
				state.carriedTasks.add(task.id);
				state.plan.appendPickup(task);
				returnStates.add(state);
			}
		} else {
			for(Task task : currentTasks) {
				State state = new State();
				state.carriedTasks.add(task.id);
				state.plan.appendPickup(task);
				HashSet<Task> deductTasks = new HashSet<Task> ();
				deductTasks = currentTasks;
				deductTasks.remove(task);
				returnStates.addAll(newTake(deductTasks));
				
			}
			state = new state(
		}
		
		State returnState = new State();
		
		returnState = state;
		for(Task task : currentTList) {
			if(taked.size() == toTake) {
				return taked;
			}
			
			if(!taked.contains(task.id) && taked.size() < toTake) {
				state.add(task.id);
				toTake --;
				return newTake(currentTasks, toTake, taked);
			}
		}
		
		return null;
	}
	
	private void constructGoal(TaskSet tasks) {
		for (Task task : tasks) {
		 // to construct the goal state	
		}
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
