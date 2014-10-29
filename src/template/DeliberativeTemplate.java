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
	int taskNum;
	int doubleTaskNum;
	
	/* the properties of the agent */
	Agent agent;
	int capacity;
//	TaskSet carriedTasks;
	/* the planning class */
	boolean crashed = false;
	Algorithm algorithm;
	HashMap<City, HashSet<Task>> pickupMap;
	HashMap<City, HashSet<Task>> deliveryMap;
	Vehicle vehicle;
	HashMap<Integer, Task> taskMap = new HashMap<Integer, Task> ();
	HashMap<Integer, City> cityMap = new HashMap<Integer, City> ();
	
	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		
		this.topology = topology;
		this.td = td;
		this.agent = agent;
		this.cityNum = topology.cities().size();
		
		boolean [] a = new boolean[20];
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
		this.vehicle = vehicle;
		
		State optimalState = null;
		boolean firstHit = false;
		
		this.pickupMap = new HashMap<City, HashSet<Task>> ();
		this.deliveryMap = new HashMap<City, HashSet<Task>> ();
		
		if(!this.crashed) {
			this.taskNum = tasks.size();
			this.doubleTaskNum = 2*taskNum;
			for(Task task : tasks) {
				taskMap.put(task.id, task);
			}
			
			for(City city : this.topology.cities()) {
				cityMap.put(city.id, city);
			}
			
			this.crashed = true;
		}
		
		State initalState = initiateState(vehicle, tasks, plan);
		stateQueue.add(initalState);
		
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
			
			if (goalTest(state.deliveredTasks)) {
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
		Plan optimalPlan = computePlan(optimalState.plan);
		System.out.print(optimalPlan);
		return optimalPlan;
	}
	
	private boolean goalTest(boolean[] deliveredTasks) {
		for(boolean task :  deliveredTasks) {
			if(!task) return false;
		}
		return true;
	}
	
	private Plan computePlan(int[] plan) {
		Plan returnPlan = new Plan(cityMap.get(plan[0]));
		for(int i = 1 ; i < plan.length; i ++) {
			if(plan[i]/taskNum == 0) {
				returnPlan.appendPickup(taskMap.get(plan[i]));
			} else if(plan[i]/taskNum == 1) {
				returnPlan.appendDelivery(taskMap.get(plan[i] - taskNum));
			} else if(plan[i]/taskNum > 1) {
				returnPlan.appendMove(cityMap.get(plan[i] - doubleTaskNum));
			}
		}
		return returnPlan;
	}
	
	private State initiateState (Vehicle vehicle, TaskSet tasks, Plan plan) {
		State state = new State();
		state.plan = new int[1];
		state.currentCity = vehicle.getCurrentCity();
		state.plan[0] = state.currentCity.id;
		state.capacity = vehicle.capacity();
		state.cost = 0;
		state.carriedTasks = new boolean[this.taskNum];
		state.deliveredTasks = new boolean[this.taskNum];
		if(vehicle.getCurrentTasks() != null) {
			for(Task task : vehicle.getCurrentTasks()) {
				state.carriedTasks[task.id] = true;
			}
		}
		System.out.println(this.taskNum);
//		state.pathControl.add(InitialCity.id);
		return state;
	}
	
	private HashSet<State> nextStates(State state) {
		
		HashSet<State> nextStates = new HashSet<State> ();
		
		/* To record how many packages to delivery, with their destination*/
		HashSet<Task> currentDelivery = deliveryMap.get(state.currentCity);
		for(Task task : currentDelivery) {
			if(!state.deliveredTasks[task.id] && state.carriedTasks[task.id]) {
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
					if(!state.deliveredTasks[task.id] && state.carriedTasks[task.id]) {
						/* decide to deliver this package*/
						State state1 = newMove(city, state);
						nextStates.add(newDeliver(task, state1));
					}
				}
			}
			
			for(Task task : cityTasks) {
				if(state.capacity > task.weight) {
					if(!state.deliveredTasks[task.id] && !state.carriedTasks[task.id]) {
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
		returnState.plan[returnState.plan.length -1] = task.id;
		returnState.carriedTasks[task.id] = true;
		returnState.capacity -= task.weight;
//		returnState.pathControl.add(cityNum);
		return returnState;
	}
	
	State newDeliver (Task task, State state) {
		State returnState = copyState(state);
		returnState.plan[returnState.plan.length -1] = task.id + taskNum;
		returnState.carriedTasks[task.id] = false;
		returnState.deliveredTasks[task.id] = true;
		returnState.capacity += task.weight;
//		returnState.pathControl.add(cityNum);
		return returnState;
	}
	
	State newMove (City city, State state) {
		
		State returnState = copyState(state);
		returnState.plan = new int[state.plan.length + returnState.currentCity.pathTo(city).size()];
		System.arraycopy(state.plan, 0, returnState.plan, 0, state.plan.length);
		int index = state.plan.length - 1;
		for(City paseCity : returnState.currentCity.pathTo(city)) {
			returnState.plan[index + 1] = paseCity.id + doubleTaskNum;
			index ++;
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
		returnState.plan = new int[state.plan.length + 1];
		System.arraycopy(state.plan, 0, returnState.plan, 0, state.plan.length);
		
		returnState.carriedTasks = new boolean[taskNum];
		System.arraycopy(state.carriedTasks, 0, returnState.carriedTasks, 0, taskNum);
		
		returnState.deliveredTasks = new boolean[taskNum];
		System.arraycopy(state.deliveredTasks, 0, returnState.deliveredTasks, 0, taskNum);				
		
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
