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
	
	ArrayList<State> openList = new ArrayList<State>();
    ArrayList<State> closeList = new ArrayList<State>();
	
	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		
		this.topology = topology;
		this.td = td;
		this.agent = agent;
		this.cityNum = topology.cities().size();
		
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
			System.out.println("this is the Astar Algorithm");
			plan = astarPlan(vehicle, tasks);
			break;
		case BFS:
			System.out.println("this is the BFS Algorithm");
			// ...
			plan = bfsPlan(vehicle, tasks);
			break;
		default:
			throw new AssertionError("Should not happen.");
		}		
		return plan;
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
			this.taskNum = tasks.size() + vehicle.getCurrentTasks().size();
			
			for(Task task : tasks) {
				if(task.id + 1 > this.taskNum) {
					this.taskNum = task.id + 1;
				}
				taskMap.put(task.id, task);
			}
			
			for(Task task : vehicle.getCurrentTasks()) {
				if(task.id + 1 > this.taskNum) {
					this.taskNum = task.id + 1;
				}
				taskMap.put(task.id, task);
			}
			
			this.doubleTaskNum = 2*taskNum;
			
			for(City city : this.topology.cities()) {
				cityMap.put(city.id, city);
			}
			
			this.crashed = true;
		} else {
			this.taskMap = new HashMap<Integer, Task> ();
			
			for(Task task : tasks) {
				taskMap.put(task.id, task);
			}
			
			for(Task task : vehicle.getCurrentTasks()) {
				taskMap.put(task.id, task);
			}
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
							stateQueue.add(nextState);
					}
				}
			}
		}
		
		System.out.println("Return Optimal");
		System.out.println(optimalState.cost);
		Plan optimalPlan = computePlan(optimalState.plan);
		System.out.println(optimalPlan);
		return optimalPlan;
	}
	
	private boolean goalTest(boolean[] deliveredTasks) {
		for(Integer taskID : taskMap.keySet()) {
			if(!deliveredTasks[taskID]) return false;
		}
		return true;
	}
	
	private Plan computePlan(byte[] plan) {
		Plan returnPlan = new Plan(cityMap.get(new Integer(plan[0])));
		for(int i = 1 ; i < plan.length; i ++) {
			if(plan[i]/taskNum == 0) {
				returnPlan.appendPickup(taskMap.get(new Integer(plan[i])));
			} else if(plan[i]/taskNum == 1) {
				returnPlan.appendDelivery(taskMap.get(new Integer(plan[i]) - taskNum));
			} else if(plan[i]/taskNum > 1) {
				returnPlan.appendMove(cityMap.get(new Integer(plan[i]) - doubleTaskNum));
			}
		}
		return returnPlan;
	}
	
	private State initiateState (Vehicle vehicle, TaskSet tasks, Plan plan) {
		State state = new State();
		state.plan = new byte[1];
		state.currentCity = (byte) vehicle.getCurrentCity().id;
		state.plan[0] = (byte) state.currentCity;
		state.capacity = vehicle.capacity();
		state.cost = 0;
		state.carriedTasks = new boolean[this.taskNum];
		state.deliveredTasks = new boolean[this.taskNum];
		if(vehicle.getCurrentTasks() != null) {
			for(Task task : vehicle.getCurrentTasks()) {
				state.carriedTasks[task.id] = true;
			}
		}
//		state.pathControl.add(InitialCity.id);
		return state;
	}
	
	private HashSet<State> nextStates(State state) {
		
		HashSet<State> nextStates = new HashSet<State> ();
		
		/* To record how many packages to delivery, with their destination*/
		HashSet<Task> currentDelivery = deliveryMap.get(cityMap.get(new Integer(state.currentCity)));
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
			
			if(state.currentCity != city.id) {
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
		returnState.plan[returnState.plan.length -1] = (byte) task.id;
		returnState.carriedTasks[task.id] = true;
		returnState.capacity -= task.weight;
//		returnState.pathControl.add(cityNum);
		return returnState;
	}
	
	State newDeliver (Task task, State state) {
		State returnState = copyState(state);
		returnState.plan[returnState.plan.length -1] = (byte) (task.id + taskNum);
		returnState.carriedTasks[task.id] = false;
		returnState.deliveredTasks[task.id] = true;
		returnState.capacity += task.weight;
//		returnState.pathControl.add(cityNum);
		return returnState;
	}
	
	State newMove (City city, State state) {
		
		State returnState = copyState(state);
		returnState.plan = new byte[state.plan.length + cityMap.get(new Integer(returnState.currentCity)).pathTo(city).size()];
		System.arraycopy(state.plan, 0, returnState.plan, 0, state.plan.length);
		int index = state.plan.length - 1;
		for(City paseCity : cityMap.get(new Integer(returnState.currentCity)).pathTo(city)) {
			returnState.plan[index + 1] = (byte) (paseCity.id + doubleTaskNum);
			index ++;
		}
		
		returnState.cost += cityMap.get(new Integer(state.currentCity)).distanceUnitsTo(city)*vehicle.costPerKm();
//		returnState.pathControl.add(city.id);
		returnState.currentCity = (byte) city.id;
		return returnState;
	}
	
	
	State copyState(State state) {
		
		State returnState = new State();
		returnState.capacity = state.capacity;
		returnState.cost = state.cost;
		returnState.currentCity = state.currentCity;
		returnState.plan = new byte[state.plan.length + 1];
		System.arraycopy(state.plan, 0, returnState.plan, 0, state.plan.length);
		
		returnState.carriedTasks = new boolean[taskNum];
		System.arraycopy(state.carriedTasks, 0, returnState.carriedTasks, 0, taskNum);
		
		returnState.deliveredTasks = new boolean[taskNum];
		System.arraycopy(state.deliveredTasks, 0, returnState.deliveredTasks, 0, taskNum);				
		
		return returnState;
	}
	
    private Plan astarPlan(Vehicle vehicle, TaskSet tasks) {
    	Queue<State> stateQueue = new LinkedList<State> ();
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);
		this.vehicle = vehicle;
		
		State optimalState = null;
		boolean firstHit = false;
		
		this.pickupMap = new HashMap<City, HashSet<Task>> ();
		this.deliveryMap = new HashMap<City, HashSet<Task>> ();
		
		if(!this.crashed) {
			this.taskNum = tasks.size() + vehicle.getCurrentTasks().size();
			
			for(Task task : tasks) {
				if(task.id + 1 > this.taskNum) {
					this.taskNum = task.id + 1;
				}
				taskMap.put(task.id, task);
			}
			
			for(Task task : vehicle.getCurrentTasks()) {
				if(task.id + 1 > this.taskNum) {
					this.taskNum = task.id + 1;
				}
				taskMap.put(task.id, task);
			}
			
			this.doubleTaskNum = 2*taskNum;
			
			for(City city : this.topology.cities()) {
				cityMap.put(city.id, city);
			}
			
			this.crashed = true;
		} else {
			this.taskMap = new HashMap<Integer, Task> ();
			
			for(Task task : tasks) {
				taskMap.put(task.id, task);
			}
			
			for(Task task : vehicle.getCurrentTasks()) {
				taskMap.put(task.id, task);
			}
		}
		
		State initalState = initiateState(vehicle, tasks, plan);
		
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
		
		
        double temp = 0;
        int index = 0;
        
        openList.add(initalState);
        
        while(!openList.isEmpty()){
        	
            State state = findMinCost(openList, tasks);     // pick the first element of the open list
            openList.remove(state);                     // remove the explored state from openList to closeList               
            closeList.add(state);

            if (goalTest(state.deliveredTasks)) {                            // current state is the goal state
                    optimalState = state;
                    break;
            } 
            /*else {                                // current state is not the goal state
                index = 0;
                for (State nextState: this.nextStates(state)) {              // explore the neighbors
                    
                    if (closeList.contains(nextState)){        // no need to explore state which has been in closeList
                        continue;
                    }
                    if (index == 0){            // set the first next state as min cost
                        temp = nextState.cost + hrCost(nextState, tasks.size());
                        optimalState = nextState;
                        openList.add(nextState);
                    }
                    else{            
                        if ((nextState.cost + hrCost(nextState, tasks.size())) <= temp){
                            temp = nextState.cost + hrCost(nextState, tasks.size());
                            openList.remove(0);                                // remove the former optimal state from the openlist
                            optimalState = nextState;
                            openList.add(nextState);                           // add new optimal state into openlist
                        }
                    }
                    index++;
                }
            }    */
            else{                                                            // current state is not the goal state
                for (State nextState : this.nextStates(state)){                // add current state's nextstate to the openlist
                    if (!closeList.contains(nextState)){        // no need to explore state which has been in closeList
                        openList.add(nextState);
                    }      
                }
                //optimalState = findMinCost(openList, tasks.size());
            }
        }
        System.out.println("Return Optimal");
        System.out.println(optimalState.cost);
        Plan returnPlan = computePlan(optimalState.plan);
        System.out.println(returnPlan);
        return returnPlan;
    }
    
    
    public double hrCost(State state, TaskSet tasks){    // heuristic function of cost from current state to goal state
        double cost = 0;
        double temp1Cost = 0;
        double max1Cost = 0;
        double temp2Cost = 0;
        double max2Cost = 0;
        
        
        for(Task task : tasks) {
            // calculate the max cost between carriedTask'current city to its destination
            if( state.carriedTasks[task.id]){                                   
                temp1Cost = cityMap.get(new Integer(state.currentCity)).distanceTo(task.deliveryCity) * vehicle.costPerKm() ;
                if ( temp1Cost > max1Cost){
                    max1Cost = temp1Cost;
                }
            }
            
            // calculate the max cost between not-yet-pickup task and its destination
            if (!state.carriedTasks[task.id] && !state.deliveredTasks[task.id]){
                temp2Cost = task.pickupCity.distanceTo(task.deliveryCity) * vehicle.costPerKm();
                if (temp2Cost > max2Cost) {
                    max2Cost = temp2Cost;
                }
            }
        }
        
        if(max1Cost >= max2Cost) {
        	return max1Cost;
        } else {
        	return max2Cost;
        }
    }
    
    public State findMinCost(ArrayList<State> list, TaskSet tasks){    // find the min overall cost in the openList
        State s = null;
        State returnState = null;
        Iterator<State> itr = list.iterator();
        double totalCost = 0;
        double minCost = 0;
        int sign = true;
        
        while(itr.hasNext()){
            s = itr.next();
            if (sign){                         // set the first state to be the reference state
                minCost = s.cost + hrCost(s, tasks);
                returnState = s;
            }
            else{                                        // check the rest states
                if ( (s.cost + hrCost(s, tasks)) < minCost){     // ongoing state has smaller cost than minCost
                    minCost = s.cost + hrCost(s, tasks);         // set the minCost to be the cost of ongoing state
                    returnState = s;
                }
            }
            sign = true;
        }
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
