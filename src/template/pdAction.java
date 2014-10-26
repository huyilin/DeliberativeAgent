package template;
import logist.topology.Topology.City;

public class pdAction {
	boolean iftake;
	City nextCity;
	String key;
	
	public pdAction() {
		this.iftake = true;
		this.nextCity = null;
		key = Integer.toString(-1);
	}
	
	public pdAction(City city) {
		this.iftake = false;
		this.nextCity = city;
		this.key = Integer.toString(city.id);
	}
}