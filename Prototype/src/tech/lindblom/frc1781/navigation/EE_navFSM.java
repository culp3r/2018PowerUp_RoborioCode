package tech.lindblom.frc1781.navigation;

import java.util.ArrayList;

import tech.lindblom.frc1781.navigation.states.INState;

public class EE_navFSM{
	
	ArrayList<INState> states = new ArrayList<INState>();
	INState current_state;
	int current_idx;
	boolean done;
	
	public EE_navFSM(){
		current_state = null;
		current_idx = -1;
		done = false;
	}
	
	public void add(INState new_state) {
		states.add(new_state);
		current_idx = 0;
	}
	
	public void update(){
		if(states.size() == 0 || current_idx == -1) current_state = null;
		else if(!done) current_state = states.get(current_idx);
		else current_state = null;
		
		if(current_state != null) current_state.update();
		
		System.out.println("Index: " + current_idx + " State: " + current_state + " Done?: " + done);
	}
	
	public void next_state(){
		if(states.size() > current_idx + 1) current_idx += 1;
		else done = true;
	}
	public boolean isDone() {
		return done;
	}
}



