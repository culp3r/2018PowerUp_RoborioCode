package tech.lindblom.frc1781.navigation.states;

import edu.wpi.first.wpilibj.Timer;
import tech.lindblom.frc1781.navigation.EE_navFSM;

public class Idle extends NState{
	
	EE_navFSM nfsm;
	Timer timer;
	double idle_time;
	boolean started;
	
	public Idle(EE_navFSM n, double t){
		super(n);
		nfsm = n;
		idle_time = t;
		timer = new Timer();
		started = false;
	}
	
	public void update(){
		if(!started) {
			timer.reset();
			timer.start();
			started = true;}
		if(timer.get() > idle_time) nfsm.next_state();
	}

}