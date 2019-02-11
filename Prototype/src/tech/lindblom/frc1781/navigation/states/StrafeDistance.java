package tech.lindblom.frc1781.navigation.states;

import tech.lindblom.frc1781.EE_Mecanum;
import tech.lindblom.frc1781.navigation.EE_navFSM;

public class StrafeDistance extends NState{
	
	EE_navFSM nfsm;
	EE_Mecanum drive;
	float dist;
	float thresh;
	
	public StrafeDistance(EE_navFSM n, EE_Mecanum m, float distance_inches, float threshold){
		super(n);
		nfsm = n;
		drive = m;
		dist = distance_inches;
		thresh = threshold;
	}
	
	public void update() {
		if (dist > 0) drive.strafe_right(dist);
		else drive.strafe_left(dist);
		
		if(Math.abs(dist) - thresh < drive.get_dist('X'))
			nfsm.next_state();
	}
	
}