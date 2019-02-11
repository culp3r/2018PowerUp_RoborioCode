package tech.lindblom.frc1781.navigation.states;

import tech.lindblom.frc1781.EE_Mecanum;
import tech.lindblom.frc1781.navigation.EE_navFSM;

public class DriveDistance extends NState{
	
	EE_navFSM nfsm;
	EE_Mecanum drive;
	float dist;
	float thresh;
	
	public DriveDistance(EE_navFSM n, EE_Mecanum m, float distance_inches, float threshold){
		super(n);
		nfsm = n;
		drive = m;
		dist = distance_inches;
		thresh = threshold;
	}
	
	public void update() {
		if (dist > 0) drive.drive_forward(dist);
		else drive.drive_back(dist);
		
		if(Math.abs(dist) - thresh < drive.get_dist('Y'))
			nfsm.next_state();
	}
	
}