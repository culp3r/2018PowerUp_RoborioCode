package tech.lindblom.frc1781.navigation.states;

import tech.lindblom.frc1781.EE_Mecanum;
import tech.lindblom.frc1781.navigation.EE_navFSM;

public class RotateToAngle extends NState{
	
	EE_navFSM nfsm;
	EE_Mecanum drive;
	float angle;
	float thresh;
	boolean setup;
	boolean reset;
	boolean reset2;
	
	public RotateToAngle(EE_navFSM n, EE_Mecanum m, float a, float t){
		super(n);
		nfsm = n;
		drive = m;
		angle = a;
		thresh = t;
		setup = false;
		
		reset = false;
		reset2 = false;
	}
	
	public void update(){
		if (Math.abs(drive.rotate_to(angle)) < .02 && 
			(Math.abs(angle) - thresh < Math.abs(drive.getAngle()) && Math.abs(angle) + thresh > Math.abs(drive.getAngle()))) 
			while(!reset2){
				if(!reset) drive.reset_nav();
				reset = true;
				if (drive.getAngle() == 0) reset2 = true;
			}
			nfsm.next_state();
	}
}