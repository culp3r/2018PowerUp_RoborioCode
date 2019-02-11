package tech.lindblom.frc1781.navigation.states;

import tech.lindblom.frc1781.navigation.EE_navFSM;

abstract class NState implements INState{
	
	EE_navFSM nfsm;
	
	NState(EE_navFSM n){
		nfsm = n;
	}
	
	public void update(){
		nfsm.next_state();
	}
}