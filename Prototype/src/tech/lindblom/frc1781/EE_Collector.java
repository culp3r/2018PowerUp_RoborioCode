package tech.lindblom.frc1781;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class EE_Collector {

	DoubleSolenoid arms, collector_lean, climb_bar;
	WPI_TalonSRX collector_right, collector_left;
	AnalogInput cube_in;
	int arm_state, tilt_state;

	public EE_Collector() {
		collector_right = new WPI_TalonSRX(6);
		collector_left = new WPI_TalonSRX(7);

		collector_lean = new DoubleSolenoid(1, 0);
		arms = new DoubleSolenoid(2, 3);
		climb_bar = new DoubleSolenoid(4, 5);
		
		cube_in = new AnalogInput(2);
	}

	public void openArms() {
		arm_state = 3;
		arms.set(DoubleSolenoid.Value.kReverse);
	}
	public void closeArms() {
		arm_state = 2;
		arms.set(DoubleSolenoid.Value.kForward);
	}
	public void toggleArms() {
		if(isDown()) arm_state += 1;
		if(arm_state % 2 == 0) closeArms();
		else openArms();
	}
	public void tiltUp() {
		tilt_state = 2;
		collector_lean.set(DoubleSolenoid.Value.kReverse);
		closeArms();
	}
	public void tiltDown() {
		tilt_state = 3;
		collector_lean.set(DoubleSolenoid.Value.kForward);
	}
	public void toggleTilt() {
		tilt_state += 1;
		if (tilt_state % 2 == 0) tiltUp();
		else tiltDown();
	}
	public boolean isDown() {
		return tilt_state % 2 != 0;
	}
	public void collect() {
		collector_right.set(.5);
		collector_left.set(-.5);
	}

	public void hold() {

		if (tilt_state % 2 == 0) {
			collector_right.set(.2);
			collector_left.set(-.2);
		}else{
			collector_right.set(0);
			collector_left.set(0);}
	}

	public void shoot() {
		collector_right.set(-.7);
		collector_left.set(.7);
	}
	public void spin() {
		collector_right.set(.5);
		collector_left.set(.5);
	}
}
