package tech.lindblom.frc1781;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class EE_Collector {

	DoubleSolenoid arms_open, collector_lean, climb_bar;
	WPI_TalonSRX collector_right, collector_left;
	AnalogInput cube_in;
	int arm_state, tilt_state;

	public EE_Collector() {
		collector_right = new WPI_TalonSRX(6);
		collector_left = new WPI_TalonSRX(7);

		collector_lean = new DoubleSolenoid(1, 0);
		arms_open = new DoubleSolenoid(2, 3);
		climb_bar = new DoubleSolenoid(4, 5);

		cube_in = new AnalogInput(2);
	}

	public void tiltUp() {
		collector_lean.set(DoubleSolenoid.Value.kReverse);
	}

	public void tiltDown() {
		collector_lean.set(DoubleSolenoid.Value.kForward);
	}

	public void toggleTilt() {
		tilt_state += 1;

		if (tilt_state % 2 == 0)
			tiltUp();
		else
			tiltDown();
	}

	public boolean isDown() {
		return tilt_state % 2 != 0;
	}

	public void collect() {
		arm_state += 1;
		if (arm_state % 2 == 0)
			arms_open.set(DoubleSolenoid.Value.kReverse);
		else
			arms_open.set(DoubleSolenoid.Value.kForward);

		collector_right.set(.75);
		collector_left.set(-.5);
	}

	public void hold() {

		if (tilt_state % 2 == 0) {
			collector_right.set(.2);
			collector_left.set(-.2);
		} else {
			collector_right.set(0);
			collector_left.set(0);
		}
	}

	public void shoot() {
		collector_right.set(-.5);
		collector_left.set(.5);
	}
}
