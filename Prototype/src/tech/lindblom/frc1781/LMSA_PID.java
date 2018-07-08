package tech.lindblom.frc1781;

import edu.wpi.first.wpilibj.PIDOutput;

public class LMSA_PID implements PIDOutput {
	double PID_out;

	public void pidWrite(double output) {
		PID_out = output;
	}

	public double get_output() {
		return PID_out;
	}
}