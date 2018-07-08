package tech.lindblom.frc1781;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



public class EE_Elevator{
	
	WPI_TalonSRX elevator;
	
	//Elevator PID
	PIDController elevatorPID;
	LMSA_PID ePID;
	Ultrasonic elevator_sonic;
	static final double elevator_kP = 0.08;
	static final double elevator_kI = 0;
	static final double elevator_kD = 0;
	static final double elevator_kF = 0;
	static final double kToleranceError = 1.0f;
	boolean elevatorPIDActive = false;
	int elevatorPID_state;
	double elevatorSetPoint = 0;
	boolean override_ePID;

	AnalogInput limit_low;
	AnalogInput limit_high;
	boolean at_top;
	
	//Some constants for field element dimensions
	static final double switch_height = 20;
	static final double scale_low = 40;
	static final double scale_high = 50;
	
	EE_Collector collector;
	
	public EE_Elevator(EE_Collector c){
		elevator = new WPI_TalonSRX(5);
		
		limit_low = new AnalogInput(0);
		limit_high = new AnalogInput(1);

		// Elevator PID
		ePID = new LMSA_PID();
		elevator_sonic = new Ultrasonic(4, 5);
		elevator_sonic.setEnabled(true);
		elevator_sonic.setAutomaticMode(true);
		elevatorPID = new PIDController(elevator_kP, elevator_kI, elevator_kD, elevator_kF, elevator_sonic, ePID);
		elevatorPID.setInputRange(0, 90);
		elevatorPID.setOutputRange(-1.0, 1.0);
		elevatorPID.setAbsoluteTolerance(kToleranceError);
		elevatorPID.setContinuous(true);
		elevatorPID_state = 2;
		
		collector = c;
	}
	public void manual_elevatorUP(){
		elevatorPIDActive = false;
		elevatorPID.disable();
		if (limit_high.getAverageVoltage() <= 3) {
			elevator.set(1);
			at_top = true;
		}else elevator.set(.1);
	}
	public void manual_elevatorDOWN(){
		at_top = false;
		elevatorPIDActive = false;
		elevatorPID.disable();
		if(limit_low.getAverageVoltage() <= 3) elevator.set(-1);
		else elevator.set(0);
	}
	
	public void stop(){
		if (!elevatorPIDActive) {
			elevatorSetPoint = elevator_sonic.getRangeInches();
			if (elevatorSetPoint < 50) {
				elevatorPIDActive = true;
				elevatorPID.setSetpoint(elevatorSetPoint);
				elevatorPID.enable();
			}
		}
		
		if (elevator_sonic.getRangeInches() < 7 || collector.isDown()) elevator.set(0);
		else if (!override_ePID) elevator.set(ePID.get_output());
		else if (at_top) elevator.set(.1);
		else elevator.set(.15);
	}
	
	public void togglePID_override(){
		elevatorPID_state += 1;

		if (elevatorPID_state % 2 == 0) override_ePID = false;
		else override_ePID = true;
	}
	void deliver_cube(double height, double thresh) {
		System.out.println("Sonic: " + elevator_sonic.getRangeInches() + " Setpoint: " + elevatorPID.getSetpoint()
				+ " Max: " + (height + thresh) + " Min: " + (height - thresh));
		elevatorPID.setSetpoint(height);
		elevatorPID.enable();
		elevator.set(ePID.get_output());
		if (elevator_sonic.getRangeInches() < height + thresh && elevator_sonic.getRangeInches() > height - thresh) {
			collector.shoot();
		} else {
			collector.hold();
		}
	}
	
}