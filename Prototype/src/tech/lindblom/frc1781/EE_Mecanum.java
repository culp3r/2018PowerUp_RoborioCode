package tech.lindblom.frc1781;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;

public class EE_Mecanum{
	
	WPI_TalonSRX front_right, back_right, front_left, back_left;
	MecanumDrive robotDrive;
	double y_speed, strafe_speed, rotate, angle;
	
	// Encoders (X: on DIOs(2(blue), 3(yellow)), Y: on DIOs(0(blue), 1(yellow))
	// encoder_y gives us 1.05mm per count (1.2mm)
	Encoder y_encoder, x_encoder;
	final double per_inchX = 22.0079;
	final double SDcountsX = 36.993;
	final double per_inchY = 24.0323;
	final double SDcountsY = 253.887;
	
	AHRS navx;
	
	// Straightening PID
	PIDController axisPID;
	LMSA_PID aPID;
	static final double axis_kP = .2;
	static final double axis_kI = 0.00;
	static final double axis_kD = 0.00;
	static final double axis_kF = 0.00;
	static final int kToleranceCounts = 60;
	
	// Turning PID
	PIDController rotatePID;
	LMSA_PID rPID;
	double rotateToAngleRate;
	static final double turn_kP = 0.03;
	static final double turn_kI = 0.00;
	static final double turn_kD = 0.00;
	static final double turn_kF = 0.00;
	static final double kToleranceDegrees = 2.0f;
	
	public EE_Mecanum(){
		// Drive System
		front_right = new WPI_TalonSRX(1);
		back_right = new WPI_TalonSRX(2);
		front_left = new WPI_TalonSRX(3);
		back_left = new WPI_TalonSRX(4);
		front_right.setNeutralMode(NeutralMode.Brake);
		back_right.setNeutralMode(NeutralMode.Brake);
		front_left.setNeutralMode(NeutralMode.Brake);
		back_left.setNeutralMode(NeutralMode.Brake);
		robotDrive = new MecanumDrive(front_left, back_left, front_right, back_right);
		y_speed = 0;
		strafe_speed = 0;
		rotate = 0;
		angle = 0;
		
		x_encoder = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
		y_encoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
		
		try {
			navx = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			System.out.println("Error instantiating navX-USB: " + ex.getMessage());}
		
		// Y-Axis Correction PID
		aPID = new LMSA_PID();
		axisPID = new PIDController(axis_kP, axis_kI, axis_kD, axis_kF, y_encoder, aPID);
		axisPID.setInputRange(Integer.MIN_VALUE, Integer.MAX_VALUE);
		axisPID.setOutputRange(-1, 1);
		axisPID.setAbsoluteTolerance(kToleranceCounts);
		axisPID.setContinuous(true);

		// Rotation PID
		rPID = new LMSA_PID();
		rotatePID = new PIDController(turn_kP, turn_kI, turn_kD, turn_kF, navx, rPID);
		rotatePID.setInputRange(-180.0f, 180.0f);
		rotatePID.setOutputRange(-1, 1);
		rotatePID.setAbsoluteTolerance(kToleranceDegrees);
		rotatePID.setContinuous(true);
	}
	
	public void drive(Joystick pilot, boolean using_navx){
		if (using_navx && pilot.getRawButtonPressed(7)) navx.reset();

		if (using_navx) angle = navx.getAngle();
		else angle = 0;
		
		if (Math.abs(pilot.getZ()) > .3) rotate = pilot.getZ() * .8;
		else rotate = 0;
		
		y_speed = pilot.getY();
		strafe_speed = pilot.getX();
		
		System.out.println("X: " + x_encoder.get() + " Y: " + y_encoder.get());

		robotDrive.driveCartesian(strafe_speed, -1 * y_speed, rotate, angle);
	}
	
	
	double turn_to(float angle) {
		double currentRotationRate = 0;
		rotatePID.enable();
		rotatePID.setSetpoint(angle);
		currentRotationRate = rPID.get_output();
		return currentRotationRate * .7;
	}
	public void reset_encoders(char reset_choice) {

		if (reset_choice == 'X') x_encoder.reset();
		else if (reset_choice == 'Y') y_encoder.reset();
		else{
			x_encoder.reset();
			y_encoder.reset();}
	}
	int to_counts(char axis, double inches) {

		if (axis == 'X') return (int) ((inches * per_inchX) - SDcountsX);
		else if (axis == 'Y') return (int) ((inches * per_inchY) - SDcountsY);
		return 0;
	}
	public void reset_nav() {
		navx.reset();
	}
	
}