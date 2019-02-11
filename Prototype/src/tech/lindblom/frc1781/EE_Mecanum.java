package tech.lindblom.frc1781;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;

public class EE_Mecanum {

	WPI_TalonSRX front_right, back_right, front_left, back_left;
	MecanumDrive robotDrive;
	double y_speed, strafe_speed, rotate, angle;

	// Encoders (X: on DIOs(2(blue), 3(yellow)), Y: on DIOs(0(blue), 1(yellow))
	// encoder_y gives us 1.05mm per count (1.2mm)
	Encoder y_encoder, x_encoder;
	final double per_inchX = 34;//22.0079;
	final double SDcountsX = 36.993;
	final double per_inchY = 34;// 24.0323;
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
	static final double turn_kP = 0.1;
	static final double turn_kI = 0.00;
	static final double turn_kD = 0.06;
	static final double turn_kF = 0.00;
	static final double kToleranceDegrees = 1.0f;
	
	// Forward PID
	PIDController forwardPID;
	LMSA_PID yPID;
	static final double forward_kP = .2;
	static final double forward_kI = 0.00;
	static final double forward_kD = 0.00;
	static final double forward_kF = 0.00;
	
	// Strafe PID
	PIDController strafePID;
	LMSA_PID xPID;
	static final double strafe_kP = .2;
	static final double strafe_kI = 0.00;
	static final double strafe_kD = 0.00;
	static final double strafe_kF = 0.00;

	double distX, distY;

	public EE_Mecanum() {
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
		reset_dist('B');

		try {
			navx = new AHRS(SerialPort.Port.kUSB);
		} catch (RuntimeException ex) {
			System.out.println("Error instantiating navX-USB: " + ex.getMessage());
		}

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
		
		// Forward PID
		yPID = new LMSA_PID();
		forwardPID = new PIDController(forward_kP, forward_kI, forward_kD, forward_kF, y_encoder, yPID);
		forwardPID.setInputRange(Integer.MIN_VALUE, Integer.MAX_VALUE);
		forwardPID.setOutputRange(-1, 1);
		forwardPID.setAbsoluteTolerance(kToleranceCounts);
		forwardPID.setContinuous(true);
		
		// Strafe PID
		xPID = new LMSA_PID();
		strafePID = new PIDController(strafe_kP, strafe_kI, strafe_kD, strafe_kF, x_encoder, xPID);
		strafePID.setInputRange(Integer.MIN_VALUE, Integer.MAX_VALUE);
		strafePID.setOutputRange(-1, 1);
		strafePID.setAbsoluteTolerance(kToleranceCounts);
		strafePID.setContinuous(true);
	}

	public void manual_drive(Joystick pilot, boolean using_navx) {
		if (using_navx && pilot.getRawButtonPressed(7))
			navx.reset();

		if (using_navx)
			angle = navx.getAngle();
		else
			angle = 0;

		if (Math.abs(pilot.getZ()) > .3)
			rotate = pilot.getZ() * .8;
		else
			rotate = 0;

		y_speed = pilot.getY();
		strafe_speed = pilot.getX();

		robotDrive.driveCartesian(strafe_speed, -1 * y_speed, rotate, angle);
	}
	public void robot_drive(double x, double y, double z, double a) {
		robotDrive.driveCartesian(x, -1 * y, z, a);
	}

	public boolean drive_forward(double dist) {
		int num_counts = to_counts('Y', dist);
		strafe_speed = 0;
		rotate = set_turnPID(0.0f);
		angle = navx.getAngle();
		forwardPID.enable();
		forwardPID.setSetpoint(num_counts);
		y_speed = yPID.get_output();
		robotDrive.driveCartesian(strafe_speed, 1 * y_speed, rotate, angle);

		return (Math.abs(num_counts - y_encoder.get()) < kToleranceCounts);
	}
	public boolean strafe(double dist) {
		int num_counts = to_counts('X', dist);
		// Stay on axis
		axisPID.enable();
		axisPID.setSetpoint(0);
		y_speed = .7 * aPID.get_output();
		
		rotate = set_turnPID(0.0f);
		angle = navx.getAngle();
		strafePID.enable();
		strafePID.setSetpoint(num_counts);
		strafe_speed = .7 * xPID.get_output();
		robotDrive.driveCartesian(strafe_speed, 1 * y_speed, rotate, angle);

		return (Math.abs(num_counts - y_encoder.get()) < kToleranceCounts);
	}

	public void drive_back(double dist) {
		int num_counts = -1*to_counts('Y', dist);
		strafe_speed = 0;
		rotate = set_turnPID(0.0f);
		angle = navx.getAngle();

		// 7FT
		if (Math.abs(num_counts - y_encoder.get()) > to_counts('Y', 84))
			y_speed = -1;
		// 5FT
		else if (Math.abs(num_counts - y_encoder.get()) > to_counts('Y', 60))
			y_speed = -.7;
		// 4FT
		else if (Math.abs(num_counts - y_encoder.get()) > to_counts('Y', 48))
			y_speed = -.5;
		// 2FT
		else if (Math.abs(num_counts - y_encoder.get()) > to_counts('Y', 24))
			y_speed = -.3;
		else if (y_encoder.get() > num_counts)
			y_speed = 0;
		else
			y_speed = -.2;
		System.out.println("FWD: current angle: \t" + navx.getAngle() + "rotation: \t" + rotate + "yencoder/target"
				+ y_encoder.get() + "/" + num_counts);

		robotDrive.driveCartesian(strafe_speed, 1 * y_speed, rotate, angle);
	}
	public void strafe_right(double dist) {
		int num_counts = to_counts('X', dist);
		axisPID.enable();
		axisPID.setSetpoint(0);
		y_speed = .7 * aPID.get_output();
		rotate = set_turnPID(0.0f);
		angle = navx.getAngle();

		// 7FT
		if (num_counts - x_encoder.get() > to_counts('X', 84))
			strafe_speed = 1;
		else if (num_counts - x_encoder.get() > to_counts('X', 72))
			strafe_speed = .7;
		else if (num_counts - x_encoder.get() > to_counts('X', 60))
			strafe_speed = .5;
		else if (num_counts - x_encoder.get() > to_counts('X', 36))
			strafe_speed = .3;
		else if (x_encoder.get() > num_counts)
			strafe_speed = 0;
		else
			strafe_speed = .2;

		robotDrive.driveCartesian(strafe_speed, -1 * y_speed, rotate, angle);
	}
	public void strafe_left(double dist) {
		int num_counts = to_counts('X', dist);
		axisPID.enable();
		axisPID.setSetpoint(0);
		y_speed = 0.7 * aPID.get_output();
		rotate = set_turnPID(0.0f);
		angle = navx.getAngle();

		// 7FT
		if (Math.abs(num_counts - x_encoder.get()) > to_counts('X', 84))
			strafe_speed = -1;
		// 5FT
		else if (Math.abs(num_counts - x_encoder.get()) > to_counts('X', 60))
			strafe_speed = -.7;
		// 4FT
		else if (Math.abs(num_counts - x_encoder.get()) > to_counts('X', 48))
			strafe_speed = -.5;
		// 2FT
		else if (Math.abs(num_counts - x_encoder.get()) > to_counts('X', 24))
			strafe_speed = -.5;
		else if (x_encoder.get() > num_counts)
			strafe_speed = 0;
		else
			strafe_speed = -.2;

		robotDrive.driveCartesian(strafe_speed, -1 * y_speed, rotate, angle);
		System.out.println("StrafeLeft: current angle: \t" + navx.getAngle() + "\trotation: \t" + rotate + "xencoder/target"
				+ x_encoder.get() + "/" + num_counts + "speed: " + strafe_speed);
	}

	public double rotate_to(float _angle) {
		strafe_speed = 0;
		y_speed = 0;
		rotate = set_turnPID(_angle);
		angle = navx.getAngle();
		robotDrive.driveCartesian(strafe_speed, -1 * y_speed, rotate, angle);
		System.out.println("RotatePWR: " + rotate + " Angle: " + angle);
		return rotate;
	}

	double set_turnPID(float angle) {
		double currentRotationRate = 0;
		rotatePID.enable();
		rotatePID.setSetpoint(angle);
		currentRotationRate = rPID.get_output();
		return currentRotationRate * .7;
	}

	public double get_dist(char axis) {
		if (axis == 'X') {
			distX += Math.abs(to_inches('X', x_encoder.get()));
			return distX;
		} else if (axis == 'Y') {
			distY += Math.abs(to_inches('Y', y_encoder.get()));
			return distY;
		}
		return -1;
	}

	public void reset_dist(char axis) {
		if (axis == 'X')
			distX = 0;
		else if (axis == 'Y')
			distY = 0;
		else {
			distX = 0;
			distY = 0;
		}
	}

	public void reset_encoders(char reset_choice) {
		System.out.println("Reseting Encoders: " + reset_choice);
		if (reset_choice == 'X')
			x_encoder.reset();
		else if (reset_choice == 'Y')
			y_encoder.reset();
		else {
			x_encoder.reset();
			y_encoder.reset();
		}
		System.out.println("Encoders X: " + x_encoder.get());
		System.out.println("Encoders Y: " + y_encoder.get());
	}

	int to_counts(char axis, double inches) {

		if (axis == 'X')
			return (int) ((inches * per_inchX));// - SDcountsX);
		else if (axis == 'Y')
			return (int) ((inches * per_inchY));// - SDcountsY);
		return 0;
	}

	double to_inches(char axis, double counts) {
		if (axis == 'X')
			return (counts + SDcountsX) / per_inchX;
		else if (axis == 'Y')
			return (counts + SDcountsY / per_inchY);
		return 0;
	}

	public double getAngle() {
		return navx.getAngle();
	}

	public void reset_nav() {
		navx.reset();
	}

}