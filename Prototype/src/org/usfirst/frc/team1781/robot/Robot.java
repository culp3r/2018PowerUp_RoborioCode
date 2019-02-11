package org.usfirst.frc.team1781.robot;

//FIRST libs
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//CANTalon libs
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//1781 custom classes
import tech.lindblom.frc1781.EE_Collector;
import tech.lindblom.frc1781.EE_Elevator;
import tech.lindblom.frc1781.EE_Mecanum;
import tech.lindblom.frc1781.navigation.EE_navFSM;
import tech.lindblom.frc1781.navigation.states.INState;
import tech.lindblom.frc1781.navigation.states.Idle;
import tech.lindblom.frc1781.navigation.states.RotateToAngle;

@SuppressWarnings({ "unused", "rawtypes", "unchecked" })
public class Robot extends IterativeRobot {

	Timer timer;

	WPI_TalonSRX right_winch, left_winch;

	// Joy-sticks
	Joystick pilot;
	Joystick co_pilot;

	EE_Mecanum drive;
	EE_Collector collector;
	EE_Elevator elevator;
	EE_navFSM auto;

	Compressor compressor;

	// For info fed from FMS
	String game_data;

	// Smart Dash-board
	SendableChooser auto_chooser, pos_chooser;
	int auto_choice;
	int pos_choice;
	char start_side;

	@Override
	public void robotInit() {
		// Camera server
		// CameraServer.getInstance().startAutomaticCapture();

		// Climb motors
		right_winch = new WPI_TalonSRX(8);
		left_winch = new WPI_TalonSRX(9);

		pilot = new Joystick(0);
		co_pilot = new Joystick(1);

		compressor = new Compressor(0);
		compressor.start();

		drive = new EE_Mecanum();
		collector = new EE_Collector();
		elevator = new EE_Elevator(collector);
		auto = new EE_navFSM();

		timer = new Timer();

		// Starting positions
		pos_chooser = new SendableChooser();
		pos_chooser.addObject("STARTING: Left", new Integer(1));
		pos_chooser.addDefault("STARTING: Middle", new Integer(2));
		pos_chooser.addObject("STARTING: Right", new Integer(3));
		SmartDashboard.putData("Starting Position", pos_chooser);

		// Goals
		auto_chooser = new SendableChooser();
		auto_chooser.addObject("Do nothing.", new Integer(-1));
		auto_chooser.addObject("TESTING: Cross auto line", new Integer(0));
		auto_chooser.addDefault("TESTING: Line up with our switch", new Integer(1));
		auto_chooser.addObject("TESTING: Always forward: Switch, then go to scale", new Integer(2));
		auto_chooser.addObject("Go to switch", new Integer(3)); // this WAS
																// scale
		auto_chooser.addObject("Just auto line", new Integer(4));
		SmartDashboard.putData("Autonomous Program", auto_chooser);

	}

	boolean reset = false;
	boolean reset2 = false;
	public void autonomousInit() {
		timer.reset();
		timer.start();
		drive.reset_encoders('B');
		drive.reset_nav();
		game_data = DriverStation.getInstance().getGameSpecificMessage();

		start_side = '?';
		auto_choice = ((Integer) (auto_chooser.getSelected())).intValue();
		pos_choice = ((Integer) (pos_chooser.getSelected())).intValue();

		switch (pos_choice) {
		case 1:
			start_side = 'L';
			break;
		case 2:
			start_side = 'M';
			break;
		case 3:
			start_side = 'R';
			break;
		}

		switch (auto_choice) {
		case 0:
			auto0(game_data, start_side);
			break;
		case 1:
			auto1(game_data, start_side);
			break;
		case 2:
			auto2(game_data, start_side);
			break;
		case 3:
			auto3(game_data, start_side);
			break;
		case 4:
			auto4(game_data, start_side);
			break;
		}

		while (!reset2) {
			if (!reset)
				drive.reset_nav();
			reset = true;
			if (drive.getAngle() == 0)
				reset2 = true;
		}
	}
	public void autonomousPeriodic() {
		auto.update();
	}

	public void teleopInit() {
		drive.reset_encoders('B');
		timer.stop();
	}
	@Override
	public void teleopPeriodic() {

		if (pilot.getPOV() == -1) {
			drive.manual_drive(pilot, false);
		} else {
			if(pilot.getPOV() == 0)
			{
				//drive.drive_forward(24.0f);
				drive.strafe(24.0f);
			}
			if(pilot.getPOV() == 180)
			{
				drive.strafe(-24.0f);
			}
			if(pilot.getPOV() == 90 || pilot.getPOV() == 270)
			{
				drive.reset_encoders('A');
				drive.reset_nav();
			}
			
		}
		handle_pneumatics(pilot, co_pilot);
		handle_shooting(pilot, co_pilot); // should not be same joy-stick for both!
		handle_elevator(co_pilot);
		handle_climbing(co_pilot);

		// System.out.println("ePID state: " + elevatorPID_state + "Sonic: " +
		// elevator_sonic.getRangeInches()
		// + " EncoderY: " + encoder_y.get() + " :: EncoderX: " + encoder_x.get() + " ::
		// Angle: "
		// + usb_navx.getAngle() + " => " + timer.get());
	}

	void handle_pneumatics(Joystick arm_joy, Joystick tilt_joy) {
		// Arms open/close
		if (arm_joy.getRawButtonPressed(12))
			collector.collect();

		// Tilt arms forward/backwards
		if (tilt_joy.getRawButtonPressed(11))
			collector.toggleTilt();
	}
	void handle_shooting(Joystick in_joy, Joystick out_joy) {
		// Pulling in cube/shooting
		if (out_joy.getRawButton(1)) { // out
			collector.shoot();
		} else if (in_joy.getRawButton(1)) { // in
			collector.collect();
		} else {
			collector.hold();
		}
	}
	void handle_elevator(Joystick elevator_joy) {
		if (elevator_joy.getRawButtonPressed(2))
			elevator.togglePID_override();

		if (elevator_joy.getRawButton(5))
			elevator.manual_elevatorUP();
		else if (elevator_joy.getRawButton(3))
			elevator.manual_elevatorDOWN();
		else
			elevator.stop();
	}
	void handle_climbing(Joystick climb_joy) {
		if (climb_joy.getRawButton(6)) { // Up
			right_winch.set(-1);
			left_winch.set(1);
		} else if (climb_joy.getRawButton(4)) { // Down
			right_winch.set(1);
			left_winch.set(-1);
		} else {
			right_winch.set(0);
			left_winch.set(0);
		}
	}

	void auto0(String data, char starting_side) {
		auto.add(new Idle(auto, 1));
		auto.add(new RotateToAngle(auto, drive, -90f, 1.5f));
		auto.add(new Idle(auto, 3.5));
		auto.add(new RotateToAngle(auto, drive, 90f, 2f));
	}
	
	void auto1(String data, char starting_side) {}
	void auto2(String data, char starting_side) {}
	void auto3(String data, char starting_side) {}
	void auto4(String data, char starting_side) {}

	// Auto used in Milwuakee; kept as example
	// void auto3(String data, char starting_side, boolean navx) {
	// // To target scale -> now switch
	// rotate = turn_to(0.0f);
	// angle = usb_navx.getAngle();
	// // encoder_y.get() < to_counts('Y', 170)
	// if (timer.get() < 4.5) {
	// axisPID.enable();
	// axisPID.setSetpoint(0);
	// strafe_speed = 0;
	// y_speed = .5;
	// elevatorPID.setSetpoint(22);
	// elevatorPID.enable();
	// elevator.set(ePID.get_output());
	// collector.hold();
	// } else {
	// y_speed = 0;
	// strafe_speed = 0;
	// if (game_data.charAt(0) == starting_side) {
	// if (elevator_sonic.getRangeInches() < 22 + 6 &&
	// elevator_sonic.getRangeInches() > 22 - 6) {
	// collector.shoot();
	// } else {
	// elevator.set(0);
	// collector.hold();
	// }
	// } else {
	// collector.hold();
	// }
	// }
	// robotDrive.driveCartesian(strafe_speed, y_speed, rotate, angle);
	// }
}
