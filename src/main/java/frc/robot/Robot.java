// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import frc.robot.Robot;
import frc.robot.systems.FSMSystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Systems
//mport frc.robot.systems.TraversalFSM;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	//private TeleopInput input;

	// Systems
	//private TraversalFSM fsmSystem;
	private LimeLight limelight;

	// ShuffleBoard and NetworkTables
	private double turnDirection;
	private double distanceToHub;
	private double shootPower;
	private FSMSystem fsmSystem;
	private TeleopInput input;
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		// Instantiate all systems here
		fsmSystem = new FSMSystem();

		limelight = new LimeLight();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		//fsmSystem.reset();
	}

	@Override
	public void autonomousPeriodic() {
		//fsmSystem.update(null);
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		//SmartDashboard.putString("Turn Direction", "Invalid");
		//SmartDashboard.putNumber("Distance To Hub", -1.0);

		//final double invalidPower = -6;
		//SmartDashboard.putNumber("Shooting Power", invalidPower);
		fsmSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		//shootPower = limelight.getMotorPower();
		//turnDirection = limelight.getTurningDirection();
		//distanceToHub = limelight.getHubDistance();
		limelight.update();
		fsmSystem.update(input);

		//if (shootPower <= 1 && shootPower >= -1) {
			//SmartDashboard.getEntry("Shooting Power").setNumber(shootPower);
		//}

		//if (turnDirection == -1) {
		//	SmartDashboard.getEntry("Turn Direction").setString("Left");
		//} else if (turnDirection == 0) {
		//	SmartDashboard.getEntry("Turn Direction").setString("Stay");
		//} else if (turnDirection == 1) {
		//	SmartDashboard.getEntry("Turn Direction").setString("Right");
		//} else {
		//	SmartDashboard.getEntry("Turn Direction").setString("Invalid Entry");
		//}

		//SmartDashboard.getEntry("Distance To Hub").setNumber(distanceToHub);
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }
}
