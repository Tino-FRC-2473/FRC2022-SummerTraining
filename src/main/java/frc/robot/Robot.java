// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;

// Systems
import frc.robot.systems.AutonomousSystem;
import frc.robot.systems.TeleOpSystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;

	// Systems
	private TeleOpSystem fsmSystem;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		// Instantiate all systems here
		fsmSystem = new TeleOpSystem();

		// Creates UsbCamera and MjpegServer [1] and connects them
		CameraServer.startAutomaticCapture();
		// Creates the CvSink and connects it to the UsbCamera
		CvSink cvSink = CameraServer.getVideo();
		// Creates the CvSource and MjpegServer [2] and connects them
		CvSource outputStream = CameraServer.putVideo("RobotFrontCamera", 640, 480);
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		fsmSystem.reset();
	}

	@Override
	public void autonomousPeriodic() {
		fsmSystem.update(null);
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		fsmSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		fsmSystem.update(input);
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
