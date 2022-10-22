package frc.robot.systems;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.drive.DriveModes;
import frc.robot.drive.DrivePower;
import frc.robot.drive.Functions;
import frc.robot.Constants;

public class DriveFSMSystem {


	// FSM state definitions
	public enum FSMState {
		TELE_STATE_2_MOTOR_DRIVE,
		TELE_STATE_MECANUM,
		PURE_PURSUIT,
		TURNING_STATE,
		IDLE,
		P1,
		// P2,
		// P3,
		// P4,
		// TURN_TO_HUB
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;

	private CANSparkMax intakeMotor;
	private CANSparkMax leftMoverMotor;
	private CANSparkMax rightMoverMotor;

	private CANSparkMax shooterFeederMotor;
	private CANSparkMax shooterShooterMotor;

	private CANSparkMax climberLeftMotor;
	private CANSparkMax climberRightMotor;

	private DoubleSolenoid intakeDeploySolenoid;

	private DoubleSolenoid solenoidNotUsed1;
	private DoubleSolenoid solenoidNotUsed2;

	private double leftPower;
	private double rightPower;

	private boolean finishedTurning;

	private boolean toggle = false;

	// private CANSparkMax bottomLeftMotorMecanum;
	// private CANSparkMax bottomRightMotorMecanum;
	// private CANSparkMax topLeftMotorMecanum;
	// private CANSparkMax topRightMotorMecanum;

	// private double bottomLeftMotorMecanumPower;
	// private double bottomRightMotorMecanumPower;
	// private double topLeftMotorMecanumPower;
	// private double topRightMotorMecanumPower;

	private boolean isInArcadeDrive = true;

	private double roboXPos = 0;
	private double roboYPos = 0;
	private double currentEncoderPos;
	private double prevEncoderPos = 0;
	private double gyroAngleForOdo = 0;
	private AHRS gyro;
	private double startAngle;

	// AUTO Constants
	private boolean turning = true;
	private boolean moving = true;
	private boolean complete = false;
	private int stateCounter = 1;

	//private static final double START_ANGLE = -19.938;
	private static final double TURN_POWER = 0.1;
	private static final double MOVE_POWER = 0.4;
	private static final double DEGREES_360 = 360;
	private static final double DEGREES_180 = 180;
	private static final double DEGREES_270 = 270;
	private static final double DEGREES_90 = 90;
	private static final double TURN_THRESHOLD = 4;
	private static final double MOVE_THRESHOLD = 2;


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT,
										CANSparkMax.MotorType.kBrushless);
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT,
										CANSparkMax.MotorType.kBrushless);

		intakeMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_INTAKE,
										CANSparkMax.MotorType.kBrushless);

		// leftMoverMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_LEFT_MOVE,
		// 								CANSparkMax.MotorType.kBrushless);

		// rightMoverMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_RIGHT_MOVE,
		// 								CANSparkMax.MotorType.kBrushless);

		// shooterFeederMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER_TRANSFERER,
		// 								CANSparkMax.MotorType.kBrushless);

		// shooterShooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER_SHOOT,
		// 								CANSparkMax.MotorType.kBrushed);

		intakeDeploySolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
										HardwareMap.PCM_CHANNEL_INTAKE_SOLENOID_EXTEND,
										HardwareMap.PCM_CHANNEL_INTAKE_SOLENOID_RETRACT);

		// solenoidNotUsed1 = new DoubleSolenoid(PneumaticsModuleType.REVPH,
		// 								HardwareMap.PCM_CHANNEL_INTAKE_SOLENOID_1,
		// 								HardwareMap.PCM_CHANNEL_INTAKE_SOLENOID_2);

		// solenoidNotUsed2 = new DoubleSolenoid(PneumaticsModuleType.REVPH,
		// 								HardwareMap.PCM_CHANNEL_INTAKE_SOLENOID_3,
		// 								HardwareMap.PCM_CHANNEL_INTAKE_SOLENOID_4);

		// climberLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_CLIMBER_SUCKS,
		// 								CANSparkMax.MotorType.kBrushless);

		// climberRightMotor = new CANSparkMax(10,
		// 								CANSparkMax.MotorType.kBrushless);

		leftPower = 0;
		rightPower = 0;


		// finishedTurning = false;

		// bottomRightMotorMecanum = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BOTTOM_RIGHT,
		// 								CANSparkMax.MotorType.kBrushless);
		// bottomLeftMotorMecanum = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BOTTOM_LEFT,
		// 								CANSparkMax.MotorType.kBrushless);
		// topLeftMotorMecanum = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_TOP_LEFT,
		// 								CANSparkMax.MotorType.kBrushless);
		// topRightMotorMecanum = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_TOP_RIGHT,
		// 								CANSparkMax.MotorType.kBrushless);

		gyro = new AHRS(SPI.Port.kMXP);
		startAngle = 0;

		// Reset state machine
		resetAutonomous();

	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void resetAutonomous() {

		// bottomLeftMotorMecanum.set(0);
		// bottomRightMotorMecanum.set(0);
		// topLeftMotorMecanum.set(0);
		// topRightMotorMecanum.set(0);

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();
		gyroAngleForOdo = 0;

		currentState = FSMState.P1;

		roboXPos = 0;
		roboYPos = 0;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}
	/**
	 * A.
	 */
	public void resetTeleop() {

		// bottomLeftMotorMecanum.set(0);
		// bottomRightMotorMecanum.set(0);
		// topLeftMotorMecanum.set(0);
		// topRightMotorMecanum.set(0);

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();
		gyroAngleForOdo = 0;

		currentState = FSMState.TELE_STATE_2_MOTOR_DRIVE;

		roboXPos = 0;
		roboYPos = 0;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {

		gyroAngleForOdo = gyro.getAngle();

		currentEncoderPos = ((leftMotor.getEncoder().getPosition()
			- rightMotor.getEncoder().getPosition()) / 2.0);

		updateLineOdometryTele(gyro.getAngle());

		switch (currentState) {
			case TELE_STATE_2_MOTOR_DRIVE:
				handleTeleOp2MotorState(input);
				break;

			// case TELE_STATE_MECANUM:
			// 	handleTeleOpMecanum(input);
			// 	break;

			// case TURNING_STATE:
			// 	handleTurnState(input, 90);
			// 	break;

			case IDLE:
				handleIdleState(input);
				break;

			case P1:
				goToPos(input, 117.047, 0);
				break;

			// case P2:
			// 	goToPos(input, 117.047, -31.623);
			// 	break;

			// case P3:
			// 	goToPos(input, 56.814, 117.516);
			// 	break;

			// case P4:
			// 	goToPos(input, 0, 0);
			// 	break;

			// case TURN_TO_HUB:
			// 	turnToHub(input);
			// 	break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {

			case TELE_STATE_2_MOTOR_DRIVE:
				return FSMState.TELE_STATE_2_MOTOR_DRIVE;

			case TELE_STATE_MECANUM:
				return FSMState.TELE_STATE_MECANUM;

			case TURNING_STATE:
				System.out.println(finishedTurning);
				if (finishedTurning) {
					return FSMState.IDLE;
				} else {
					return FSMState.TURNING_STATE;
				}

			case IDLE:
				return FSMState.IDLE;

			case P1:
				if (stateCounter == 1) {
					return FSMState.P1;
				} else if (stateCounter == 2) {
					return FSMState.TELE_STATE_2_MOTOR_DRIVE;
				// } else if (stateCounter == 3) {
				// 	return FSMState.P2;
				// } else if (stateCounter == 4) {
				// 	return FSMState.TURN_TO_HUB;
				// } else if (stateCounter == 5) {
				// 	return FSMState.P3;
				// } else if (stateCounter == 6) {
				// 	return FSMState.TURN_TO_HUB;
				// } else if (stateCounter == 7) {
				// 	return FSMState.P4;
				}

			// case P2:
			// 	if (stateCounter == 1) {
			// 		return FSMState.P1;
			// 	} else if (stateCounter == 2) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 3) {
			// 		return FSMState.P2;
			// 	} else if (stateCounter == 4) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 5) {
			// 		return FSMState.P3;
			// 	} else if (stateCounter == 6) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 7) {
			// 		return FSMState.P4;
			// 	}

			// case P3:
			// 	if (stateCounter == 1) {
			// 		return FSMState.P1;
			// 	} else if (stateCounter == 2) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 3) {
			// 		return FSMState.P2;
			// 	} else if (stateCounter == 4) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 5) {
			// 		return FSMState.P3;
			// 	} else if (stateCounter == 6) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 7) {
			// 		return FSMState.P4;
			// 	}

			// case P4:
			// 	if (stateCounter == 1) {
			// 		return FSMState.P1;
			// 	} else if (stateCounter == 2) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 3) {
			// 		return FSMState.P2;
			// 	} else if (stateCounter == 4) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 5) {
			// 		return FSMState.P3;
			// 	} else if (stateCounter == 6) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 7) {
			// 		return FSMState.P4;
			// 	}

			// case TURN_TO_HUB:
			// 	if (stateCounter == 1) {
			// 		return FSMState.P1;
			// 	} else if (stateCounter == 2) {
			// 		return FSMState.TURN_TO_HUB;
			// 	} else if (stateCounter == 3) {
			// 		return FSMState.TELE_STATE_2_MOTOR_DRIVE;
			// 	}
				// } else if (stateCounter == 3) {
				// 	return FSMState.P2;
				// } else if (stateCounter == 4) {
				// 	return FSMState.TURN_TO_HUB;
				// } else if (stateCounter == 5) {
				// 	return FSMState.P3;
				// } else if (stateCounter == 6) {
				// 	return FSMState.TURN_TO_HUB;
				// } else if (stateCounter == 7) {
				// 	return FSMState.P4;


			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in TELE_STATE_2_MOTOR_DRIVE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleOp2MotorState(TeleopInput input) {
		if (input == null) {
			return;
		}

		if (isInArcadeDrive) {

			currentEncoderPos = ((leftMotor.getEncoder().getPosition()
				- rightMotor.getEncoder().getPosition()) / 2.0);

			updateLineOdometryTele(gyroAngleForOdo);

			double steerAngle = input.getSteerAngle();
			double currentLeftPower = leftMotor.get();
			double currentRightPower = rightMotor.get();


			DrivePower targetPower = DriveModes.arcadeDrive(input.getLeftJoystickY(),
				steerAngle, currentLeftPower,
				currentRightPower, true);

			// multiple speed modes
			if (input.isLeftJoystickTriggerPressedRaw()) {
				targetPower.scale(Constants.MAX_POWER);
			} else {
				targetPower.scale(Constants.REDUCED_MAX_POWER);
			}

			DrivePower power;

			// acceleration
			power = Functions.accelerate(targetPower, new DrivePower(currentLeftPower,
				currentRightPower));

			// turning in place
			if (Math.abs(input.getLeftJoystickY()) < Constants.TURNING_IN_PLACE_THRESHOLD) {
				power = Functions.turnInPlace(input.getRightJoystickY(), steerAngle);
			}

			System.out.println("ANGLE: " + getAngleToHub());

			leftPower = power.getLeftPower();
			rightPower = power.getRightPower();


			// if (input.isIntakeButtonPressed()) {
			// 	intakeDeploySolenoid.set(DoubleSolenoid.Value.kForward);
			// 	intakeMotor.set(0.8);
			// } else {
			// 	intakeDeploySolenoid.set(DoubleSolenoid.Value.kReverse);
			// 	intakeMotor.set(0);
			// }

			// if (toggle) {
			// 	intakeDeploySolenoid.set(DoubleSolenoid.Value.kForward);

			// 	intakeMotor.set(0.8);
			// 	// leftMoverMotor.set(-0.2);
			// 	// rightMoverMotor.set(0.2);
			// } else {
			// 	intakeDeploySolenoid.set(DoubleSolenoid.Value.kReverse);
			// 	intakeMotor.set(0);
			// 	// leftMoverMotor.set(0);
			// 	// rightMoverMotor.set(0);
			// }

			// Intake Code: OG (Mayand)
			if (input.isIntakeButtonPressed()) {
				intakeDeploySolenoid.set(DoubleSolenoid.Value.kForward);

				intakeMotor.set(0.8);
	
			} else {
				intakeDeploySolenoid.set(DoubleSolenoid.Value.kReverse);
				intakeMotor.set(0);
			}

			// if (input.isShootButtonPressed()) {
			// 	leftMoverMotor.set(-0.2);
			// 	rightMoverMotor.set(0.2);


			// 	shooterShooterMotor.set(-0.9);
			// 	shooterFeederMotor.set(0.6);
			// } else {

			// 	leftMoverMotor.set(0);
			// 	rightMoverMotor.set(0);

			// 	shooterShooterMotor.set(0);
			// 	shooterFeederMotor.set(0);
			// }

			// rightMotor.set(rightPower);
			// leftMotor.set(leftPower);

			//clinber
			// if(input.lC()){
			// 	climberRightMotor.set(-0.4);
			// 	climberLeftMotor.set(-0.4);
			// }else if(input.rC()){
			// 	climberRightMotor.set(0.4);
			// 	climberLeftMotor.set(0.4 );
			// }else{
			// 	climberRightMotor.set(0);
			// 	climberLeftMotor.set(0);
			// }

			// rightMotor Edits
			// if (input.rC()) {
			// 	climberRightMotor.set(0.4);
			// } else if (input.rCC()) {
			// 	climberRightMotor.set(-0.4);
			// } else {
			// 	climberRightMotor.set(0);
			// }

			// leftMotor edits
			// if (input.lC()) {
			// 	climberLeftMotor.set(0.4);
			// } else if (input.lCC()) {
			// 	climberLeftMotor.set(-0.4);
			// } else {
			// 	climberLeftMotor.set(0);
			// }



		} else {
			leftMotor.set((input.getLeftJoystickY()));
			rightMotor.set(-(input.getRightJoystickY()));
		}

		leftMotor.set(0);
		rightMotor.set(0);
		

	}

	/**
	 * Handle behavior in TURNING_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @param degrees How many degrees the robot is to turn
	 */
	public void handleTurnState(TeleopInput input, double degrees) {
		if (input != null) {
			return;
		}

		double error = degrees - getHeading();
		if (error > 180) {
			error -= 360;
		}
		if (Math.abs(error) <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
			finishedTurning = true;
			leftMotor.set(0);
			rightMotor.set(0);
			return;
		}
		double power = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
		if (power < Constants.MIN_TURN_POWER) {
			power = Constants.MIN_TURN_POWER;
		}

		power *= (error < 0 && error > -180) ? -1 : 1;

		leftMotor.set(power);
		rightMotor.set(power);
	}

	/**
	 * Handle behavior in IDlE State.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleIdleState(TeleopInput input) {
		leftMotor.set(0);
		rightMotor.set(0);
	}

	/**
	 * Handle behavior in TELE_STATE_MECANUM.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	// private void handleTeleOpMecanum(TeleopInput input) {

	// 	if (input == null) {
	// 		return;
	// 	}

	// 	double hypot = Math.hypot(input.getLeftJoystickX(), input.getLeftJoystickY());
	// 	double rightX = -input.getRightJoystickX();

	// 	double robotAngleFrontBack = (Math.atan2(input.getLeftJoystickY(),
	// 		input.getLeftJoystickX())) - Constants.QUARTER_PI;

	// 	topLeftMotorMecanumPower = hypot * Math.cos(robotAngleFrontBack) + rightX;
	// 	topRightMotorMecanumPower = hypot * Math.sin(robotAngleFrontBack) - rightX;
	// 	bottomLeftMotorMecanumPower = hypot * Math.sin(robotAngleFrontBack) + rightX;
	// 	bottomRightMotorMecanumPower = hypot * Math.cos(robotAngleFrontBack) - rightX;

	// 	topLeftMotorMecanumPower = ensureRange(topLeftMotorMecanumPower, -1, 1);
	// 	topRightMotorMecanumPower = ensureRange(topRightMotorMecanumPower, -1, 1);
	// 	bottomLeftMotorMecanumPower = ensureRange(bottomLeftMotorMecanumPower, -1, 1);
	// 	bottomRightMotorMecanumPower = ensureRange(bottomRightMotorMecanumPower, -1, 1);

	// 	System.out.println(topLeftMotorMecanumPower);

	// 	if (input.isLeftJoystickTriggerPressedRaw()) {
	// 		bottomLeftMotorMecanum.set(-bottomLeftMotorMecanumPower);
	// 		bottomRightMotorMecanum.set(bottomRightMotorMecanumPower);
	// 		topLeftMotorMecanum.set(-topLeftMotorMecanumPower);
	// 		topRightMotorMecanum.set(topRightMotorMecanumPower);
	// 	} else {
	// 		bottomLeftMotorMecanum.set(-bottomLeftMotorMecanumPower / 2);
	// 		bottomRightMotorMecanum.set(bottomRightMotorMecanumPower / 2);
	// 		topLeftMotorMecanum.set(-topLeftMotorMecanumPower / 2);
	// 		topRightMotorMecanum.set(topRightMotorMecanumPower / 2);
	// 	}
	// }

	// private double ensureRange(double value, double min, double max) {
	// 	return Math.min(Math.max(value, min), max);
	// }

	/**
	* Gets the heading from the gyro.
	* @return the gyro heading
	*/
	public double getHeading() {
		double angle = startAngle - gyro.getYaw();
		if (angle < 0) {
			angle += 360;
		}
		if (angle > 360) {
			angle -= 360;
		}
		return angle;
	}

	/**
	 * Tracks the robo's position on the field.
	 * @param gyroAngle robot's angle
	 */
	public void updateLineOdometryTele(double gyroAngle) {

		double dEncoder = (currentEncoderPos - prevEncoderPos)
			/ Constants.REVOLUTIONS_PER_INCH;
		double dX = dEncoder * Math.cos(Math.toRadians(gyroAngleForOdo))
			* Constants.DX_INCHES_CONST;
		double dY = dEncoder * Math.sin(Math.toRadians(gyroAngleForOdo))
			* Constants.DY_INCHES_CONST;

		roboXPos += dX;
		roboYPos += dY;

		prevEncoderPos = this.currentEncoderPos;

		System.out.println("X Pos: " + roboXPos);
		System.out.println("Y Pos: " + roboYPos);
		// System.out.println("Gyro: " + gyroAngleForOdo);
	}

	/**
	 * Identifies the x pos, y pos, and gyro angle for shooting point.
	 * @return distanceToTravel the distance that is to be travelled by
	 * the robot so it is in shooting position.
	 */
	public double identifyDistanceForShootingCircle() {
		// Slope of the line that intersects between robo x and y and origin (0,0)
		double m = roboYPos / roboXPos;

		// X1 value of intersection point between line and shooting circle
		double xIntersection1Val = (Constants.SHOOTING_CIRCLE_RADIUS
			* Math.sqrt(1 + Math.pow(m, 2)) / (1 + Math.pow(m, 2)));

		// X2 value of intersection point between line and shooting circle
		double xIntersection2Val = -((Constants.SHOOTING_CIRCLE_RADIUS
			* Math.sqrt(1 + Math.pow(m, 2)) / (1 + Math.pow(m, 2))));

		// Y1 value of intersection point between line and shooting circle
		double yIntersection1Val = (Constants.SHOOTING_CIRCLE_RADIUS * m
			* Math.sqrt(1 + Math.pow(m, 2)) / (1 + Math.pow(m, 2)));

		// Y2 value of intersection point between line and shooting circle
		double yIntersection2Val = -((Constants.SHOOTING_CIRCLE_RADIUS * m
			* Math.sqrt(1 + Math.pow(m, 2)) / (1 + Math.pow(m, 2))));

		double distance1 = Math.sqrt(Math.pow((roboXPos - xIntersection1Val), 2)
			+ Math.pow((roboYPos - yIntersection1Val), 2));

		double distance2 = Math.sqrt(Math.pow((roboXPos - xIntersection2Val), 2)
			+ Math.pow((roboYPos - yIntersection2Val), 2));

		double distanceToTravel;

		if (distance1 > distance2) {
			distanceToTravel = distance2;
		} else {
			distanceToTravel = distance1;
		}

		return distanceToTravel;
	}

	/**
	 * Makes the robot turn so it is facing the origin.
	 */
	public void turnToFaceOrigin() {

		double degreesToTurn = getAngleToHub();
		// turn clockwise if the angle is 0 < θ < 180
		double power = Functions.accelerateDecelerateTurn(0, getAngleToHub());
		if (degreesToTurn > 0) {
			leftMotor.set(power);
			rightMotor.set(power);
		} else if (degreesToTurn < 0) {
			leftMotor.set(-power);
			rightMotor.set(-power);
		} else {
			leftMotor.set(0);
			rightMotor.set(0);
		}
		// double degreesToTurn = Math.atan2(roboYPos, roboXPos);

		// double error = degreesToTurn - getHeading();
		// if (error > 180) {
		// 	error -= 360;
		// }
		// if (Math.abs(error) <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
		// 	leftMotor.set(0);
		// 	rightMotor.set(0);
		// 	return;
		// }
		// double power = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
		// if (power < Constants.MIN_TURN_POWER) {
		// 	power = Constants.MIN_TURN_POWER;
		// }

		// power *= (error < 0 && error > -180) ? -1 : 1;

		// leftMotor.set(power);
		// rightMotor.set(power);
	}

	/**
	 * Identifies angle to face hub.
	 * @return angle to hub
	 */
	public double getAngleToHub() {
		// double heading = (getHeading() + 180) % 360 - 180;
		double heading = (getHeading() % 360);
		double angleToHub = Math.toDegrees(Math.atan2(roboXPos + Constants.HUB_X_COORDINATE,
			-roboYPos + Constants.HUB_Y_COORDINATE)) - 90;

		//calculating the difference between the two angles
		double angleDifference = -((angleToHub - heading + 180) % 360 - 180);
		if (angleDifference >= 360) {
			angleDifference = 0;
		}
		return angleDifference;
	}

	/**
	 * Handle behavior in PURE_PERSUIT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @param x go to x position
	 * @param y go to y position
	 */
	public void goToPos(TeleopInput input, double x, double y) {

		if (input != null) {
			return;
		}

		System.out.println("t " + turning + " m " + moving);

		double roboX = -roboXPos;
		double roboY = roboYPos;
		double deltaX = (x - roboX);
		double deltaY = (y - roboY);
		if (deltaX > -1 && deltaX < 1) {
			deltaX = 0;
		}
		if (deltaY > -1 && deltaY < 1) {
			deltaY = 0;
		}
		System.out.println("dx " + deltaX + " dy " + deltaY);

		// assume unit circle angles (east = 0, positive counterclockwise)
		double currentAngle = (-gyro.getAngle()) % DEGREES_360;
		System.out.println("current angle " + currentAngle);

		// calculates turn angle
		double angle;
		if (deltaX == 0 && deltaY >= 0) {
			angle = DEGREES_90;
		} else if (deltaX == 0 && deltaY < 0) {
			angle = DEGREES_270;
		} else {
			angle = Math.toDegrees(Math.atan(deltaY / deltaX));
		}

		if (deltaX < 0) {
			angle += DEGREES_180;
		}
		if (deltaX > 0 && deltaY < 0) {
			angle += DEGREES_360;
		}

		System.out.println("turn angle " + angle);

		// calculate turn amount
		double turnAmount = angle - currentAngle;

		if (Math.abs(turnAmount - DEGREES_360) < Math.abs(turnAmount)) {
			turnAmount -= DEGREES_360;
		}

		System.out.println("turn amount: " + turnAmount);

		// calculates distance
		double dist = Math.sqrt(deltaY * deltaY + deltaX * deltaX);
		// System.out.println("dist: " + dist);
		System.out.print("(" + roboX + "," + roboYPos + ")");
		System.out.println("x " + x + "y " + y);

		// fix threshold errors
		dist += MOVE_THRESHOLD / 2;
		turnAmount += TURN_THRESHOLD / 2;

		// set motor power
		if ((turnAmount < -TURN_THRESHOLD || turnAmount > TURN_THRESHOLD) && !complete && turning) {
			System.out.println("turning");
			if (turnAmount > 0) {
				leftMotor.set(TURN_POWER);
				rightMotor.set(TURN_POWER);
			} else if (turnAmount < 0) {
				leftMotor.set(-TURN_POWER);
				rightMotor.set(-TURN_POWER);
			}
		} else  if (dist > MOVE_THRESHOLD && !complete && moving) {
			System.out.println("moving");
			leftMotor.set(-MOVE_POWER);
			rightMotor.set(MOVE_POWER);
		} else if (!complete) {
			leftMotor.set(0);
			rightMotor.set(0);
			System.out.println("STOP");
			complete = true;
			turning = true;
			moving = true;
			stateCounter++;
		}

		// complete or not
		if (((Math.abs(roboY) > Math.abs(y) + Math.abs(deltaX) / 2 || Math.abs(roboX) > Math.abs(x) + Math.abs(deltaY) / 2)) && !complete) {
			moving = false;
			//System.out.println("HERE");
		} else if (!(turnAmount >= -TURN_THRESHOLD && turnAmount <= TURN_THRESHOLD)) {
			complete = false;
		} else if (dist > MOVE_THRESHOLD) {
			complete = false;
			turning = false;
		} else {
			complete = true;
			turning = true;
			moving = true;
		}
	}

	public void turnToHub(TeleopInput input) {

		double roboX = -roboXPos;
		double roboY = roboYPos;
		double deltaX = (-roboX);
		double deltaY = (-roboY);
		if (deltaX > -1 && deltaX < 1) {
			deltaX = 0;
		}
		if (deltaY > -1 && deltaY < 1) {
			deltaY = 0;
		}
		//System.out.println("dx " + deltaX + " dy " + deltaY);

		// assume unit circle angles (east = 0, positive counterclockwise)
		double currentAngle = (-gyro.getAngle()) % DEGREES_360;
		System.out.println("current angle " + currentAngle);

		// calculates turn angle
		double angle;
		if (deltaX == 0 && deltaY >= 0) {
			angle = DEGREES_90;
		} else if (deltaX == 0 && deltaY < 0) {
			angle = DEGREES_270;
		} else {
			angle = Math.toDegrees(Math.atan(deltaY / deltaX));
		}

		if (deltaX < 0) {
			angle += DEGREES_180;
		}
		if (deltaX > 0 && deltaY < 0) {
			angle += DEGREES_360;
		}

		if (angle < 0) {
			angle -= 5;
		} else {
			angle += 5;
		}

		System.out.println("turn angle hub " + angle);

		// calculate turn amount
		double turnAmount = angle - currentAngle;

		System.out.println("hub turn amount: " + turnAmount);

		if (turnAmount < -TURN_THRESHOLD || turnAmount > TURN_THRESHOLD) {
			System.out.println("turning to hub");
			if (turnAmount > 0) {
				leftMotor.set(TURN_POWER);
				rightMotor.set(TURN_POWER);
			} else if (turnAmount < 0) {
				leftMotor.set(-TURN_POWER);
				rightMotor.set(-TURN_POWER);
			}
		} else {
			System.out.println("STOP");
			leftMotor.set(0);
			rightMotor.set(0);
			stateCounter++;
		}
	}

	/**
	 * Tracks the robo's position on the field.
	 * @param gyroAngle robot's angle
	 * @param currentEncoderPos robot's current position
	 */

}
