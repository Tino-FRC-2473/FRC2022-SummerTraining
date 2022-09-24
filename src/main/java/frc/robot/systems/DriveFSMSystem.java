package frc.robot.systems;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.LimeLight;
import frc.robot.drive.DriveModes;
import frc.robot.drive.DrivePower;
import frc.robot.drive.Functions;
import frc.robot.Constants;

public class Drive {


	// FSM state definitions
	public enum FSMState {
		TELE_STATE_2_MOTOR_DRIVE,
		TELE_STATE_MECANUM,
		PURE_PURSUIT,
		TURNING_STATE,
		IDLE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;
	private double leftPower;
	private double rightPower;
	private LimeLight limelight;

	private boolean finishedTurning;

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


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public Drive() {
		// Perform hardware init
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT,
										CANSparkMax.MotorType.kBrushless);
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT,
										CANSparkMax.MotorType.kBrushless);

		leftPower = 0;
		rightPower = 0;

		limelight = new LimeLight();

		finishedTurning = false;

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
		reset();

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
	public void reset() {

		// bottomLeftMotorMecanum.set(0);
		// bottomRightMotorMecanum.set(0);
		// topLeftMotorMecanum.set(0);
		// topRightMotorMecanum.set(0);

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();

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

		switch (currentState) {
			case TELE_STATE_2_MOTOR_DRIVE:
				handleTeleOp2MotorState(input);
				break;

			// case TELE_STATE_MECANUM:
			// 	handleTeleOpMecanum(input);
			// 	break;

			case TURNING_STATE:
				handleTurnState(input, 90);
				break;

			case IDLE:
				handleIdleState(input);
				break;

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

			// System.out.println("VELOC: " + gyro.getVelocityX());
			// System.out.print("POPO: " + leftMotor.get());

			currentEncoderPos = ((leftMotor.getEncoder().getPosition()
				- rightMotor.getEncoder().getPosition()) / 2.0);

			// updateLineOdometryTele(gyroAngleForOdo);

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

			// System.out.println("ANGLE: " + getAngleToHub());

			leftPower = power.getLeftPower();
			rightPower = power.getRightPower();


			rightMotor.set(rightPower);
			leftMotor.set(leftPower);

			System.out.println("distance: " + limelight.getHubDistance());


		} else {
			leftMotor.set((input.getLeftJoystickY()));
			rightMotor.set(-(input.getRightJoystickY()));
		}

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
		} else if(degreesToTurn < 0) {
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

}