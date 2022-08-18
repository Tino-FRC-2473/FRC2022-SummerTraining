package frc.robot.systems;

// Third party Hardware Imports
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.Constants;

public class FSMSystem {


	// FSM state definitions
	public enum FSMState {
		TELE_STATE_2_MOTOR_DRIVE,
		TELE_STATE_MECANUM,
		PURE_PURSUIT
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;

	private double roboXPos = 0;
	private double roboYPos = 0;
	private double currentEncoderPos;
	private double prevEncoderPos = 0;
	private double gyroAngleForOdo = 0;
	private AHRS gyro;


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT,
										CANSparkMax.MotorType.kBrushless);
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT,
										CANSparkMax.MotorType.kBrushless);

		// leftPower = 0;
		// rightPower = 0;

		gyro = new AHRS(SPI.Port.kMXP);

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

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();

		currentState = FSMState.PURE_PURSUIT;

		roboXPos = 0;
		roboYPos = 0;
		System.out.println(roboXPos + " " + roboYPos);

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
		
		updateLineOdometryTele(gyro.getAngle(), currentEncoderPos);

		switch (currentState) {

			case PURE_PURSUIT:
				handlePurePersuit(input, 0, 15, 0);
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

			case PURE_PURSUIT:
				return FSMState.PURE_PURSUIT;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in PURE_PERSUIT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handlePurePersuit(TeleopInput input, double startAngle, double x, double y) {

		if(input == null) {
			return;
		}

		double roboX = -roboXPos;

		// assume unit circle angles (east = 0, positive counterclockwise)
		double currentAngle = calculateCurrentAngle(gyroAngleForOdo, startAngle);

		// calculates turn angle
		double angle;
		if (x == 0 && y > 0) angle = 90;
		else if (x == 0 && y < 0) angle = 270;
		else angle = Math.toDegrees(Math.atan(y/x));
		
		if (x < 0) angle += 180;
		if (x > 0 && y < 0) angle += 360;

		// calculate turn amount
		double turnAmount = angle - currentAngle;
		System.out.println("turn amount: " + turnAmount);

		// calculates distance
		double dist = Math.sqrt((y - roboYPos) * (y - roboYPos) + (x - roboX) * (x - roboX));
		System.out.println("dist: " + dist);
		System.out.println("curX: " + roboX + " curY: " + roboYPos);

		// set motor power
		if (!(turnAmount >= -5 && turnAmount <= 5)) {
			System.out.println("turning");
			if (turnAmount > 0) {
				leftMotor.set(0.1);
				rightMotor.set(0.1);
			} else if (turnAmount < 0) {
				leftMotor.set(-0.1);
				rightMotor.set(-0.1);
			}
		} else  if (dist > 0.4) {
			System.out.println("moving");
			leftMotor.set(-0.1);
			rightMotor.set(0.1);
		} else {
			leftMotor.set(0);
			rightMotor.set(0);
		}
	}

	/**
	 * Tracks the robo's position on the field.
	 * @param gyroAngle robot's angle
	 * @param currentEncoderPos robot's current position
	 */
	public void updateLineOdometryTele(double gyroAngle, double currentEncoderPos) {

		// double currentEncoderPos = ((-leftEncoderPos + rightEncoderPos) / 2.0);
		double dEncoder = (currentEncoderPos - prevEncoderPos)
			/ Constants.REVOLUTIONS_PER_INCH;
		double dX = dEncoder * Math.cos(Math.toRadians(gyroAngleForOdo)) * 0.8880486672;
		double dY = dEncoder * Math.sin(Math.toRadians(gyroAngleForOdo)) * 1.1742067733;

		roboXPos += dX;
		roboYPos += dY;

		prevEncoderPos = this.currentEncoderPos;

		System.out.println("X Pos: " + roboXPos);
		System.out.println("Y Pos: " + roboYPos);
		System.out.println("Gyro: " + gyroAngleForOdo);
		// return new Translation2d(robotPos.getX() + dX, robotPos.getY() + dY);
	}
	
	/**
	 * Calculates current angle in field.
	 * assume unit circle angles (east = 0, positive counterclockwise)
	 * @param gyroAngle robot's angle
	 * @param startAngle starting angle of robot
	 * @return robot's angle in global plane
	 */
	public double calculateCurrentAngle(double gyroAngle, double startAngle) {
		double angle = gyroAngle % 360;
		angle += startAngle;
		return angle;
	}
}
