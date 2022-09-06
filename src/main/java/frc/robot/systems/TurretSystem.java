package frc.robot.systems;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;



//########## TODO: Turret Homing: drive turret to one end
//########## of its range of motion to "zero" the turret position



public class TurretSystem {

	// FSM state definitions
	public enum FSMState {
		INTAKING,
		LIVE_TURRET
	}

	/**
	 * DEFINITION OF TURRET COORDINATE SYSTEM:
	 * 
	 * MAXIMUM RANGE OF MOTION:
	 * \           /
	 *  \         /
	 *   \       /
	 *    \     /
	 *     \   /
	 *      \ /
	 *       O
	 * 
	 * LEFT_LIMIT: the furthest counterclockwise the turret can face
	 * LEFT_LIMIT < 0
	 * referencePoint = LEFT_LIMIT
	 * \
	 *  \
	 *   \
	 *    \
	 *     \
	 *      \
	 *       O
	 * 
	 * CENTER: facing straight ahead, shooter is facing directly ahead of chassis
	 * referencePoint = 0
	 *       |
	 *       |
	 *       |
	 *       |
	 *       |
	 *       |
	 *       O
	 * 
	 * RIGHT_LIMIT: the furthest clockwise the turret can face
	 * RIGHT_LIMIT > 0
	 * referencePoint = RIGHT_LIMIT
	 *            /
	 *           /
	 *          /
	 *         /
	 *        /
	 *       O
	 */

	/* ======================== Private variables ======================== */
	//Motor Ticks required to rotate turret by one degree
	private static final double REVS_PER_DEGREE = 70.0/360; //i have no idea

	//Range of motion: The turret can rotate 45 degrees in either direction
	private static final double RANGE_OF_MOTION = 90;

	private static final double LEFT_LIMIT = -REVS_PER_DEGREE * RANGE_OF_MOTION / 2;
	private static final double RIGHT_LIMIT = REVS_PER_DEGREE * RANGE_OF_MOTION / 2;

	private static final double TURRET_MAX_SPEED = 0.2;

	private double referenceAngle;

	private FSMState currentState;

	private final ShooterSystem SHOOTER;
	private final DriveSystem DRIVER;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	private final CANSparkMax turretMotor;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public TurretSystem(ShooterSystem shooter, DriveSystem driver) {
		// Perform hardware init
		turretMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_TURRET_DRIVER,
										CANSparkMax.MotorType.kBrushless);

		SHOOTER = shooter;
		DRIVER = driver;

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
		currentState = FSMState.LIVE_TURRET;

		turretMotor.set(0);
		turretMotor.getEncoder().setPosition(0);

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
		switch (currentState) {
			case INTAKING:
				handleIntakingState(input);
				break;
			
			case LIVE_TURRET
:
				handleLiveTurretState(input);
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
		if(input == null){
			return currentState;
		}
		if(SHOOTER.getCurrentState() == ShooterSystem.FSMState.INTAKING){
			return FSMState.INTAKING;
		}else{
			return FSMState.LIVE_TURRET;
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in INTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakingState(TeleopInput input) {
		referenceAngle = -10;

		driveTurret();
	}

	/**
	 * Handle behavior in INTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleLiveTurretState(TeleopInput input) {
		//Convert a continuous angle (could go beyond 360) to [-180, 180]
		
		double heading = (DRIVER.getHeading() + 180) % 360 - 180;
		Point position = DRIVER.getPosition();
		double angleToHub = Math.toDegrees(Math.atan2(position.x - DriveSystem.HUB_LOCATION.x,
													  - position.y)) - 90;

		//calculating the difference between the two angles
		double angleDifference = (angleToHub - heading + 180) % 360 - 180;

		if(input!=null)System.out.println(input.getSteerAngle());
		System.out.println("heading:      "+heading);
		System.out.println("position:     "+position);
		System.out.println("angle to hub: "+angleToHub);
		System.out.println("angle diff:   "+angleDifference);

		referenceAngle = angleDifference;

		driveTurret();
	}

	/* ------------------------ Helpers ------------------------ */
	private final void driveTurret() {
		double target = Math.min(Math.max(referenceAngle * REVS_PER_DEGREE, LEFT_LIMIT), RIGHT_LIMIT);
		turretMotor.set((target - turretMotor.getEncoder().getPosition()) / REVS_PER_DEGREE / RANGE_OF_MOTION * TURRET_MAX_SPEED/2 - 0.03);
		System.out.println((target - turretMotor.getEncoder().getPosition()) );
	}
}