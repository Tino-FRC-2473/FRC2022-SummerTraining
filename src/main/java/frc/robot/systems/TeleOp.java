package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;
// WPILib Imports
// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardwareMap;
// Robot Imports
import frc.robot.TeleopInput;

public class TeleOp {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		MOVE,
		IDLE,
		AUTO
	}

	// private static final float MOTOR_RUN_POWER = 0.1f;
	/* ======================== Private variables ======================== */
	private FSMState currentState;
	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax motor;
	// private CANSparkMax joystick;

	private AHRS gyro = new AHRS(SPI.Port.kMXP);


	private final int potRange = 180;
	private AnalogPotentiometer pot = new AnalogPotentiometer(0, potRange, 1);

	private final int cameraWidth = 640;
	private final int cameraHeight = 480;

	private DigitalInput limitSwitch = new DigitalInput(0);

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public TeleOp() {
		// Perform hardware init
		motor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,
				CANSparkMax.MotorType.kBrushless);

		CameraServer.startAutomaticCapture();
		CvSink cvsink = CameraServer.getVideo();
		CvSource outputStream = CameraServer.putVideo("Camera", cameraWidth, cameraHeight);

		// jotstick = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
		// CANSparkMax.MotorType.);
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
	 * when the robot is enabled.a
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.MOVE;
		gyro.reset();
		gyro.calibrate();
		// Call one tick of update to ensure outputs reflect start state
		// update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case MOVE:
				moveHandle(input);
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
	 *              the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		return FSMState.MOVE;
	}


	private void moveHandle(TeleopInput input) {

		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Potentiometer Voltage", pot.get());
		SmartDashboard.putBoolean("Switch", limitSwitch.get()); // temp false
		SmartDashboard.putNumber("Get Left Encoder Ticks", motor.getEncoder().getPosition());
	}
}