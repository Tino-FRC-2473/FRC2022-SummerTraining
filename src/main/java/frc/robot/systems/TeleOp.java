package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleOp {

	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP
	}
	private AHRS gyro;
	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

    private CANSparkMax left, right;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public TeleOp() {
		// Perform hardware init
        right = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,
										CANSparkMax.MotorType.kBrushless);
        left = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT,
										CANSparkMax.MotorType.kBrushless);
		left.setInverted(true);
		// Creates UsbCamera and MjpegServer [1] and connects them
		
		gyro = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);
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
		currentState = FSMState.TELEOP;
		gyro.reset();
		gyro.calibrate();
		// Call one tick of update to ensure outputs reflect start state
	}
	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		currentState = FSMState.TELEOP;
		handleTELEOP(input);
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
		return FSMState.TELEOP;
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTELEOP(TeleopInput input) {
		//SmartDashboard.putNumber("Left Motor Ticks", left.)
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putBoolean("Shooter Button", input.isShooterButtonPressed());
		SmartDashboard.putBoolean("Intake Button", input.isIntakeButtonPressed());
	}
}
