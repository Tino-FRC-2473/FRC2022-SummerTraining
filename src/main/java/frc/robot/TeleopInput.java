package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int LEFT_JOYSTICK_PORT = 0;
	private static final int RIGHT_JOYSTICK_PORT = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick leftJoystick;
	private Joystick rightJoystick;

	private static final int ONE = 1;
	private static final int TWO = 2;
	private static final int THREE = 3;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);

		rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Left Joystick ------------------------ */
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickX() {
		return leftJoystick.getX();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickY() {
		return leftJoystick.getY();
	}
	/**
	 * Get the value of the shooter button.
	 * @return True if button is pressed
	 */
	public boolean isShooterButtonPressed() {
		return leftJoystick.getRawButton(ONE);
	}
	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return leftJoystick.getRawButton(TWO);
	}

	/**
	 * Get the value of the intake button.
	 * @return True if button is released
	 */
	public boolean isIntakeButtonReleased() {
		return leftJoystick.getRawButtonReleased(TWO);
	}

	/**
	 * Get the value of the climber button.
	 * @return True if button is pressed
	 */
	public boolean isClimberButtonPressed() {
		return leftJoystick.getRawButton(THREE);
	}

	/**
	 * Get the value of the climber button.
	 * @return True if button is released
	 */
	public boolean isClimberButtonReleased() {
		return leftJoystick.getRawButtonReleased(THREE);
	}

	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickX() {
		return rightJoystick.getX();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickY() {
		return rightJoystick.getY();
	}

	/* ======================== Private methods ======================== */

}
