package frc.robot.systems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class CompressorSystem {

	private Compressor pneumaticsCompressor;

	/**
	 * Constructor for general CompressorSystem.
	 * @param runCompressor Whether or not the compressor
	 * 		should be running (must be TRUE for games, or
	 *		anytime pneumatics are being used)
	 */
	public CompressorSystem(boolean runCompressor) {
		pneumaticsCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

		if (runCompressor) {
			pneumaticsCompressor.enableDigital();
		} else {
			pneumaticsCompressor.disable();
		}
	}

	/**
	 * Get the current pressure of the pressure tanks.
	 * @return current pressure of the pressure tanks
	 */
	public double getCompressorPressure() {
		return pneumaticsCompressor.getPressure();
	}

	/**
	 * Get the current consumption of the Compressor in Amps.
	 * @return current consumption of the Compressor in Amps
	 */
	public double getCompressorCurrent() {
		return pneumaticsCompressor.getCurrent();
	}

}
