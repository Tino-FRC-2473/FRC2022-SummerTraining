package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class LimeLight {

	private PhotonCamera camera;
	private static final double HUB_CAMERA_ANGLE = Math.toRadians(30.25); //RADIANS
	//private static final double BALL_CAMERA_ANGLE = Math.toRadians(); //RADIANS
	private static final double HUB_HEIGHT = 2.6; //METERS
	//private static final double BALL_HEIGHT = ; //METERS
	private static final double CAMERA_HEIGHT = 0.55; //METERS
	public static final double INVALID_RETURN = -2;
	//Vision pipeline: 0 is reflective tape, 1 is ball, 2 is april tag  
	/**
	 * LimeLight Constructor.
	 */
	public LimeLight() {
		camera = new PhotonCamera("gloworm");
	}

	public void update() {
		SmartDashboard.putNumber("Distance", getHubDistance());
		SmartDashboard.updateValues();

	}

	public double getHubDistance() {
		camera.setPipelineIndex(0);
		var result = camera.getLatestResult();
		if (result.hasTargets()) {
			return PhotonUtils.calculateDistanceToTargetMeters(
								CAMERA_HEIGHT,
								HUB_HEIGHT,
								HUB_CAMERA_ANGLE,
								Math.toRadians(result.getBestTarget().getPitch()));
		}
		return -1;
	}
	
	/* 
	public double getBallDistance() {
		camera.setPipelineIndex(1);
		var result = camera.getLatestResult();
		if (result.hasTargets()) {
			return PhotonUtils.calculateDistanceToTargetMeters(
								CAMERA_HEIGHT,
								BALL_HEIGHT,
								BALL_CAMERA_ANGLE,
								Math.toRadians(result.getBestTarget().getPitch()));
		}
		return -1;
	}
	*/
	
	public double getHubTurningPower() {
		camera.setPipelineIndex(0);
		var result = camera.getLatestResult();
		if (!result.hasTargets()) {
			return INVALID_RETURN;
		}
		double angle = result.getBestTarget().getYaw();
		if (Math.abs(angle) < 6) {
			return 0;
		}
		return Math.abs(angle) / -angle;
	}
	/*
	public double getBallTurningPower() {
		camera.setPipelineIndex(1);
		var result = camera.getLatestResult();
		if (!result.hasTargets()) {
			return INVALID_RETURN;
		}
		double angle = result.getBestTarget().getYaw();
		if (Math.abs(angle) < 6) {
			return 0;
		}
		return Math.abs(angle) / -angle;
	}
	*/
}
