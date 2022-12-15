package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class LimeLight {

	private PhotonCamera camera;
	private static final double HUB_CAMERA_ANGLE = Math.toRadians(10); //RADIANS //30.25
	//private static final double BALL_CAMERA_ANGLE = Math.toRadians(); //RADIANS
	private static final double HUB_HEIGHT = 2.6; //METERS
	private static final double APRIL_TAG_HEIGHT = 0.4699; //METERS
	//private static final double BALL_HEIGHT = ; //METERS
	private static final double CAMERA_HEIGHT = 0.584; //METERS
	public static final double INVALID_RETURN = -2;
	private double last_seen_location = -1;
	//Vision pipeline: 0 is reflective tape, 1 is april tag, 2 is ball  
	/**
	 * LimeLight Constructor.
	 */
	public LimeLight() {
		camera = new PhotonCamera("gloworm");
	}

	public void update() {
		SmartDashboard.putNumber("Distance", getAprilTagDistance());
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

	public double getAprilTagTurningPower() {
		camera.setPipelineIndex(1);
		var result = camera.getLatestResult();
		if (!result.hasTargets()) {
			return last_seen_location;
		}
		double angle = result.getBestTarget().getYaw();
		//if (Math.abs(angle) < 6) {
			//return 0;
		//}
		//last_seen_location =  Math.abs(angle) / -angle;
		//return last_seen_location;
		if (Math.abs(angle) < 6) {
			last_seen_location = 0;
		}else{
			last_seen_location = Math.abs(angle) / -angle;
		}
		/*
		if (angle < 0) {
			last_seen_location = 1 - (1/((angle * angle / 10) + 1));
		} else {
			last_seen_location = (1 / ((angle * angle / 10) + 1)) - 1;
		}
		*/
		SmartDashboard.putNumber("Turning Power", last_seen_location);
		System.out.println(angle);
		SmartDashboard.updateValues();
		return last_seen_location;
	}

	public double getAprilTagDistance() {
		camera.setPipelineIndex(1);
		var result = camera.getLatestResult();
		if (result.hasTargets()) {
			//SmartDashboard.putNumber("angle", result.getBestTarget().getPitch());
			//SmartDashboard.updateValues();
			/*
			return PhotonUtils.calculateDistanceToTargetMeters(
								CAMERA_HEIGHT,
								APRIL_TAG_HEIGHT,
								HUB_CAMERA_ANGLE,
								Math.toRadians(result.getBestTarget().getPitch()));
			*/
			return (APRIL_TAG_HEIGHT - CAMERA_HEIGHT)/Math.tan(HUB_CAMERA_ANGLE+Math.toRadians(result.getBestTarget().getPitch()));

		}
		return -1;
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
