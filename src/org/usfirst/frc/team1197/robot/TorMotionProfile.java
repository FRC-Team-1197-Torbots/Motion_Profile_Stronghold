package org.usfirst.frc.team1197.robot;

public class TorMotionProfile {
	
	private static boolean isActive = false;
	private static TorTrajectory activeTrajectory = null;
	
	public static double lookUpDisplacement(long t){
		return activeTrajectory.lookUpDisplacement(t);
	}
	public static double lookUpVelocity(long t){
		return activeTrajectory.lookUpVelocity(t);
	}
	public static double lookUpAcceleration(long t){
		return activeTrajectory.lookUpAcceleration(t);
	}
	
	public static double lookUpHeading(long t){
		return activeTrajectory.lookUpHeading(t);
	}
	public static double lookUpOmega(long t){
		return activeTrajectory.lookUpOmega(t);
	}
	public static double lookUpAlpha(long t){
		return activeTrajectory.lookUpAlpha(t);
	}
	
	public static boolean lookUpIsLast(long t){
		return activeTrajectory.lookUpIsLast(t);
	}
	
	public static void loadTrajectory(TorTrajectory traj){
		TorCAN.INSTANCE.resetHeading();
		TorCAN.INSTANCE.resetEncoder();
		activeTrajectory = traj;
	}
	
	public static void setActive(){
		isActive = true;
	}
	public static void setInactive(){
		isActive = false;
	}
	public static boolean isActive(){
		return isActive;
	}
}
