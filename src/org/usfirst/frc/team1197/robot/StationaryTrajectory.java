package org.usfirst.frc.team1197.robot;

public class StationaryTrajectory extends TorTrajectory{
	
	public StationaryTrajectory() {
		type = new String("Stationary");
	}
	
	public double lookUpDisplacement(long t){
		return goal_pos;
	}
	
	public double lookUpVelocity(long t){
		return 0.0;
	}
	
	public double lookUpAcceleration(long t){
		return 0.0;
	}
	
	public boolean lookUpIsLast(long t){
		return false;
	}
	
	public double lookUpOmega(long t){
		return 0;
	}
	
	public double lookUpAlpha(long t){
		return 0;
	}
	
	public void execute(){
		goal_pos = TorMotionProfile.getWayPoint();
		TorMotionProfile.loadTrajectory(this);
	}

}
