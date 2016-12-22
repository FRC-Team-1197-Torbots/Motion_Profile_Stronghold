package org.usfirst.frc.team1197.robot;

public class LinearTrajectory extends TorTrajectory{
	
	public LinearTrajectory(double goal) {
		super(goal);
		type = new String("Linear");
		max_vel = 2.5;
		max_acc = 8.0;
		max_jerk = 13.0;
		build();
	}
	
	public double lookUpDisplacement(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return displacement.elementAt(i);
	}
	
	public double lookUpVelocity(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return velocity.elementAt(i);
	}
	
	public double lookUpAcceleration(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return acceleration.elementAt(i);
	}
	
	public boolean lookUpIsLast(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return true;
		}
		return isLast.elementAt(i);
	}
	
	public double lookUpOmega(long t){
		return 0;
	}
	
	public double lookUpAlpha(long t){
		return 0;
	}
	
}