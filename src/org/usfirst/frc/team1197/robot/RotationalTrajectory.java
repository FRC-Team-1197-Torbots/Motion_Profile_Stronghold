package org.usfirst.frc.team1197.robot;

public class RotationalTrajectory extends TorTrajectory {
	
	public RotationalTrajectory(double goal){
		super(goal * (Math.PI/180.0));
		type = new String("Rotational");
		max_vel = 9.050;
		max_acc = 28.96;
		max_jerk = 47.06;
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