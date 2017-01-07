package org.usfirst.frc.team1197.robot;

import java.util.LinkedList;
import java.util.List;

public class PivotTrajectory extends TorTrajectory {
	
	public PivotTrajectory(double goal){
		super(goal * (Math.PI/180.0));
		type = new String("Pivot");
		max_omg = 4.3 * 0.5; //4.3 * 0.5
		max_alf = 2*(Math.PI); //2*(Math.PI)
		max_jeta = 6.0; //6.0
		build(goal_pos, max_omg, max_alf, max_jeta, heading, omega, alpha);
	}
	
	public double lookUpHeading(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return goal_pos;
		}
		return heading.elementAt(i);
	}
	public double lookUpOmega(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return omega.elementAt(i);
	}
	public double lookUpAlpha(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return alpha.elementAt(i);
	}
	
	public double lookUpDisplacement(long t){
		return 0;
	}
	public double lookUpVelocity(long t){
		return 0;
	}
	public double lookUpAcceleration(long t){
		return 0;
	}
}