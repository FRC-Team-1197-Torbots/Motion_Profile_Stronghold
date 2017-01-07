package org.usfirst.frc.team1197.robot;

public class JoystickTrajectory extends TorTrajectory{
	
	private double pos;
	private double vel;
	private double acc;
	private double jrk;
	
	private double hed;
	private double omg;
	private double alf;
	private double jta;
	
	private double p;
	private double v;
	
	private double tgt_vel;
	private double tgt_omg;
	
	public JoystickTrajectory(){
		
	}
	
	public double lookUpDisplacement(long t){
		return 0.0;
	}
	public double lookUpVelocity(long t){
		return 0.0;
	}
	public double lookUpAcceleration(long t){
		return 0.0;
	}
	
	public double lookUpHeading(long t){
		return 0.0;
	}
	public double lookUpOmega(long t){
		return 0.0;
	}
	public double lookUpAlpha(long t){
		return 0.0;
	}
	
	public void setTargets(double v, double w){
		tgt_vel = v;
		tgt_omg = w;
	}
	
	public void execute(){
		
	}
	
	public void update(double tgt_vel, double pos, double vel, double acc, double jrk){
		double a_sign;
		if (tgt_vel < vel)
			a_sign = -1;
		else if (tgt_vel > vel)
			a_sign = 1;
		else a_sign = 0;
		double reachable_acc = Math.abs(acc) + max_jerk*dt;
		double reachable_vel = Math.abs(vel) + Math.abs(acc);
		acc = a_sign*Math.min(max_acc, reachable_acc);
		
		
		
		
	}
	
	public void updateVelocity(){
		
	}
	
	public void updateOmega(){
		
	}
}
