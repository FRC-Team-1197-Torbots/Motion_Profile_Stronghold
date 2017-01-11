package org.usfirst.frc.team1197.robot;

public class JoystickTrajectory extends TorTrajectory{
	
	private Motion linearMotion;
	private Motion rotationalMotion;
	private double tgt_vel;
	private double tgt_omg;
	
	public JoystickTrajectory(){
//		super(0.0);
		linearMotion = new Motion(0.0, 0.0, 0.0);
		rotationalMotion = new Motion(0.0, 0.0, 0.0);
	}
	
	public double lookUpDisplacement(long t){
		return linearMotion.pos;
	}
	public double lookUpVelocity(long t){
		return linearMotion.vel;
	}
	public double lookUpAcceleration(long t){
		return linearMotion.acc;
	}
	
	public double lookUpHeading(long t){
		return rotationalMotion.pos;
	}
	public double lookUpOmega(long t){
		return rotationalMotion.vel;
	}
	public double lookUpAlpha(long t){
		return rotationalMotion.acc;
	}
	
	public boolean lookUpIsLast(long t){
		return false;
	}
	
	public void setTargets(double v, double w){
		tgt_vel = v;
		tgt_omg = w;
	}
	
	public void execute(double p_init, double v_init, double h_init, double w_init){
		linearMotion.pos = p_init;
		linearMotion.vel = v_init;
		linearMotion.acc = 0.0;
		
		rotationalMotion.pos = h_init;
		rotationalMotion.vel = w_init;
		rotationalMotion.acc = 0.0;
		
		TorMotionProfile.INSTANCE.loadTrajectory(this);
	}
	
	public void update(double tgt_vel, Motion m){
		// Target (requested) acceleration:
		double tgt_acc = (tgt_vel - m.vel) / dt;
		double a_sign = sign(tgt_acc);
		tgt_acc = a_sign*Math.min(Math.abs(tgt_acc), max_acc);
		// Target (requested) jerk:
		double tgt_jrk = (tgt_acc - m.acc) / dt;
		double j_sign = sign(tgt_jrk);
		// Actual jerk, based on constraints:
		double jerk = j_sign*Math.min(Math.abs(tgt_jrk), max_jerk);
		// Actual acceleration:
		m.acc = m.acc + jerk*dt;
		// Velocity:
		m.vel = m.vel + m.acc*dt + 0.5*jerk*dt*dt;
		// Position:
		m.pos = m.pos + m.vel*dt + 0.5*m.acc*dt*dt + 0.1666667*jerk*dt*dt*dt;	
	}
	
	public void updateVelocity(){
		update(tgt_vel, linearMotion);
	}
	
	public void updateOmega(){
		update(tgt_omg, rotationalMotion);
	}
	
	public void updateDt(double dt){
		if(dt == 0.0){
			dt = 0.005;
		}
		else{
			this.dt = dt;
		}
	}
	
	private double sign(double x){
		if (x > 0)
			return 1.0;
		else if (x < 0)
			return -1.0;
		else return 0.0;
	}
}
