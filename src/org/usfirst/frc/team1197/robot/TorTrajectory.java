
package org.usfirst.frc.team1197.robot;

import java.util.Vector;
import java.util.List;

public abstract class TorTrajectory {
	protected double goal_pos = 0.0;
	
	protected double max_vel = 1000.0;
	protected double max_acc = 1000.0;
	protected double max_jerk = 1000.0;
	
	protected double max_omg;
	protected double max_alf;
	protected double max_jeta;
	
	protected Vector<Long> time;
	protected Vector<Boolean> isLast;
	
	protected Vector<Double> displacement;
	protected Vector<Double> velocity;
	protected Vector<Double> acceleration;
	
	protected Vector<Double> omega;
	protected Vector<Double> alpha;
	protected Vector<Double> heading;
	
	protected static long startTime;
	protected String type;
	
	public TorTrajectory(double goal){
		goal_pos = goal;
		type = new String("null");
		
		time = new Vector<Long>();
		isLast = new Vector<Boolean>();
		
		displacement = new Vector<Double>();
		velocity = new Vector<Double>();
		acceleration = new Vector<Double>();
		
		omega = new Vector<Double>();
		alpha = new Vector<Double>();
		heading = new Vector<Double>();
	}
	
	public TorTrajectory(){
		
	}
	
	protected void build(){
		
	}
	
	protected void secondOrderFilter(int f0_length, int f1_length, int f2_length, double dt, double max_vel, int length) {

	}
	
	public void execute(){
		
	}
	
	public abstract double lookUpDisplacement(long t);
	public abstract double lookUpVelocity(long t);
	public abstract double lookUpAcceleration(long t);
	
	public abstract double lookUpAlpha(long t);
	public abstract double lookUpOmega(long t);
	public abstract double lookUpHeading(long t);	
	
	public abstract boolean lookUpIsLast(long t);
	
	public double average(List<Double> list){
		double avg = 0;
		for(Double element : list){
			avg += element;
		}
		avg /= list.size();
		return avg;
	}
}
