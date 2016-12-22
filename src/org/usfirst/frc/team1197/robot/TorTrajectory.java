package org.usfirst.frc.team1197.robot;

import java.util.Vector;
import java.util.LinkedList;
import java.util.List;

public abstract class TorTrajectory {
	
	protected double goal_pos = 0.0;
	protected double max_vel = 1000.0;
	protected double max_acc = 1000.0;
	protected double max_jerk = 1000.0;
	protected static final double dt = 0.010;
	
	protected Vector<Long> time;
	protected Vector<Double> displacement;
	protected Vector<Double> velocity;
	protected Vector<Boolean> isLast;
	protected Vector<Double> acceleration;
	
	protected static long startTime;
	
	protected String type;
	
	public TorTrajectory(double goal){
		goal_pos = goal;
		type = new String("null");
		displacement = new Vector<Double>();
		velocity = new Vector<Double>();
		time = new Vector<Long>();
		isLast = new Vector<Boolean>();
		acceleration = new Vector<Double>();
//		build();
	}
	
	public TorTrajectory(){
		
	}
	
	protected void build(){
		double adjusted_max_acc = Math.min(max_acc, Math.pow((0.5 * goal_pos * max_jerk * max_jerk), (1.0/3.0)));
		double adjusted_max_vel = Math.min(max_vel,
				(-adjusted_max_acc * adjusted_max_acc + Math.sqrt(adjusted_max_acc
						* adjusted_max_acc * adjusted_max_acc * adjusted_max_acc
						+ 4 * max_jerk * max_jerk * adjusted_max_acc
						* Math.abs(goal_pos))) / (2 * max_jerk));

		// Compute the length of the linear filters and impulse.
		int f0_length = (int) Math.round((dt + goal_pos / adjusted_max_vel) * (1/dt));
		adjusted_max_vel = goal_pos/(f0_length * dt);
		int f1_length = (int) Math.ceil((adjusted_max_vel / adjusted_max_acc) / dt);
		int f2_length = (int) Math.ceil((adjusted_max_acc / max_jerk) / dt);
//		double impulse = (Math.abs(goal_pos) / adjusted_max_vel) / dt;
		int time = (int) (Math.ceil(f1_length + f2_length + f0_length));
		secondOrderFilter(f0_length, f1_length, f2_length, dt, 0, adjusted_max_vel, time);
	}
	
	public abstract double lookUpDisplacement(long t);
	public abstract double lookUpVelocity(long t);
	public abstract double lookUpAcceleration(long t);
	public abstract double lookUpAlpha(long t);
	public abstract double lookUpOmega(long t);
//	public abstract double lookUpHeading(long t);	
	public abstract boolean lookUpIsLast(long t);
	
	protected void secondOrderFilter(
			int f0_length,
			int f1_length,
			int f2_length,
			double dt,
			double start_vel,
			double max_vel,
			int length ) {

		// f2 is the average of the last f2_length samples from f1, so while we
		// can recursively compute f2's sum, we need to keep a buffer for f1.
		List<Double> f1 = new LinkedList<Double>();
		for(int i = 0; i < f1_length; i++){
			f1.add(new Double(0));
		}
//		f1[0] = (start_vel / max_vel) * f1_length;
		List<Double> f2 = new LinkedList<Double>();
		for(int i = 0; i < f2_length; i++){
			f2.add(new Double(0));
		}
		
		double pos = 0.0;
		double vel = 0.0;
		double acc = 0.0;
		double jerk = 0.0;
		
		double last_pos = 0.0;
		double last_vel = 0.0;
		double last_acc = 0.0;
		double last_jerk = 0.0;
		
		double input;
		double FL1out;
		double FL2out;
		
		for (int i = 0; i < length; ++i) {
			// Apply input
//			double input = Math.min(f0_length, 1);
			
			last_pos = pos;
			last_vel = vel;
			last_acc = acc;
			last_jerk = jerk;
			
			if(f0_length > 0){
				input = max_vel;
			}
			else{
				input = 0.0;
			}
			
			f0_length--;
			
//			if (input < 1) {
//				// The impulse is over, so decelerate
//				input -= 1;
//				total_impulse = 0;
//			} 
//			else {
//				total_impulse -= input;
//			}

			// Filter through F1
//			double f1_last;
//			if (i > 0) {
//				f1_last = f1[i - 1];
//			} else {
//				f1_last = f1[0];
//			}
//			f1[i] = Math.max(0.0, Math.min(f1_length, f1_last + input));
			

//			f2 = 0;
			// Filter through F2
//			for (int j = 0; j < f2_length; ++j) {
//				if (i - j < 0) {
//					break;
//				}
//
//				f2 += f1[i - j];
//			}
//			f2 = f2 / f1_length;
			
			f1.add(new Double(input));
			f1.remove(0);
			
			FL1out = average(f1);
			
			f2.add(new Double(FL1out));
			f2.remove(0);
			
			FL2out = average(f2);
			
			// Velocity is the normalized sum of f2 * the max velocity
//			vel = f2 / f2_length * max_vel;
			
			vel = FL2out;
			velocity.addElement(new Double(vel));
			
			pos = (last_vel + vel) / 2.0 * dt + last_pos;
			displacement.addElement(new Double(pos));
			
			acc = (vel - last_vel) / dt;
			acceleration.addElement(new Double(acc));

			// Acceleration and jerk are the differences in velocity and
			// acceleration, respectively.
			jerk = (acc - last_acc) / dt;

//			last_pos = pos;
//			last_vel = vel;
//			last_acc = acc;
//			last_jerk = jerk;
		}
		if(goal_pos < 0.0){
			for(int i = 0; i < length; i++){
				displacement.set(i, -displacement.elementAt(i));
				velocity.set(i, -velocity.elementAt(i));
			}
		}
	}
	
	public double average(List<Double> list){
		double avg = 0;
		for(Double element : list){
			avg += element;
		}
		avg /= list.size();
		return avg;
	}
	
	public void execute(){
		startTime = System.currentTimeMillis();
		startTime = startTime - (startTime % 10);
		int length = displacement.size();
		time.clear();
		isLast.clear();
		for(int i = 0; i < length; i++){
			displacement.set(i, displacement.elementAt(i) + TorMotionProfile.getWayPoint());
			time.addElement(new Long(startTime + (10 * i)));
			isLast.addElement(false);
			if(i + 1 == length){
				isLast.set(i, true);
			}
		}
		TorMotionProfile.loadTrajectory(this);
	}
}
