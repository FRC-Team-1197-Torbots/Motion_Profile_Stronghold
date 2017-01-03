package org.usfirst.frc.team1197.robot;

import java.util.LinkedList;
import java.util.List;

public class LinearTrajectory extends TorTrajectory{
	
	public LinearTrajectory(double goal) {
		super(goal);
		type = new String("Linear");
		max_vel = 2.5;
		max_acc = 6.0;
		max_jerk = 6.0;
		build();
	}
	
	public boolean lookUpIsLast(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return true;//TODO: explain why 
		}
		return isLast.elementAt(i);
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
	
	
	public double lookUpOmega(long t){
		return 0;
	}
	public double lookUpAlpha(long t){
		return 0;
	}
	public double lookUpHeading(long t){
		return 0;
	}
	
	// The following magic was adapted from 254's TrajectoryLib.
	protected void build(){
		// This guarantees that if we don't have the runway to accelerate up to top speed
		// or to "jerk" up to top acceleration, we set more realistic goals:
		double disp = Math.abs(goal_pos); // Worry about sign later.
		double adjusted_max_acc = Math.min(max_acc,
										   Math.min(Math.sqrt(max_vel * max_jerk),
										            Math.pow((0.5 * disp * max_jerk * max_jerk), (1.0/3.0))));
		double adjusted_max_vel = Math.min( max_vel,
				(-adjusted_max_acc * adjusted_max_acc + Math.sqrt(adjusted_max_acc
						* adjusted_max_acc * adjusted_max_acc * adjusted_max_acc
						+ 4 * max_jerk * max_jerk * adjusted_max_acc * disp)) 
				/ (2 * max_jerk) );
		
		// Since each filter has a discrete number of slots, we need this "fancy rounding code" 
		// to make sure the robot always drives exactly the correct distance, and always
		// moves in a way that does not exceed the maximum velocity, acceleration, or
		// jerk. Without this, the use of a digital filter to produce motion profiles can
		// give unexpected results that violate these constraints, especially when
		// the path is short or the maximum velocity, acceleration, and/or jerk are high.
		double dt = TorMotionProfile.getTimeInterval();
		int f0_length = (int) (1 + Math.round(disp / (adjusted_max_vel * dt)));
		adjusted_max_vel = disp/(f0_length * dt);
		int f1_length = (int) Math.ceil(adjusted_max_vel / (adjusted_max_acc * dt));
		int f2_length = (int) Math.ceil(adjusted_max_acc / (max_jerk * dt));
		int tot_length = f0_length + f1_length + f2_length;
		secondOrderFilter(f0_length, f1_length, f2_length, dt, adjusted_max_vel, tot_length);
	}
	
	protected void secondOrderFilter(int f0_length, int f1_length, int f2_length, double dt, double max_vel, int tot_length) {
		// Why no "f0"? Because the zero-filter can be equivalently implemented more
		// simply by just feeding a constant velocity value into the first filter for
		// the correct length of time. The "real" filters, on the other hand, MUST be 
		// implemented as lists of numbers:
		List<Double> f1 = new LinkedList<Double>();
		for(int i = 0; i < f1_length; i++){
			f1.add(new Double(0));
		}
		List<Double> f2 = new LinkedList<Double>();
		for(int i = 0; i < f2_length; i++){
			f2.add(new Double(0));
		}
		
		// Inputs and outputs of the filters:
		double input;
		double FL1out;
		double FL2out;
		// Individual data values:
		double pos = 0.0;
		double vel = 0.0;
		double acc = 0.0;
		// record previous values so we can do integration/differentiation:
		double last_pos = 0.0;
		double last_vel = 0.0;
		
		for (int i = 0; i < tot_length; ++i) {
			last_pos = pos;
			last_vel = vel;
			// As long as the zero filter is not empty, feed-forward the target velocity:
			if(f0_length > 0){
				input = max_vel;
				f0_length--; // Decrement the zero filter.
			}
			else{
				input = 0.0;
			}

			// Input goes at the beginning of the filter (end of the list):
			f1.add(new Double(input));
			// Throw away the element at the end of the filter (beginning of the list)
			f1.remove(0);
			//  Output is the average of the elements in the first filter:
			FL1out = average(f1);
			// Repeat this procedure for the second filter:
			f2.add(new Double(FL1out));
			f2.remove(0);
			FL2out = average(f2);
			
			// The output of the filter is velocity:
			vel = FL2out;
			velocity.addElement(new Double(vel));
		    // We have to integrate to get position. This uses trapezoidal integration,
		    // but the choice of integration strategy probably doesn't matter:
			pos = last_pos + 0.5 * (last_vel + vel) * dt;
			displacement.addElement(new Double(pos));
			// We have to differentiate to get acceleration:
			acc = (vel - last_vel) / TorMotionProfile.getTimeInterval();
			acceleration.addElement(new Double(acc));
		}
		
		// It's later, time to worry about sign:
		if(goal_pos < 0.0){
			for(int i = 0; i < displacement.size(); i++){
				displacement.set(i, -displacement.elementAt(i));
				velocity.set(i, -velocity.elementAt(i));
				acceleration.set(i, -acceleration.elementAt(i));
			}
		}
	}
	
	public void execute(){
		startTime = System.currentTimeMillis();
		startTime = startTime - (startTime % 10);
		int length = displacement.size();
		time.clear();
		isLast.clear();
		for(int i = 0; i < length; i++){
			time.addElement(new Long(startTime + (long)(1000 * i * TorMotionProfile.getTimeInterval())));
			isLast.addElement(false);
			if(i + 1 == length){
				isLast.set(i, true);
			}
		}
		TorMotionProfile.loadTrajectory(this);
	}
}