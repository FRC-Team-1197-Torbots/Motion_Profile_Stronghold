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
			return true;
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
	
	protected void build(){
		double adjusted_max_acc = Math.min(max_acc, Math.pow((0.5 * goal_pos * max_jerk * max_jerk), (1.0/3.0)));
		double adjusted_max_vel = Math.min(max_vel,
				(-adjusted_max_acc * adjusted_max_acc + Math.sqrt(adjusted_max_acc
						* adjusted_max_acc * adjusted_max_acc * adjusted_max_acc
						+ 4 * max_jerk * max_jerk * adjusted_max_acc
						* Math.abs(goal_pos))) / (2 * max_jerk));

		int f0_length = (int) Math.round((TorMotionProfile.getTimeInterval() + goal_pos / adjusted_max_vel) * (1/TorMotionProfile.getTimeInterval()));
		adjusted_max_vel = goal_pos/(f0_length * TorMotionProfile.getTimeInterval());
		int f1_length = (int) Math.ceil((adjusted_max_vel / adjusted_max_acc) / TorMotionProfile.getTimeInterval());
		int f2_length = (int) Math.ceil((adjusted_max_acc / max_jerk) / TorMotionProfile.getTimeInterval());
		int time = (int) (Math.ceil(f0_length + f1_length + f2_length));
		secondOrderFilter(f0_length, f1_length, f2_length, TorMotionProfile.getTimeInterval(), adjusted_max_vel, time);
	}
	
	protected void secondOrderFilter(int f0_length, int f1_length, int f2_length, double dt, double max_vel, int length) {
		List<Double> f1 = new LinkedList<Double>();
		for(int i = 0; i < f1_length; i++){
			f1.add(new Double(0));
		}
		List<Double> f2 = new LinkedList<Double>();
		for(int i = 0; i < f2_length; i++){
			f2.add(new Double(0));
		}
		
		long t = 0;
		
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
			
			f1.add(new Double(input));
			f1.remove(0);
			
			FL1out = average(f1);
			
			f2.add(new Double(FL1out));
			f2.remove(0);
			
			FL2out = average(f2);
			
			vel = FL2out;
			velocity.addElement(new Double(vel));
			
			pos = (last_vel + vel) / 2.0 * TorMotionProfile.getTimeInterval() + last_pos;
			displacement.addElement(new Double(pos));
			
			acc = (vel - last_vel) / TorMotionProfile.getTimeInterval();
			acceleration.addElement(new Double(acc));

			jerk = (acc - last_acc) / TorMotionProfile.getTimeInterval();
			
//			time.addElement(new Long(t));
//			t += (long) (1000 * TorMotionProfile.getTimeInterval());
		}
		
		if(goal_pos < 0.0){
			for(int i = 0; i < length; i++){
				displacement.set(i, -displacement.elementAt(i));
				velocity.set(i, -velocity.elementAt(i));
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