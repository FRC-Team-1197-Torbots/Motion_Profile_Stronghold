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
		build();
	}
	
	public boolean lookUpIsLast(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return true;
		}
		return isLast.elementAt(i);
	}
	
	public double lookUpHeading(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
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
	
	protected void build(){
		double adjusted_max_alf = Math.min(max_alf, Math.pow((0.5 * goal_pos * max_jeta * max_jeta), (1.0/3.0)));
		double adjusted_max_omg = Math.min(max_omg,
				(-adjusted_max_alf * adjusted_max_alf + Math.sqrt(adjusted_max_alf
						* adjusted_max_alf * adjusted_max_alf * adjusted_max_alf
						+ 4 * max_jeta * max_jeta * adjusted_max_alf
						* Math.abs(goal_pos))) / (2 * max_jeta));

		int f0_length = (int) Math.round((TorMotionProfile.getTimeInterval() + goal_pos / adjusted_max_omg) * (1/TorMotionProfile.getTimeInterval()));
		adjusted_max_omg = goal_pos/(f0_length * TorMotionProfile.getTimeInterval());
		int f1_length = (int) Math.ceil((adjusted_max_omg / adjusted_max_alf) / TorMotionProfile.getTimeInterval());
		int f2_length = (int) Math.ceil((adjusted_max_alf / max_jeta) / TorMotionProfile.getTimeInterval());
		int time = (int) (Math.ceil(f1_length + f2_length + f0_length));
		secondOrderFilter(f0_length, f1_length, f2_length, TorMotionProfile.getTimeInterval(), adjusted_max_omg, time);
	}
	
	protected void secondOrderFilter(int f0_length, int f1_length, int f2_length, double dt, double max_omg, int length) {

		List<Double> f1 = new LinkedList<Double>();
		for(int i = 0; i < f1_length; i++){
			f1.add(new Double(0));
		}
		List<Double> f2 = new LinkedList<Double>();
		for(int i = 0; i < f2_length; i++){
			f2.add(new Double(0));
		}
		
		double head = 0.0;
		double omg = 0.0;
		double alf = 0.0;
		double jeta = 0.0;
		
		double last_head = 0.0;
		double last_omg = 0.0;
		double last_alf = 0.0;
		double last_jeta = 0.0;
		
		double input;
		double FL1out;
		double FL2out;
		
		for (int i = 0; i < length; ++i) {
			
			last_head = head;
			last_omg = omg;
			last_alf = alf;
			last_jeta = jeta;
			
			if(f0_length > 0){
				input = max_omg;
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
			
			omg = FL2out;
			omega.addElement(new Double(omg));
			
			head = (last_omg + omg) / 2.0 * TorMotionProfile.getTimeInterval() + last_head;
			heading.addElement(new Double(head));
			
			alf = (omg - last_omg) / TorMotionProfile.getTimeInterval();
			alpha.addElement(new Double(alf));

			jeta = (alf - last_alf) / TorMotionProfile.getTimeInterval();
		}
		if(goal_pos < 0.0){
			for(int i = 0; i < length; i++){
				heading.set(i, -heading.elementAt(i));
				omega.set(i, -omega.elementAt(i));
			}
		}
	}
	public void execute(){
		startTime = System.currentTimeMillis();
		startTime = startTime - (startTime % 10);
		int length = heading.size();
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