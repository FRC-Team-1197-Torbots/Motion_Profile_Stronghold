package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TorMotionProfile {
	
	private static boolean isActive = false;
	private static TorTrajectory activeTrajectory = null;
	private static final double timeInterval = 0.005;
	
	private static double lastOmega;
	private static double lastVelocity;
	
	private static double targetVelocity;
	private static double targetAcceleration;
	private static double targetDisplacement;
	
	private static double currentVelocity;
	private static double currentAcceleration;
	private static double currentDisplacement;
	
	private static double targetOmega;
	private static double targetAlpha;
	private static double targetHeading;
	
	private static double currentOmega;
	private static double currentAlpha;
	private static double currentHeading;
	
	private static double v;
	private static double w;
	
	private static double displacementError;
	private static double headingError;
	private static double lastHeadingError;
	private static double totalDisplacementError;
	private static double totalHeadingError;
	
	private static double kA = 0*0.02; //0.02
	private static double kP = 0*1.5;  //2.0
	private static double kI = 0*0.0;  //0.0
	private static double kD = 0*0.5;  //0.5

	private static double kw = 10.0;
	private static double ka = 0*1.8;//0.05
	private static double kp = 0*2.3;//1.0
	private static double ki = 0.0;
	private static double kd = 0*1.1;//0.5

	private static double dt = 0.005;
	
	private static long currentTime;
	private static long lastTime;
	
	private static double dDispErrordt;
	private static double dHeadErrordt;
	
	private static StationaryTrajectory stationaryTraj = new StationaryTrajectory();
	private static TorDerivative displacementDerivative = new TorDerivative(TorMotionProfile.getTimeInterval());
	private static TorDerivative headingDerivative = new TorDerivative(TorMotionProfile.getTimeInterval());
	
	public static double lookUpDisplacement(long t){
		return activeTrajectory.lookUpDisplacement(t);
	}
	public static double lookUpVelocity(long t){
		return activeTrajectory.lookUpVelocity(t);
	}
	public static double lookUpAcceleration(long t){
		return activeTrajectory.lookUpAcceleration(t);
	}
	
	public static double lookUpHeading(long t){
		return activeTrajectory.lookUpHeading(t);
	}
	public static double lookUpOmega(long t){
		return activeTrajectory.lookUpOmega(t);
	}
	public static double lookUpAlpha(long t){
		return activeTrajectory.lookUpAlpha(t);
	}
	
	public static boolean lookUpIsLast(long t){
		return activeTrajectory.lookUpIsLast(t);
	}
	
	public static void loadTrajectory(TorTrajectory traj){
		TorCAN.INSTANCE.resetEncoder();
		TorCAN.INSTANCE.resetHeading();
		displacementError = 0.0;
		headingError = 0.0;
		displacementDerivative.reset();
		headingDerivative.reset();
		activeTrajectory = traj;
	}
	
	public static void setActive(){
		isActive = true;
	}
	public static void setInactive(){
		isActive = false;
	}
	public static boolean isActive(){
		return isActive;
	}
	
	public static double getTimeInterval(){
		return timeInterval;
	}
	
	public static void run(){
		if(TorMotionProfile.isActive()){			
			currentTime = System.currentTimeMillis();
			dt = (currentTime - lastTime) * 0.001;
			if(dt == 0){
				dt = 0.001;
			}
			
			currentDisplacement = TorCAN.INSTANCE.getDisplacement();
			currentVelocity = TorCAN.INSTANCE.getVelocity();
			currentAcceleration = (currentVelocity - lastVelocity) / dt;
			
			currentHeading = TorCAN.INSTANCE.getHeading();
			currentOmega = TorCAN.INSTANCE.getOmega();
			currentAlpha = (currentOmega - lastOmega) / dt;
			
			currentTime = (currentTime - (currentTime % ((long)(TorMotionProfile.getTimeInterval() * 1000))));
			if(TorMotionProfile.lookUpIsLast(currentTime)){
				stationaryTraj.execute();
			}
			
			targetDisplacement = TorMotionProfile.lookUpDisplacement(currentTime);
			targetVelocity = TorMotionProfile.lookUpVelocity(currentTime);
			targetAcceleration = TorMotionProfile.lookUpAcceleration(currentTime);	
			
//			SmartDashboard.putNumber("targetVelocity", targetVelocity);
//			SmartDashboard.putNumber("targetAcceleration", targetAcceleration);
//			SmartDashboard.putNumber("targetDisplacement", targetDisplacement);
			SmartDashboard.putNumber("currentDisplacement", currentDisplacement);
//			SmartDashboard.putNumber("currentVelocity", currentVelocity);
//			SmartDashboard.putNumber("currentAcceleration", currentAcceleration);
			
			displacementError = targetDisplacement - currentDisplacement;
//			SmartDashboard.putNumber("displacementError", displacementError);

			totalDisplacementError += displacementError * dt;
			dDispErrordt = displacementDerivative.Estimate(displacementError);
			
			//TODO figure why we need to do this.
			if(Math.abs(dDispErrordt) > 5.0){ 
				dDispErrordt = 0.0;
			}
			
//			SmartDashboard.putNumber("dDispErrordt", dDispErrordt);
			
			if(targetAcceleration > 0){
				currentDisplacement -= (0.5 * TorCAN.INSTANCE.getBacklash());
			}
			else if(targetAcceleration < 0){
				currentDisplacement += (0.5 * TorCAN.INSTANCE.getBacklash());
			}
			else{
				if(Math.abs(displacementError) < (0.5 * TorCAN.INSTANCE.getBacklash())){
					displacementError = 0;
				}
			}
			
			v = targetVelocity 
					+ (kA * (targetAcceleration *(1.0 + 0.0 * 0.5 * Math.pow(targetVelocity, 2))))
					+ (kP * displacementError) 
					+ (kI * totalDisplacementError)
					+ (kD * dDispErrordt);

			targetHeading = TorMotionProfile.lookUpHeading(currentTime);
			while(targetHeading < -Math.PI) targetHeading += 2*Math.PI;
			while(targetHeading > Math.PI) targetHeading -= 2*Math.PI;
			
			targetOmega = TorMotionProfile.lookUpOmega(currentTime);
			targetAlpha = TorMotionProfile.lookUpAlpha(currentTime);
			
			while(currentHeading < -Math.PI){currentHeading += 2*Math.PI;}
			while(currentHeading > Math.PI){currentHeading -= 2*Math.PI;}

			headingError = targetHeading - currentHeading;
			if (Math.abs(headingError) > (Math.PI)) {
				if (headingError > 0.0) {
					headingError -= (2*(Math.PI));
				} else {
					headingError += (2*(Math.PI));
				}
			}
			totalHeadingError += headingError*dt;
			dHeadErrordt = headingDerivative.Estimate(headingError);
			
			//TODO figure out why we need to do this
			if(Math.abs(dHeadErrordt) > 5.0){
				dHeadErrordt = 0.0;
			}
			
			SmartDashboard.putNumber("targetOmega", targetOmega);
//			SmartDashboard.putNumber("targetAlpha", targetAlpha);
			SmartDashboard.putNumber("targetHeading", targetHeading);
			SmartDashboard.putNumber("currentHeading", currentHeading);
			SmartDashboard.putNumber("currentOmega", currentOmega);
//			SmartDashboard.putNumber("currentAlpha", currentAlpha);
			SmartDashboard.putNumber("headingError", headingError);
			SmartDashboard.putNumber("dHeadErrordt", dHeadErrordt);
			
//			System.out.println(currentOmega);
			
			w = kw * targetOmega 
					+ (ka * (targetAlpha *(1.0 + 0.5*Math.pow(targetOmega, 2))))
					+ (kp * headingError) 
					+ (ki * totalHeadingError)
					+ (kd * dHeadErrordt);

			TorCAN.INSTANCE.setTargets(v, w);
			
			lastHeadingError = headingError;
			
			lastTime = currentTime;
			
			lastVelocity = currentVelocity;
			lastOmega = currentOmega;
		}
	}
}
