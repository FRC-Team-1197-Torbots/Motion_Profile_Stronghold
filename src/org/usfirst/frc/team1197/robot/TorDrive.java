package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TorDrive
{	
	private boolean isHighGear = true;
	private Solenoid m_solenoidshift;

	private double rightMotorSpeed;
	private double leftMotorSpeed;
	private TorJoystickProfiles joystickProfile;
	private double targetSpeed;
	private double trackWidth = 0.5525; //meters, in inches 21.75
	private double halfTrackWidth = trackWidth / 2.0;
	private double centerRadius = 0.0;
	private double maxThrottle;
	private double approximateSensorSpeed;

	private double lastOmega;
	
	private double targetVelocity;
	private double targetAcceleration;
	private double targetDisplacement;
	
	private double targetOmega;
	private double targetAlpha;
	private double targetHeading;
	
	private double currentHeading;
	
	private double v;
	private double w;
	
	private double displacementError;
	private double headingError;
	private double lastDisplacementError;
	private double lastHeadingError;
	private double totalDisplacementError;
	private double totalHeadingError;
	
	private double kA = 0.2; //0.025
	private double kP = 0.01;  //1.0
	private double kI = 0.0;  //0.0
	private double kD = 0.005;  //0.5

	private double ka = 0.0;//0.2
	private double kp = 0.0;//1.0
	private double ki = 0.0;
	private double kd = 0.0;//0.5

	private double dt = 0.01;
	
	private TorTrajectory linearTrajectory;
	private TorTrajectory pivotTrajectory;
	
	private long currentTime;
	
	private StationaryTrajectory stationaryTraj;
	
	class PeriodicRunnable implements java.lang.Runnable {
		public void run() {
			if(TorMotionProfile.isActive()){
				currentTime = System.currentTimeMillis();
				currentTime = (currentTime - (currentTime % ((long)(dt * 1000))));
				if(TorMotionProfile.lookUpIsLast(currentTime)){
					stationaryTraj.execute();
				}
				
				targetDisplacement = TorMotionProfile.lookUpDisplacement(currentTime);
				targetVelocity = TorMotionProfile.lookUpVelocity(currentTime);
				targetAcceleration = TorMotionProfile.lookUpAcceleration(currentTime);	
				
//				SmartDashboard.putNumber("targetVelocity", targetVelocity);
//				SmartDashboard.putNumber("targetAcceleration", targetAcceleration);
//				SmartDashboard.putNumber("targetDisplacement", targetDisplacement);
//				SmartDashboard.putNumber("getDisplacement", TorCAN.INSTANCE.getDisplacement());
//				SmartDashboard.putNumber("getVelocity", TorCAN.INSTANCE.getVelocity());
//				SmartDashboard.putNumber("getAcceleration", ((TorCAN.INSTANCE.getVelocity() - lastVelocity) / dt));
//				lastVelocity = TorCAN.INSTANCE.getVelocity();
				
				displacementError = targetDisplacement - TorCAN.INSTANCE.getDisplacement();
				totalDisplacementError += displacementError*dt;
				
				v = targetVelocity 
						+ (kA * (targetAcceleration *(1.0 + 0.5*Math.pow(targetVelocity, 2))))
						+ (kP * displacementError) 
						+ (kI * totalDisplacementError)
						+ (kD * (((displacementError - lastDisplacementError) / (dt))));

				targetHeading = TorMotionProfile.lookUpHeading(currentTime);
				while(targetHeading < 0) targetHeading += 2*Math.PI;
				while(targetHeading > 2*Math.PI) targetHeading -= 2*Math.PI;
				
				targetOmega = TorMotionProfile.lookUpOmega(currentTime);
				targetAlpha = TorMotionProfile.lookUpAlpha(currentTime);
				currentHeading = TorCAN.INSTANCE.getHeading();
				
			//	while(currentHeading < 0){currentHeading += 2*Math.PI;}
			//	while(currentHeading > 2*Math.PI){currentHeading -= 2*Math.PI;}

				headingError = (targetHeading - currentHeading);
				if (Math.abs(headingError) > (Math.PI)) {
					if (headingError > 0.0D) {
						headingError -= (2*(Math.PI));
					} else {
						headingError += (2*(Math.PI));
					}
				}
				totalHeadingError += headingError*dt;
				
				SmartDashboard.putNumber("targetOmega", targetOmega);
				SmartDashboard.putNumber("targetAlpha", targetAlpha);
				SmartDashboard.putNumber("targetHeading", targetHeading);
				SmartDashboard.putNumber("getHeading", currentHeading);
				SmartDashboard.putNumber("getOmega", TorCAN.INSTANCE.getOmega());
				SmartDashboard.putNumber("getAlpha", ((TorCAN.INSTANCE.getOmega() - lastOmega) / dt));
				lastOmega = TorCAN.INSTANCE.getOmega();
				
				w = targetOmega 
						+ (ka * (targetAlpha *(1.0 + 0.5*Math.pow(targetOmega, 2))))
						+ (kp * headingError) 
						+ (ki * totalHeadingError)
						+ (kd * (((headingError - lastHeadingError) / (dt))));

				TorCAN.INSTANCE.setTargets(v, w);
				
				lastDisplacementError = displacementError;
				lastHeadingError = headingError;
			}
		}
	}
	Notifier mpNotifier = new Notifier(new PeriodicRunnable());

	public TorDrive(Joystick stick, Solenoid shift, double approximateSensorSpeed)
	{
		joystickProfile = new TorJoystickProfiles();
		linearTrajectory = new LinearTrajectory(2.0);
		pivotTrajectory = new PivotTrajectory(90);
		stationaryTraj = new StationaryTrajectory();
		
		maxThrottle = (5.0/6.0) * (joystickProfile.getMinTurnRadius() / (joystickProfile.getMinTurnRadius() + halfTrackWidth));
		
		m_solenoidshift = shift;
		this.approximateSensorSpeed = approximateSensorSpeed;
		mpNotifier.startPeriodic(0.010);
	}
	
	
	public void driving(double throttleAxis, double arcadeSteerAxis, double carSteerAxis, boolean shiftButton, double rightTrigger,
			boolean buttonA, boolean buttonB, boolean buttonX, boolean buttonY, boolean rightBumper){
		//Only switch to carDrive in high gear
		if(isHighGear){
//			carDrive(throttleAxis, carSteerAxis);
			buttonDrive(buttonA, buttonB, buttonX, buttonY, rightTrigger);
			
			//When you hold down the shiftButton (left bumper), then shift to low gear.
			if(shiftButton){
				shiftToLowGear();
			}
		}
		
		//Only switch to ArcadeDrive in low gear
		else{	
			ArcadeDrive(throttleAxis, arcadeSteerAxis);
			
			//When you release the shiftButton (left bumper), then shift to high gear.
			if(!shiftButton){
				shiftToHighGear();
			}
			
		}
	}
	
	//Shifts the robot to high gear and change the talon's control mode to speed.
	public void shiftToHighGear(){
		if (!isHighGear){
			m_solenoidshift.set(false);
			TorCAN.INSTANCE.chooseVelocityControl();
			isHighGear = true;
			stationaryTraj.execute();
			TorMotionProfile.setActive();
		}
	}
	
	//Shifts the robot to low gear and change the talon's control mode to percentVbus.
	public void shiftToLowGear(){
		if (isHighGear){
			m_solenoidshift.set(true);
			TorCAN.INSTANCE.choosePercentVbus();
			isHighGear = false;
			TorMotionProfile.setInactive();
		}
	}

	public void ArcadeDrive(double throttleAxis, double arcadeSteerAxis){
		throttleAxis = -throttleAxis;
		if (Math.abs(arcadeSteerAxis) <= 0.1) {
			arcadeSteerAxis = 0.0D;
		}
		if (Math.abs(throttleAxis) <= 0.2D) {
			throttleAxis = 0.0D;
		}

		if (arcadeSteerAxis >= 0.0D) {
			arcadeSteerAxis *= arcadeSteerAxis;
		} else {
			arcadeSteerAxis = -(arcadeSteerAxis * arcadeSteerAxis);
		}
		if (throttleAxis >= 0.0D) {
			throttleAxis *= throttleAxis;
		} else {
			throttleAxis = -(throttleAxis * throttleAxis);
		}
		
		double rightMotorSpeed;
		double leftMotorSpeed;

		if (throttleAxis > 0.0D)
		{
			if (arcadeSteerAxis > 0.0D)
			{
				leftMotorSpeed = throttleAxis - arcadeSteerAxis;
				rightMotorSpeed = Math.max(throttleAxis, arcadeSteerAxis);
			}
			else
			{
				leftMotorSpeed = Math.max(throttleAxis, -arcadeSteerAxis);
				rightMotorSpeed = throttleAxis + arcadeSteerAxis;
			}
		}
		else
		{
			if (arcadeSteerAxis > 0.0D)
			{
				leftMotorSpeed = -Math.max(-throttleAxis, arcadeSteerAxis);
				rightMotorSpeed = throttleAxis + arcadeSteerAxis;
			}
			else
			{
				leftMotorSpeed = throttleAxis - arcadeSteerAxis;
				rightMotorSpeed = -Math.max(-throttleAxis, -arcadeSteerAxis);
			}
		}
		TorCAN.INSTANCE.SetDrive(rightMotorSpeed, -leftMotorSpeed);
	}

	
	public void carDrive(double throttleAxis, double carSteeringAxis){
		
		//Flipping the sign so it drives forward when you move the analog stick up and vice versa
		throttleAxis = -throttleAxis;
		carSteeringAxis = -carSteeringAxis;
		
		targetSpeed = joystickProfile.findSpeed(throttleAxis) * approximateSensorSpeed;
		
		targetSpeed *= maxThrottle;
		
		/* The centerRadius is the value we gain from findRadiusExponential method in the joystickProfile class.
		   The Math.abs(throttleAxis) / throttleAxis is to make sure the sign of the centerRadius is right. */
		centerRadius = (Math.abs(throttleAxis) / throttleAxis) * joystickProfile.findRadiusExponential(carSteeringAxis);
		
		//If the centerRadius is greater than the maxTurnRadius or the centerRadius is 0, then drive forward and vice versa.
		if (Math.abs(centerRadius) > joystickProfile.getMaxTurnRadius() || Math.abs(centerRadius) == 0.0)
		{
			leftMotorSpeed = targetSpeed;
			rightMotorSpeed = targetSpeed;
		}
		
		//Else, steer
		else {
			rightMotorSpeed = targetSpeed * ((centerRadius - halfTrackWidth) / centerRadius);
			leftMotorSpeed = targetSpeed * ((centerRadius + halfTrackWidth) / centerRadius);
		}

		//Setting the rightMotorSpeed and the leftMotorSpeed so that it actually drives.
		TorCAN.INSTANCE.SetDrive(rightMotorSpeed, -leftMotorSpeed);
		
	}
	
	public void buttonDrive(boolean buttonA, boolean buttonB, boolean buttonX, boolean buttonY, double rightTrigger){
		if(buttonB){
			pivotTrajectory.execute();
		}
		else if(buttonX){

		}
		else if(buttonY){
			linearTrajectory.execute();
		}
		else if(buttonA){

		}
		else{
		
		}
	}
}
