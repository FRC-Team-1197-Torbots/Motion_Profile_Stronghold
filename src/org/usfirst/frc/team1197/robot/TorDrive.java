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
	private double targetOmega;
	private double trackWidth = 0.5525; //meters, in inches 21.75
	private double halfTrackWidth = trackWidth / 2.0;
	private double centerRadius = 0.0;
	private double maxThrottle;
	private double approximateSensorSpeed;

	private static TorTrajectory linearTrajectory;
	private static TorTrajectory pivotTrajectory;
	public TorTrajectory stationaryTraj;
	
	private boolean buttonYlast;
	private boolean buttonBlast;
	
	class PeriodicRunnable implements java.lang.Runnable {
		public void run() {
			TorMotionProfile.INSTANCE.run();
		}
	}
	Notifier mpNotifier = new Notifier(new PeriodicRunnable());

	public TorDrive(Joystick stick, Solenoid shift, double approximateSensorSpeed)
	{
		joystickProfile = new TorJoystickProfiles();
		linearTrajectory = new LinearTrajectory(2.0);
		pivotTrajectory = new PivotTrajectory(-270);
		stationaryTraj = new StationaryTrajectory();
		
		maxThrottle = (5.0/6.0) * (joystickProfile.getMinTurnRadius() / (joystickProfile.getMinTurnRadius() + halfTrackWidth));
		
		m_solenoidshift = shift;
		this.approximateSensorSpeed = approximateSensorSpeed;
		mpNotifier.startPeriodic(TorMotionProfile.INSTANCE.getTimeInterval());
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
//			TorMotionProfile.INSTANCE.joystickTraj.execute(0.0, 0.0, 0.0, 0.0);
			TorMotionProfile.INSTANCE.setActive();
		}
	}
	
	//Shifts the robot to low gear and change the talon's control mode to percentVbus.
	public void shiftToLowGear(){
		if (isHighGear){
			m_solenoidshift.set(true);
			TorCAN.INSTANCE.choosePercentVbus();
			isHighGear = false;
			stationaryTraj.execute();
			TorMotionProfile.INSTANCE.setInactive();
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
//			leftMotorSpeed = targetSpeed;
//			rightMotorSpeed = targetSpeed;
			targetOmega = 0.0;
		}
		
		//Else, steer
		else {
//			rightMotorSpeed = targetSpeed * ((centerRadius - halfTrackWidth) / centerRadius);
//			leftMotorSpeed = targetSpeed * ((centerRadius + halfTrackWidth) / centerRadius);
			targetOmega = targetSpeed / centerRadius;
		}

		//Setting the rightMotorSpeed and the leftMotorSpeed so that it actually drives.
//		TorCAN.INSTANCE.SetDrive(rightMotorSpeed, -leftMotorSpeed);
		//TorMotionProfile.INSTANCE.joystickTraj.setTargets(targetSpeed, targetOmega);
		
	}
	
	public void buttonDrive(boolean buttonA, boolean buttonB, boolean buttonX, boolean buttonY, double rightTrigger){
		if(buttonB && !buttonBlast){
			pivotTrajectory.execute();
		}
		else if(buttonX){

		}
		else if(buttonY && !buttonYlast){
			linearTrajectory.execute();
		}
		else if(buttonA){

		}
		else{
			
		}
		buttonBlast = buttonB;
		buttonYlast = buttonY;
	}
}
