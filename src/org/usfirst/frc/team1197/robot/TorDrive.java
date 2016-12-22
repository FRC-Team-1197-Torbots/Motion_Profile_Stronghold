package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.livewindow.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TorDrive
{	
	private Joystick stick;
	private Joystick overrideStick;
	private Encoder m_encoder;
	private float negOvershoot;
	
	private TorCAN m_jagDrive;
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
	
	private CANTalon rightTalon;
	private CANTalon leftTalon;
	
	private double targetVelocity;
	private double targetAcceleration;
	private double targetDisplacement;
	
	private double displacementError;
	private double headingError;
	private double v;
	private double omega;
	private double lastDisplacementError;
	private double lastHeadingError;
	private double totalError;
	
	private double kA = 0.3;

	private double kP = 1.0;  //0.001
	private double kI = 0.0;  //0.0
	private double kD = 0.5;  //0.01

	private double dt = 0.01;
	
	private TorTrajectory trajectory;	
	private long currentTime;
	
	private StationaryTrajectory stationaryTraj;
	
	class PeriodicRunnable implements java.lang.Runnable {
		public void run() {
			if(TorMotionProfile.isTurnedOn()){
				currentTime = System.currentTimeMillis();
				currentTime = currentTime - (currentTime % 10);
				targetDisplacement = TorMotionProfile.lookUpDisplacement(currentTime);
				targetVelocity = TorMotionProfile.lookUpVelocity(currentTime);
				targetAcceleration = TorMotionProfile.lookUpAcceleration(currentTime);
				if(TorMotionProfile.lookUpIsLast(currentTime)){
					m_jagDrive.resetEncoder();
					TorMotionProfile.setWayPoint(0.0);
					TorMotionProfile.stop();
				}
				
				SmartDashboard.putNumber("targetAcceleration", targetAcceleration);
				SmartDashboard.putNumber("targetDisplacement", targetDisplacement);
				SmartDashboard.putNumber("getDisplacement", m_jagDrive.getDisplacement());
				

				displacementError = targetDisplacement - m_jagDrive.getDisplacement();
				totalError += displacementError;
				v = targetVelocity + (kA * targetAcceleration) + (kP * displacementError) + (kI * totalError) + (kD * (((displacementError - lastDisplacementError) / (dt))));

				omega = 0;
//				}
//				else if(TorMotionProfile.isRotationalTrajectory()){
//					headingError = targetDisplacement - m_jagDrive.getHeading();
//					totalError += displacementError;
//					omega = targetVelocity + (kP * headingError) + (kI * totalError) + (kD * (headingError - lastHeadingError));
//					v = 0;
//				}
				SmartDashboard.putNumber("targetVelocity", targetVelocity);
				m_jagDrive.setTargets(v, omega);
				lastDisplacementError = displacementError;
				lastHeadingError = headingError;
			}
			else{
				TorMotionProfile.setWayPoint(m_jagDrive.getDisplacement());
			}
		}
	}
	Notifier mpNotifier = new Notifier(new PeriodicRunnable());

	public TorDrive(Joystick stick, Joystick stick2, TorCAN cans, Encoder encoder, Solenoid shift, double approximateSensorSpeed)
	{
		joystickProfile = new TorJoystickProfiles();
		trajectory = new LinearTrajectory(1.0);
		maxThrottle = (5.0/6.0) * (joystickProfile.getMinTurnRadius() / (joystickProfile.getMinTurnRadius() + halfTrackWidth));
		stick = this.stick;
		overrideStick = stick2;
		m_jagDrive = cans;
		m_encoder = encoder;
		m_solenoidshift = shift;
		this.approximateSensorSpeed = approximateSensorSpeed;
		mpNotifier.startPeriodic(0.010);
		stationaryTraj = new StationaryTrajectory();
	}
	
	public TorDrive(Joystick stick, TorCAN cans, CANTalon rightTalon, CANTalon leftTalon){
		this.stick = stick;
		m_jagDrive = cans;
		this.rightTalon = rightTalon;
		this.leftTalon = leftTalon;
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
			m_jagDrive.chooseVelocityControl();
			isHighGear = true;
			m_jagDrive.resetEncoder();
			TorMotionProfile.setWayPoint(0.0);
			TorMotionProfile.turnOn();
			stationaryTraj.execute();
		}
	}
	
	//Shifts the robot to low gear and change the talon's control mode to percentVbus.
	public void shiftToLowGear(){
		if (isHighGear){
			m_solenoidshift.set(true);
			m_jagDrive.choosePercentVbus();
			isHighGear = false;
			TorMotionProfile.turnOff();
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
		m_jagDrive.SetDrive(rightMotorSpeed, -leftMotorSpeed);
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
		m_jagDrive.SetDrive(rightMotorSpeed, -leftMotorSpeed);
		
	}
	
	public void buttonDrive(boolean buttonA, boolean buttonB, boolean buttonX, boolean buttonY, double rightTrigger){
		if(buttonB){

		}
		else if(buttonX){

		}
		else if(buttonY){
			trajectory.execute();
		}
		else if(buttonA){

		}
		else{
		
		}
	}
}
