package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;

public class Robot extends SampleRobot {
	private Compressor compressor;
	private Solenoid S1;
	private Joystick stick;
	private TorDrive drive;
	private double approximateSensorSpeed = 4550.0;
	private CameraServer camera;

	public Robot() {
		stick = new Joystick(0);	
		S1 = new Solenoid(0);		
		compressor = new Compressor();
		drive = new TorDrive(stick, S1, approximateSensorSpeed); 
//		CameraServer camera = CameraServer.getInstance();
//		camera.startAutomaticCapture("cam0", 0);
	}

	public void autonomous() {
		
	}

	public void operatorControl() {
		drive.shiftToHighGear();
		while (isEnabled())
		{   
			drive.driving(getLeftY(), getLeftX(), getRightX(), getShiftButton(), getRightTrigger(), 
					getButtonA(), getButtonB(), getButtonX(), getButtonY(), getRightBumper());
		}
	}

	public void test() {
		while(isEnabled()){
			compressor.start();
		}
	}

	//Low-gear software wise, High-gear mechanically
	public void disabled() {
		drive.shiftToLowGear();
		S1.set(false); 
		TorCAN.INSTANCE.SetDrive(0.0, 0.0);
	}

	// Getting the left analog stick X-axis value from the xbox controller. 
	public double getLeftX(){
		return stick.getRawAxis(0);
	}

	// Getting the left analog stick Y-axis value from the xbox controller. 
	public double getLeftY(){
		return stick.getRawAxis(1);
	}

	// Getting the right analog stick X-axis value from the xbox controller. 
	public double getRightX(){
		return stick.getRawAxis(4);
	}

	// Getting the right trigger value from the xbox controller.
	public double getRightTrigger(){
		return stick.getRawAxis(3);
	}

	// Getting the left bumper button value from the xbox controller. 
	public boolean getShiftButton(){
		return stick.getRawButton(5);
	}

	public boolean getRightBumper(){
		return stick.getRawButton(6);
	}

	public boolean getButtonA(){
		return stick.getRawButton(1);
	}

	public boolean getButtonB(){
		return stick.getRawButton(2);
	}

	public boolean getButtonX(){
		return stick.getRawButton(3);
	}

	public boolean getButtonY(){
		return stick.getRawButton(4);
	}

}