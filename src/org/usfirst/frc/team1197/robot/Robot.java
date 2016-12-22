package org.usfirst.frc.team1197.robot;

import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;




public class Robot
extends SampleRobot
{
	private Compressor compressor;
	
	private CANTalon R1;
	private CANTalon R2;
	private CANTalon L1;
	private CANTalon L2;
	private CANTalon T1;
	private CANTalon P1;
	private CANTalon P2;
	private CANTalon P3;
	private CANTalon P4;
	private CANTalon shooter1;
	private CANTalon shooter2;
	private CANTalon hood;
	
	private Solenoid S1;
	private Encoder encoder;
	
	private Joystick stick;
	private Joystick stick2;
	private Joystick cypress;
	
	private TorCAN driveCANS;
	private TorDrive drive;
	private TorSiege siege;
	private TorIntake intakee;
	private TorAuto auto;
	private TorShooter shoot;
	private TorCamera camera;
	
	private AHRS gyro;
	
	private DigitalInput breakBeam;
	private DigitalInput breakBeam2;
	
	final String drawBridgeAuto = "Draw Bridge";
	final String sallyPortAuto = "Sally Port";
	final String chevelAuto = "Chevel";
	final String portcullisAuto = "Portcullis";
	final String rampartsAuto = "Ramparts";
	final String rockWallAuto = "Rock Wall";
	final String roughTerrainAuto = "Rough Terrain";
	final String moatAuto = "Moat";
	
	private NetworkTable table;
    StringBuilder _sb = new StringBuilder();
	private int _loops = 0; 
	
	//see Talon SRX Software Reference Manual page 75-78
	private double approximateSensorSpeed = 4550.0;

	public Robot()
	{
		gyro = new AHRS(SPI.Port.kMXP);
		stick = new Joystick(0);
		stick2 = new Joystick(1);
		cypress = new Joystick(2);

		L1 = new CANTalon(1);
		L2 = new CANTalon(2);

		R1 = new CANTalon(5);
		R2 = new CANTalon(6);

		T1 = new CANTalon(7);

		double RampRate = 0.0D;
		RampRate = 0.0D;
		
		T1.changeControlMode(CANTalon.TalonControlMode.Position);
		T1.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		
		T1.setPID(20.0D, 0.0D, 0.0D, 0.0D, 0, RampRate, 1);
		T1.setPID(16.0D, 0.0D, 0.0D, 0.0D, 0, RampRate, 0);
		//profile 1 = 20 profile 0 = 16
		P1 = new CANTalon(8);
		P2 = new CANTalon(9);
	
		P3 = new CANTalon(4);
		P4 = new CANTalon(3);
		S1 = new Solenoid(0);

		shooter1 = new CANTalon(11);
		shooter2 = new CANTalon(12);
	
		hood = new CANTalon(10);

		breakBeam = new DigitalInput(4);
		breakBeam2 = new DigitalInput(5);
		
		gyro = new AHRS(SerialPort.Port.kMXP);
		
		encoder = new Encoder(0, 1);
		encoder.setDistancePerPulse(0.017857142857142856D);
		
		driveCANS = new TorCAN(R1, R2, L1, L2);
		intakee = new TorIntake(stick, stick2, P1, P2, P3, P4, breakBeam, breakBeam2, siege, shoot); 
		drive = new TorDrive(stick2, stick, driveCANS, encoder, S1, approximateSensorSpeed); 
		siege = new TorSiege(T1, stick2, driveCANS, stick, intakee, drive, encoder, gyro, camera);
		camera = new TorCamera(table, gyro, driveCANS, siege, intakee, stick2);
		shoot = new TorShooter(intakee, shooter1, shooter2, hood, P2, P1, stick2, gyro, driveCANS, camera);
		auto = new TorAuto(cypress, stick, stick2, gyro, encoder, driveCANS, S1, siege, intakee, drive, shoot, this, camera);
	}

	public void autonomous()
	{
		shooter1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		shooter2.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		shooter1.set(1);
		shooter2.set(1);
		encoder.reset();
		siege.PID();
		auto.ModeChooser();
		shooter1.changeControlMode(CANTalon.TalonControlMode.Voltage);
		shooter2.changeControlMode(CANTalon.TalonControlMode.Voltage);
	}

	public void operatorControl()
	{
		shooter1.changeControlMode(CANTalon.TalonControlMode.Voltage);
		shooter2.changeControlMode(CANTalon.TalonControlMode.Voltage);
		gyro.reset();
		siege.PID();
		drive.shiftToHighGear();
		while (isEnabled())
		{
			double leftYjoystick = -1 * getLeftY();
//			System.out.println("ENCODER: " + R1.getAnalogInRaw());
			
//			SmartDashboard.putNumber("Velocity", driveCANS.getVelocity());
//			SmartDashboard.putNumber("Position", driveCANS.getDisplacement());
			
//			double targetSpeed = getLeftY() * 0.417 * 4550;
//			double RmotorOutput = R1.getOutputVoltage() / R1.getBusVoltage();
//	    	double LmotorOutput = L1.getOutputVoltage() / L1.getBusVoltage();
//	    	
//			_sb.append("\tRout:");
//			_sb.append(RmotorOutput);
//	        _sb.append("\tRspd:");
//	        _sb.append(R1.getSpeed());
//	        
//	        _sb.append("\tRerr:");
//            _sb.append(R1.getClosedLoopError());
//            _sb.append("\tRtrg:");
//            _sb.append(targetSpeed);
//	        
//	        _sb.append("\tLout:");
//			_sb.append(LmotorOutput);
//	        _sb.append("\tLspd:");
//	        _sb.append(L1.getSpeed() );
//            
//            _sb.append("\tLerr:");
//            _sb.append(L1.getClosedLoopError());
//            _sb.append("\tLtrg:");
//            _sb.append(targetSpeed);
//            
//            if(++_loops >= 10) {
//	        	_loops = 0;
//	        	System.out.println(_sb.toString());
//	        }
//	        _sb.setLength(0);
//            
//	        SmartDashboard.putNumber("RightSpeed: ", R1.getSpeed());
//	        SmartDashboard.putNumber("LeftSpeed: ", L1.getSpeed());
//	        
			siege.SiegeArmUpdate();
			siege.intakeTele();
			intakee.intake();
			if (!siege.enabled) {
				drive.driving(getLeftY(), getLeftX(), getRightX(), getShiftButton(), getRightTrigger(), 
						getButtonA(), getButtonB(), getButtonX(), getButtonY(), getRightBumper());
			}
			shoot.shooter();
		}
	}

	public void test()
	{
		compressor = new Compressor();
		compressor.start();
		
		gyro.reset();
		
		while (isEnabled())
		{
//			System.out.println("POT: " + T1.getAnalogInRaw());
//			System.out.println("ENCODER: " + R1.getAnalogInRaw());

	        
	        if(stick2.getRawButton(3)){
	        	S1.set(true);
	        }
	        else{
	        	S1.set(false);
	        }
		}
	}
	
	public void robotInit(){
		
	}
	
	public void disabled() {
		TorMotionProfile.turnOff();
		driveCANS.resetEncoder();
		TorMotionProfile.setWayPoint(0.0);
		driveCANS.choosePercentVbus();
		driveCANS.SetDrive(0.0, 0.0);
	}

	// Getting the left analog stick X-axis value from the xbox controller. 
	public double getLeftX(){
		return stick2.getRawAxis(0);
	}
	
	// Getting the left analog stick Y-axis value from the xbox controller. 
	public double getLeftY(){
		return stick2.getRawAxis(1);
	}
	
	// Getting the right analog stick X-axis value from the xbox controller. 
	public double getRightX(){
		return stick2.getRawAxis(4);
	}
	
	// Getting the right trigger value from the xbox controller.
	public double getRightTrigger(){
		return stick2.getRawAxis(3);
	}
	
	// Getting the left bumper button value from the xbox controller. 
	public boolean getShiftButton(){
		return stick2.getRawButton(5);
	}
	
	public boolean getRightBumper(){
		return stick2.getRawButton(6);
	}
	
	public boolean getButtonA(){
		return stick2.getRawButton(1);
	}
	
	public boolean getButtonB(){
		return stick2.getRawButton(2);
	}
	
	public boolean getButtonX(){
		return stick2.getRawButton(3);
	}
	
	public boolean getButtonY(){
		return stick2.getRawButton(4);
	}
	
}