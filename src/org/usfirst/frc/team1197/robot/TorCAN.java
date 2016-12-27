package org.usfirst.frc.team1197.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.StatusFrameRate;

public enum TorCAN
{
	INSTANCE;
	
	private CANTalon m_Rtalon1;
	private CANTalon m_Rtalon2;
	private CANTalon m_Ltalon1;
	private CANTalon m_Ltalon2;
	
	private AHRS gyro;
	
	private double encoderTicksPerMeter = 8814.0;
	private double approximateSensorSpeed = 4550.0;
	private double quadEncNativeUnits = 512.0;
	private double kF = (1023.0) / ((approximateSensorSpeed * quadEncNativeUnits) / (600.0));
	private double kP = 1.5; //1.5 change it back down to 1.0 if you get the jitters
	private double kI = 0.0; //0.0
	private double kD = 50.0; //50.0
	
	private double trackWidth = 0.5525; //meters, in inches 21.75
	private double halfTrackWidth = trackWidth / 2.0;
	
	private double backlash = 0.015;
	
	private TorCAN(){
		
		gyro = new AHRS(SerialPort.Port.kMXP);
		
		m_Rtalon1 = new CANTalon(5);
		m_Rtalon2 = new CANTalon(6);
		m_Ltalon1 = new CANTalon(1);
		m_Ltalon2 = new CANTalon(2);
		
		m_Rtalon1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		m_Rtalon1.reverseSensor(false);
		m_Rtalon1.configNominalOutputVoltage(+0.0f, -0.0f);
		m_Rtalon1.configPeakOutputVoltage(+12.0f, -12.0f);
		m_Rtalon1.setProfile(0);
		m_Rtalon1.setF(kF); 
		m_Rtalon1.setP(kP); 
		m_Rtalon1.setI(kI); 
		m_Rtalon1.setD(kD);
		
		m_Rtalon2.changeControlMode(CANTalon.TalonControlMode.Follower);
		m_Rtalon2.set(m_Rtalon1.getDeviceID());
		
		m_Ltalon1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		m_Ltalon1.reverseSensor(true);
		m_Ltalon1.configNominalOutputVoltage(+0.0f, -0.0f);
		m_Ltalon1.configPeakOutputVoltage(+12.0f, -12.0f);
		m_Ltalon1.setProfile(0);
		m_Ltalon1.setF(kF);  
		m_Ltalon1.setP(kP); 
		m_Ltalon1.setI(kI); 
		m_Ltalon1.setD(kD); 
		
		m_Ltalon2.changeControlMode(CANTalon.TalonControlMode.Follower);
		m_Ltalon2.set(m_Ltalon1.getDeviceID());

		m_Rtalon1.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 2);
		m_Ltalon1.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 2);
	}

	public void SetDrive(double leftSpeed, double rightSpeed)
	{
		SetLeft(leftSpeed);
		SetRight(rightSpeed);
	}
	
	//Setting the left master Talon's speed to the given parameter
	public void SetLeft(double speed)
	{
		m_Ltalon1.set(-speed);
	}

	//Setting the right master Talon's speed to the given parameter
	public void SetRight(double speed)
	{
		m_Rtalon1.set(speed);
	}
	
	/* A method to change the right and left master Talon's control mode
	   to percentVbus, so we can use it when the robot is in low gear. */
	public void choosePercentVbus(){
		m_Rtalon1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		m_Ltalon1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}
	
	/* A method to change the right and left master Talon's control mode
	   to speed, so we can use it when the robot is in high gear. */
	public void chooseVelocityControl(){
		m_Rtalon1.changeControlMode(CANTalon.TalonControlMode.Speed);
		m_Ltalon1.changeControlMode(CANTalon.TalonControlMode.Speed);
	}
	
	public void chooseMotionProfileControl(){
		m_Rtalon1.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
		m_Ltalon1.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
	}
	
	public double getDisplacement(){
		return -(m_Rtalon1.getPosition() + m_Ltalon1.getPosition()) * 0.5 / encoderTicksPerMeter;
	}
	public double getVelocity(){
		return -(m_Rtalon1.getSpeed() + m_Ltalon1.getSpeed()) * 0.5 * 10 / encoderTicksPerMeter;
	}
	
	public double getHeading(){
		return (gyro.getAngle() * (Math.PI / 180));
	}
	public double getOmega(){
		return (gyro.getRate());
	}
	
	//1.555 is the conversion factor that we found experimentally.
	public void setTargets(double v, double omega){ 
		m_Rtalon1.set(-(v - omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter);
		m_Ltalon1.set(-(v + omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter);
	}
	
	public void resetEncoder(){
		m_Rtalon1.setPosition(0);
		m_Ltalon1.setPosition(0);
	}
	
	public void resetHeading(){
		gyro.reset();
	}
	
	public double getBacklash(){
		return backlash;
	}
}
