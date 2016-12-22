package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.StatusFrameRate;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TorCAN

implements PIDOutput
{
	CANTalon m_Ltalon3;
	CANTalon m_Ltalon2;
	CANTalon m_Ltalon1;
	CANTalon m_Rtalon3;
	CANTalon m_Rtalon2;
	CANTalon m_Rtalon1;
	float lowgear = 20.0F;
	float highgear = 15.0F;
	int numOfJags;
	
	private double encoderTicksPerMeter = 8814.0;
	private double approximateSensorSpeed = 4550.0; //4550
	private double quadEncNativeUnits = 512.0;
	private double kF = (1023.0) / ((approximateSensorSpeed * quadEncNativeUnits) / (600.0));
	private double kP = 2.0; //7.0mp, 2.0vel
	private double kI = 0.0; //0.0mp, 0.0vel
	private double kD = 20.0; //250.0mp, 20.0vel
	
	private double trackWidth = 0.5525; //meters, in inches 21.75
	private double halfTrackWidth = trackWidth / 2.0;
	
	//Constructor for two Talon (one on each side) driving.
	public TorCAN(CANTalon R1, CANTalon L1)
	{
		numOfJags = 2;
		m_Rtalon1 = R1;
		m_Ltalon1 = L1;
	}

	//Constructor for four Talon (two on each side) driving.
	public TorCAN(CANTalon R1, CANTalon R2, CANTalon L1, CANTalon L2)
	{
		numOfJags = 4;
		
		//m_Rtalon1 (R1) is the right master Talon.
		m_Rtalon1 = R1; 
		
		//This will make the m_Rtalon2 follow m_Rtalon1.
		m_Rtalon2 = R2;
		m_Rtalon2.changeControlMode(CANTalon.TalonControlMode.Follower);
		m_Rtalon2.set(m_Rtalon1.getDeviceID());

		//m_Ltalon1 (L1) is the left master Talon.
		m_Ltalon1 = L1;
		
		//This will make the m_Ltalon2 follow m_Ltalon1.
		m_Ltalon2 = L2;
		m_Ltalon2.changeControlMode(CANTalon.TalonControlMode.Follower);
		m_Ltalon2.set(m_Ltalon1.getDeviceID());
		
		/* first choose the sensor */
	      m_Rtalon1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	      m_Rtalon1.reverseSensor(false);
        //m_Rtalon1.configEncoderCodesPerRev(128); // if using FeedbackDevice.QuadEncoder
	      //m_Rtalon1.configPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

	      /* set the peak and nominal outputs, 12V means full */
	      m_Rtalon1.configNominalOutputVoltage(+0.0f, -0.0f);
	      m_Rtalon1.configPeakOutputVoltage(+12.0f, -12.0f);
	      /* set closed loop gains in slot0 */
	      m_Rtalon1.setProfile(0);
	      m_Rtalon1.setF(kF); 
	      m_Rtalon1.setP(kP); 
	      m_Rtalon1.setI(kI); 
	      m_Rtalon1.setD(kD);
	      
	      /* first choose the sensor */
	      m_Ltalon1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	      m_Ltalon1.reverseSensor(true);
        //m_Ltalon1.configEncoderCodesPerRev(128); // if using FeedbackDevice.QuadEncoder
	      //m_Ltalon1.configPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

	      /* set the peak and nominal outputs, 12V means full */
	      m_Ltalon1.configNominalOutputVoltage(+0.0f, -0.0f);
	      m_Ltalon1.configPeakOutputVoltage(+12.0f, -12.0f);
	      /* set closed loop gains in slot0 */
	      m_Ltalon1.setProfile(0);
	      m_Ltalon1.setF(kF); //0.263 
	      m_Ltalon1.setP(kP); //1
	      m_Ltalon1.setI(kI); 
	      m_Ltalon1.setD(kD); //10
	      
	      m_Rtalon1.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 10);
	      m_Ltalon1.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 10);
	}

	//Constructor for six Talon (three on each side) driving.
	public TorCAN(CANTalon R1, CANTalon R2, CANTalon R3, CANTalon L1, CANTalon L2, CANTalon L3)
	{
		numOfJags = 6;

		//m_Rtalon1 (R1) is the right master Talon.
		m_Rtalon1 = R1;
		
		//This will make the m_Rtalon2 follow m_Rtalon1.
		m_Rtalon2 = R2;
		m_Rtalon2.changeControlMode(CANTalon.TalonControlMode.Follower);
		m_Rtalon2.set(m_Rtalon1.getDeviceID());
		
		//This will make the m_Rtalon3 follow m_Rtalon1.
		m_Rtalon3 = R3;
		m_Rtalon3.changeControlMode(CANTalon.TalonControlMode.Follower);
		m_Rtalon3.set(m_Rtalon1.getDeviceID());

		//m_Ltalon1 (L1) is the left master Talon.
		m_Ltalon1 = L1;
		
		//This will make the m_Ltalon2 follow m_Ltalon1.
		m_Ltalon2 = L2;
		m_Ltalon2.changeControlMode(CANTalon.TalonControlMode.Follower);
		m_Ltalon2.set(m_Ltalon1.getDeviceID());
		
		//This will make the m_Ltalon3 follow m_Ltalon1.
		m_Ltalon3 = L3;
		m_Ltalon3.changeControlMode(CANTalon.TalonControlMode.Follower);
		m_Ltalon3.set(m_Ltalon1.getDeviceID());
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

	public void pidWrite(double output)
	{
		SetRight(output);
		SetLeft(-output);
	}
	
	public double getDisplacement(){
		return -(m_Rtalon1.getPosition() + m_Ltalon1.getPosition()) * 0.5 / encoderTicksPerMeter;
	}
	public double getVelocity(){
		return -(m_Rtalon1.getSpeed() + m_Ltalon1.getSpeed()) * 0.5 * 10 / encoderTicksPerMeter;
	}
	
	public void setTargets(double v, double omega){
		m_Rtalon1.set(-(v + omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter);
		m_Ltalon1.set(-(v - omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter);
	}
	
	public void resetEncoder(){
		m_Rtalon1.setPosition(0);
		m_Ltalon1.setPosition(0);
	}
	
	public double getHeading(){
		return -(m_Rtalon1.getPosition() - m_Ltalon1.getPosition()) * 0.5 / (encoderTicksPerMeter * halfTrackWidth);
	}

}
