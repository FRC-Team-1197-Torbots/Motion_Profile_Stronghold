package org.usfirst.frc.team1197.robot;

public class TorJoystickProfiles {
	
	private double k = 0.3;
	private double A = (k * k) / (1 - (2 * k));
	private double B = (k - 1) * (k - 1) / (k * k);	
	
	private double maxTurnRadius = 10.0;
	private double minTurnRadius = 0.5;
	private double steeringDeadBand = 0.1;
	private double throttleDeadBand = 0.15;
	private double C = (Math.log(minTurnRadius / maxTurnRadius)) / (steeringDeadBand - 1);
	private double D = maxTurnRadius * Math.exp(C * steeringDeadBand);
	private double negSteeringInertia = 0.0;
	private double previous_r = 0.0;
	private double r = 0.0;
	private double negThrottleInertia = 0.0;
	private double previous_y = 0.0;
	private double y = 0.0;
	
	public TorJoystickProfiles(){
		
	}
	
	public double findRadiusExponential(double x){
		
		/* The exponential equation to find the radius.
		   See https://www.desmos.com/calculator/g0st5i5ajy 
		   for the turning profile equations */
		r = (Math.abs(x) / x) * (D * Math.exp(-C * Math.abs(x)));

		/* Calculating the negative Steering Inertia.
		   See the negative inertia section
		   http://frc971.org/content/programming. */
		negSteeringInertia = (r - previous_r) * 100.0;
		previous_r = r;
		
		if(x == 0.0){
			return 0.0;
		}
		else{
			//Adding the negative Steering Inertia to prevent the "sluggish" steering. 
			return r + negSteeringInertia;
		}
	}
	
	public double findSpeed(double x){
		y = (Math.abs(x) / x) * A * ((Math.pow(B, Math.abs(x))) - 1.0);
		negThrottleInertia = (y - previous_y) * 0.0;
		previous_y = y;
	
		if(Math.abs(x) >= throttleDeadBand){
			return y + negThrottleInertia;
		}
		
		else{
			return 0.0;
		}
	}
	
	//Mutator method for other classes to access the variable minTurnRadius.
	public double getMinTurnRadius(){
		return minTurnRadius;
	}
	
	//Mutator method for other classes to access the variable maxTurnRadius.
	public double getMaxTurnRadius(){
		return maxTurnRadius;
	}
}
