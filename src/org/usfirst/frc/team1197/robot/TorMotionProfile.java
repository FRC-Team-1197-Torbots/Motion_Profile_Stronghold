package org.usfirst.frc.team1197.robot;

import java.util.Vector;

public class TorMotionProfile {
	
	private static Vector<Long> time;
	private static Vector<Double> displacement;
	private static Vector<Double> velocity;
	private static Vector<Boolean> isLast;
	
	private static boolean isMoving = false;
	private static boolean isTurnedOn = false;
	
	private static double wayPoint;
	
	private static TorTrajectory activeTrajectory = null;
	
	public static double lookUpDisplacement(long t){
		return activeTrajectory.lookUpDisplacement(t);
	}
	
	public static double lookUpVelocity(long t){
		return activeTrajectory.lookUpVelocity(t);
	}
	
	public static boolean lookUpIsLast(long t){
		return activeTrajectory.lookUpIsLast(t);
	}
	
	public static double lookUpAcceleration(long t){
		return activeTrajectory.lookUpAcceleration(t);
	}
	
	public static void loadTrajectory(TorTrajectory traj){
		activeTrajectory = traj;
	}
	
	public static void start(){
		isMoving = true;
	}
	
	public static void turnOn(){
		isTurnedOn = true;
	}
	
	public static void turnOff(){
		isTurnedOn = false;
	}
	
	public static void stop(){
		isMoving = false;
//		isTurnedOn = false;
		
		time.clear();
		velocity.clear();
		displacement.clear();
		isLast.clear();
	}
	
	public static boolean isMoving(){
		return isMoving;
	}
	
	public static boolean isTurnedOn(){
		return isTurnedOn;
	}
	
	public static void setWayPoint(long t){
		int i = time.indexOf(t);
		wayPoint = displacement.elementAt(i);
	}
	
	public static void setWayPoint(double pos){
		wayPoint = pos;
	}
	
	public static double getWayPoint(){
		return wayPoint;
	}
}
