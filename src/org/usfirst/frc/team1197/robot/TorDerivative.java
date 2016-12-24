package org.usfirst.frc.team1197.robot;

public class TorDerivative {
	
	private double[] data = new double[20];
	private double tMean;
	private double yMean;
	private double dt;
	private double SStt;
	
	public TorDerivative(double dt){
		this.dt = dt;
		tMean = (data.length-1) * dt * 0.5;
		SStt = 0;
	    for(float t = 0;  t < data.length * dt; t+=dt){
			SStt += (t - tMean)*(t - tMean);
		}
	}
	
	public double Estimate(double y){
		for(int i=data.length-1; i>0; i--){
			data[i] = data[i - 1];
		}
		data[0] = y;
		return (SSty() / SStt);
	}
	
	public void Mean(){
		yMean = 0;
		for(int i=0; i < data.length; i++){
			yMean += data[i];
		}
		yMean = yMean / data.length;
	}
	
	private double SSty(){
		double sum = 0;
	    for(int i=0; i<data.length; i++){
			sum += (i * dt - tMean)*(data[i] - yMean);
		}
	    return sum;
	}
	
	public void reset(){
		for(int i = 0; i < data.length; i++){
			data[i] = 0;
		}
	}
}
