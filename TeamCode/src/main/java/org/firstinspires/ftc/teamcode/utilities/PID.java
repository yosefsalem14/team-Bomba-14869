package org.firstinspires.ftc.teamcode.utilities;
public class PID  {
    private double KP;
    private double KI;
    private double KD;
    private double errorSum;
    private double currentTime;
    private double previousTime;
    private double previousError;
    public PID(double KP,double KI,double KD){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.errorSum = 0;
        this.currentTime = System.currentTimeMillis();
        this.previousTime = 0;
    }
    public double getError(double current,double target){

        return target-current;
    }
    public double getPower(double error){
        errorSum += error;
        currentTime = System.currentTimeMillis();
        double DT = currentTime - previousTime;
        double P = error;
        double I = errorSum;
        double D = (error - previousError)/DT;
        previousError = error;
        previousTime = System.currentTimeMillis();

        return P*KP + I*KI + D*KD;
    }

    public double getKP(){
        return KP;
    }
    public double getKI(){
        return KI;
    }
    public double getKD(){
        return KD;
    }
    public void doubleSetCoefficients(double KP,double KI,double KD){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
    }
    public void reset(){
        this.previousError = 0;
        this.errorSum = 0;
        this.previousTime = 0;
    }
}
