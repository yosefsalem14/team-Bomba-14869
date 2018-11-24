package org.firstinspires.ftc.teamcode.utilities;
public class PID  {
    private double KP;
    private double KI;
    private double KD;
    private final long DT = 100;
    private double errorSum;
    private double previousError = 0;
    public PID(double KP,double KI,double KD){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.errorSum = 0;
    }
    public double getError(double current,double target){

        return target-current;
    }
    public double getPower(double error){
        errorSum += error;
        double P = error;
        double I = errorSum;
        double D = (error - previousError)/DT;
        previousError = error;
        try{
        wait(DT);
        }catch(Exception e){ }

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
        previousError = 0;
        errorSum = 0;
    }
}
