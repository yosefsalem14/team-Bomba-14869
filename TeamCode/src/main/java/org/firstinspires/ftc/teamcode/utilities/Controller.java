package org.firstinspires.ftc.teamcode.utilities;
import android.util.Log;

import java.util.logging.*;
public class Controller {
    private PID moveRamper;
    private PID turnRamper;
    private double currentPower;
    public Controller(double P,double I,double D){
        this.moveRamper = new PID( 0.2,0,0.000135);
        this.turnRamper = new PID(P,I,D);
        this.currentPower = 0;
    }
    public void reset(){
        moveRamper.reset();
        turnRamper.reset();
    }
    public double rampUp(double target,boolean condition){
        PID cont = this.turnRamper;
        if(condition){
           cont = this.moveRamper;
        }else{
            if(target==0){
                currentPower = 0;
                return 0;
            }
        }
        double error = target - this.currentPower;
        this.currentPower += cont.getPower(error);
        return this.currentPower;
    }


}
