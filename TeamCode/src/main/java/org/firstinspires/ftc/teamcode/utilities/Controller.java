package org.firstinspires.ftc.teamcode.utilities;
import android.util.Log;

import java.util.logging.*;
public class Controller {
    private PID moveRamper;
    private PID turnRamper;
    private double currentPowerMain;
    private double currentPowerArm;
    public Controller(){
        this.moveRamper = new PID( 0.18,0,0);
        this.turnRamper = new PID(1, 0.0, -0.2);
        this.currentPowerMain = 0;
        this.currentPowerArm = 0;
    }
    public Controller(PID pid){
        this.moveRamper = new PID(pid.getKP(), pid.getKI(), pid.getKD());
    }
    public void reset(){
        moveRamper.reset();
        turnRamper.reset();
    }

        public double rampUp(double target,boolean condition){
        if(Math.abs(currentPowerMain)<=0.001){
            reset();
            currentPowerMain=0;
        }
        PID cont = this.moveRamper;
        if(condition){
           cont = this.turnRamper;
        }
        double error = target - currentPowerMain;
        this.currentPowerMain += cont.getPower(error);

        return this.currentPowerMain;
    }
    public double useSmoothing(double input){
        if(input>0) {
            return 3*1*(1-input)*Math.pow(input,2)+1*Math.pow(input,3);
        }
        return -
                .3*1*(1-input)*Math.pow(input,2)+1*Math.pow(input,3);
    }
      public   double rampUp(double target){
        if(Math.abs(currentPowerArm)<=0.001){
            moveRamper.reset();
            currentPowerArm=0;
        }
        double error = target - this.currentPowerArm;
        this.currentPowerArm += moveRamper.getPower(error) ;
        return this.currentPowerArm;
    }


}
