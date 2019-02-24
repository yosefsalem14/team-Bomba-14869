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
        this.turnRamper = new PID(0.8, 0.0, 0);
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
