package org.firstinspires.ftc.teamcode.utilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public  class commands  {
    //a simple command interface to work with the autonomous system
    //warning: using the same motors with different commands will result in complications

    //TODO:
    //-all done [☺]
    //-calculate magic number[ ]
    //-maybe add an interrupt function [ ]
    //-make a custom way of moving to goal [☺]
    private DcMotor[] motors;
    private double power;
    private int targetMotor ;
    private static final double MAGIC_NUMBER = 500;
    public commands(DcMotor[] motors,double power){
        this.motors = motors;
        this.power = power;
        this.targetMotor = 0;
    }
    //get all the motors,that this command uses
    public DcMotor[] getMotors(){
        return this.motors;
    }
    //get the power that the motors work at
    public double getPower(){
        return this.power;
    }
    //functions:
    //execute the command, in this case, it's just move to position
    public void execute() {
        for (int i = 0;i<this.motors.length;i++) {
                motors[i].setPower(this.power);
        }
    }

    //stop executing the command,this makes sure all the motors stop
    public void stop(){
        for (int i = 0;i<this.motors.length;i++) {
            motors[i].setPower(0);
        }
    }
    //initialize the command,I.e, calculate the distance and change the run mode
    public void init(double dst){
        for(int i =0;i<this.motors.length;i++){
        //reset the encoder, change the behaviour, calculate position
        //then set the target position and change the mode
        motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  //      motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   //     motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int pos = motors[i].getCurrentPosition() + (int) ((dst) * MAGIC_NUMBER);
        motors[i].setTargetPosition(pos);
        motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    }
    public void updatePower(double power){
        for(DcMotor motor : this.motors){
            motor.setPower(Range.clip(power,-this.power,this.power));
        }
    }
    public boolean canMove() {
        boolean execute = true;
        for(int i =0;i<this.getMotors().length;i++){
            if(getDist(i)<=0){
                execute = false;
                targetMotor = i;
            }
        }
        return execute;
    }
    public double getDist(int i){
        double current = Math.abs(this.getMotors()[i].getCurrentPosition());


        double target = Math.abs(this.getMotors()[i].getTargetPosition());


        return (target-current);
    }
}


