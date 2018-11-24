package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.PID;
@Autonomous(name="TEST AUTONOMOUS")
/*
    a testing module for the autonomous class
    NEEDS TESTING
 */
public class auto extends LinearOpMode {
    private final double MAGIC_NUMBER = 100;
    private robot rover = new robot();
    private ElapsedTime matchTime = new ElapsedTime();
    PID turn = new PID(0.01049998542908,0.0,0.0000988/58);
    PID move = new PID(0.56,0.0,0.81);
        //0.016302

    @Override
    public void runOpMode(){
        rover.init(hardwareMap);
        matchTime.reset();
        waitForStart();
        turn(90,1);

    }

    public void moveForwards(double goal,double dir){
        rover.resetEncoders();
        double dist = Math.abs(goal*MAGIC_NUMBER) - Math.abs(rover.getDistance());
        double power = move.getPower(dist);
        while(Math.abs(dist)>0.05 && opModeIsActive()){
         rover.setMainMovePower(power*dir);
            telemetry.addData("goal",goal);
            telemetry.addData("current",rover.getDistance());
            telemetry.update();
         dist = Math.abs(goal*MAGIC_NUMBER) - Math.abs(rover.getDistance());;
         power = move.getPower(dist);
        }
        rover.setMainMovePower(0);
    }
    public void turn(double goal,double dir){
        rover.resetEncoders();
        double dist = Math.abs(goal) - Math.abs(rover.getAngle());
        double power = turn.getPower(dist);
        while(Math.abs(dist)>0 && opModeIsActive()){
            rover.setMainTurnPower(power*dir);
            telemetry.addData("goal",goal);
            telemetry.addData("current",rover.getAngle());
            telemetry.addData("difference",Math.abs(dist));
            telemetry.update();
            dist = Math.abs(goal) - Math.abs(rover.getAngle());
            power = turn.getPower(dist);
        }
        rover.setMainMovePower(0);
    }
}


