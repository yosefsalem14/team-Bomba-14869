package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.PID;
@Autonomous(name="TEST AUTONOMOUS")
/*
    a testing module for the autonomous class
    NEEDS TESTING
 */
public class auto extends LinearOpMode {
    private final double MAGIC_NUMBER = 5000;
    private robot rover = new robot();
    PID turn = new PID(0.0002,0.0,0.00646);
    PID move = new PID(0.1,0.0,0.1);
    @Override
    public void runOpMode(){
        rover.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("angle",rover.getAngle());
            telemetry.update();
        }
    }

    public void moveForwards(double goal){
        rover.resetEncoders();
        double dist = Math.abs(goal*MAGIC_NUMBER) - rover.getDistance();
        double power = move.getPower(dist);
        while(Math.abs(power)>0.05 && opModeIsActive()){
         rover.setMainMovePower(power);
         dist = Math.abs(goal*MAGIC_NUMBER) - rover.getDistance();
         power = move.getPower(dist);
        }
        rover.setMainMovePower(0);
    }
    public void turn(double goal){
        rover.resetEncoders();
        double dist = Math.abs(goal) - rover.getAngle();
        double power = turn.getPower(dist);
        while(Math.abs(power)>0.05 && opModeIsActive()){
            rover.setMainTurnPower(power);
            dist = Math.abs(goal) - rover.getAngle();
            power = turn.getPower(dist);
        }
        rover.setMainMovePower(0);
    }
}


