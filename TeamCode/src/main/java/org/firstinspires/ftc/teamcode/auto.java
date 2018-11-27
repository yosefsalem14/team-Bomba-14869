package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;
import org.firstinspires.ftc.teamcode.utilities.PID;
@Autonomous(name="TEST AUTONOMOUS")
/*
    a testing module for the autonomous class
    NEEDS TESTING
 */

public class auto extends LinearOpMode {
    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     MAGIC_NUMBER         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private Robot rover = new Robot();
    private ElapsedTime matchTime = new ElapsedTime();
    PID turn = new PID(0.01049998542908,0.0,0.0000988/58);
    PID move = new PID(0.004854899,0.0,0.000004959);
        //0.016302

    @Override
    public void runOpMode(){
        rover.init(hardwareMap);
        matchTime.reset();
        telemetry.addData("status","ready for start!");
        telemetry.update();
        waitForStart();
        moveForwards(32,1);
        turn(90,-1);
        moveForwards(16,-1);
        turn(90,1);
    }

    public void moveForwards(double goal,double dir){
        rover.resetEncoders();
        move.reset();
        double dist = Math.abs(goal*MAGIC_NUMBER) - Math.abs(rover.getDistance());
        double power = move.getPower(dist);
        while(Math.abs(dist)>7 && opModeIsActive()){
         rover.setMainMovePower(power*dir);
            telemetry.addData("goal",goal);
            telemetry.addData("current",rover.getDistance());
            telemetry.addData("dist", dist);
            telemetry.update();
         dist = Math.abs(goal*MAGIC_NUMBER) - Math.abs(rover.getDistance());;
         power = move.getPower(dist);
        }

        rover.setMainMovePower(0);
    }
    public void turn(double goal,double dir){
        rover.resetEncoders();
        turn.reset();
        double dist = Math.abs(goal) - Math.abs(rover.getAngle());
        double power = turn.getPower(dist);
        while(Math.abs(dist)>3 && opModeIsActive()){
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


