package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.AutoDrivetype;
import org.firstinspires.ftc.teamcode.utilities.Auto;
import org.firstinspires.ftc.teamcode.utilities.Commands;
@Autonomous(name="TESTING")
public class AUTOT extends Auto {
    Robot rover = new Robot();

    @Override
    public void runOpMode(){
        rover.init(hardwareMap);
        initialize();
        DcMotor[] right = {rover.mainMotors[0],rover.mainMotors[2]};
        DcMotor[] left = {rover.mainMotors[1],rover.mainMotors[3]};
        DcMotor[] x1 = {rover.mainMotors[0],rover.mainMotors[3]};
        DcMotor[] x2 = {rover.mainMotors[1],rover.mainMotors[2]};
        //TODO:fix command dir name
        Commands rightFwd = new Commands(right,0.8,Commands.Direction.FORWARD);
        Commands rightRvrs = new Commands(right,0.8,Commands.Direction.REVERSE);
        Commands leftFwd = new Commands(left,0.8,Commands.Direction.FORWARD);
        Commands strafe1 = new Commands(x1,0.3,Commands.Direction.REVERSE);
        Commands strafe2 = new Commands(x2,0.3,Commands.Direction.FORWARD);
        for(DcMotor M : right){
            telemetry.addData("Motor port",M.getPortNumber());
        }
        telemetry.update();
        Commands[] move = {leftFwd,rightFwd};
        Commands[] turn = {rightRvrs,leftFwd};
        Commands[] strafe = {strafe2,strafe1};
        waitForStart();
        autoDrive(AutoDrivetype.ENCODER_MOVE,move,16,80);
        autoDrive(AutoDrivetype.IMU_TURN,turn,45,80);
        autoDrive(AutoDrivetype.ENCODER_MOVE,move,16,80);
        autoDrive(AutoDrivetype.ENCODER_MOVE,move,-14,80);
        autoDrive(AutoDrivetype.IMU_TURN,turn,-90,80);
        autoDrive(AutoDrivetype.ENCODER_MOVE,move,48,80);
        autoDrive(AutoDrivetype.IMU_TURN,turn,-135,80);
        autoDrive(AutoDrivetype.ENCODER_MOVE,move,56,80);
        autoDrive(AutoDrivetype.ENCODER_STRAFE,strafe,8,50);
    }
}
