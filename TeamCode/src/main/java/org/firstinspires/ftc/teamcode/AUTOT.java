package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.AutoDrivetype;
import org.firstinspires.ftc.teamcode.utilities.Auto;
import org.firstinspires.ftc.teamcode.utilities.Commands;
@Autonomous(name="TESTING")
public class AUTOT extends Auto{
    Robot rover = new Robot();
    @Override
    public void runOpMode(){
        initialize();
        rover.init(hardwareMap);
        DcMotor[] right = {rover.mainMotors[0],rover.mainMotors[2]};
        DcMotor[] left = {rover.mainMotors[1],rover.mainMotors[3]};
        //TODO:fix command dir name
        Commands rightFwd = new Commands(right,1,Commands.Direction.forward);
        Commands rightRvrs = new Commands(right,1, Commands.Direction.reverse);
        Commands leftFwd = new Commands(left,1, Commands.Direction.forward);
        Commands leftrvrs = new Commands(left,1, Commands.Direction.reverse);

        Commands[] moveFwd = {rightFwd,leftFwd};
        Commands[] turnRight = {rightRvrs,leftFwd};
        telemetry.addData("status","ready");
        telemetry.update();
        waitForStart();
        autoDrive(AutoDrivetype.ENCODER_MOVE,moveFwd,32,80);
        autoDrive(AutoDrivetype.IMU_TURN,turnRight,90,80);
    }
}
