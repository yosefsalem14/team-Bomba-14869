package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.autonomous;
import org.firstinspires.ftc.teamcode.utilities.commands;
import org.firstinspires.ftc.teamcode.utilities.type;

@Autonomous(name="TESTING")
public class AUTOT extends autonomous{
    Robot rover = new Robot();
    @Override
    public void runOpMode(){
        rover.init(hardwareMap);
        DcMotor[] right = {rover.mainMotors[1],rover.mainMotors[3]};
        DcMotor[] left = {rover.mainMotors[0],rover.mainMotors[2]};
        commands rightFwd = new commands(right,0.5);
        commands rightRvrs = new commands(right,-0.5);
        commands leftFwd = new commands(left,0.5);
        commands leftrvrs = new commands(left,-0.5);

        commands[] moveFwd = {rightFwd,leftFwd};
        commands[] turnRight = {rightRvrs,leftFwd};
        waitForStart();

        autoDrive(type.MOVE,moveFwd,32,10);
        autoDrive(type.TURN,turnRight,90,5);
    }
}
