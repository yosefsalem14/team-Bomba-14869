package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utilities.Auto;
import org.firstinspires.ftc.teamcode.utilities.AutoDrivetype;

@Autonomous(name="DEPOT")
public class Auto2019_DEPOT extends Auto {
    Robot rover = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto(rover);

        telemetry.addData("status", "ready for start!");
        telemetry.update();
        waitForStart();
        execute(AutoDrivetype.ARM_MOVE,-10,1);
        execute(AutoDrivetype.CLOSE_SERVOS);
        execute(AutoDrivetype.ARM_MOVE,-10,1);
        execute(AutoDrivetype.ARM_MOVE,0.3,20,2);
        execute(AutoDrivetype.ENCODER_MOVE,-5,1);
        execute(AutoDrivetype.LATCH_MOVE,25,2);
        double gold = getGoldPosition(5);

        execute(AutoDrivetype.ARM_MOVE, 1, -40, 2);
        execute(AutoDrivetype.IMU_TURN, gold * 25, 4);
        execute(AutoDrivetype.ENCODER_MOVE, 22 + Math.abs(gold) * 2, 4);
        execute(AutoDrivetype.ENCODER_MOVE, -10, 4);
        execute(AutoDrivetype.IMU_TURN, 85, 4);
        execute(AutoDrivetype.ENCODER_MOVE, 27 - gold * 6, 4);
        execute(AutoDrivetype.IMU_TURN, -35, 4);
        execute(AutoDrivetype.ENCODER_MOVE, 30, 4);
        execute(AutoDrivetype.INTAKE_MOVE, 1, 0.5);
        execute(AutoDrivetype.IMU_TURN, -50, 4);
        execute(AutoDrivetype.ENCODER_MOVE, -40, 5);
    }
}
