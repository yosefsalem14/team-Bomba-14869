package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.utilities.Auto;

@Autonomous (name="VISION_TEST")
public class test extends Auto {
    Robot rover = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        initAuto(rover);

        telemetry.addData("status", "ready for start!");
        telemetry.update();
        waitForStart();
        double gold = getGoldPosition(5);
        while (opModeIsActive()) {
            telemetry.addData("gold pos", gold);
            telemetry.update();
        }



        }
}
