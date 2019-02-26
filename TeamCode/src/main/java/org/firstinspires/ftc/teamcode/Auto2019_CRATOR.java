package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utilities.Auto;
import org.firstinspires.ftc.teamcode.utilities.AutoDrivetype;

//Log.i("info","going forwards");
//Use Log.i() for debugging!
//Can now debug easilly :>


//TODO: fix PID coefficients, make sure recognition works first!
@Autonomous(name="CRATOR")
public class Auto2019_CRATOR extends Auto {
    Robot rover = new Robot();
    @Override
    public void runOpMode() throws InterruptedException{
        initAuto(rover);

        telemetry.addData("status","ready for start!");
        telemetry.update();
        waitForStart();
        execute(AutoDrivetype.ARM_MOVE,-7,1);
        execute(AutoDrivetype.CLOSE_SERVOS);
        execute(AutoDrivetype.ARM_MOVE,-7,1);
        execute(AutoDrivetype.ARM_MOVE,0.3,20,80);
        execute(AutoDrivetype.ENCODER_MOVE,-5,1);
        execute(AutoDrivetype.LATCH_MOVE,25,2);
        double gold = getGoldPosition(5);
        execute(AutoDrivetype.ARM_MOVE,1,-40,2);
        execute(AutoDrivetype.IMU_TURN,gold*25,4);
        execute(AutoDrivetype.ENCODER_MOVE,22 + Math.abs(gold)*2,4);
        execute(AutoDrivetype.ENCODER_MOVE,-10,4);
        execute(AutoDrivetype.IMU_TURN,85,4);
        execute(AutoDrivetype.ENCODER_MOVE,27 - gold*6,4);
        execute(AutoDrivetype.IMU_TURN,125,4);
        execute(AutoDrivetype.ENCODER_MOVE,23,4);
        execute(AutoDrivetype.INTAKE_MOVE,1,0.5);
        execute(AutoDrivetype.IMU_TURN,140,4);
        execute(AutoDrivetype.ENCODER_MOVE,-40,5);
    }

}

