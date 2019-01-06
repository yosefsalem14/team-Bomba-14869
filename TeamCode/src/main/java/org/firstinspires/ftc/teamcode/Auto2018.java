package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.utilities.Auto;
import org.firstinspires.ftc.teamcode.utilities.AutoDrivetype;

//Log.i("info","going forwards");
//Use Log.i() for debugging!
//Can now debug easilly :>


//TODO: fix PID coefficients, make sure recognition works first!
@Autonomous(name="unlatch")
public class Auto2018 extends Auto {
    Robot rover = new Robot();
    @Override
    public void runOpMode() throws InterruptedException{
        initAuto(rover);

        telemetry.addData("status","ready for start!");
        telemetry.update();
        waitForStart();
        execute(AutoDrivetype.ENCODER_MOVE, -20, 30);
        execute(AutoDrivetype.IMU_TURN,180,30);
        execute(AutoDrivetype.ENCODER_MOVE,20,30);
        execute(AutoDrivetype.IMU_TURN,90,30);
        execute(AutoDrivetype.ARM_MOVE,0.3,-5,0.5);
        execute(AutoDrivetype.ENCODER_MOVE,45,30);
        execute(AutoDrivetype.IMU_TURN,45,30);
        execute(AutoDrivetype.ENCODER_MOVE,8,30);
    }

}

