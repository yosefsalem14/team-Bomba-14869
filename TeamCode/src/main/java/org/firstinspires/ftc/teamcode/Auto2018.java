package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.utilities.Auto;

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

    }

}

