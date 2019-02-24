package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.utilities.Auto;
import org.firstinspires.ftc.teamcode.utilities.AutoDrivetype;
import org.firstinspires.ftc.teamcode.utilities.Robot;

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
        double gold = getGoldPosition(5);
        while(opModeIsActive()){
            telemetry.addData("gold os", gold);
            telemetry.update();
        }
    }

}

