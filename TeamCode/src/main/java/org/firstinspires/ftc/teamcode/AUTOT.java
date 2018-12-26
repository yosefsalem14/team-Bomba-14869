package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import android.util.Log;
import org.firstinspires.ftc.teamcode.utilities.AutoDrivetype;
import org.firstinspires.ftc.teamcode.utilities.Auto;
import org.firstinspires.ftc.teamcode.utilities.Commands;

//Log.i("info","going forwards");
//Use Log.i() for debugging!
//Can now debug easilly :>


//TODO: fix PID coefficients, make sure recognition works first!
@Autonomous(name="TESTING")
public class AUTOT extends Auto {
    Robot rover = new Robot();
    @Override
    public void runOpMode() throws InterruptedException{
        initAuto(rover);
        telemetry.addData("status","ready for start!");
        telemetry.update();
        waitForStart();
        double gold = getGoldPosition();
        execute(AutoDrivetype.IMU_TURN,gold,6);
        execute(AutoDrivetype.ENCODER_MOVE,25,6);
    }

}

