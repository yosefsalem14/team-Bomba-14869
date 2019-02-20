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
        double angle = getGoldPosition(10);
        execute(AutoDrivetype.IMU_TURN,angle,2);
        execute(AutoDrivetype.ARM_MOVE,-50,0.5);
        execute(AutoDrivetype.ENCODER_MOVE,20,2);
        execute(AutoDrivetype.ENCODER_MOVE,-10,2);
        execute(AutoDrivetype.IMU_TURN,90,2);
        execute(AutoDrivetype.ENCODER_MOVE,20,2);
        execute(AutoDrivetype.IMU_TURN,135,2);
        execute(AutoDrivetype.ENCODER_MOVE,35,2);
        execute(AutoDrivetype.ARM_MOVE,-50,1);
        execute(AutoDrivetype.INTAKE_MOVE,50,1);
        execute(AutoDrivetype.IMU_TURN,-45,4);
        execute(AutoDrivetype.ENCODER_MOVE,60,2);
    }

}

