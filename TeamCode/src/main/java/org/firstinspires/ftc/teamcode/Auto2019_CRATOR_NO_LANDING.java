package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utilities.Auto;
import org.firstinspires.ftc.teamcode.utilities.AutoDrivetype;
@Autonomous(name="CRATOR_NO_LANDING")
public class Auto2019_CRATOR_NO_LANDING extends Auto {
    Robot rover = new Robot();
    @Override
    public void runOpMode() throws InterruptedException{
        initAuto(rover);
        telemetry.addData("status","ready for start!");
        telemetry.update();
        waitForStart();
        execute(AutoDrivetype.ARM_MOVE,23,2);
        double gold = getGoldPosition(5);
        execute(AutoDrivetype.ARM_MOVE,1,-25,2);
        execute(AutoDrivetype.IMU_TURN,1,gold*25 ,4);
        execute(AutoDrivetype.ENCODER_MOVE,0.7,20 + Math.abs(gold)*2,4);
        execute(AutoDrivetype.ENCODER_MOVE,0.7,-12 + Math.abs(gold)*2,4);
        execute(AutoDrivetype.IMU_TURN,0.8,75,4);
        if(gold==1) {
            execute(AutoDrivetype.ENCODER_MOVE, 0.7, 25, 4);
        }else if(gold==0){
            execute(AutoDrivetype.ENCODER_MOVE, 0.7, 32, 4);
        }else{
            execute(AutoDrivetype.ENCODER_MOVE, 0.7, 34, 4);
        }

        execute(AutoDrivetype.IMU_TURN,0.8,128,4);
        execute(AutoDrivetype.ENCODER_MOVE,0.7,30,4);
        execute(AutoDrivetype.INTAKE_MOVE,1,0.5);
        execute(AutoDrivetype.IMU_TURN,0.8,140,4);
        execute(AutoDrivetype.ENCODER_MOVE,0.7,-45,5);
    }
}
