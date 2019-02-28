package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utilities.Auto;
import org.firstinspires.ftc.teamcode.utilities.AutoDrivetype;
@Autonomous(name="DEPOT_NO_LANDING")
public class AUTO2019_DEPOT_NO_LANDING extends Auto {
    Robot rover = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto(rover);

        telemetry.addData("status", "ready for start!");
        telemetry.update();
        waitForStart();
        execute(AutoDrivetype.ARM_MOVE,25,2);
        double gold = getGoldPosition(5);
        execute(AutoDrivetype.ARM_MOVE, 1, -25, 2);
        execute(AutoDrivetype.IMU_TURN, gold * 25 , 2);
        execute(AutoDrivetype.ENCODER_MOVE, 0.7,22 + Math.abs(gold) * 2, 4);
        if(gold!=0) {
            execute(AutoDrivetype.ENCODER_MOVE, 0.7, -15, 4);
        }else{
            execute(AutoDrivetype.ENCODER_MOVE, 0.7, -11, 4);
        }
        execute(AutoDrivetype.IMU_TURN, 0.8,75, 4);
        if(gold==1) {
            execute(AutoDrivetype.ENCODER_MOVE, 0.7, 23, 4);
        }else if(gold==0){
            execute(AutoDrivetype.ENCODER_MOVE, 0.7, 30, 4);
        }else{
            execute(AutoDrivetype.ENCODER_MOVE, 0.7, 34, 4);
        }
        execute(AutoDrivetype.IMU_TURN, 0.8,-45, 2);//put to - 30 if doesn't ework
        execute(AutoDrivetype.ENCODER_MOVE,0.7,32, 4);
        execute(AutoDrivetype.INTAKE_MOVE, 1, 0.5);
        execute(AutoDrivetype.IMU_TURN, 0.8,-45, 4);
        execute(AutoDrivetype.ENCODER_MOVE, 0.7,-49 , 5);
    }
}
