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
@Autonomous(name="TESTING")
public class AUTOT extends Auto {
    Robot rover = new Robot();
    private Commands[] move;
    private Commands[] turn;
    private Commands[] strafe;
    @Override
    public void runOpMode() throws InterruptedException{
        Init();
        telemetry.addData("status","ready for start!");
        telemetry.update();
        waitForStart();
        if(opModeIsActive()) {
            double goldPos = getGoldPosition();
            execute(AutoDrivetype.ENCODER_MOVE,16,3);
            execute(AutoDrivetype.IMU_TURN, (int)goldPos, 4*sign(goldPos));
            execute(AutoDrivetype.ENCODER_MOVE,28,4);
            execute(AutoDrivetype.ENCODER_MOVE,-28,4);
            execute(AutoDrivetype.IMU_TURN,0,4);
        }
    }
    public int sign(double num){
        if(num>0){
            return 1;
        }else if(num<0){
            return -1;
        }
        return 0;
    }
    public void Init(){
        rover.init(hardwareMap);
        initialize();
        DcMotor[] right = {rover.mainMotors[0],rover.mainMotors[2]};
        DcMotor[] left = {rover.mainMotors[1],rover.mainMotors[3]};
        DcMotor[] x1 = {rover.mainMotors[0],rover.mainMotors[3]};
        DcMotor[] x2 = {rover.mainMotors[1],rover.mainMotors[2]};


        Commands rightFwd = new Commands(right,0.5,Commands.Direction.FORWARD);
        Commands rightRvrs = new Commands(right,0.5,Commands.Direction.REVERSE);
        Commands leftFwd = new Commands(left,0.5,Commands.Direction.FORWARD);
        Commands strafe1 = new Commands(x1,0.5,Commands.Direction.REVERSE);
        Commands strafe2 = new Commands(x2,0.5,Commands.Direction.FORWARD);
        Commands[] move_I = {leftFwd,rightFwd};
        Commands[] turn_I = {rightRvrs,leftFwd};
        Commands[] strafe_I = {strafe2,strafe1};
         move = move_I;
         turn = turn_I;
         strafe = strafe_I;
    }
    public void execute(AutoDrivetype movement,int goal,int timeOut)throws InterruptedException{
        switch(movement){
            case ENCODER_MOVE:
                autoDrive(movement,move,goal,timeOut);
                break;
            case IMU_TURN:
                autoDrive(movement,turn,goal,timeOut);
                break;
            case ENCODER_STRAFE:
                autoDrive(movement,strafe,goal,timeOut);
                break;
        }

    }
}
