package org.firstinspires.ftc.teamcode.utilities;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public abstract class autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private boolean canRun = true;
    private PID motorDist;
    private PID motorTurn;
    private Orientation angles;
    BNO055IMU imu;
    // State used for updating telemetry
    //these values wont work!
    //TODO:
    //VERY IMPORTANT:
    //REFACTOR EVERYTHING, MAKE IT READABLE, TIDY THINGS UP
    //FIX CODE, TRANSFER THE AUTO CODE TO HERE AND PUT THE
    // MAGIC NUMBER CALC
    public void getIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void reset(){
        motorTurn.reset();
        motorDist.reset();
    }
    public void initialize(){
         angles=new Orientation();
        getIMU();
        motorDist = new PID(0.1,0,0.02);
        motorTurn = new PID(0.002,0,0.004);
    }
    public void turn(commands[] comms,double angle,double timeout){

        if (opModeIsActive()) {
            reset();
            for(commands command : comms) {
                command.init(0);//SET TARGET SO IT DOESN'T CRASH,
                //// THIS IS POINTLESS
            }


            //rest run time
            runtime.reset();
            //set all the powers
            for(commands command:comms) {
                command.execute();
            }

            //wait until the motors reach their goal
            //or until the time runs out
            boolean condition;
//            angle +=Math.abs(angles.firstAngle);
            condition = Math.abs(angle)-Math.abs(angles.firstAngle)>0;
            while (opModeIsActive() &&condition&&
                        runtime.seconds() < timeout) {
                    //work until all the motors stop or until the time runs out
                    telemetry.addData("status", "busy");
                    telemetry.addData("angle",angles.firstAngle);
                telemetry.addData("Target angle",angle);
                    telemetry.update();
                    angles =imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    condition = Math.abs(angle)-Math.abs(angles.firstAngle)>0;
                    for(commands command : comms){
                        command.updatePower(Math.abs(angle)-Math.abs(angles.firstAngle));
                    }
                    idle();
             }
                //MIGHT REMOVE STOP!
            //stop all motors
            for(commands command:comms) {
                command.stop();
            }

            telemetry.addData("status","finished");


            telemetry.update();
            //reset all the encoders and the PID controller
            motorTurn.reset();
            for(commands command:comms) {
                for (DcMotor motor : command.getMotors()) {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }


        }
    }
    public void execute(commands[] comms,double distance,double timeout) {

        if (opModeIsActive()) {
            reset();
            for(commands command : comms) {
                command.init(distance);
            }


            //rest run time
            runtime.reset();

            //set all the powers
            for(commands command:comms) {
                command.execute();
            }

            //wait until the motors reach their goal
            //or until the time runs out
            for(commands command:comms) {
                while (opModeIsActive() &&canRun&&
                        runtime.seconds() < timeout) {
                    double power = motorDist.getPower(command.getDist(0));
                    for(commands c:comms){
                        c.updatePower(power);
                    }
                    //work until all the motors stop or until the time runs out
                    telemetry.addData("status", "busy");
                    telemetry.addData("power!",
                            power);
                    telemetry.update();
                    canRun = isBusy(command, 0);
                    idle();
                }
            }

            //stop all motors
            for(commands command:comms) {
                command.stop();
            }
            telemetry.addData("status","finished");
            telemetry.update();
            //reset all the encodes and the PID controller
            motorDist.reset();
            for(commands command:comms) {
                for (DcMotor motor : command.getMotors()) {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }


            }
        }
    public boolean isBusy(commands command,int i){
        // checks if all the motors are busy
        if(i<=command.getMotors().length-1)
            return command.canMove() && isBusy(command,i+1);
        return true;
    }
    public void autoDrive(type T,commands[] command,double dist,double time){
        if(T==T.MOVE){
            this.execute(command,dist,time);
        }else if(T==T.TURN){
            this.turn(command,dist,time);
        }
    }
}
