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
    private boolean canRun      = true;
    private PID motorDist;
    private PID motorTurn;
    private Orientation angles;
    BNO055IMU imu;
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
        motorDist = new PID(0.01049998542908,0.0,0.000098858);
        motorTurn = new PID(0.004854899,0.0,0.000004959);
    }
    public void turn(commands[] comms,double angle,double timeout){

        if (opModeIsActive()) {
            reset();
            //rest run time
            runtime.reset();
            //set all the powers
            for(commands command:comms) {
                command.execute();
            }

            //wait until the moto1rs reach their goal
            //or until the time runs out
            boolean condition;
//            angle +=Math.abs(angles.firstAngle);
            //might add some offset to the condition, if a perfect PID isn't feasible
            condition = Math.abs(angle)-Math.abs(angles.firstAngle)>=4;
            while (opModeIsActive() &&condition&&
                        runtime.seconds() < timeout) {
                    angles =imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    double dist = Math.abs(angle)-Math.abs(angles.firstAngle);
                    double power = motorTurn.getPower(dist);
                    //work until all the motors stop or until the time runs out
                    telemetry.addData("status", "busy");
                    telemetry.addData("angle",angles.firstAngle);
                    telemetry.addData("Target angle",angle);
                    telemetry.addData("dist", dist);
                    telemetry.update();
                    for(commands command : comms){
                        command.updatePower(power);
                    }
                    condition = Math.abs(angle)-Math.abs(angles.firstAngle)>0;
                    idle();
             }
                //MIGHT REMOVE STOP!
            //stop all motors
            //this is to make sure it comes to a full stop, it's kinda pointless
            for(commands command:comms) {
                command.stop();
            }

            telemetry.addData("status","finished");
            telemetry.update();
        }
    }
    public void move(commands[] comms,double distance,double timeout) {
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
            //will remove later if I don't find a perfect PID
            for(commands command:comms) {
                command.stop();
            }
            telemetry.addData("status","finished");
            telemetry.update();
            }
        }
    public boolean isBusy(commands command,int i){
        // checks if all the motors are busy
        if(i<=command.getMotors().length-1)
            return command.canMove() && isBusy(command,i+1);
        return true;
    }
    public void autoDrive(type moveType,commands[] command,double dist,double time){
        switch(moveType){
            case MOVE:
                this.move(command,dist,time);
                break;
            case TURN:
                this.turn(command,dist,time);
                break;
        }
    }
}
