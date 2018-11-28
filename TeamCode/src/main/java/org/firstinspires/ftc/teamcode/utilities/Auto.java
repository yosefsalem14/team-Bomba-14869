package org.firstinspires.ftc.teamcode.utilities;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
/////////////////// COULD BE DONE //////////////
///////////////////       :>      //////////////

///LAST TODO: fix Direction handling
public abstract class Auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private PID motorDist;
    private PID motorTurn;
    private Orientation angles;
    private BNO055IMU imu;


    /* get the IMU sensor

     */
    public void getIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
    }


    /* reset all the PID's used in the Auto class
     */
    public void reset(){
        motorTurn.reset();
        motorDist.reset();
    }


    /*
     initialize all the variables(including PIDs)
     */
    public void initialize(){
         angles=new Orientation();
        getIMU();
        motorDist = new PID(0.03809998542908,0.0,0.005398858);
        motorTurn = new PID(0.034854899,0.0,0.000564959);
        //motorTurn = new PID(0.01849998542908,0.0,0.000598858);
    }

    /*
        ******NOTE******
        * Both turn and move take an array of commands,
        * the functions will execute these commands at the same time
        * this is done to give the programmer more flexibility,
     */


    /*
        IMU_TURN function:
        robot will work with a PID until it reaches the desired angle
        the robot will automatically stop moving if the command times out
        (if the timer count exceeds timeout)
     */
    public void turn(Commands[] comms,double angle,double timeout){

        if (opModeIsActive()) {
            reset();
            //rest run time
            runtime.reset();
            /*
            command.execute() might be pointless here!
            check it next time!
            this is part of the new Direction handling :>
             */
//            //set all Che powers
//            for(Commands command:comms) {
//                command.execute();
//            }

            //wait until the moto1rs reach their goal
            //or until the time runs out
//            angle +=Math.abs(angles.firstAngle);
            //might add some offset to the confition, if a perfect PID isn't feasible
            boolean canRun;
            canRun = Math.abs(angle)-Math.abs(angles.firstAngle)>0;
            while (opModeIsActive() &&canRun&&
                        runtime.seconds() < timeout) {
                    angles =imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    double dist = Math.abs(angle)-Math.abs(angles.firstAngle);
                    double power = motorTurn.getPower(dist);
                    for(Commands command : comms){command.updatePower(power);}
                    //work until all the motors stop or until the time runs out
                telemetry.addData("angle",angles.firstAngle);
                    telemetry.addData("Target angle",angle);
                    telemetry.addData("dist", dist);
                    telemetry.update();
                    canRun = Math.abs(dist)>0;
             }
             for(Commands C : comms){
                C.stop();
             }
            telemetry.addData("status","finished");
            telemetry.update();
        } }



    /*
        ENCODER_DRIVE(MOVE) function:
        robot will keep moving until it reachess the desired distance
        the function will automatically stop when the timer count exceeds
        the timeout variable

     */
    public void move(Commands[] comms,double distance,double timeout) {
        if (opModeIsActive()) {
            reset();
            for(Commands command : comms) {
                command.init(distance);
            }
            //rest run time
            runtime.reset();
            /*
            command.execute() might be pointless here!
            check it next time!
            this is part of the new Direction handling :>
             */
//            //set all powers
//            for(Commands command:comms) {
//                command.execute();
//            }
            //Cait until the motors reach their goal
            //or until the time runs out
            boolean canRun = true;
                while (opModeIsActive() &&canRun&&
                        runtime.seconds() < timeout) {
                    for(Commands command:comms) {
                    double power = motorDist.getPower(command.getDist(0));
                    command.updatePower(power);
                    //work until all the motors stop or until the time runs out
                    telemetry.addData("dist", command.getDist(0));
                    telemetry.update();
                    canRun = isBusy(command, 0);
                    }
                }

            for(Commands command : comms){
                command.stop();
            }
            telemetry.addData("status","finished");
            telemetry.update();
            }
        }
    /*
        this is used in the move() function to check
        whether the one of the commands in the command array reached it's desired
        goal, it uses a bit of recursion to go through every command
        and check, all the commands in the Commands array will abort if
        it reaches the goal, this is done because we're only using 2
        encoders, the encoders are wired so each possible Commands array
        has at least one encoder.
     */
    public boolean isBusy(Commands command,int i){
        // checks if all the motors are busy
        if(i<=command.getMotors().length-1)
            return command.canMove() && isBusy(command,i+1);
        return true;
    }


    /*
        this will be used by the programmer to call the movement type
        that he/she wants, this is done so the programmer doesn't have
        to memorise functions, more functions could potentially get added later

     */
    public void autoDrive(AutoDrivetype moveType,Commands[] command,double goal,double time){
        switch(moveType){
            case ENCODER_MOVE:
                this.move(command,goal,time);
                break;
            case IMU_TURN:
                this.turn(command,goal,time);
                break;
        }
    }
}
