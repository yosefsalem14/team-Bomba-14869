package org.firstinspires.ftc.teamcode.utilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
/////////////////// COULD BE DONE //////////////
///////////////////       :>      //////////////
public class Commands  {
    //a simple command interface to work with the autonomous system
    //warning: using the same motors with different Commands will result in complications
    public enum Direction {
        FORWARD,
        REVERSE
    }


    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     MAGIC_NUMBER         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private boolean busy;
    private DcMotor[] motors;
    private double power;
    private int direction = 0;
    private ElapsedTime runTime;
    private double timeGoal;

    /*
        Commands constructor, takes an array of DcMotors, their desired power
        and their Direction.
     */
    public Commands(){
        this.power = 0;
        this.direction = 0;
        this.busy = false;
        this.runTime = new ElapsedTime();
        this.timeGoal = 0;
    }
    public Commands(DcMotor[] motors,double power,Direction D){
        this.motors = motors;
        this.power = power;
        this.busy = false;
        this.runTime = new ElapsedTime();
        this.timeGoal = 0;
        switch(D){
            case FORWARD:
                this.direction = 1;
                break;
            case REVERSE:
                this.direction = -1;
                break;
            default:
                this.direction = 0;
                break;
        }
    }





    /*
        sets the target distance to the dst variable, although this looks
        like a RUN_TO_POSITION config, it in-fact uses RUN_WITHOUT_ENCODER
        setTargetDist() is used cause am lazy!
     */
    public void init(double dst,double timeGoal){
        if(!busy) {
            for (int i = 0; i < this.motors.length; i++) {
                //reset the encoder, change the behaviour, calculate position
                //then set the target position and change the mode
                motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                int pos = motors[i].getCurrentPosition() + (int) ((dst) * MAGIC_NUMBER);
                motors[i].setTargetPosition(pos);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            runTime.reset();
            this.timeGoal = timeGoal;
            busy = true;
        }
    }
    public void init(double dst){
            timeGoal = 500;
            for (int i = 0; i < this.motors.length; i++) {
                //reset the encoder, change the behaviour, calculate position
                //then set the target position and change the mode
                motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                int pos = motors[i].getCurrentPosition() + (int) ((dst) * MAGIC_NUMBER);
                motors[i].setTargetPosition(pos);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
    }



    /*
        return the Commands' Dcmotor array
     */
    public DcMotor[] getMotors(){
        return this.motors;
    }




    /*
        return the Commands' deired power
     */
    public double getPower()
    {
        return this.power;
    }






    /*
        stop all the DcMotors that belong to this Command
     */
    public void stop(){
        for (DcMotor motor :this.motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
        busy = false;
        timeGoal = 0;
    }
    public void stopMotors(){
        for (DcMotor motor :this.motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
    }
    public void setState(boolean newState){
        this.busy = newState;
    }

    /*
        change the motors' power to the power function, this is used along
        with a PID to tune the commands power
     */
    public void updatePower(double power){
        for(DcMotor motor : this.motors){
            motor.setPower(Range.clip(power*this.direction,-this.power,this.power));
        }
    }

    public void updatePower(double power,double limit){
        for(DcMotor motor : this.motors){
            motor.setPower(Range.clip(power*this.direction,-limit,limit));
        }
    }

    /*
        goes through all the DcMotors and checks if one reached
        it's goal, if it 1`did it stops all the command
     */
    public boolean canMove() {
        return (Math.abs(getDist())>=50)&&(runTime.seconds()<timeGoal);
    }



    /*
        returns the dist between the current encoder value and the desired dist
    */
    public double getDist(){
        double dist=0;
        double DA = 0;
        double minDist = this.getMotors()[0].getTargetPosition();
        for(int i =0;i<this.getMotors().length;i++) {
            double current = this.getMotors()[i].getCurrentPosition();
            double target = this.getMotors()[i].getTargetPosition();
            DA = Math.abs(target) - Math.abs(current);
            dist = target - current;
            if(Math.abs(DA)<Math.abs(minDist)){
                minDist = dist;
            }
        }
        return (minDist);
    }
}


