package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class robot {
     DcMotor[] mainMotors = null;

     DcMotor[] armMotors =  null;

     DcMotor stretcher =    null;

     DcMotor collector =    null;

    BNO055IMU imu=null;

     Servo[] latches =      null;
   //  CRServo backMotor = null;

    /** Define the powers
     *
     */
    final double movePower = 0.6;
    final double armPower = 0.7;
    final double collectPower = 1;
    final double stretchPower = 1;
    /**
     * get the hardware map
     */
    private HardwareMap hw = null;

    /**
     * initialise the robot class
     */
    public robot(){
        mainMotors = new DcMotor[4];
        armMotors  = new DcMotor[2];
        latches    = new Servo[2];
    }
    public void init(HardwareMap hw){
        this.hw=hw;
        /**
         * get the movement motors
         *
         */
        try {
            mainMotors[0] = this.hw.get(DcMotor.class, "backLeft");
            mainMotors[1] = this.hw.get(DcMotor.class, "backRight");
            mainMotors[2] = this.hw.get(DcMotor.class, "frontLeft");
            mainMotors[3] = this.hw.get(DcMotor.class, "frontRight");
            //run them without an encoder
            for(int i =0;i<mainMotors.length;i++){
                mainMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mainMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            //set the direction
//            for(int i =0;i<mainMotors.length;i++){
//                if((i & 0x1)==0)
//                    mainMotors[i].setDirection(DcMotor.Direction.FORWARD);
//                else
//                    mainMotors[i].setDirection(DcMotor.Direction.REVERSE);
//
//            }
        }catch(Exception notF){
            for(int i =0;i<mainMotors.length;i++){
                mainMotors[i]=null;
            }
        }

        /**
        get the arm motors
         */
        try {
            armMotors[0] = this.hw.get(DcMotor.class, "armRight");
            armMotors[1] = this.hw.get(DcMotor.class, "armLeft");

            for (int i = 0; i < armMotors.length; i++) {
                armMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


            for(int i =0;i<armMotors.length;i++){
                if((i & 0x1)==0)
                    armMotors[i].setDirection(DcMotor.Direction.FORWARD);
                else
                    armMotors[i].setDirection(DcMotor.Direction.REVERSE);

                armMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }


        }catch(Exception notF){
            for(int i =0;i<armMotors.length;i++){
                armMotors[i] = null;
            }
        }
        /**
         * get the stretcher motors
         */
        try {
            stretcher = this.hw.get(DcMotor.class, "stretcher");

            stretcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            stretcher.setDirection(DcMotor.Direction.REVERSE);


        }catch(Exception notF){
            stretcher=null;
        }
        /**
         * get the collector motor
         */
        try {
            collector = this.hw.get(DcMotor.class, "collector");

            collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            collector.setDirection(DcMotor.Direction.REVERSE);
        }catch(Exception notF){

            collector = null;
        }


        /*
        get the latch servos
         */
        try{
            latches[0] = this.hw.get(Servo.class,"latchRight");
            latches[1] = this.hw.get(Servo.class,"latchLeft");
        }catch(Exception notF){
            for(int i =0;i<latches.length;i++){
                latches[i] = null;
            }
        }

        try{
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            imu = hw.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }catch(Exception notF){
            imu = null;
        }

    }
    public void resetEncoders(){
        for(int i =0;i<mainMotors.length;i++){
            mainMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mainMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public double getDistance(){
        return mainMotors[0].getCurrentPosition();
    }
    public void setMainMovePower(double power){
        for(int i =0;i<mainMotors.length;i++){
            mainMotors[i].setPower(power);
        }
    }
    public double getAngle(){
        double angle= imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return angle;
    }
    public void setMainTurnPower(double power){
        for(int i =0;i<mainMotors.length;i++){
            if((i&0x01)==0) {
                mainMotors[i].setPower(power);
            }else{
                mainMotors[i].setPower(-power);
            }
        }
    }
    public void setMainStrafePower(double power){
        for(int i =0;i<mainMotors.length;i++){
            mainMotors[i].setPower(power);
        }
    }
}
