package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
public class Robot {
    DcMotor[] mainMotors = null;

    DcMotor[] armMotors = null;

    DcMotor stretcher = null;

    DcMotor collector = null;

    BNO055IMU imu = null;
    Servo[] latches = null;

    Servo[] cubeIntakes = null;
    //  CRServo backMotor = null;
    /* Define the powers
     *
     */
    //EDIT THESE VALUES TO CHANGE POWERS//
    ////////////////////////////////////
    final double movePower = 0.7;     //
    final double turnPower = 0.8;       //
    final double strafePower = 1;   //
    final double armPower = 0.8;      //
    final double collectPower = 1;    //                                                                                                                                                                                                                                                                                                                                                                            `
    final double stretchPower = 1;    //
    ////////////////////////////////////
    /*
     * get the hardware map
     */

    private HardwareMap hw = null;

    /*
     * initialise the robot class
     */
    public Robot() {
        mainMotors = new DcMotor[4];
        armMotors = new DcMotor[2];
        latches = new Servo[2];
        cubeIntakes = new Servo[2];
    }

    /*
        this function handles hardWare, it initialize everything that will be used
        in this year's competition,
        NOTE: might use a bit of threading to make this more efficient

     */
    public void init(HardwareMap hw) {
        this.hw = hw;
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
            for (int i = 0; i < mainMotors.length; i++) {
                mainMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
//            //set the direction
            for(int i =0;i<mainMotors.length;i++){
                if((i & 0x1)==0)
                    mainMotors[i].setDirection(DcMotor.Direction.FORWARD);
                else
                    mainMotors[i].setDirection(DcMotor.Direction.REVERSE);

            }
        } catch (Exception notF) {
            for (int i = 0; i < mainMotors.length; i++) {
                mainMotors[i] = null;
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


            for (int i = 0; i < armMotors.length; i++) {
                if ((i & 0x1) == 0)
                    armMotors[i].setDirection(DcMotor.Direction.REVERSE);
                else
                    armMotors[i].setDirection(DcMotor.Direction.FORWARD);

                armMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }


        } catch (Exception notF) {
            for (int i = 0; i < armMotors.length; i++) {
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


        } catch (Exception notF) {
            stretcher = null;
        }
        /**
         * get the collector motor
         */
        try {
            collector = this.hw.get(DcMotor.class, "collector");

            collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            collector.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception notF) {

            collector = null;
        }


        /*
        get the latch servos
         */
        try {
            latches[0] = this.hw.get(Servo.class, "latchRight");
            latches[1] = this.hw.get(Servo.class, "latchLeft");
        } catch (Exception notF) {
            for (int i = 0; i < latches.length; i++) {
                latches[i] = null;
            }
        }
        try{
            cubeIntakes[0] = this.hw.get(Servo.class,"cubeIntakeLeft");
            cubeIntakes[1] = this.hw.get(Servo.class,"cubeIntakeRight");
        }catch(Exception e){
            for(int i =0;i<cubeIntakes.length;i++){
                cubeIntakes[i] = null;
            }
        }
        for(int i =0;i<latches.length;i++){
            latches[i].setDirection(Servo.Direction.FORWARD);
        }

            cubeIntakes[0].setDirection(Servo.Direction.FORWARD);
        cubeIntakes[1].setDirection(Servo.Direction.REVERSE);

    }
    }

