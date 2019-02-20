package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.utilities.Commands;

public class Robot {
    public DcMotor[] mainMotors = null;

    public  DcMotor[] armMotors = null;

    public DcMotor latchMotor = null;

    public DcMotor collector = null;

    public Servo[] supportServos;

    public Servo[] latches = null;
    public Servo[] cubeIntakes = null;

    //2018 auto stuff
    public Commands[] move;

    public Commands[] turn;

    public Commands[] intakeMove;

    public Commands[] latchMove;

    public Commands[] armMove;
    //  CRServo backMotor = null;
    /* Define the powers
     *
     */
    //EDIT THESE VALUES TO CHANGE POWERS//
    ////////////////////////////////////
    public final double movePower = 1;     //
    public final double turnPower = 1;     //
    public final double armPower = 1;      //
    public final double collectPower = 0.7;    //                                                                                                                                                                                                                                                                                                                                                                            `
    public final double latchPower = 1;    //
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
        supportServos = new Servo[2];
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
                        mainMotors[i].setDirection(DcMotor.Direction.REVERSE);
                    else
                        mainMotors[i].setDirection(DcMotor.Direction.FORWARD);
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
                    armMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                armMotors[0].setDirection(DcMotor.Direction.REVERSE);
                armMotors[1].setDirection(DcMotor.Direction.FORWARD);
            } catch (Exception notF) {
                for (int i = 0; i < armMotors.length; i++) {
                    armMotors[i] = null;
                }
            }
            /**
             * get the stretcher motors
             */
            try {
                latchMotor = this.hw.get(DcMotor.class, "latchMotor");
                latchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                latchMotor.setDirection(DcMotor.Direction.REVERSE);
            } catch (Exception notF) {
                latchMotor = null;
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
                for(int i =0;i<latches.length;i++){
                    latches[i].setDirection(Servo.Direction.FORWARD);
                }
            } catch (Exception notF) {
                for (int i = 0; i < latches.length; i++) {
                    latches[i] = null;
                }
            }
            try{
                cubeIntakes[0] = this.hw.get(Servo.class,"cubeIntakeLeft");
                cubeIntakes[1] = this.hw.get(Servo.class,"cubeIntakeRight");
                cubeIntakes[0].setDirection(Servo.Direction.FORWARD);
                cubeIntakes[1].setDirection(Servo.Direction.FORWARD);
            }catch(Exception e){
                for(int i =0;i<cubeIntakes.length;i++){
                    cubeIntakes[i] = null;
                }
            }


            //get the marker Servo

            try{
                this.supportServos[0] = hw.get(Servo.class,"supportRight");
                this.supportServos[1] = hw.get(Servo.class,"supportLeft");
                supportServos[0].setDirection(Servo.Direction.FORWARD);
                supportServos[1].setDirection(Servo.Direction.REVERSE);
            }catch(Exception e){
                for(int i =0;i<supportServos.length;i++){
                    supportServos[i] = null;
                }
            }
        }
        public void init2018Auto() {
            DcMotor[] right = {this.mainMotors[0], this.mainMotors[2]};
            DcMotor[] left = {this.mainMotors[1], this.mainMotors[3]};
            DcMotor[] arm = {this.armMotors[0], this.armMotors[1]};
            DcMotor[] latch = {latchMotor,};
            DcMotor[] intake = {collector,};
            Commands rightFwd = new Commands(right, 0.7, Commands.Direction.REVERSE);
            Commands rightRvrs = new Commands(right, 0.7, Commands.Direction.FORWARD);
            Commands leftFwd = new Commands(left, 0.7, Commands.Direction.REVERSE);
            Commands Arm = new Commands(arm,1, Commands.Direction.FORWARD);
            Commands Latch = new Commands(latch,1,Commands.Direction.FORWARD);
            Commands Intake = new Commands(intake,1,Commands.Direction.FORWARD);
            Commands[] move_I = {leftFwd, rightFwd};
            Commands[] turn_I = {rightRvrs, leftFwd};
            Commands[] arm_I = {Arm, };
            Commands[] Latch_I = {Latch, };
            Commands[] Intake_I = {Intake,};
            this.move = move_I;
            this.turn = turn_I;
            this.armMove = arm_I;
            this.latchMove = Latch_I;
            this.intakeMove = Intake_I;
        }
    }

