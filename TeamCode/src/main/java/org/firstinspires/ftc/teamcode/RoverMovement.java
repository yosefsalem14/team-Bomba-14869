package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Arrays;
import android.util.Log;
/*
    ///////main TeleOP class/////
    this will be used for the main robot
    this is almost done, just fix the comments!
 */

@TeleOp(name="rover movement13",group="movement")

public class RoverMovement extends LinearOpMode {
    Robot rover = new Robot();
    private double move = 0.0;
    private double turn = 0.0;
    @Override
    public void runOpMode(){//throws InterruptedException {
        rover.init(hardwareMap);
        //define controller input variables
            boolean latchOpen =      false;
            boolean cubeIntakesOpen= false;
            boolean support =        false;
            double armMove =         0.0;
            double leftTrigger =     0.0;
            double com2DpadLeft =    0.0;
            double com2DpadRight =   0.0;
            double rightTrigger =    0.0;
            double com2LeftBumper =  0.0;
            double com2RightBumper = 0.0;
            double collect =         0.0;
            double latchOn = 0.0;
            double latchMove = 0.0;
        boolean XPressed;
        boolean YPressed;
        boolean APressed;
        boolean onceSupport = true;
        boolean onceLatches = true;
        boolean onceIntakes = true;
        telemetry.addData("status","ready for start!");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {

            //get all the variables:
                //parse the Dpad clicks
                    latchOn = Int(gamepad2.dpad_up);
                //parse the bumper clicks
                    com2LeftBumper =  Int(gamepad2.left_bumper);
                    com2RightBumper = Int(gamepad2.right_bumper);
                    com2DpadRight = Int(gamepad2.dpad_right);
                    com2DpadLeft = Int(gamepad2.dpad_left);
                    XPressed = gamepad2.x;
                    YPressed = gamepad2.y;
                    APressed = gamepad2.a;
                //parse the trigger pushes
                    leftTrigger =   Math.pow(gamepad1.left_trigger,3);
                    rightTrigger =  Math.pow(gamepad1.right_trigger,3);


                //calculate the turn & strafe & move factor

                    this.move =     (Math.pow(gamepad1.left_stick_y,3))*rover.movePower;
                    this.turn =     (rightTrigger-leftTrigger)*rover.turnPower;
                    double armY = -gamepad2.left_stick_y;
                //calculate the arm & stetcher & collector move factor
                    double armPower = rover.armPower;
                    if(armY >= 0){
                        armPower = rover.armPower;
                    }else if(armY < 0){
                        armPower = 0.2;
                    }
                    armMove = (armY*armPower) - (latchOn);
                    collect = com2RightBumper - com2LeftBumper;
                //open and close the latches
                boolean[] latches=servoSwitch(XPressed,latchOpen,onceLatches);
                boolean[] intakes = servoSwitch(YPressed,cubeIntakesOpen,onceIntakes);
                boolean[] Support = servoSwitch(APressed,support,onceSupport);
                latchOpen =latches[0];
                onceLatches = latches[1];
                cubeIntakesOpen =intakes[0];
                onceIntakes = intakes[1];
                support = Support[0];
                onceSupport = Support[1];
            /*
             * MAIN MOTOR MOVEMENT
             *
             */
            //calculate the specific motor power
            double leftBack =   move - turn;
            double leftFront =  move + turn;
            double rightBack =  (move - (turn));
            double rightFront = (move + (turn));
            double[] powers = {leftBack, leftFront, rightBack, rightFront,};
            normalizeInputs(powers);

            //do the movements:
            if(armMove==0) {
                rover.mainMotors[0].setPower(powers[0]);
                rover.mainMotors[1].setPower(powers[1]);
                rover.mainMotors[2].setPower(powers[2]);
                rover.mainMotors[3].setPower(powers[3]);
            }else{
                rover.mainMotors[0].setPower(0);
                rover.mainMotors[1].setPower(0);
                rover.mainMotors[2].setPower(0);
                rover.mainMotors[3].setPower(0);
            }



            /*
              PICKING UP MECHANISM MOVEMENT
             */
            //arm  movement

            for (int i = 0; i < rover.armMotors.length; i++) {
                    rover.armMotors[i].setPower(armMove);
            }

            //collector movement
            rover.collector.setPower(-collect *
                    rover.collectPower);
            for(int i =0;i<rover.latches.length;i++) {
                if (latchOpen) {
                    Log.i("entered","entered latch open");
                    if(i%2 == 0) {
                        rover.latches[i].setPosition(0);
                    }else{
                        rover.latches[i].setPosition(1);

                    }
                } else {
                    if(i%2 == 0) {
                        rover.latches[i].setPosition(1);
                    }else{
                        rover.latches[i].setPosition(0);
                    }
                }
            }
            for(int i =0;i<rover.cubeIntakes.length;i++) {
                if (cubeIntakesOpen) {
                    if(i%2==0){
                        rover.cubeIntakes[i].setPosition((0.0/180.0));
                    }else
                        rover.cubeIntakes[i].setPosition((20.0/180.0));
                } else {
                    if(i%2==0){
                        rover.cubeIntakes[i].setPosition((90.0/180.0));
                    }else
                        rover.cubeIntakes[i].setPosition((120.0/180.0));

                }
            }

            for(int i =0;i<rover.supportServos.length;i++) {
                if (support) {
                    if(i%2==0){
                        rover.supportServos[i].setPosition((180.0/180.0));
                    }else
                        rover.supportServos[i].setPosition((180.0/180.0));

                } else {
                    if(i%2==0){
                        rover.supportServos[i].setPosition((0/180.0));
                    }else
                        rover.supportServos[i].setPosition((0/180.0));
                }
            }
            //latchMotor move
            latchMove = com2DpadRight - com2DpadLeft;
            rover.latchMotor.setPower(latchMove * rover.latchPower);

            //output

            //show all the input values
            telemetry.addData("move",
                    "move= " + formatString(move));

            telemetry.addData("turn",
                    "turn value= " +
                           formatString(turn));
            telemetry.addData("arm",
                    "arm value= " +
                            formatString(armMove));

            telemetry.addData("collector",
                    "collector value= " +
                            formatString(collect));
            telemetry.addLine();


            telemetry.addData("status",
                    "running");
            telemetry.update();
        }


    }
    public void normalizeInputs(double[] powers){
        double[] clone = powers.clone();
        Arrays.sort(clone);
        if(clone[0]>1) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= Math.abs(clone[0]);
            }
        }
    }
    public boolean[] servoSwitch(boolean in,boolean out,boolean once){

        if(in){
            if(once){
                out = !out;
                once = false;
            }
        }else{
            once = true;
        }
        boolean[] output = {out,once};
        return output;

    }
    public String formatString(double value){
        return (Math.abs(Math.floor(value*100)) + "%");
    }
    public int Int(boolean bool){
        return (bool ?  1:0);
    }
    public int sign(double num){
        if(num>0){
            return 1;
        }else if(num<0){
            return -1;
        }
        return 0;
    }
}


