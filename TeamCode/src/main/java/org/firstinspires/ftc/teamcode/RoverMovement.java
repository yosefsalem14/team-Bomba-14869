package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.Auto;
import org.firstinspires.ftc.teamcode.utilities.AutoDrivetype;
import org.firstinspires.ftc.teamcode.utilities.Controller;
import org.firstinspires.ftc.teamcode.utilities.Robot;
import java.util.Arrays;
/*
    ///////main TeleOP class/////
    this will be used for the main robot
    this is almost done, just fix the comments!
 */

@TeleOp(name="rover movement13",group="movement")

public class RoverMovement extends Auto {
    Robot rover = new Robot();
    Controller[] ramper = new Controller[4];
    Controller armRamper = new Controller(0.02199,0,0.002156);
    private double move = 0.0;
    private double turn = 0.0;
    @Override
    public void runOpMode(){//throws InterruptedException {
        initControlled(rover);
        for(int i =0;i<rover.mainMotors.length;i++){
            ramper[i] = new Controller(0.9,0,0.02156);
            rover.mainMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //define controller input variables
            boolean latchOpen =      false;
            boolean cubeIntakesOpen= false;
            boolean support =        false;
            double leftTrigger =     0.0;
            double com2DpadLeft =    0.0;
            double com2DpadRight =   0.0;
            double rightTrigger =    0.0;
            double com2RightTrigger = 0.0;
            double com2DpadUp =      0.0;
            double armMove = 0.0;
            double com2LeftBumper =  0.0;
            double com2RightBumper = 0.0;
            double collect =         0.0;
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
                //parse the bumper clicks
                    com2LeftBumper =  toInt(gamepad2.left_bumper);
                    com2RightBumper = toInt(gamepad2.right_bumper);
                    com2DpadRight = toInt(gamepad2.dpad_right);
                    com2DpadLeft = toInt(gamepad2.dpad_left);
                    com2DpadUp = toInt(gamepad2.dpad_up);
                    com2RightTrigger = Math.ceil(gamepad2.right_trigger);
                    armMove = -gamepad2.left_stick_y* rover.armPower;
                    XPressed = gamepad2.x;
                    YPressed = gamepad2.y;
                    APressed = gamepad2.a;
                //parse the trigger pushes
                    leftTrigger =   gamepad1.left_trigger;
                    rightTrigger =  gamepad1.right_trigger;


                //calculate the turn & strafe & move factor

                this.move =     -gamepad1.left_stick_y*rover.movePower;
                this.turn =     -(rightTrigger-leftTrigger)*rover.turnPower;

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
            double leftBack =   (move - turn);
            double leftFront =  (move + turn);
            double rightBack =  (move - turn);
            double rightFront = (move + turn);
            double[] powers = {leftBack, leftFront, rightBack, rightFront};
            normalizeInputs(powers);

            //do the movements:
                for (int i = 0; i < rover.mainMotors.length; i++) {
                        double currentPower = ramper[i].rampUp(powers[i], turn == 0);
                        if(rover.mainMotors[i]!=null)
                        rover.mainMotors[i].setPower(currentPower);
                }


            /*
              PICKING UP MECHANISM MOVEMENT
             */
            //arm  movement & latch movement
            goToPos((int)-com2DpadUp, AutoDrivetype.ARMS,22);
            goToPos((int)(com2DpadLeft - com2DpadRight), AutoDrivetype.LATCH,22);
            for (int i = 0; i < rover.armMotors.length; i++) {
                    if(rover.armMotors[i]!=null)
                    rover.armMotors[i].setPower(armRamper.rampUp(armMove,false));
            }
            //collector movement
            collect = com2RightBumper - com2LeftBumper + com2RightTrigger*rover.collectPower;
            if(rover.collector!=null)
            rover.collector.setPower(-collect *
                    rover.collectPower);
            if(rover.latches[0]!=null&&rover.latches!=null) {
                for (int i = 0; i < rover.latches.length; i++) {
                    if (latchOpen) {
                        rover.latches[0].setPosition(0);
                        rover.latches[1].setPosition(1);
                    } else {
                        rover.latches[0].setPosition(1);
                        rover.latches[1].setPosition(0);
                    }
                }
            }
            if(rover.cubeIntakes[0]!=null&&rover.cubeIntakes!=null) {
                for (int i = 0; i < rover.cubeIntakes.length; i++) {
                    if (cubeIntakesOpen) {
                        rover.cubeIntakes[0].setPosition((90.0 / 180.0));
                        rover.cubeIntakes[1].setPosition((0.0 / 180.0));
                    } else {
                        rover.cubeIntakes[0].setPosition((0.0 / 180.0));
                        rover.cubeIntakes[1].setPosition((90.0 / 180.0));

                    }
                }
            }
            if(rover.supportServos[0]!=null&&rover.supportServos!=null) {
                for (int i = 0; i < rover.supportServos.length; i++) {
                    if (support) {
                        rover.supportServos[i].setPosition((85.0 / 180.0));
                    } else {
                        rover.supportServos[i].setPosition((5 / 180.0));
                    }
                }
            }
            //latchMotor move

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
        for(int i =0;i<clone.length;i++){
            clone[i] = Math.abs(clone[i]);
        }
        Arrays.sort(clone);
        if(Math.abs(clone[2])>=1) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= clone[2];
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
    public int toInt(boolean bool){
        return (bool ?  1:0);
    }
}


