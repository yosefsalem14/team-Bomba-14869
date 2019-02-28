package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.PID;
import org.firstinspires.ftc.teamcode.utilities.Auto;
import org.firstinspires.ftc.teamcode.utilities.AutoDrivetype;
import org.firstinspires.ftc.teamcode.utilities.Controller;

import java.util.Arrays;
/*
    ///////main TeleOP class/////
    this will be used for the main robot
    this is almost done, just fix the comments!
 */

@TeleOp(name="TELEOP",group="movement")

 public class RoverMovement extends Auto {
    Robot rover = new Robot();
    Controller[] ramper = new Controller[4];
    Controller armRamper = new Controller(new PID(0.0843 ,0,0));
    private double move = 0.0;
    private double turn = 0.0;
    @Override
    public void runOpMode(){//throws InterruptedException {
        initControlled(rover);
        for(int i =0;i<rover.mainMotors.length;i++){
            ramper[i] = new Controller();
            if(rover.mainMotors[i]!=null)
            rover.mainMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //define controller input variables
        boolean latchOpen = false;
        ElapsedTime TT = new ElapsedTime();
        boolean ghostOpen = false;
        boolean stopperOpen = false;
        double leftTrigger = 0.0;
        double com2DpadLeft = 0.0;
        double com2DpadRight = 0.0;
        double rightTrigger = 0.0;
        double com2RightTrigger = 0.0;
        double armMove = 0.0;
        double com2LeftBumper = 0.0;
        double com2RightBumper = 0.0;
        double collect = 0.0;
        boolean[] XPressed={false,false};
        boolean[] YPressed={false,false};
        boolean[] APressed={false,false};
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
                    com2RightTrigger = Math.ceil(gamepad2.right_trigger);
                    armMove = -gamepad2.left_stick_y;
                    XPressed[0] = gamepad2.x;
                    YPressed[0] = gamepad2.y;
                    APressed[0] = gamepad2.a;
                //parse the trigger pushes
                    leftTrigger =   gamepad1.left_trigger;
                    rightTrigger =  gamepad1.right_trigger;


                //calculate the turn & strafe & move factor

                this.move =     -gamepad1.left_stick_y*rover.movePower;
                this.turn =     -(rightTrigger-leftTrigger)*rover.turnPower;
                if(servoSwitch(XPressed)) {
                    latchOpen = !latchOpen;
                }
                if(servoSwitch(YPressed)) {
                    ghostOpen = !ghostOpen;
                }
                if(servoSwitch(APressed)) {
                    stopperOpen = !stopperOpen;
                }
                //open and close the latches

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
                        double currentPower = ramper[i].rampUp(powers[i], turn != 0);
                        if(rover.mainMotors[i]!=null)
                        rover.mainMotors[i].setPower(currentPower);// + 0.5*(gamepad1.right_stick_x)*((i%2)*2 - 1)
                }


            /*
              PICKING UP MECHANISM MOVEMENT
             */
            //arm  movement & latch movement
            goToPos((int)(com2DpadRight - com2DpadLeft), AutoDrivetype.LATCH,15);
            double pow=0;
            for (int i = 0; i < rover.armMotors.length; i++) {
                    if(rover.armMotors[i]!=null)
                        pow = armRamper.useSmoothing(armRamper.rampUp(armMove));

                    rover.armMotors[i].setPower(pow - toInt(gamepad2.dpad_down));
            }
            //collector movement
            collect = com2RightBumper - com2LeftBumper + com2RightTrigger*rover.collectPower;
            if(rover.collector!=null)
            rover.collector.setPower(-collect);
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
                    if (ghostOpen) {
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
                    if (stopperOpen) {
                        rover.supportServos[i].setPosition((0.0 / 180.0));
                    } else {
                        rover.supportServos[i].setPosition((90.0 / 180.0));
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
    public boolean servoSwitch(boolean[] bools){
        if(bools[0]&&!bools[1]){
            bools[1] = bools[0];
            return true;
        }
        bools[1] = bools[0];
        return false;
    }
    public double norm(double num){
//
//        if(num<0.9&&num>0.1){
//            return 0.5;
//        }
//        if(num>-0.9 &&num<-0.1){
//            return -0.5;
//        }
        return num;
    }
    public String formatString(double value){
        return (Math.abs(Math.floor(value*100)) + "%");
    }
    public int toInt(boolean bool){
        return (bool ?  1:0);
    }
}


