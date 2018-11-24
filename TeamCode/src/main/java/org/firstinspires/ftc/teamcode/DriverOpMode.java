/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**    waseem
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="Basic: Driver", group="Linear Opmode")

public class DriverOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fDrive_left = null;
    private DcMotor fDrive_right = null;
    private DcMotor bDrive_left = null;
    private DcMotor bDrive_right = null;
    private DcMotor leftSuck = null;
    private DcMotor rightSuck = null;
    private DcMotor Tolift = null;
    private DcMotor safra = null;
    private Servo ServoSafra = null;
    private Servo handServo = null;
    private Servo handBallServo = null;
    private Servo moveBallServo = null;
    private static double power = 1.0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fDrive_left = hardwareMap.get(DcMotor.class, "lfmotor");
        fDrive_right = hardwareMap.get(DcMotor.class, "rfmotor");
        bDrive_left = hardwareMap.get(DcMotor.class, "lbmotor");
        bDrive_right = hardwareMap.get(DcMotor.class, "rbmotor");
//        leftSuck = hardwareMap.servo.get("leftSuck");
//        rightSuck = hardwareMap.servo.get("rightSuck");
        leftSuck = hardwareMap.get(DcMotor.class, "frontmotorleft");
        rightSuck = hardwareMap.get(DcMotor.class, "frontmotorright");
        Tolift = hardwareMap.get(DcMotor.class, "Tolift");
        safra = hardwareMap.get(DcMotor.class, "Safra");
        ServoSafra = hardwareMap.get(Servo.class, "ServoSafra");
        handServo = hardwareMap.get(Servo.class, "handServo");
        moveBallServo = hardwareMap.get(Servo.class, "moveBallServo");
        handBallServo = hardwareMap.get(Servo.class, "handBallServo");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fDrive_left.setDirection(DcMotor.Direction.REVERSE);
        fDrive_right.setDirection(DcMotor.Direction.FORWARD);
        bDrive_left.setDirection(DcMotor.Direction.REVERSE);
        bDrive_right.setDirection(DcMotor.Direction.FORWARD);

        fDrive_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fDrive_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bDrive_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bDrive_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        ServoSafra.setPosition(0.4);
        handServo.setPosition(0);
        handBallServo.setPosition(0.12);
        moveBallServo.setPosition(0.7);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            ///////////////////////////////////////////
            if (gamepad1.right_stick_y < -0.7)  /// Move Forward - Backward
            {
               MoveForward();
            } else if (gamepad1.right_stick_y > 0.7) {
                MoveBackwards();
            } else if (gamepad1.right_stick_x < -0.7)  /// Move Right - Left
            {
                MoveLeft();
            } else if (gamepad1.right_stick_x > 0.7) {
                MoveRight();
            } else if (gamepad1.left_stick_x < -0.7)  /// Move Right - Left
            {
                TurnLeft();
            } else if (gamepad1.left_stick_x > 0.7) {
                TurnRight();
            } else {
                FullStop();
            }


            //******************Move Diagonal **************************

            if (gamepad1.right_bumper) {
                fDrive_left.setDirection(DcMotorSimple.Direction.FORWARD);
                bDrive_right.setDirection(DcMotorSimple.Direction.REVERSE);
                fDrive_left.setPower(power);
                bDrive_right.setPower(power);
                fDrive_left.setPower(0);
                bDrive_right.setPower(0);
            } else if (gamepad1.right_trigger > 0.0) {
                fDrive_right.setDirection(DcMotorSimple.Direction.FORWARD);
                bDrive_left.setDirection(DcMotorSimple.Direction.REVERSE);
                fDrive_right.setPower(power);
                bDrive_left.setPower(power);
                fDrive_right.setPower(0);
                bDrive_left.setPower(0);
            }
            if (gamepad1.left_bumper) {
                fDrive_left.setDirection(DcMotorSimple.Direction.REVERSE);
                bDrive_right.setDirection(DcMotorSimple.Direction.FORWARD);
                fDrive_left.setPower(power);
                bDrive_right.setPower(power);
                fDrive_left.setPower(0);
                bDrive_right.setPower(0);
            } else if (gamepad1.left_trigger > 0.0) {
                fDrive_right.setDirection(DcMotorSimple.Direction.REVERSE);
                bDrive_left.setDirection(DcMotorSimple.Direction.FORWARD);
                fDrive_right.setPower(power);
                bDrive_left.setPower(power);
                fDrive_right.setPower(0);
                bDrive_left.setPower(0);
            }


            //****************************************

            if (gamepad2.right_bumper) {
                rightSuck.setDirection(DcMotor.Direction.REVERSE);
                rightSuck.setPower(power);
                rightSuck.setPower(0);
            } else if (gamepad2.right_trigger > 0.0) {
                rightSuck.setDirection(DcMotor.Direction.FORWARD);
                rightSuck.setPower(power);
                rightSuck.setPower(0);
            }
            if (gamepad2.left_bumper) {
                leftSuck.setDirection(DcMotor.Direction.FORWARD);
                leftSuck.setPower(power);
                leftSuck.setPower(0);
            } else if (gamepad2.left_trigger > 0.0) {
                leftSuck.setDirection(DcMotor.Direction.REVERSE);
                leftSuck.setPower(power);
                leftSuck.setPower(0);
            }
            // right stick for both engines
            if (gamepad2.right_stick_y < -0.7)  /// FRONT BOX HANDLING
            {
                leftSuck.setDirection(DcMotor.Direction.FORWARD);
                rightSuck.setDirection(DcMotor.Direction.REVERSE);
                leftSuck.setPower(power);
                rightSuck.setPower(power);
            } else if (gamepad2.right_stick_y > 0.7) {
                leftSuck.setDirection(DcMotor.Direction.REVERSE);
                rightSuck.setDirection(DcMotor.Direction.FORWARD);
                leftSuck.setPower(power);
                rightSuck.setPower(power);
            }else {
                leftSuck.setPower(0);
                rightSuck.setPower(0);
            }


            ////////////

            if (gamepad2.left_stick_y > 0.7) {

                Tolift.setDirection(DcMotor.Direction.FORWARD);
                Tolift.setPower(0.6);
            } else if (gamepad2.left_stick_y < -0.7) {
                Tolift.setDirection(DcMotor.Direction.REVERSE);
                Tolift.setPower(0.2);
            } else {
                Tolift.setPower(0);
            }

            ///////


            //////////

            if (gamepad1.y) {

                ServoSafra.setPosition(0.4);

            } else {
                ServoSafra.setPosition(0.8);
            }
            ///////


            // Show the elapsed game time and wheel power.
            telemetry.addData("fdrive_right",fDrive_right.getCurrentPosition());
            telemetry.addData("fdrive_left",fDrive_left.getCurrentPosition());
            telemetry.addData("bdrive_right",bDrive_right.getCurrentPosition());
            telemetry.addData("bdrive_left",bDrive_left.getCurrentPosition());
            telemetry.addData("Position", safra.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("gamepad1 left stick y", gamepad1.left_stick_y);
            telemetry.update();
        }
    }

    public void MoveForward(){

        fDrive_right.setDirection(DcMotor.Direction.FORWARD);
        fDrive_left.setDirection(DcMotor.Direction.REVERSE);
        bDrive_right.setDirection(DcMotor.Direction.FORWARD);
        bDrive_left.setDirection(DcMotor.Direction.REVERSE);
        fDrive_right.setPower(power);
        fDrive_left.setPower(power);
        bDrive_right.setPower(power);
        bDrive_left.setPower(power);
    }
    public void MoveBackwards(){
        fDrive_right.setDirection(DcMotor.Direction.REVERSE);
        fDrive_left.setDirection(DcMotor.Direction.FORWARD);
        bDrive_right.setDirection(DcMotor.Direction.REVERSE);
        bDrive_left.setDirection(DcMotor.Direction.FORWARD);
        fDrive_right.setPower(power);
        fDrive_left.setPower(power);
        bDrive_right.setPower(power);
        bDrive_left.setPower(power);
    }
    public void MoveLeft(){
        fDrive_right.setDirection(DcMotor.Direction.FORWARD);
        fDrive_left.setDirection(DcMotor.Direction.FORWARD);
        bDrive_right.setDirection(DcMotor.Direction.REVERSE);
        bDrive_left.setDirection(DcMotor.Direction.REVERSE);
        fDrive_right.setPower(power);
        fDrive_left.setPower(power);
        bDrive_right.setPower(power);
        bDrive_left.setPower(power);
    }
    public void MoveRight(){
        fDrive_right.setDirection(DcMotor.Direction.REVERSE);
        fDrive_left.setDirection(DcMotor.Direction.REVERSE);
        bDrive_right.setDirection(DcMotor.Direction.FORWARD);
        bDrive_left.setDirection(DcMotor.Direction.FORWARD);
        fDrive_right.setPower(power);
        fDrive_left.setPower(power);
        bDrive_right.setPower(power);
        bDrive_left.setPower(power);
    }
    public void TurnLeft(){
        fDrive_right.setDirection(DcMotor.Direction.FORWARD);
        fDrive_left.setDirection(DcMotor.Direction.FORWARD);
        bDrive_right.setDirection(DcMotor.Direction.FORWARD);
        bDrive_left.setDirection(DcMotor.Direction.FORWARD);
        fDrive_right.setPower(power);
        fDrive_left.setPower(power);
        bDrive_right.setPower(power);
        bDrive_left.setPower(power);
    }
    public void TurnRight(){
        fDrive_right.setDirection(DcMotor.Direction.REVERSE); //turn right if ball is blue
        fDrive_left.setDirection(DcMotor.Direction.REVERSE);
        bDrive_right.setDirection(DcMotor.Direction.REVERSE);
        bDrive_left.setDirection(DcMotor.Direction.REVERSE);
        fDrive_right.setPower(power);
        fDrive_left.setPower(power);
        bDrive_right.setPower(power);
        bDrive_left.setPower(power);
    }
    public void FullStop(){
        fDrive_right.setPower(0);
        fDrive_left.setPower(0);
        bDrive_right.setPower(0);
        bDrive_left.setPower(0);
    }
}