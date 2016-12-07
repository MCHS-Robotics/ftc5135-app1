/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp v1.2", group="TeleOp")  // @Autonomous(...) is the other common choice
//@Disabled
public class TeleOp1 extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor left = null;
    private DcMotor right = null;

    private DcMotor shootA = null;
    private DcMotor shootB = null;

    private Servo bacon = null;
    private double midPos = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        left  = hardwareMap.dcMotor.get("L");
        right = hardwareMap.dcMotor.get("R");

        shootA = hardwareMap.dcMotor.get("A");
        shootB = hardwareMap.dcMotor.get("B");

        bacon = hardwareMap.servo.get("bcn");
        midPos = 0.5; //bacon.getPosition();

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        shootA.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        shootB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        //tank drive
        left.setPower(-gamepad1.left_stick_y);
        right.setPower(gamepad1.right_stick_y);

        //shooter--press buttons for speed multiplier
        if(gamepad2.dpad_up){
            if(gamepad2.a){
                shootA.setPower(0.625);
                shootB.setPower(0.625);
            }
            else if(gamepad2.b){
                shootA.setPower(0.75);
                shootB.setPower(0.75);
            }
            else if(gamepad2.x){
                shootA.setPower(0.875);
                shootB.setPower(0.875);
            }
            else if(gamepad2.y){
                shootA.setPower(1);
                shootB.setPower(1);
            }
            else{
                shootA.setPower(0.5);
                shootB.setPower(0.5);
            }
        }
        else if(gamepad2.dpad_down){
            if(gamepad2.a){
                shootA.setPower(-0.625);
                shootB.setPower(-0.625);
            }
            else if(gamepad2.b){
                shootA.setPower(-0.75);
                shootB.setPower(-0.75);
            }
            else if(gamepad2.x){
                shootA.setPower(-0.875);
                shootB.setPower(-0.875);
            }
            else if(gamepad2.y){
                shootA.setPower(-1);
                shootB.setPower(-1);
            }
            else{
                shootA.setPower(-0.5);
                shootB.setPower(-0.5);
            }
        }
        else{
            shootA.setPower(0);
            shootB.setPower(0);
        }

        //beacon manipulator--R/L triggers
        if(gamepad2.right_trigger > 0){
            bacon.setPosition(Range.scale(gamepad2.right_trigger, 0, 1, midPos, 1));
        }
        else if(gamepad2.left_trigger > 0){
            bacon.setPosition(Range.scale(gamepad2.left_trigger, 0, 1, midPos, 0));
        }
        else{
            bacon.setPosition(midPos);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

}
