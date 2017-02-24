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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoShoot 1.1.0", group="Autonomous")  // @Autonomous(...) is the other common choice
//@Disabled
public class Auto_Shoot extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor left = null;
    private DcMotor right = null;

    private DcMotor shootA = null;
    private DcMotor shootB = null;

    private DcMotor ballArm = null;
    private DcMotor ballLift = null;

    private CRServo bacon = null;

    //ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;
    AnalogInput linSens;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        left  = hardwareMap.dcMotor.get("L");
        right = hardwareMap.dcMotor.get("R");

        shootA = hardwareMap.dcMotor.get("A");
        shootB = hardwareMap.dcMotor.get("B");

        ballArm = hardwareMap.dcMotor.get("arm");
        ballLift = hardwareMap.dcMotor.get("lift");

        bacon = hardwareMap.crservo.get("bcn");
        bacon.setPower(0);

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        //sensorRGB = hardwareMap.colorSensor.get("color");
        linSens = hardwareMap.analogInput.get("line");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        shootA.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        shootB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        ballLift.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        runtime.reset();
        double sTime = runtime.milliseconds();

        while (opModeIsActive()) {
            while(runtime.milliseconds() <= 10000) {
                if(runtime.milliseconds() <= sTime + 1500){
                    shootA.setPower(-0.5);
                    shootB.setPower(0.5);
                }
                else
                    ballLift.setPower(0.4);
            }
            shootA.setPower(0);
            shootB.setPower(0);
            ballLift.setPower(0);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
