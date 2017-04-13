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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoDrive", group="Auto")  // @Autonomous(...) is the other common choice
//@Disabled
public class AutoDrive extends LinearVisionOpMode  {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private HardwarePushbot robot = new HardwarePushbot();
   /* private shootThread shoot;
    private shoot_servo reloader;
    private int distance_First_goShoot=4100;
    private int distance_pushBall_go=3800;
    final static boolean isReturn = false;
    final static int destColour = 0 ;*/
    subAuto_tan sub = new subAuto_tan(robot,this,1.0);
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        robot.gyro.calibrate();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        /*STEP1: go and shoot *//*
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pushGamepad(0,-0.5);
        while (opModeIsActive()) {
            telemetry.addData("Status L1", "Run Time: " + runtime.toString());
            if(Math.abs(robot.l2.getCurrentPosition())>=distance_First_goShoot) break;
            telemetry.update();
        }
        robot.pushGamepad(0,0);
//////////////////////////////////////////////////////////////////////////////////////////////////////
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pushGamepad(0.5,0);
//        robot.TargetPosition(1850);
        while (opModeIsActive()) {
            telemetry.addData("Status L1", "Run Time: " + runtime.toString());
            if(robot.l1.getCurrentPosition()<=-1860) break;
            telemetry.update();
        }
        robot.pushGamepad(0,0);
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pushGamepad(0,-0.5 + destColour*1.0);
        while (opModeIsActive()) {
            telemetry.addData("Status L1", "Run Time: " + runtime.toString());
            if(Math.abs(robot.l2.getCurrentPosition())>=distance_First_goShoot/2+500 + (destColour-1)*200) break;
            telemetry.update();
        }
        robot.pushGamepad(0,0);
/////////////////////////////////////////////////////////////////////////////////////////////////////
        robot.isLoaded=true;
        while (opModeIsActive()) {
            telemetry.addData("Status L2", "Run Time: " + runtime.toString());

            robot.miniGun.setPower(0.58);
            while (!robot.TCH.isPressed()) {
            }
            while (robot.TCH.isPressed()) {
            }
            while (!robot.TCH.isPressed()) {
            }
            robot.miniGun.setPower(0);
            while (true) {
                if (((reloader == null || !reloader.isAlive()))) {
                    reloader = new shoot_servo(robot);
                    reloader.start();
                    break;
                }
            }
            try {
                Thread.sleep(1500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            robot.miniGun.setPower(0.58);
            while (!robot.TCH.isPressed()) {
            }
            while (robot.TCH.isPressed()) {
            }
            while (!robot.TCH.isPressed()) {
            }
            robot.miniGun.setPower(0);
            break;
        }
        if(!isReturn) {
////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
            robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pushGamepad(0, 0.9);
            while (opModeIsActive()) {
                telemetry.addData("Status T2", "Run Time: %d " + runtime.toString(), robot.l1.getCurrentPosition());
                if (robot.l1.getCurrentPosition() >= Math.abs((destColour-1)*5000)) {
                    robot.pushGamepad(0, 0);
                    break;
                }
                telemetry.update();
            }

            robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pushGamepad(-0.5, 0);
            while (opModeIsActive()) {
                telemetry.addData("Status T2", "Run Time: %d " + runtime.toString(), robot.l1.getCurrentPosition());
                if (robot.l1.getCurrentPosition() >= 600 + destColour*1675 ) {
                    robot.pushGamepad(0, 0);
                    break;
                }
                telemetry.update();
            }

//////////////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////
            robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pushGamepad(0, -0.8);
            while (opModeIsActive()) {
                telemetry.addData("Status L1", "Run Time: " + runtime.toString());
                if (Math.abs(robot.l1.getCurrentPosition()) >= Math.abs(distance_pushBall_go/3+400)) break;

            }
            robot.pushGamepad(0, 0);
        }
        else
        {//////////////////////////////////////////////////////////////////////////////////////////////////////
            robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pushGamepad(-0.5,0);
//        robot.TargetPosition(1850);3975
            while (opModeIsActive()) {
                telemetry.addData("Status L1", "Run Time: " + runtime.toString());
                if(robot.l1.getCurrentPosition()>=1050) break;
                telemetry.update();
            }
            robot.pushGamepad(0,0);
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.pushGamepad(0,0.5);
            robot.TargetPosition(-distance_First_goShoot);
            while (opModeIsActive()) {
                telemetry.addData("Status L1", "Run Time: " + runtime.toString());
                if(robot.l2.getCurrentPosition()<=-distance_First_goShoot) break;
                telemetry.update();
            }
            robot.pushGamepad(0,0);

        }

        telemetry.addData("Info: ","Finished");

        robot.stop();
    }
}
        /*
        if(!isReturn) {
////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
            robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pushGamepad(-0.9, 0);
            while (opModeIsActive()) {
                telemetry.addData("Status T2", "Run Time: %d " + runtime.toString(), robot.l1.getCurrentPosition());
                if (robot.l1.getCurrentPosition() >= 1950 /*900) {
                    robot.pushGamepad(0, 0);
                    break;
                }
                telemetry.update();
            }
            robot.pushGamepad(0,0);
//////////////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////
            robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pushGamepad(0, -0.8);
            while (opModeIsActive()) {
                telemetry.addData("Status L1", "Run Time: " + runtime.toString());
                if (robot.l2.getCurrentPosition() >= distance_pushBall_go) break;
            }
            robot.pushGamepad(0, 0);
        }
        else
        {//////////////////////////////////////////////////////////////////////////////////////////////////////
            robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pushGamepad(-0.9,0);
//        robot.TargetPosition(1850);3550
            while (opModeIsActive()) {
                telemetry.addData("Status L1", "Run Time: " + runtime.toString());
                if(robot.l1.getCurrentPosition()>=1050) break;
                telemetry.update();
            }
            robot.pushGamepad(0,0);
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.pushGamepad(0,0.5);
            robot.TargetPosition(-distance_First_goShoot);
            while (opModeIsActive()) {
                telemetry.addData("Status L1", "Run Time: " + runtime.toString());
                if(robot.l2.getCurrentPosition()<=-distance_First_goShoot) break;
                telemetry.update();
            }
            robot.pushGamepad(0,0);
        }*/
        sub.goShoot();
        sub.pushBall();
        telemetry.addData("Info: ","Finished");
        robot.stop();
    }
}
