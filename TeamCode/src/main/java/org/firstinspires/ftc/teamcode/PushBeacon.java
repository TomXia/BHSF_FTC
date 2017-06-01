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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

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
//BLUE
//@Disabled
@Autonomous(name="PushBeaconB", group="AutoDrive")  // @Autonomous(...) is the other common choice
public class PushBeacon extends LinearVisionOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    HardwarePushbot robot = new HardwarePushbot();
    subAuto_right sub = new subAuto_right(robot,this,1.3);
    int times;
    double a;

    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException{
        sub.t=telemetry;
        telemetry.update();
        robot.init(hardwareMap);
        waitForVisionStart();

        telemetry.addData("Status", "Initialized");
        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
        sub.setBeacon(beacon);
        times=0;
        waitForStart();
        runtime.reset();
        telemetry.addData("ultrasonicf:","%f",robot.ulsf.getUltrasonicLevel());
        telemetry.addData("ultrasonicb:","%f",robot.ulsb.getUltrasonicLevel());
        /*
        a = (robot.ulsf.getUltrasonicLevel());
        b = (robot.ulsb.getUltrasonicLevel());
        degree=a-b;
        while(robot.ods.getLightDetected()<0.1 && opModeIsActive()){
            a = (robot.ulsf.getUltrasonicLevel());
            b = (robot.ulsb.getUltrasonicLevel());
            degree=a-b;
            delta=b-19;
            if(delta!=0){

            }
            if (delta > 20) {
                delta = 20;
            }
            else if (delta < -20) {
                delta = -20;
            }
            robot.pushOnebyOne(-0.3+delta/30,-0.3-delta/30);
        }*/
      //  sub.ultrasonicgo(false);
      //  Thread.sleep(10000);
       // sub.ultrasonicgo(false);
        //try {
        //     Thread.sleep(20000);
        //}catch (InterruptedException e) {
        //    e.printStackTrace();
        //}
        sub.pushLight_goShoot();
        sub.shootBall(2);
        sub.pushLight_goLight();
        robot.ultrasonic.setPosition(0.46);
        Thread.sleep(100);
//////////////////////////////////////////////////////////////////////////////////////////
        sub.degree();
        sub.ultrasonicgo(true);
        sub.degreee();
        sub.findTopline(0.1);

        /*do {
            times++;
        'telemetry.addData("times","%d",times);*/
            if (sub.Bea_findBeacon())
                    sub.Bea_pushBeacon();
       /*     else
                break;
        }while(times < 3);*/
/*
        a = (robot.ulsf.getUltrasonicLevel());
        b = (robot.ulsb.getUltrasonicLevel());
        degree=a-b;
        while(Math.abs(degree)!=0 && opModeIsActive()){
            a = (robot.ulsf.getUltrasonicLevel());
            b = (robot.ulsb.getUltrasonicLevel());
            degree=a-b;
            if (degree > 20) {
                degree = 20;
            }
            else if (degree < -20) {
                degree = -20;
            }
            robot.pushOnebyOne(degree/30,-degree/30);
        }、、、、、、、、、、、、、、、、、、、、、、、、、、、、按完第一个灯之后矫正*/

        sub.pushDeg(450,0,0.25,false);

        /*for(double i = 0; i < 0.4; i+=0.001){
            robot.pushGamepad(0,i);
        }
        robot.pushGamepad(0,0.4);
        while(robot.ods.getLightDetected() < 0.05 && opModeIsActive()){
        }
        robot.pushGamepad(0,0);
        \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\已用超声波后退取代*/
        sub.ultrasonicgo(false);
        //sub.degreee();
        sub.degree();
        //sub.findTopline(0.18);

        /*do {
            times++;
            //'telemetry.addData("times","%d",times);*/
            if (sub.Bea_findBeacon())
                sub.Bea_pushBeacon();
        //////////////////////////////////////////////////////////
        /// /////////////////////////////////////////////
        robot.ultrasonic.setPosition(1.0);
        if(sub.destColour==subAuto.BEACON_COLOUR_BLUE){
            sub.pushDeg(500,0,-0.8,false);
            sub.pushDeg(500,1.0,0.0,true);
            sub.pushDeg(4000,0,0.5,false);
        }else{
            sub.pushDeg(170,-1.0,0.0,true);
            sub.pushDeg(12000,0,-0.9,false);
        }
           /* else
                break;
        }while(times < 3);*/
        robot.stop();
        return;

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
    }
}
