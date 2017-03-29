package org.firstinspires.ftc.teamcode;

import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.BeaconExtension;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by Administrator on 2017/3/14.
 */

public class subAuto {
    final static int isBlue = 0 ;
    public Telemetry t;
    Boolean dest,analysis=true;
    HardwarePushbot robot;
    private shoot_servo reloader;
    public LinearVisionOpMode opmode;
    public double qSpeed;
    public BeaconExtension beacon = null;
    public  subAuto(HardwarePushbot r, LinearVisionOpMode op, double q)
    {
        this.robot = r;
        opmode = op;
        reloader = new shoot_servo(robot);
        qSpeed = q;
        dest = isBlue==0;
    }
    public void shootBall(int times){
        for(int i=0;i<times;++i)
        {
            robot.miniGun.setPower(0.58);
            while (opmode.opModeIsActive() && (!robot.TCH.isPressed())) {
            }
            while (opmode.opModeIsActive() && robot.TCH.isPressed()) {
            }
            while (opmode.opModeIsActive() && (!robot.TCH.isPressed())) {
            }
            robot.miniGun.setPower(0);
            if (i==0 && !reloader.isAlive())            reloader.start();
            try {
                Thread.sleep((i==0 ? 1:0) * 500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
    public void setBeacon(BeaconExtension b){
        beacon = b;
    }

    public void ultrasonicgo(Boolean isForward)
    {
//            while (opmode.opModeIsActive() && robot.ods.getLightDetected() < 0.2) {
                double a = (robot.ulsf.getUltrasonicLevel());
                double b = (robot.ulsb.getUltrasonicLevel());
                double degree=a-b;
                double delta=degree/6;
                double x, y;


                if(isForward){
                    if (delta > 0.2) {
                        delta = 0.2;
                    } else if (delta < -0.2) {
                        delta = -0.2;
                    }
                    x = 0.1 + delta;
                    y = 0.1 - delta;
                }
                else {
                    if (delta > 0.2) {
                        delta = 0.2;
                    } else if (delta < -0.2) {
                        delta = -0.2;
                    }
                    x = -0.1 - delta;
                    y = -0.1 + delta;
                }


                if (a == 0 || a >= 150 || b == 0 || b >= 150) {
                    robot.pushOnebyOne(0, 0);
                } else if (a == 24 || b == 24) {
                    robot.pushOnebyOne(0.1, 0.1);
                } else {
                    robot.pushOnebyOne(x, y);
                }
        t.addData("delta:","%f",delta);
 //           }

    }
    public void test()
    {
        CaptureResult r;
        CaptureRequest q;
    }
    public void pushDeg(int deg,double x,double y){
        robot.resetMotors();
        robot.pushGamepad(x,y);
        while (opmode == null || opmode.opModeIsActive()) {
            if(Math.abs(robot.l2.getCurrentPosition()) >= deg) break;
        }
        robot.pushGamepad(0,0);
    }

    public boolean Bea_findBeacon(){
//        t.addData("find","1");
        robot.resetMotors();
        Boolean Founded = false;
        int deg=0;
        if(beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().getConfidence()>=0.80f) {
            Founded=true;
        }
        else{
            robot.pushGamepad(0, -0.3);
            while (opmode.opModeIsActive() && (!(beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().getConfidence() >= 0.80f))) {
                if (Math.abs(robot.l2.getCurrentPosition()) >= 150) {
                    Founded = true;
                    break;
                }
            }
            robot.pushGamepad(0, 0);
            deg += robot.l2.getCurrentPosition();
            robot.resetMotors();
            if (Founded) {
                robot.pushGamepad(0, 0.3);
                while (opmode.opModeIsActive() && (!(beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().getConfidence() >= 0.80f))) {
//                    t.addData("find2x", "%d", robot.l2.getCurrentPosition());
                    if (Math.abs(robot.l2.getCurrentPosition()) >= 550) {
                        Founded = true;
                        break;
                    }
                }
                robot.pushGamepad(0, 0);
            }
            deg += robot.l2.getCurrentPosition();
        }
        if(Founded) {
            analysis=beacon.getAnalysis().isLeftBlue();
        }
        if(Math.abs(deg) >= 50) pushDeg(Math.abs(deg),0,(Math.abs(deg)/deg)*(-0.35));

        return  Founded;
    }
    public void Bea_pushBeacon(){
//        t.addData("push","1");
        if(analysis == dest){
            robot.pushLight.setPosition(1.0);
            try {
                Thread.sleep(1000);
            }catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.pushLight.setPosition(0);
            try {
                Thread.sleep(1000);
            }catch (InterruptedException e) {
                e.printStackTrace();
            }
//            t.addData("push","2");
        }
        else{
            pushDeg(700,0,0.35);
            robot.pushLight.setPosition(1.0);
            try {
                Thread.sleep(1000);
            }catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.pushLight.setPosition(0);
            try {
                Thread.sleep(1000);
            }catch (InterruptedException e) {
                e.printStackTrace();
            }
            pushDeg(700,0,-0.35);
//            t.addData("push","3");
        }
    }

}
