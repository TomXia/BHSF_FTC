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
    public int scanresault,nores;
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
                double degree=(a-b)/3;
                double delta=(19-a)/2;
                double deltab=(19-b)/2;
                double x=0, y=0;
                if(Math.abs(delta) <= 0.5 && Math.abs(degree) <= 0.35){
                    x=0.2;
                    y=0.2;
                }
                else{
                    x = 0.2 - delta + degree;
                    y = 0.2 + delta - degree;
                    /*while(Math.abs(delta) > 0.5){
                        a = (robot.ulsf.getUltrasonicLevel());
                        b = (robot.ulsb.getUltrasonicLevel());
                        degree=(a-b)/3;
                        delta=(19-a)/2;
                        if(isForward){
                            if (delta > 0.2) {
                                delta = 0.2;
                            } else if (delta < -0.2) {
                                delta = -0.2;
                            }
                            x = 0.2 - delta;
                            y = 0.2 + delta;
                        }
                        else {
                            if (delta > 0.2) {
                                delta = 0.2;
                            } else if (delta < -0.2) {
                                delta = -0.2;
                            }
                            x = -0.2 + delta;
                            y = -0.2 - delta;
                        }
                    }
                    while(Math.abs(degree) > 0.35){
                        a = (robot.ulsf.getUltrasonicLevel());
                        b = (robot.ulsb.getUltrasonicLevel());
                        degree=(a-b)/3;
                        delta=(19-a)/2;
                        if(isForward){
                            if (degree > 0.2) {
                                degree = 0.2;
                            } else if (delta < -0.2) {
                                degree = -0.2;
                            }
                            x = 0.2 + degree;
                            y = 0.2 - degree;
                        }
                        else {
                            if (degree > 0.2) {
                                degree = 0.2;
                            } else if (degree < -0.2) {
                                degree = -0.2;
                            }
                            x = -0.2 - degree;
                            y = -0.2 + degree;
                        }
                    }*/
                }




                if (a == 0 || a >= 150 || b == 0 || b >= 150) {
                    robot.pushOnebyOne(0, 0);
                } else if (a == 24 || b == 24) {
                    robot.pushOnebyOne(0.2, 0.2);
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
        Boolean Founded = false, isMotorsRunning = false;
        int deg=0;
        scanresault=nores=0;

            while(!Founded && opmode.opModeIsActive()) {
                if (beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().getConfidence() >= 0.80f)
                    scanresault += beacon.getAnalysis().isLeftBlue() ? 1 : -1;
                else
                    nores++;
                if (Math.abs(scanresault) >= 1500/*times*/) {
                    isMotorsRunning = false;
                    robot.pushGamepad(0, 0);
                    Founded = true;
                    analysis = scanresault > 0;
                    break;
                }
                if (nores > 1500/*times*/ && !isMotorsRunning) {
                    isMotorsRunning = true;
                    robot.pushGamepad(0, -0.35);
                }
                if (Math.abs(robot.l2.getCurrentPosition()) >= 300 ) {
                    isMotorsRunning = false;
                    robot.pushGamepad(0,0);
                    break;
                }
//                t.addData("res","%d",scanresault);
//                t.addData("nores","%d",nores);
//                t.update();
            }
            deg += robot.l2.getCurrentPosition();
            robot.resetMotors();

        while(!Founded && opmode.opModeIsActive()) {
            if (beacon.getAnalysis().isLeftKnown() && beacon.getAnalysis().getConfidence() >= 0.80f)
                scanresault += beacon.getAnalysis().isLeftBlue() ? 1 : -1;
            else
                nores++;
           if (Math.abs(scanresault) >= 1500) {
                isMotorsRunning = false;
                robot.pushGamepad(0, 0);
                Founded = true;
                analysis = scanresault > 0;
                break;
            }
            if (nores > 1500 && !isMotorsRunning) {
                isMotorsRunning = true;
                robot.pushGamepad(0, 0.35);
            }
            if (Math.abs(robot.l2.getCurrentPosition()) >= 700) {
                isMotorsRunning = false;
                robot.pushGamepad(0,0);
                break;
            }
///            t.addData("res","%d",scanresault);
///            t.addData("nores","%d",nores);
///            t.update();
        }
        robot.pushGamepad(0,0);
        deg+=robot.l2.getCurrentPosition();

        if(Math.abs(deg) >= 50) pushDeg(Math.abs(deg),0,(Math.abs(deg)/deg)*(-0.35));
        return  Founded;
    }

    public void Bea_pushBeacon(){
//        t.addData("push","1");
        if(analysis == dest){
            robot.pushLight.setPosition(1.0);
            try {
                Thread.sleep(500);
            }catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.pushLight.setPosition(0);
            try {
                Thread.sleep(500);
            }catch (InterruptedException e) {
                e.printStackTrace();
            }
//            t.addData("push","2");
        }
        else{
            pushDeg(600,0,0.4);
            robot.pushLight.setPosition(1.0);
            try {
                Thread.sleep(500);
            }catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.pushLight.setPosition(0);
            try {
                Thread.sleep(500);
            }catch (InterruptedException e) {
                e.printStackTrace();
            }
            pushDeg(600,0,-0.4);
//            t.addData("push","3");
        }
    }

}
