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
    final static int BEACON_COLOUR_BLUE = 0;
    final static int BEACON_COLOUR_RED = 1;
    final static int destColour = BEACON_COLOUR_BLUE ;
    public Telemetry t;
    Boolean dest,analysis=true;
    HardwarePushbot robot;
    private shoot_servo reloader;
    public LinearVisionOpMode opmode;
    public double qSpeed;
    public BeaconExtension beacon = null;
    public int scanresault,nores;
    public int distance_pushLight_goShootGo = 4500;
    public int distance_pushLight_goShootTurn = 1100;
    public int distance_pushLight_goLightGo = 3300;
    public int distance_pushLight_goLightTurn = 2755;

    public int RED_distance_pushLight_goShootGo = 3500;
    public int RED_distance_pushLight_goShootTurn = 800;
    public int RED_distance_pushLight_goLightGo = 6000;
    public int RED_distance_pushLight_goLightTurn = 800;

    public  subAuto(HardwarePushbot r, LinearVisionOpMode op, double q)
    {
        this.robot = r;
        opmode = op;
        reloader = new shoot_servo(robot);
        qSpeed = q;
        dest = destColour==0;
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
        double a = (robot.ulsf.getUltrasonicLevel());
        double b = (robot.ulsb.getUltrasonicLevel());
        double degree=a-b;
        double delta=(19-a)/4;
        double x=0, y=0;
        double k;
        if(isForward) {
            while (opmode.opModeIsActive() && robot.ods.getLightDetected() < 0.1) {
                a = (robot.ulsf.getUltrasonicLevel());
                b = (robot.ulsb.getUltrasonicLevel());
                delta = 19 - a;
                if (Math.abs(delta) > 20) {
                    if (delta > 0)
                        delta = 20;
                    else
                        delta = -20;
                }
                k = Math.sqrt(100 - (Math.abs(delta) - 10) * (Math.abs(delta) - 10)) / 50;
                if (isForward) {
                    if (k > 0.2) {
                        k = 0.2;
                    } else if (k < -0.2) {
                        k = -0.2;
                    }
                    if (delta > 0) {
                        x = 0.3 - k;
                        y = 0.3 + k;
                    } else {
                        x = 0.3 + k;
                        y = 0.3 - k;
                    }

                }


                if (a == 0 || a >= 150 || b == 0 || b >= 150) {
                    robot.pushOnebyOne(0, 0);
                } else if (a == 24 || b == 24) {
                    robot.pushOnebyOne(0.2, 0.2);
                } else {
                    robot.pushOnebyOne(x, y);

                }
            }
            a = (robot.ulsf.getUltrasonicLevel());
            b = (robot.ulsb.getUltrasonicLevel());
            degree = a - b;
            while (Math.abs(degree) != 0 && opmode.opModeIsActive()) {
                a = (robot.ulsf.getUltrasonicLevel());
                b = (robot.ulsb.getUltrasonicLevel());
                degree = a - b;
                if (degree > 20) {
                    degree = 20;
                } else if (degree < -20) {
                    degree = -20;
                }
                robot.pushOnebyOne(degree / 30, -degree / 30);
            }
        }
            else{
                a = (robot.ulsf.getUltrasonicLevel());
            b = (robot.ulsb.getUltrasonicLevel());
            degree=a-b;
            while(robot.ods.getLightDetected()<0.1 && opmode.opModeIsActive()){
                a = (robot.ulsf.getUltrasonicLevel());
                b = (robot.ulsb.getUltrasonicLevel());
                degree=a-b;
                if (degree > 20) {
                    degree = 20;
                }
                else if (degree < -20) {
                    degree = -20;
                }
                robot.pushOnebyOne(-0.3+degree/30,-0.3-degree/30);
            }
            }
        }
    public void test()
    {
        robot.pushGamepad(0,-0.5);
        robot.resetMotors();
        int k=0;
        while(opmode.opModeIsActive()) {
            t.addData("deg","%d",robot.l2.getCurrentPosition() - k);
            k=robot.l2.getCurrentPosition();
        }
    }
    public void pushDeg(int deg,double x,double y,boolean isStop){
        robot.resetMotors();
        double k = y==0 ? 0:Math.abs(y)/y;
        double ix;
        while (opmode == null || opmode.opModeIsActive()) {
            ix=Math.abs((double)robot.l2.getCurrentPosition())/(double)deg;
            robot.pushGamepad(x,k*Math.min(-4.5*ix*ix+4.5*ix+0.25,Math.abs(y)));
            if(Math.abs(robot.l2.getCurrentPosition()) >= deg) break;
        }
        if(isStop) robot.pushGamepad(0,0);
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
                if (Math.abs(scanresault) >= 1000/*times*/) {
                    isMotorsRunning = false;
                    robot.pushGamepad(0, 0);
                    Founded = true;
                    analysis = scanresault > 0;
                    break;
                }
                if (nores > 1200/*times*/ && !isMotorsRunning) {
                    isMotorsRunning = true;
                    robot.pushGamepad(0, -0.25);
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
           if (Math.abs(scanresault) >= 1000) {
                isMotorsRunning = false;
                robot.pushGamepad(0, 0);
                Founded = true;
                analysis = scanresault > 0;
                break;
            }
            if (nores > 1200 && !isMotorsRunning) {
                isMotorsRunning = true;
                robot.pushGamepad(0, 0.25);
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

        if(Math.abs(deg) >= 50) pushDeg(Math.abs(deg),0,(Math.abs(deg)/deg)*(-0.35),true);
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
            pushDeg(450,0,0.2,true);
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
            pushDeg(450,0,-0.2,true);
//            t.addData("push","3");
        }
    }

    public void findTopline(double s){
        pushDeg(150,0,-0.25,false);
        robot.pushGamepad(0,s);
        while(robot.ods.getLightDetected() < 0.05 && opmode.opModeIsActive()){
        }
        robot.pushGamepad(0,0);
    }

    public void pushLight_goShoot(){
        pushDeg(destColour==BEACON_COLOUR_BLUE ? distance_pushLight_goShootGo : RED_distance_pushLight_goShootGo,0,(destColour==BEACON_COLOUR_BLUE ? 1 : -1)*(qSpeed)*(-0.9),true);
        pushDeg(destColour==BEACON_COLOUR_BLUE ? distance_pushLight_goShootTurn : RED_distance_pushLight_goShootTurn ,(destColour==BEACON_COLOUR_BLUE ? 1 : -1)*0.4,0,true);
    }

    public void pushLight_goLight(){
        pushDeg(destColour==BEACON_COLOUR_BLUE ? distance_pushLight_goLightGo : RED_distance_pushLight_goLightGo,0,(destColour==BEACON_COLOUR_BLUE ? 1 : -1)*(-0.7)*qSpeed,true);
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(destColour == BEACON_COLOUR_BLUE)
            robot.pushOnebyOne(0,0.4);
        else
            robot.pushGamepad(0.4,0);

        while (opmode == null || opmode.opModeIsActive()) {
            if(Math.abs(robot.r1.getCurrentPosition()) >= (destColour==BEACON_COLOUR_BLUE ? distance_pushLight_goLightTurn : RED_distance_pushLight_goLightTurn) ) break;
        }
        if(destColour == BEACON_COLOUR_RED) pushDeg(1200,0,0.6,true);
    }

}
