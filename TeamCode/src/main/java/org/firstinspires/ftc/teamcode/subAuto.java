package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.BeaconExtension;

/**
 * Created by Administrator on 2017/3/14.
 */

public class subAuto{
    final static int BEACON_COLOUR_BLUE = 0;
    final static int BEACON_COLOUR_RED = 1;
    final static int destColour = BEACON_COLOUR_BLUE;
    public Telemetry t;
    Boolean dest,analysis=true;
    HardwarePushbot robot;
    private shoot_servo reloader;
    public LinearVisionOpMode opmode;
    public double qSpeed;
    public BeaconExtension beacon = null;
    public int scanresault,nores;
    public final int Q = 1;
    public int distance_pushLight_goShootGo = 4500;
    public int distance_pushLight_goShootTurn = 900;
    public int distance_pushLight_goLightGo = 3700;
    public int distance_pushLight_goLightTurn = 2055;

    public int RED_distance_pushLight_goShootGo = 3500;
    public int RED_distance_pushLight_goShootTurn = 1000;
    public int RED_distance_pushLight_goLightGo = 4900;
    public int RED_distance_pushLight_goLightTurn = 990;



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
                Thread.sleep((i==0 ? 1:0) * 300);
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
        double a;
        double b;
        double degree;
        double delta;
        double x=0, y=0;
        double k;
        if(isForward) {
            while (opmode.opModeIsActive() && robot.ods.getLightDetected() < 0.45) {//
                a = (robot.ulsf.getUltrasonicLevel());
                b = (robot.ulsb.getUltrasonicLevel());
                delta = 17 - a;
                if (Math.abs(delta) > 10) {
                    if (delta > 0)
                        delta = 10;
                    else
                        delta = -10;
                }
                k = Math.sqrt(100 - (Math.abs(delta) - 10) * (Math.abs(delta) - 10)) / 50;
                    if (k > 0.2) {
                        k = 0.2;
                    } else if (k < -0.2) {
                        k = -0.2;
                    }
                    if (delta > 0) {
                        x = 0.3 - k;
                        y = 0.3 + k;
                    } else if(delta==0) {
                        x=0.6;
                        y=0.6;
                    }else {
                        x = 0.3 + k;
                        y = 0.3 - k;
                    }




                if (a == 0 || a >= 150 || b == 0 || b >= 150) {
                    robot.pushOnebyOne(0, 0);
                } else if (a == 24 || b == 24) {
                    robot.pushOnebyOne(0.5, 0.5);
                } else {
                    robot.pushOnebyOne(x, y);

                }
            }
            //degree();
        }
        else{
            while (opmode.opModeIsActive() && robot.ods.getLightDetected() < 0.45) {//
                a = (robot.ulsf.getUltrasonicLevel());
                b = (robot.ulsb.getUltrasonicLevel());
                delta = 18 - b;
                if (Math.abs(delta) > 10) {
                    if (delta > 0)
                        delta = 10;
                    else
                        delta = -10;
                }
                k = Math.sqrt(100 - (Math.abs(delta) - 10) * (Math.abs(delta) - 10)) / 50;

                    if (k > 0.2) {
                        k = 0.2;
                    } else if (k < -0.2) {
                        k = -0.2;
                    }
                    if (delta > 0) {
                        x = -0.3 + k;
                        y = -0.3 - k;
                    } else if(delta==0) {
                        x=-0.6;
                        y=-0.6;
                    }else {
                        x = -0.3 - k;
                        y = -0.3 + k;
                    }



                if (a == 0 || a >= 150 || b == 0 || b >= 150) {
                    robot.pushOnebyOne(0, 0);
                } else if (a == 24 || b == 24) {
                    robot.pushOnebyOne(-0.5, -0.5);
                } else {
                    robot.pushOnebyOne(x, y);

                }
            }
            //degree();
            }
            /*
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
                if(degree==0){
                    robot.pushOnebyOne(-0.6,-0.6);
                }else{
                    robot.pushOnebyOne(-0.3+degree/30,-0.3-degree/30);
                }

            }*/
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
        deg/=Q;
        double k = y==0 ? 0:Math.abs(y)/y;
        double ix;
        robot.pushGamepad(x,0.1*k);
        while (opmode == null || opmode.opModeIsActive()) {
            ix=Math.abs((double)robot.l2.getCurrentPosition())/(double)deg;
            if(ix>1) ix=1;
            robot.pushGamepad(x,k*Math.min(-4.5*ix*ix+4.5*ix+0.09,Math.abs(y)));
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
            for(int i=0;i<2;++i) {
                robot.pushLight.setPosition(1.0);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.pushLight.setPosition(0);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
//            t.addData("push","2");
        }
        else{
            pushDeg(550,0,0.2,true);
            for(int i=0;i<2;++i) {
                robot.pushLight.setPosition(1.0);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.pushLight.setPosition(0);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            //pushDeg(450,0,-0.2,true);
//            t.addData("push","3");
        }
    }

    public void findTopline(double s){
        robot.resetMotors();
        robot.pushGamepad(0,-0.18);
        while(opmode == null || opmode.opModeIsActive()){
            if(Math.abs(robot.l2.getCurrentPosition()) >= 150) break;
        }
        robot.pushGamepad(0,s);
        while(robot.ods.getLightDetected() < 0.45 && opmode.opModeIsActive()){
        }
        robot.pushGamepad(0,0);
    }

    public void pushLight_goShoot(){
        pushDeg(destColour==BEACON_COLOUR_BLUE ? distance_pushLight_goShootGo : RED_distance_pushLight_goShootGo,0,(destColour==BEACON_COLOUR_BLUE ? 1 : -1)*(qSpeed)*(-0.7),true);
        try{
            Thread.sleep(60);
        }catch (InterruptedException e){
            e.printStackTrace();
        }
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
        //if(destColour == BEACON_COLOUR_RED) pushDeg(1200,0,0.6,true);
    }
    public void degreee() {
        double a = (robot.ulsf.getUltrasonicLevel());
        double b = (robot.ulsb.getUltrasonicLevel());
        double degree = a - b + 1;
        while (Math.abs(degree) > 1 && opmode.opModeIsActive()) {
            a = (robot.ulsf.getUltrasonicLevel());
            b = (robot.ulsb.getUltrasonicLevel());
            degree = a - b + 1;
            if (degree > 10) {
                degree = 10;
            } else if (degree < -10) {
                degree = -10;
            } else {

            }
            if (Math.abs(degree * degree * 0.025) < 0.1) {
                break;
            }
            //robot.pushOnebyOne(degree/30,-degree/30);
            if (degree > 0) {
                robot.pushOnebyOne(degree * degree * 0.0025, degree * degree * (-0.0025));
            } else {
                robot.pushOnebyOne(degree * degree * (-0.0025), degree * degree * 0.0025);
            }
        }
    }
    public void degree(){
        double a = (robot.ulsf.getUltrasonicLevel());
        double b = (robot.ulsb.getUltrasonicLevel());
        double degree=a-b+1;
        while(Math.abs(degree) !=0 && opmode.opModeIsActive()){
            a = (robot.ulsf.getUltrasonicLevel());
            b = (robot.ulsb.getUltrasonicLevel());
            degree=a-b+1;
            if (degree > 10) {
                degree = 10;
            }
            else if (degree < -10) {
                degree = -10;
            }else{

            }
            if(Math.abs(degree*degree*0.025)<0.1){
                break;
            }
            //robot.pushOnebyOne(degree/30,-degree/30);
            if(degree>0){
                robot.pushOnebyOne(degree*degree*0.0025,degree*degree*(-0.0025));
            }
            else{
                robot.pushOnebyOne(degree*degree*(-0.0025),degree*degree*0.0025);
            }
        }
        robot.pushOnebyOne(0,0);
    }


}
