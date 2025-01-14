package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous(name = "roadrunnertest", group = "Autonomous")
public class roadrunnertest extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightExtend = null;
    private DcMotor rightTilt = null;
    private DcMotor leftExtend = null;
    private DcMotor leftTilt = null;
    private CRServo Intake;
    public void runOpMode() throws InterruptedException {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front left");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back right");
        //rightTilt = hardwareMap.get(DcMotor.class, "righttilt");
        //leftTilt = hardwareMap.get(DcMotor.class, "lefttilt");
        //leftExtend = hardwareMap.get(DcMotor.class, "leftextend");
        //rightExtend = hardwareMap.get(DcMotor.class, "rightextend");
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        rightLift rightLift = new rightLift(hardwareMap);
        leftLift leftLift = new leftLift(hardwareMap);
        intake intake = new intake(hardwareMap);


        //###########################_NOTE:_#############################
        //     Below is where the programming for the autonomous       //
        //     should be set up: in the Action.runBlocking() function. //
        //     everything else is for set up purposes only             //
        //###############################################################



        Actions.runBlocking(
                new SequentialAction(
                        leftLift.tilt(32.3)
                )
        );







    }
    public class rightLift{
        private DcMotor RightLift;
        private DcMotor RightTilt;
        public rightLift(HardwareMap hardwareMap){
            RightLift = hardwareMap.get(DcMotor.class, "rightextend");
            RightTilt = hardwareMap.get(DcMotor.class, "righttilt");
            RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RightLift.setDirection(DcMotorSimple.Direction.FORWARD);
            RightTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RightTilt.setDirection(DcMotorSimple.Direction.FORWARD);



        }
    }
    public class leftLift{
        private DcMotorEx LeftLift;
        private DcMotorEx LeftTilt;
        private double CurrentTilt = 90.0;
        public leftLift(HardwareMap hardwareMap){
            LeftLift = hardwareMap.get(DcMotorEx.class, "leftextend");
            LeftTilt = hardwareMap.get(DcMotorEx.class, "lefttilt");
            LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LeftLift.setDirection(DcMotorSimple.Direction.FORWARD);
            LeftTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LeftTilt.setDirection(DcMotorSimple.Direction.FORWARD);
            LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftTilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public Action extend(double inches){
            //1.483
            //537.7
            double targetPos = (1.483*Math.PI)*537.7;

            LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            class Extend implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    LeftLift.setTargetPosition((int) targetPos);
                    if (!(LeftLift.isBusy() && opModeIsActive())) {
                        return false;
                    }else{
                        LeftLift.setPower(0);
                        return true;
                    }
                }
            }
            return new Extend();
        }
        public Action tilt(double theta){
            //1.483
            //537.7
            double temp = theta-CurrentTilt;
            final double target = (temp*537.7)/(28*360);
            LeftTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftTilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            class Tilt implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    LeftTilt.setTargetPosition((int) target);
                    if (!(LeftTilt.isBusy() && opModeIsActive())) {
                        return false;
                    }else{
                        LeftTilt.setPower(0);
                        CurrentTilt = theta;
                        return true;
                    }
                }
            }
                return new Tilt();
        }
        //public void tilt(double theta) {
        //    //worm gear 28:1
        //    double temp = theta-CurrentTilt;
        //    temp = (temp*537.7)/(28*360);
        //    LeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //    LeftLift.setTargetPosition((int) temp);
        //    if(!(LeftLift.isBusy() && opModeIsActive())) {
        //        LeftLift.setPower(0);//Loop body can be empty
        //    }
        //}
    }
    public class intake{
        private CRServo Intake;
        public intake(HardwareMap hardwareMap){
            Intake = hardwareMap.get(CRServo.class, "intake");
        }
        public void in(){
            Intake.setPower(-1);
        }
        public void out(){
            Intake.setPower(1);
        }
        public void stop(){
            Intake.setPower(0);
        }

    }

}
