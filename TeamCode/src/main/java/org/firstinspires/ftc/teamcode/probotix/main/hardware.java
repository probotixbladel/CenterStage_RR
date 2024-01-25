package org.firstinspires.ftc.teamcode.probotix.main;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;


public class hardware {
        public DcMotorEx wheelLeftFront, wheelRightFront, wheelLeftRear, wheelRightRear, liftMotor, armMotor;
        public Servo grabServo, dropServo, flipServo;
        public Encoder leftEncoder,rightEncoder,frontEncoder;


        private HardwareMap hardwareMap;
        private Gear gear;
        private Constants constants;






        public hardware(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            init();
        }

        public void init() {
            this.wheelLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("wheelLeftFront");
            this.wheelRightFront = (DcMotorEx) hardwareMap.dcMotor.get("wheelRightFront");
            this.wheelLeftRear = (DcMotorEx) hardwareMap.dcMotor.get("wheelLeftRear");
            this.wheelRightRear = (DcMotorEx) hardwareMap.dcMotor.get("wheelRightRear");
            this.liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("liftMotor");
            this.armMotor = (DcMotorEx) hardwareMap.dcMotor.get("armMotor");
            this.grabServo = (Servo) hardwareMap.servo.get("grabServo");
            this.dropServo = (Servo) hardwareMap.servo.get("dropServo");
            this.flipServo = (Servo) hardwareMap.servo.get("flipServo");
            reset();
        }

        public void reset() {
            wheelLeftFront.setPower(0);
            wheelRightFront.setPower(0);
            wheelLeftRear.setPower(0);
            wheelRightRear.setPower(0);

            liftMotor.setPower(0);
            liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setTargetPosition(0);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            armMotor.setPower(0);
            armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


            dropServo.setPosition(DriveConstants.dropServoInit);
            grabServo.setPosition(DriveConstants.grabServoInit);
            flipServo.setPosition(DriveConstants.flipServoInit); //init stand
            // drive = 0.70
            //open = 0.43
            //closed = 0.50



        }

        public void setGear(Gear gear) {
            this.gear = gear;
        }

        public Gear getGear() {
            return this.gear;
        }

        public void setConstants(Constants constants) {
        this.constants = constants;
    }

        public Constants getConstants() {
        return this.constants;
    }

            //niet gebruiken werkt nie

        public enum Constants {
            liftUp(800),liftDown(250),
            armPickUp(1700),armDeliver(0),
            dropServoInit(0.5),dropServoOpen(0.43),dropServoClose(0.70),
            flipServoInit(0.33),flipServoPickUp(0.75),flipServoDeliver(0.25),
            grabServoInit(0.5),grabServoOpen(0.43),grabServoClose(0.70);


            private double Positions;

            Constants(double positions){this.Positions = positions;}

            public double getPositions() {return Positions;}
        }

        public enum  Gear {
            FIRST(0.25), SECOND(0.5), THIRD(0.75), FOURTH(0.90);
            //FOURTH was 1.0


            private double MaxSpeed;

            Gear(double maxSpeed){
                this.MaxSpeed = maxSpeed;
            }

            public double getMaxSpeed() {
                return MaxSpeed;
            }
        }



        public DcMotorEx getWheelLeftFront() {
            return wheelLeftFront;
        }

        public DcMotorEx getWheelRightFront() {
            return wheelRightFront;
        }

        public DcMotorEx getWheelLeftRear() {
            return wheelLeftRear;
        }

        public DcMotorEx getWheelRightRear() {
            return wheelRightRear;
        }

        public DcMotorEx getLiftMotor() {return liftMotor;}

        public DcMotorEx getArmMotor() {return armMotor;}

        public Servo getGrabServo(){return grabServo;}

        public Servo getDropServo(){return dropServo;}

        public Servo getFlipServo(){return flipServo;}






    }