package org.firstinspires.ftc.teamcode.teleop.transport;

import static org.firstinspires.ftc.teamcode.teleop.AllianceStorage.isRed;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

public class Transport {
    private Toggle automode, clawLeft, clawRight;
    private DcMotorEx slidesMotor;

    private DcMotorEx armMotor;
    private ServoImplEx rightIntake;
    private ServoImplEx leftIntake;
    private ServoImplEx intakeRotation;
    private TouchSensor clawSensor;
    private TouchSensor zeroLimit;
    private PIDController slidesController;

    public static double slidesp = .005, slidesi = 0, slidesd = 0.00038;
    public static int slidesTarget = 0;
    private PIDController armController;

    public static double armp = 0.007, armi = 0, armd = 0.0004;
    public static double armf = 0.06;
    public static int armTarget = 0;

    private final double arm_ticks_in_degrees = 1425.1 / 1512;

    public int mode = 0;

    //Safe, Intaking Ground, Intaking Med, Intaking Top, Parallels (.5, 1st, 1.5, 2nd, 2.5, 3rd, 3.5), Hang
    public static final int[] armPositions = {0, 3100, 2975, 2900, 450, 500, 600, 775, 775, 850, 900, 1500};

    //Safe, Extended, Mid-Way
    public static final int[] slidesPositions = {0, 1500, 3000, 1500, 1750, 2700, 3000};
    //Safe, Deploy, Intaking, Intaking Off-Ground, Parallels (.5, 1st, 1.5, 2 - 3.5), Hang
    public static final double[] intakeRotPositions = {0, .9, .45, .42, .95, 1, .6};

    //Idx
    public static int leftClawPos = 0;
    public static int rightClawPos = 0;


    //Closed, Inter, Open
    public static final double[] clawPositions = {0, .55, 1};

    public boolean armInRange;

    public boolean slidesInRange;
    public boolean slidesAtZero;

    //Neutral, In-taking, Out-taking
    public enum TPos {
        //Reset:
        RESET("RESET", armPositions[0], intakeRotPositions[0], slidesPositions[0]),

        //Deploy:
        DEPLOY("DEPLOY", armPositions[1], intakeRotPositions[2], slidesPositions[0]),

        //Intaking Positions:
        INTAKING_MED_GROUND("INTAKING_CLOSE_GROUND", armPositions[1], intakeRotPositions[2], slidesPositions[1]),
        INTAKING_FAR_GROUND("INTAKING_FAR_GROUND", armPositions[1], intakeRotPositions[2], slidesPositions[6]),
        INTAKING_CLOSE_MEDSTACK("INTAKING_CLOSE_MEDSTACK", armPositions[2], intakeRotPositions[6], slidesPositions[0]),
        //TODO: TUNE ^^^
        INTAKING_FAR_MEDSTACK("INTAKING_FAR_MEDSTACK", armPositions[2], intakeRotPositions[6], slidesPositions[1]),
        //TODO: TUNE ^^^
        INTAKING_CLOSE_TOPSTACK("INTAKING_CLOSE_TOPSTACK", armPositions[3], intakeRotPositions[6], slidesPositions[0]),
        INTAKING_FAR_TOPSTACK("INTAKING_FAR_TOPSTACK", armPositions[3], intakeRotPositions[6], slidesPositions[1]),
        //TODO: TUNE ^^^
        //Outtaking Positions:
        OUTTAKING_1("OUTTAKING_1", armPositions[4], intakeRotPositions[4], slidesPositions[1]),

        OUTTAKING_2("OUTTAKING_2", armPositions[5], intakeRotPositions[4], slidesPositions[1]),

        OUTTAKING_3("OUTTAKING_3", armPositions[6], intakeRotPositions[4], slidesPositions[1]),

        OUTTAKING_4("OUTTAKING_4", armPositions[7], intakeRotPositions[5], slidesPositions[1]),
        OUTTAKING_5("OUTTAKING_5", armPositions[7], intakeRotPositions[5], slidesPositions[5]),
        OUTTAKING_6("OUTTAKING_6", armPositions[9], intakeRotPositions[5], slidesPositions[5]),
        OUTTAKING_7("OUTTAKING_7", armPositions[9], intakeRotPositions[5], slidesPositions[2]),

        HANG("HANG", armPositions[11], intakeRotPositions[6], slidesPositions[2]);

        private final String debug;
        private final int armPosition;
        private final double intakeRotPosition;

        private final int slidesPosition;

        TPos(String debug, int armPosition, double intakeRotPosition, int slidesPosition) {
            this.debug = debug;
            this.armPosition = armPosition;
            this.intakeRotPosition = intakeRotPosition;
            this.slidesPosition = slidesPosition;
        }

        public String toString() {
            return debug;
        }

        public int armPos() {
            return armPosition;
        }

        public double intakeRotPos() {
            return intakeRotPosition;
        }

        public int slidesPos() {
            return slidesPosition;
        }
    }

    public TPos transportPos = TPos.RESET;

    public Transport(HardwareMap hardwareMap) {
        automode = new Toggle(false);
        clawLeft = new Toggle(false);
        clawRight = new Toggle(false);
        clawSensor = hardwareMap.get(TouchSensor.class, "clawSensor");
        zeroLimit = hardwareMap.get(TouchSensor.class, "zeroLimit");

        slidesController = new PIDController(slidesp, slidesi, slidesd);
        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesController.setPID(slidesp, slidesi, slidesd);
        slidesController = new PIDController(armp, armi, armd);
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armController = new PIDController(armp, armi, armd);
        armController.setPID(armp, armi, armd);

        intakeRotation = hardwareMap.get(ServoImplEx.class, "intakeRotation");
        rightIntake = hardwareMap.get(ServoImplEx.class, "rightIntake");
        leftIntake = hardwareMap.get(ServoImplEx.class, "leftIntake");

        intakeRotation.setDirection(ServoImplEx.Direction.FORWARD);
        rightIntake.setDirection(ServoImplEx.Direction.FORWARD);
        leftIntake.setDirection(ServoImplEx.Direction.REVERSE);

        intakeRotation.setPosition(.05);
        rightIntake.setPosition(0);
        leftIntake.setPosition(0);
    }


    public void setTPos() {
        armInRange = Math.abs(transportPos.armPos() - armMotor.getCurrentPosition()) < 50;
        slidesInRange = Math.abs(transportPos.slidesPos() - slidesMotor.getCurrentPosition()) < 15;
        slidesAtZero = slidesMotor.getCurrentPosition() < 50;
        if (!slidesAtZero && !armInRange) {
            slidesTarget = 0;
        }
        if (slidesAtZero && !armInRange) {
            armTarget = transportPos.armPos();
        }
        if (!slidesInRange && armInRange) {
            slidesTarget = transportPos.slidesPos();
        }

        if (mode == 1 && slidesAtZero && armInRange) {
            leftIntake.setPosition(clawPositions[2] - .12);
            rightIntake.setPosition(clawPositions[2]);
        } else if (mode == 0 && transportPos.debug != "AUTO_DEPLOY" || slidesAtZero && transportPos.debug != "AUTO_DEPLOY") {
            leftIntake.setPosition(clawPositions[0] - .12);
            rightIntake.setPosition(clawPositions[0]);
        } else {
            leftIntake.setPosition(clawPositions[leftClawPos] - .12);
            rightIntake.setPosition(clawPositions[rightClawPos]);
        }

        if (mode != 2) {
            intakeRotation.setPosition(transportPos.intakeRotPos());
        }
        if (mode == 2 && !armInRange) {
            intakeRotation.setPosition(0.35);
        }
        if (mode == 2 && armInRange) {
            intakeRotation.setPosition(transportPos.intakeRotPos());
        }
    }

    public void update() {
        int armPos = armMotor.getCurrentPosition();
        double armpid = armController.calculate(armPos, armTarget);
        double armff = Math.cos(Math.toRadians(armTarget / arm_ticks_in_degrees)) * armf;
        double armPower = armpid + armff;

        armMotor.setPower(armPower);

        int slidesPos = slidesMotor.getCurrentPosition();
        double slidespid = slidesController.calculate(slidesPos, slidesTarget);
        double slidesPower = slidespid;

        slidesMotor.setPower(slidesPower);
        setTPos();
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        automode.update(gamepad2.back);
        clawLeft.update(gamepad1.right_bumper);
        clawRight.update(gamepad1.left_bumper);

        int armPos = armMotor.getCurrentPosition();
        double armpid = armController.calculate(armPos, armTarget);
        double armff = Math.cos(Math.toRadians(armTarget / arm_ticks_in_degrees)) * armf;
        double armPower = armpid + armff;

        armMotor.setPower(armPower);

        int slidesPos = slidesMotor.getCurrentPosition();
        double slidespid = slidesController.calculate(slidesPos, slidesTarget);
        double slidesPower = slidespid;

        slidesMotor.setPower(slidesPower);

        if (zeroLimit.isPressed()) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (automode.value() == true && clawSensor.isPressed()) {
            fullLeftClaw();
            fullRightClaw();
        }
        if (clawLeft.value() == true) {
            fullLeftClaw();
        } else {
            closeLeftClaw();
        }
        if (clawRight.value() == true) {
            fullRightClaw();
        } else {
            closeRightClaw();
        }
        if (gamepad1.b) {
            reset();
        }
        if (gamepad1.x) {
            closeIntaking();
        }
        if (gamepad1.y) {
            closeTopStack();
        }
        if (gamepad1.a) {
            medIntaking();
        }
        if (gamepad2.a) {
            half();
        }
        if (gamepad2.b) {
            one();
        }
        if (gamepad2.x) {
            oneHalf();
        }
        if (gamepad2.y) {
            two();
        }
        if (gamepad2.dpad_down) {
            twoHalf();
        }
        if (gamepad2.dpad_left) {
            three();
        }
        if (gamepad2.dpad_right) {
            threeHalf();
        }
        if (gamepad2.dpad_up) {
            hang();
        }
        setTPos();
    }
    public void reset() {
        transportPos = TPos.RESET;
        mode = 0;
    }
    //Claw:
    public void closeLeftClaw() { leftClawPos = 0; }
    public void closeRightClaw() { rightClawPos = 0; }
    public void medLeftClaw() { leftClawPos = 1; }
    public void medRightClaw() { rightClawPos = 1; }
    public void fullLeftClaw() {
        leftClawPos = 2;
    }
    public void fullRightClaw() {
        rightClawPos = 2;
    }
    //Intaking:
    public void closeIntaking() {
        transportPos = TPos.DEPLOY;
        mode = 1;
    }
    public void medIntaking() {
        transportPos = TPos.INTAKING_MED_GROUND;
        mode = 1;
    }
    public void farIntaking() {
        transportPos = TPos.INTAKING_FAR_GROUND;
        mode = 1;
    }

    public void closeMedStack() {
        transportPos = TPos.INTAKING_CLOSE_MEDSTACK;
        mode = 1;
    }
    public void closeTopStack() {
        transportPos = TPos.INTAKING_CLOSE_TOPSTACK;
        mode = 1;
    }

    public void farMedStack() {
        transportPos = TPos.INTAKING_FAR_MEDSTACK;
        mode = 1;
    }
    public void farTopStack() {
        transportPos = TPos.INTAKING_FAR_TOPSTACK;
        mode = 1;
    }
    //Outtaking:
    public void half() {
        transportPos = TPos.OUTTAKING_1;
        mode = 2;
    }
    public void one() {
        transportPos = TPos.OUTTAKING_2;
        mode = 2;
    }
    public void oneHalf() {
        transportPos = TPos.OUTTAKING_3;
        mode = 2;
    }
    public void two() {
        transportPos = TPos.OUTTAKING_4;
        mode = 2;
    }
    public void twoHalf() {
        transportPos = TPos.OUTTAKING_5;
        mode = 2;
    }
    public void three() {
        transportPos = TPos.OUTTAKING_6;
        mode = 2;
    }
    public void threeHalf() {
        transportPos = TPos.OUTTAKING_7;
        mode = 2;
    }
    public void hang() {
        transportPos = TPos.HANG;
        mode = 2;
    }
}
