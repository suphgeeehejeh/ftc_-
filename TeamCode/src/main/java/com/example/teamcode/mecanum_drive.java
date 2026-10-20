package com.example.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Robot-centric mecanum drive.
 *
 * The left stick is read in POLAR form: the direction (angle) and the
 * magnitude (how far the stick is pushed) are handled separately, so a
 * "small diagonal" and a "large diagonal" keep the exact same heading and
 * differ only in speed.
 *
 * gamepad1 left stick   : translation, any angle, any magnitude
 * gamepad1 right stick x: rotation
 * gamepad1 right bumper : booster (tap)
 * gamepad1 X            : kicker (1st tap = start charging, 2nd tap = kick)
 *
 * Booster cycle: READY -> BOOST -> PENALTY -> COOLDOWN -> READY
 *   BOOST    : faster than normal for BOOST_TIME
 *   PENALTY  : slower than normal for PENALTY_TIME
 *   COOLDOWN : normal speed for COOLDOWN_TIME, booster still locked
 *
 * Kicker cycle: IDLE -> CHARGING -> KICK -> RETURN -> COOLDOWN -> IDLE
 *   CHARGING : kick power climbs from KICK_MIN_POWER to 1.0 over KICK_CHARGE_TIME
 *   KICK     : swing up at the charged power until KICK_ANGLE_DEG
 *   RETURN   : let the leg stop, then lower it slowly at KICK_RETURN_DEG_S
 *   COOLDOWN : X ignored for KICK_COOLDOWN so the motor is not hammered
 * The leg must hang at rest when INIT is pressed: that is angle 0.
 *
 * The robot stops by itself MATCH_TIME seconds after START is pressed.
 *
 * Configure names must match exactly: "motor1" .. "motor4", and "kicker"
 * on the Expansion Hub.
 */
@TeleOp(name = "mecanum_drive")
public class mecanum_drive extends LinearOpMode {

    static final double DEADZONE   = 0.06; // stick slop to ignore
    static final double EXPO       = 2.0;  // 1.0 = linear, higher = softer centre
    static final double TURN_SCALE = 0.80; // rotation is usually too twitchy at 1.0
    static final double MATCH_TIME = 120.0; // seconds after START before auto-stop

    // Booster (right bumper). Scales multiply speed_degree.
    static final double BOOST_SCALE   = 1.8; // 0.5 -> 0.9
    static final double PENALTY_SCALE = 0.5; // 0.5 -> 0.25
    static final double BOOST_TIME    = 3.0; // seconds of boost
    static final double PENALTY_TIME  = 2.0; // seconds forced slow after boost
    static final double COOLDOWN_TIME = 5.0; // seconds at normal speed before boost is ready again

    enum BoostState { READY, BOOST, PENALTY, COOLDOWN }

    // Kicker leg (X). goBILDA 5203, 1150 rpm: 145.1 encoder ticks per output rev.
    static final double KICK_TICKS_PER_DEG   = 145.1 / 360.0;
    static final double KICK_ANGLE_DEG       = 90.0; // cut power once the leg has swung this far
    static final double KICK_MIN_POWER       = 0.4;  // power if kicked right after charging starts
    static final double KICK_CHARGE_TIME     = 1.5;  // seconds of charging to reach full power
    static final double KICK_MAX_TIME        = 0.5;  // give up the swing if the angle isn't reached
    static final double KICK_RETURN_DEG_S    = 60.0; // lowering speed, degrees per second
    static final double KICK_RETURN_MAX_TIME = 4.0;  // give up lowering after this long
    static final double KICK_HOME_DEG        = 5.0;  // this close to 0 counts as back down
    static final double KICK_COOLDOWN        = 1.5;  // seconds after landing before X works again

    enum KickState { IDLE, CHARGING, KICK, RETURN, COOLDOWN }

    /**
     * @param ux,uy unit vector of the travel direction (length 1, or 0 if idle)
     * @param mag   how fast to travel in that direction, 0..1
     * @param rx    rotation command, -1..1 (positive = clockwise)
     */
    public void drive(DcMotor l_f, DcMotor r_f, DcMotor l_b, DcMotor r_b,
                      double ux, double uy, double mag, double rx,double speed_degree) {

        // Wheel pattern for the pure direction. Same signs as straight/right.
        double lfT = uy + ux;
        double rfT = uy - ux;
        double lbT = uy - ux;
        double rbT = uy + ux;

        // Rescale so the LARGEST wheel command equals mag. Full stick therefore
        // reaches full power in every direction, not just along the axes.
        double tMax = Math.max(Math.abs(lfT), Math.abs(rfT));
        if (tMax > 1e-9) {
            double k = mag / tMax;
            lfT *= k;
            rfT *= k;
            lbT *= k;
            rbT *= k;
        }

        // Add rotation. Same signs as r_r.
        double lfP = lfT + rx;
        double rfP = rfT - rx;
        double lbP = lbT + rx;
        double rbP = rbT - rx;

        // Translation + rotation together can exceed 1.0; keep the ratios.
        double max = Math.max(1.0, Math.max(Math.abs(lfP), Math.max(Math.abs(rfP),
                     Math.max(Math.abs(lbP), Math.abs(rbP)))));

        l_f.setPower(lfP / max*speed_degree);
        r_f.setPower(rfP / max*speed_degree);
        l_b.setPower(lbP / max*speed_degree);
        r_b.setPower(rbP / max*speed_degree);
    }

    /** Deadzone with no step at the edge, followed by an expo response curve. */
    private double shape(double v) {
        double sign = Math.signum(v);
        double a = Math.abs(v);
        if (a < DEADZONE) return 0.0;
        a = (a - DEADZONE) / (1.0 - DEADZONE); // re-stretch to 0..1, no jump
        a = Math.pow(a, EXPO);
        return sign * a;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor r_f = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor r_b = hardwareMap.get(DcMotor.class, "motor2");
        DcMotor l_b = hardwareMap.get(DcMotor.class, "motor3");
        DcMotor l_f = hardwareMap.get(DcMotor.class, "motor4");

        l_f.setDirection(DcMotor.Direction.REVERSE);
        l_b.setDirection(DcMotor.Direction.REVERSE);

        l_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotorEx kicker = hardwareMap.get(DcMotorEx.class, "kicker");
        kicker.setDirection(DcMotor.Direction.FORWARD); // flip if the leg swings the wrong way
        kicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // limits overshoot
        kicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        // resting leg = 0 deg
        kicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speed_degree=0.5;

        telemetry.addLine("Ready. Left stick = move, right stick = turn.");
        telemetry.update();

        waitForStart();

        // Starts counting the moment START is pressed.
        ElapsedTime matchTimer = new ElapsedTime();

        BoostState boostState = BoostState.READY;
        ElapsedTime boostTimer = new ElapsedTime(); // reset on every state change

        KickState kickState = KickState.IDLE;
        ElapsedTime kickTimer = new ElapsedTime(); // reset on every state change
        double kickPower = 0.0;
        boolean kickLowering = false; // RETURN: true once the leg is being lowered

        while (opModeIsActive() && matchTimer.seconds() < MATCH_TIME) {
            double sx = gamepad1.left_stick_x;
            double sy = -gamepad1.left_stick_y; // stick is negative when pushed up




            
            // Polar split: direction is taken from the raw stick and never
            // touched, only the magnitude goes through deadzone + curve.
            double r = Math.min(1.0, Math.hypot(sx, sy));
            double mag = shape(r);

            double ux = 0.0, uy = 0.0;
            if (r > 1e-9) {
                ux = sx / r;
                uy = sy / r;
            }

            double rx = shape(gamepad1.right_stick_x) * TURN_SCALE;

            // Read every loop so a press during the lockout is not saved up
            // and fired the instant the booster becomes READY.
            boolean boostPressed = gamepad1.rightBumperWasPressed();

            double t = boostTimer.seconds();
            switch (boostState) {
                case READY:
                    if (boostPressed) {
                        boostState = BoostState.BOOST;
                        boostTimer.reset();
                    }
                    break;
                case BOOST:
                    if (t >= BOOST_TIME) {
                        boostState = BoostState.PENALTY;
                        boostTimer.reset();
                    }
                    break;
                case PENALTY:
                    if (t >= PENALTY_TIME) {
                        boostState = BoostState.COOLDOWN;
                        boostTimer.reset();
                    }
                    break;
                case COOLDOWN:
                    if (t >= COOLDOWN_TIME) {
                        boostState = BoostState.READY;
                    }
                    break;
            }

            double boostScale = 1.0;
            if (boostState == BoostState.BOOST)   boostScale = BOOST_SCALE;
            if (boostState == BoostState.PENALTY) boostScale = PENALTY_SCALE;

            // Seconds left in the forced-slow phase, and until R1 works again.
            t = boostTimer.seconds();
            double penaltyLeft = 0.0;
            double cooldownLeft = 0.0;
            switch (boostState) {
                case BOOST:
                    penaltyLeft  = PENALTY_TIME;
                    cooldownLeft = (BOOST_TIME - t) + PENALTY_TIME + COOLDOWN_TIME;
                    break;
                case PENALTY:
                    penaltyLeft  = PENALTY_TIME - t;
                    cooldownLeft = (PENALTY_TIME - t) + COOLDOWN_TIME;
                    break;
                case COOLDOWN:
                    cooldownLeft = COOLDOWN_TIME - t;
                    break;
                default:
                    break;
            }

            drive(l_f, r_f, l_b, r_b, ux, uy, mag, rx, speed_degree * boostScale);

            // Kicker. Read every loop, same reason as the booster button.
            boolean kickPressed = gamepad1.xWasPressed();
            double legDeg = kicker.getCurrentPosition() / KICK_TICKS_PER_DEG;
            double kt = kickTimer.seconds();

            switch (kickState) {
                case IDLE:
                    if (kickPressed) {
                        kickState = KickState.CHARGING;
                        kickTimer.reset();
                    }
                    break;
                case CHARGING:
                    kickPower = KICK_MIN_POWER
                            + (1.0 - KICK_MIN_POWER) * Math.min(1.0, kt / KICK_CHARGE_TIME);
                    if (kickPressed) {
                        kicker.setPower(kickPower);
                        kickState = KickState.KICK;
                        kickTimer.reset();
                    }
                    break;
                case KICK:
                    if (legDeg >= KICK_ANGLE_DEG || kt >= KICK_MAX_TIME) {
                        kicker.setPower(0); // still RUN_WITHOUT_ENCODER, so BRAKE just slows it
                        kickLowering = false;
                        kickState = KickState.RETURN;
                        kickTimer.reset();
                    }
                    break;
                case RETURN:
                    if (legDeg <= KICK_HOME_DEG || kt >= KICK_RETURN_MAX_TIME) {
                        kicker.setPower(0);
                        kicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        kickState = KickState.COOLDOWN;
                        kickTimer.reset();
                    } else if (!kickLowering && kicker.getVelocity() <= 0) {
                        // Wait until the leg stops rising on its own, so the gearbox
                        // is never slammed into reverse at full speed.
                        kicker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        kickLowering = true;
                    }
                    if (kickLowering && kickState == KickState.RETURN) {
                        kicker.setVelocity(-KICK_RETURN_DEG_S * KICK_TICKS_PER_DEG);
                    }
                    break;
                case COOLDOWN:
                    if (kt >= KICK_COOLDOWN) {
                        kickState = KickState.IDLE;
                    }
                    break;
            }

            kt = kickTimer.seconds();
            double kickCooldownLeft = 0.0;
            if (kickState == KickState.COOLDOWN) kickCooldownLeft = KICK_COOLDOWN - kt;

            telemetry.addData("time left", "%.1f s", MATCH_TIME - matchTimer.seconds());
            telemetry.addData("booster", "%s  (speed x%.2f)", boostState, boostScale);
            telemetry.addData("boost left", "%.1f s",
                    boostState == BoostState.BOOST ? BOOST_TIME - t : 0.0);
            telemetry.addData("penalty left", "%.1f s", penaltyLeft);
            telemetry.addData("booster cooldown", "%.1f s", cooldownLeft);
            telemetry.addData("kicker", "%s", kickState);
            telemetry.addData("kick power", "%.0f %%",
                    kickState == KickState.IDLE ? 0.0 : kickPower * 100);
            telemetry.addData("leg angle", "%.0f deg", legDeg);
            telemetry.addData("kick cooldown", "%.1f s", kickCooldownLeft);
            telemetry.addData("heading", "%.0f deg", Math.toDegrees(Math.atan2(uy, ux)));
            telemetry.addData("stick / mag", "%.2f -> %.2f", r, mag);
            telemetry.addData("turn", "%.2f", rx);
            telemetry.addData("powers", "lf %.2f  rf %.2f  lb %.2f  rb %.2f",
                    l_f.getPower(), r_f.getPower(), l_b.getPower(), r_b.getPower());
            telemetry.update();
        }

        l_f.setPower(0);
        r_f.setPower(0);
        l_b.setPower(0);
        r_b.setPower(0);
        kicker.setPower(0);

        telemetry.addLine("Time up. Robot stopped.");
        telemetry.update();
    }
}
