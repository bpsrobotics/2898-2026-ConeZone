package frc.robot.subsystems

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.utils.DashboardMeasure
import frc.robot.utils.InvertibleDigitalInput
import frc.robot.utils.SparkWrapper
import frc.robot.utils.asMeters
import frc.robot.utils.controls.ElevatorFFSendable
import frc.robot.utils.controls.ElevatorFeedForwardConstants
import frc.robot.utils.controls.LiveGains
import frc.robot.utils.controls.PIDConstants
import frc.robot.utils.convert
import frc.robot.utils.inches
import frc.robot.utils.meters
import frc.robot.utils.metersPerSecond
import frc.robot.utils.metersPerSecondSquared
import frc.robot.utils.sec
import frc.robot.utils.volts

object Elevator : SubsystemBase() {

    object Constants {
        const val LEADER_MOTOR_ID: Int = 13
        const val FOLLOWER_MOTOR_ID: Int = 14

        // const val ENCODER_LA_ID: Int = 2
        // const val ENCODER_LB_ID: Int = 3

        var MIN_HEIGHT: Distance by DashboardMeasure(Meters, 12.inches, "Elevator")
        var MAX_HEIGHT: Distance by DashboardMeasure(Meters, 62.inches, "Elevator")
        var MAX_VELOCITY: LinearVelocity by
            DashboardMeasure(MetersPerSecond, 3.metersPerSecond, "Elevator")
        var MAX_ACCELERATION: LinearAcceleration by
            DashboardMeasure(MetersPerSecondPerSecond, 2.metersPerSecondSquared, "Elevator")

        /** Encoder pulses per centimeter */
        // const val ENCODER_PPC = 100.0
        const val BOTTOM_LIMIT_SWITCH_ID: Int = 0

        @LiveGains("/SmartDashboard/Elevator/PID", "p,i,d")
        val PID_GAINS = PIDConstants(23.166, 0.0, 1.7529)

        @LiveGains("/SmartDashboard/Elevator/FF", "kS,kG,kV,kA")
        val FF_GAINS = ElevatorFeedForwardConstants(0.27524, 0.10578, 3.033, 0.21036)

        // Tolerance for position
        var POS_TOLERANCE: Distance by DashboardMeasure(Meters, 0.5.inches, "Elevator")
        var OPERATOR_STEP: Distance by DashboardMeasure(Meters, 0.10.meters, "Elevator")

        var GRAVITY_TRAVEL: Distance by DashboardMeasure(Meters, 3.inches, "Elevator/Gravity")
        var GRAVITY_MOTION_SPEED: LinearVelocity by
            DashboardMeasure(MetersPerSecond, 0.02.metersPerSecond, "Elevator/Gravity")
        /** Voltage added or removed each second during gravity analysis. */
        var GRAVITY_RAMP_PER_SECOND: Voltage by
            DashboardMeasure(Volts, 0.4.volts, "Elevator/Gravity")
        var GRAVITY_MAX_VOLTAGE: Voltage by DashboardMeasure(Volts, 3.0.volts, "Elevator/Gravity")
        var GRAVITY_PHASE_TIMEOUT: Time by DashboardMeasure(Seconds, 20.sec, "Elevator/Gravity")

        val ELEVATOR_MOTION_CONSTRAINTS: TrapezoidProfile.Constraints
            get() =
                TrapezoidProfile.Constraints(
                    MAX_VELOCITY.convert(MetersPerSecond),
                    MAX_ACCELERATION.convert(MetersPerSecondPerSecond),
                )
    }

    private val leadMotor =
        SparkWrapper(
            Constants.LEADER_MOTOR_ID,
            MotorType.kBrushless,
            4.1875.inches * 2.0, // Multiply by 2 because elevator rigging
        ) {
            idleMode(IdleMode.kBrake)
            smartCurrentLimit(40)
            val factor = 1 / 5.0
            encoder.positionConversionFactor(factor).velocityConversionFactor(factor)
        }

    private val followerMotor =
        SparkWrapper(Constants.FOLLOWER_MOTOR_ID, MotorType.kBrushless) {
            apply(leadMotor.config)
            follow(Constants.LEADER_MOTOR_ID, true)
        }

    // private val encoder: Encoder = Encoder(Constants.ENCODER_LA_ID, Constants.ENCODER_LB_ID)

    private val elevatorFeedforward =
        ElevatorFeedforward(
            Constants.FF_GAINS.kS,
            Constants.FF_GAINS.kG,
            Constants.FF_GAINS.kV,
            Constants.FF_GAINS.kA,
        )

    private var targetHeight: Distance = Constants.MIN_HEIGHT

    private var isHomed: Boolean = false

    private enum class GravityPhase {
        UP,
        DOWN,
        RETURN,
        COMPLETE,
        FAILED,
    }

    private var bottomLimitSwitch =
        InvertibleDigitalInput(
            Constants.BOTTOM_LIMIT_SWITCH_ID,
            inverted = true,
        )

    private val pid: ProfiledPIDController =
        ProfiledPIDController(
            Constants.PID_GAINS.p,
            Constants.PID_GAINS.i,
            Constants.PID_GAINS.d,
            Constants.ELEVATOR_MOTION_CONSTRAINTS,
        )

    private val sysId =
        SysIdRoutine(
            SysIdRoutine.Config(Volts.per(Seconds).of(0.5), 4.0.volts, 4.0.sec),
            SysIdRoutine.Mechanism(
                { voltage ->
                    val position = getPos()
                    val movingUp = voltage.convert(Volts) > 0.0
                    val atLimit =
                        position == null ||
                            if (movingUp) position.asMeters >= Constants.MAX_HEIGHT.asMeters
                            else
                                bottomLimitSwitch.get() ||
                                    position.asMeters <= Constants.MIN_HEIGHT.asMeters
                    if (!isHomed || atLimit) stop()
                    else leadMotor.setVoltage(voltage.convert(Volts))
                },
                { log ->
                    val entry = log.motor("elevator")
                    entry.voltage(leadMotor.lastControl.volts)
                    getPos()?.let(entry::linearPosition)
                    leadMotor.linearVelocity?.let(entry::linearVelocity)
                },
                this,
            ),
        )

    init {
        // encoder.distancePerPulse = 0.01 / Constants.ENCODER_PPC

        pid.setTolerance(Constants.POS_TOLERANCE.asMeters)
        SmartDashboard.putData("Elevator/PID", pid)
        SmartDashboard.putData("Elevator/FF", ElevatorFFSendable(elevatorFeedforward))

        Alerts.register("Elevator") {
            buildList {
                if (!isHomed) add(Alerts.Alert("Elevator", "not homed", Alerts.Level.WARNING))
            }
        }
        defaultCommand = holdPosition()
    }

    override fun periodic() {
        SmartDashboard.putBoolean("Elevator/limitSwitch", bottomLimitSwitch.get())
        SmartDashboard.putNumber("Elevator/targetHeight", targetHeight.asMeters)
        SmartDashboard.putNumber("Elevator/position", getPos()?.asMeters ?: Double.NaN)
        SmartDashboard.putNumber(
            "Elevator/velocity",
            leadMotor.linearVelocity?.convert(MetersPerSecond) ?: Double.NaN,
        )
    }

    /** Stops elevator by setting lead motor speed to 0.0 */
    fun stop() {
        leadMotor.stopMotor()
    }

    /** Clamps a requested height to the elevator's configured travel. */
    fun clampToValidHeight(inputPosition: Distance): Distance {
        return Meters.of(
            MathUtil.clamp(
                inputPosition.asMeters,
                Constants.MIN_HEIGHT.asMeters,
                Constants.MAX_HEIGHT.asMeters,
            ),
        )
    }

    /** Returns encoder distance from the home position. */
    fun getPos(): Distance? {
        return (leadMotor.distance ?: return null) + Constants.MIN_HEIGHT
    }

    /** Sets the position of the motor given a Distance value in centimeters. */
    fun setPosition(positionCentimeters: Distance) {
        if (
            !isHomed ||
                leadMotor.initError != REVLibError.kOk ||
                followerMotor.initError != REVLibError.kOk
        ) {
            stop()
            return
        }
        targetHeight = clampToValidHeight(positionCentimeters)
        pid.setGoal(targetHeight.asMeters)
        controlToTarget()
    }

    private fun controlToTarget() {
        pid.constraints = Constants.ELEVATOR_MOTION_CONSTRAINTS
        pid.setTolerance(Constants.POS_TOLERANCE.asMeters)
        val pos = getPos()
        if (pos == null) {
            stop()
            return
        }
        val pidOutput = pid.calculate(pos.asMeters)
        val ffOutput = elevatorFeedforward.calculate(pid.setpoint.velocity)
        SmartDashboard.putNumber("Elevator/PID output", pidOutput)
        SmartDashboard.putNumber("Elevator/Feedforward output", ffOutput)
        SmartDashboard.putNumber("Elevator/voltage", pidOutput + ffOutput)
        leadMotor.setVoltage(pidOutput + ffOutput)
    }

    /** Homes the elevator and resets encoder when plant triggers bottom limit switch. */
    fun homeElevator(): Command = run {
        leadMotor.set(-0.2)
    }
        .until { bottomLimitSwitch.get() }
        .finallyDo { interrupted ->
            stop()
            if (!interrupted && bottomLimitSwitch.get()) {
                // encoder.reset()
                isHomed = leadMotor.encoder?.setPosition(0.0) == REVLibError.kOk
                targetHeight = Constants.MIN_HEIGHT
                pid.reset(Constants.MIN_HEIGHT.asMeters)
            }
        }

    /** Checks if the elevator position is within a certain tolerance of the goal position. */
    fun isAtPosition(positionCentimeters: Distance): Boolean {
        val pos = getPos()
        return isHomed &&
            pos != null &&
            pid.atGoal() &&
            kotlin.math.abs(
                pos.asMeters - clampToValidHeight(positionCentimeters).asMeters,
            ) <= Constants.POS_TOLERANCE.asMeters
    }

    /** Move to target height in centimeters. */
    fun moveTo(targetPosition: () -> Distance): Command = run {
        setPosition(targetPosition())
    }
        .until { isAtPosition(targetPosition()) }
        .finallyDo(Runnable(::stop))

    fun moveTo(targetPosition: Distance): Command = moveTo { targetPosition }

    fun moveBy(distanceToMove: () -> Distance): Command = runOnce {
        targetHeight = clampToValidHeight(targetHeight + distanceToMove())
    }
        .andThen(moveToTarget())

    fun moveBy(distanceToMove: Distance): Command = moveBy { distanceToMove }

    private fun moveToTarget(): Command = run {
        setPosition(targetHeight)
    }
        .until { isAtPosition(targetHeight) }
        .finallyDo(Runnable(::stop))

    fun holdPosition(): Command {
        return run { setPosition(targetHeight) }.finallyDo(Runnable(::stop))
    }

    private fun sysIdLimitReached(direction: SysIdRoutine.Direction): Boolean {
        val position = getPos() ?: return true
        return !isHomed ||
            if (direction == SysIdRoutine.Direction.kForward)
                position.asMeters >= Constants.MAX_HEIGHT.asMeters
            else bottomLimitSwitch.get() || position.asMeters <= Constants.MIN_HEIGHT.asMeters
    }

    private fun finishSysId() {
        stop()
        getPos()?.let { position ->
            targetHeight = clampToValidHeight(position)
            pid.reset(position.asMeters)
        }
    }

    fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command =
        sysId
            .quasistatic(direction)
            .until { sysIdLimitReached(direction) }
            .finallyDo(Runnable(::finishSysId))

    fun sysIdDynamic(direction: SysIdRoutine.Direction): Command =
        sysId
            .dynamic(direction)
            .until { sysIdLimitReached(direction) }
            .finallyDo(Runnable(::finishSysId))

    /**
     * Measures the upward and downward motion thresholds twice. Their midpoint estimates kG while
     * canceling much of the static friction. Start homed at the bottom with a clear elevator path.
     */
    fun analyzeGravity(): Command {
        var phase = GravityPhase.FAILED
        var pass = 0
        var phaseStart = 0.0
        var lastTime = 0.0
        var volts = 0.0
        var riseStart = 0.0
        var upThreshold = 0.0
        val estimates = mutableListOf<Double>()
        var leadIdle: IdleMode? = null
        var followerIdle: IdleMode? = null
        var failure = ""

        return run {
                if (phase == GravityPhase.FAILED || phase == GravityPhase.COMPLETE) return@run
                val now = Timer.getFPGATimestamp()
                val position = getPos()?.asMeters
                val speed = leadMotor.linearVelocity?.convert(MetersPerSecond)
                if (
                    position == null || speed == null || !position.isFinite() || !speed.isFinite()
                ) {
                    failure = "encoder reading unavailable"
                    phase = GravityPhase.FAILED
                    stop()
                    return@run
                }
                if (now - phaseStart > Constants.GRAVITY_PHASE_TIMEOUT.convert(Seconds)) {
                    failure = "timed out during $phase"
                    phase = GravityPhase.FAILED
                    stop()
                    return@run
                }
                val dt = (now - lastTime).coerceIn(0.0, 0.1)
                lastTime = now
                val step = Constants.GRAVITY_RAMP_PER_SECOND.convert(Volts) * dt
                val maxVoltage = Constants.GRAVITY_MAX_VOLTAGE.convert(Volts)
                val motionSpeed = Constants.GRAVITY_MOTION_SPEED.convert(MetersPerSecond)
                when (phase) {
                    GravityPhase.UP -> {
                        if (position >= Constants.MAX_HEIGHT.asMeters) {
                            failure = "upper travel limit reached"
                            phase = GravityPhase.FAILED
                        } else {
                            volts = (volts + step).coerceAtMost(maxVoltage)
                            leadMotor.setVoltage(volts)
                            if (upThreshold == 0.0 && speed >= motionSpeed) upThreshold = volts
                            if (position - riseStart >= Constants.GRAVITY_TRAVEL.asMeters) {
                                if (upThreshold == 0.0) {
                                    failure = "upward motion threshold was not measured"
                                    phase = GravityPhase.FAILED
                                } else {
                                    phase = GravityPhase.DOWN
                                    phaseStart = now
                                }
                            } else if (volts >= maxVoltage && speed < motionSpeed) {
                                failure = "no upward motion at voltage limit"
                                phase = GravityPhase.FAILED
                            }
                        }
                    }
                    GravityPhase.DOWN -> {
                        if (bottomLimitSwitch.get() || position <= Constants.MIN_HEIGHT.asMeters) {
                            failure = "bottom reached before downward threshold"
                            phase = GravityPhase.FAILED
                        } else {
                            volts = (volts - step).coerceAtLeast(-maxVoltage)
                            leadMotor.setVoltage(volts)
                            if (speed <= -motionSpeed) {
                                estimates.add((upThreshold + volts) / 2.0)
                                SmartDashboard.putNumber(
                                    "Elevator/Gravity/pass${pass + 1}EstimateVolts",
                                    estimates.last(),
                                )
                                stop()
                                if (++pass == 2) phase = GravityPhase.COMPLETE
                                else {
                                    phase = GravityPhase.RETURN
                                    phaseStart = now
                                }
                            } else if (volts <= -maxVoltage) {
                                failure = "no downward motion at voltage limit"
                                phase = GravityPhase.FAILED
                            }
                        }
                    }
                    GravityPhase.RETURN -> {
                        stop()
                        if (bottomLimitSwitch.get()) {
                            volts = 0.0
                            upThreshold = 0.0
                            riseStart = position
                            phase = GravityPhase.UP
                            phaseStart = now
                        }
                    }
                    else -> Unit
                }
                if (phase == GravityPhase.FAILED) stop()
                SmartDashboard.putString("Elevator/Gravity/status", "pass ${pass + 1}: $phase")
                SmartDashboard.putNumber("Elevator/Gravity/appliedVolts", volts)
            }
            .beforeStarting(
                {
                    estimates.clear()
                    pass = 0
                    volts = 0.0
                    upThreshold = 0.0
                    failure = ""
                    phase = GravityPhase.FAILED
                    SmartDashboard.putNumber("Elevator/Gravity/estimateVolts", Double.NaN)
                    if (!isHomed || !bottomLimitSwitch.get()) {
                        failure = "home the elevator at the bottom before analysis"
                    } else if (leadMotor.dashboardControl || followerMotor.dashboardControl) {
                        failure = "disable direct motor dashboard control before analysis"
                    } else {
                        leadIdle = leadMotor.idleMode
                        followerIdle = followerMotor.idleMode
                        if (leadIdle == null || followerIdle == null)
                            failure = "could not read motor idle modes"
                        else if (
                            !leadMotor.setTemporaryIdleMode(IdleMode.kCoast) ||
                                !followerMotor.setTemporaryIdleMode(IdleMode.kCoast)
                        )
                            failure = "could not set both motors to coast"
                        else {
                            phase = GravityPhase.UP
                            riseStart = getPos()?.asMeters ?: Constants.MIN_HEIGHT.asMeters
                            phaseStart = Timer.getFPGATimestamp()
                            lastTime = phaseStart
                        }
                    }
                    if (phase == GravityPhase.FAILED)
                        SmartDashboard.putString("Elevator/Gravity/status", failure)
                },
            )
            .until { phase == GravityPhase.COMPLETE || phase == GravityPhase.FAILED }
            .finallyDo { interrupted ->
                stop()
                val restoredLead = leadIdle?.let(leadMotor::setTemporaryIdleMode) ?: true
                val restoredFollower =
                    followerIdle?.let(followerMotor::setTemporaryIdleMode) ?: true
                getPos()?.let { position ->
                    targetHeight = clampToValidHeight(position)
                    pid.reset(position.asMeters)
                }
                val message =
                    when {
                        !restoredLead || !restoredFollower -> "could not restore motor idle mode"
                        interrupted -> "gravity analysis interrupted"
                        phase == GravityPhase.FAILED -> "gravity analysis failed: $failure"
                        else -> {
                            val estimate = estimates.average()
                            SmartDashboard.putNumber("Elevator/Gravity/estimateVolts", estimate)
                            "Elevator kG estimate: %.4f V (two passes)".format(estimate)
                        }
                    }
                SmartDashboard.putString("Elevator/Gravity/status", message)
                DriverStation.reportWarning(message, false)
            }
    }
}
