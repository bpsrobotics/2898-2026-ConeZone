package frc.robot.commands.swerve

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.DashboardString
import frc.robot.utils.metersPerSecond
import frc.robot.utils.radiansPerSecond

class DriveManager() : Command() {
    interface DriveRequest {
        val vx: LinearVelocity?
        val vy: LinearVelocity?
        val omega: AngularVelocity?
        val doLock: Boolean?
        val isEnabled: Boolean
        val priority: Int
        /**
         * Whether this request's speeds stay meaningful robot-oriented when the gyro is
         * untrustworthy. Only teleop driving opts in; field-frame vision aligners go quiet instead.
         */
        val allowRobotOrientedFallback: Boolean
            get() = false
    }

    abstract class DriveRequestBase : Command(), DriveRequest {
        override var vx: LinearVelocity? = null
        override var vy: LinearVelocity? = null
        override var omega: AngularVelocity? = null
        override var doLock: Boolean? = null
        override var isEnabled: Boolean = false

        override fun initialize() {
            isEnabled = true
        }

        override fun end(interrupted: Boolean) {
            isEnabled = false
        }

        /** Runs this driver in its own DriveManager. */
        fun solitarily(): Command = alongWith(DriveManager(listOf(this)))
    }

    constructor(initRequests: List<DriveRequest>) : this() {
        requests.addAll(initRequests)
    }

    init {
        addRequirements(Drivetrain)
    }

    var requestsDebug by DashboardString("", "DriveManager")
    var requestsEnabledDebug by DashboardString("", "DriveManager")

    private val requests = mutableListOf<DriveRequest>()

    fun <T : DriveRequest> defineDriver(rq: T): T {
        requests.add(rq)
        return rq
    }

    override fun execute() {
        requestsDebug = requests.map { it.priority }.joinToString()
        requestsEnabledDebug = requests.filter { it.isEnabled }.map { it.priority }.joinToString()

        val doLock =
            requests
                .filter { it.doLock != null && it.isEnabled }
                .maxByOrNull { it.priority }
                ?.doLock ?: false
        if (doLock) {
            Drivetrain.lock()
            return
        }

        if (Drivetrain.fieldOrientedAvailable) {
            val vx =
                requests.filter { it.vx != null && it.isEnabled }.maxByOrNull { it.priority }?.vx
                    ?: 0.0.metersPerSecond
            val vy =
                requests.filter { it.vy != null && it.isEnabled }.maxByOrNull { it.priority }?.vy
                    ?: 0.0.metersPerSecond
            val omega =
                requests
                    .filter { it.omega != null && it.isEnabled }
                    .maxByOrNull { it.priority }
                    ?.omega ?: 0.0.radiansPerSecond
            Drivetrain.driveFieldOriented(ChassisSpeeds(vx, vy, omega))
            return
        }

        // Gyro untrustworthy: field-frame requests (vision aligners) are disabled outright. Only
        // teleop falls back to robot-oriented so the driver keeps a drivable robot.
        val vx =
            requests
                .filter { it.vx != null && it.isEnabled && it.allowRobotOrientedFallback }
                .maxByOrNull { it.priority }
                ?.vx
        val vy =
            requests
                .filter { it.vy != null && it.isEnabled && it.allowRobotOrientedFallback }
                .maxByOrNull { it.priority }
                ?.vy
        val omega =
            requests
                .filter { it.omega != null && it.isEnabled && it.allowRobotOrientedFallback }
                .maxByOrNull { it.priority }
                ?.omega
        if (vx == null && vy == null && omega == null) {
            Drivetrain.stop()
            return
        }
        Drivetrain.driveRobotOriented(
            ChassisSpeeds(
                vx ?: 0.0.metersPerSecond,
                vy ?: 0.0.metersPerSecond,
                omega ?: 0.0.radiansPerSecond,
            )
        )
    }
}
