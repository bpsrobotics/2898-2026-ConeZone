package frc.robot.commands.vision

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import frc.robot.utils.asRadians
import frc.robot.utils.convert
import frc.robot.utils.degrees
import frc.robot.utils.getAlliance
import frc.robot.utils.radiansPerSecond
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sign
import org.photonvision.targeting.PhotonTrackedTarget

private val alignConeRotationPID = PIDController(3.0, 0.0, 0.1)

/** How long the last seen cone keeps steering the robot after the pipeline stops reporting it. */
private const val ALIGN_CONE_TARGET_TIMEOUT_SECONDS = 0.15

/** Time for the camera to settle after switching pipelines before its targets are trusted. */
private const val ALIGN_CONE_PIPELINE_SETTLE_SECONDS = 0.15

/**
 * Turns the robot toward the largest cone of the current alliance color. The AR0234 pipeline is
 * switched to red or blue for the duration and restored to the driver feed afterward. Runs until
 * the cone is centered for a few frames, or until interrupted.
 */
fun alignCone(
    pipelineSupplier: () -> Vision.ConePipeline = {
        Vision.ConePipeline.forAlliance(getAlliance())
    },
    maxRotSpeed: AngularVelocity = 1.0.radiansPerSecond,
): Command {
    val rotationPID = alignConeRotationPID
    rotationPID.enableContinuousInput(-PI, PI)
    val timer = Timer()
    val maxOmega = maxRotSpeed.convert(Units.RadiansPerSecond)

    var subscription: Vision.Subscription? = null
    var cone: PhotonTrackedTarget? = null
    var timeSinceConeSeen = 0.0
    var alignedFrames = 0

    fun computeSpeeds(): ChassisSpeeds {
        val target = cone
        if (target == null || timer.get() - timeSinceConeSeen > ALIGN_CONE_TARGET_TIMEOUT_SECONDS) {
            return ChassisSpeeds()
        }

        var rotationSpeed =
            -rotationPID.calculate(target.yaw.degrees.asRadians).coerceIn(-maxOmega, maxOmega)
        val deadzone = 0.02
        if (rotationSpeed.absoluteValue < deadzone) rotationSpeed = 0.0
        else rotationSpeed += deadzone * rotationSpeed.sign

        return ChassisSpeeds(0.0, 0.0, rotationSpeed)
    }

    return Drivetrain.driveLiveRobotOriented(::computeSpeeds)
        .beforeStarting({
            timer.restart()
            Vision.pipeline = pipelineSupplier()
            subscription = Vision.onConeResult { result, _ ->
                // The camera takes a few frames to adjust after a pipeline switch; ignore
                // targets until it has settled.
                if (timer.get() < ALIGN_CONE_PIPELINE_SETTLE_SECONDS) {
                    return@onConeResult
                }
                // The pipelines are largest-first, but pick explicitly so ordering changes are
                // harmless.
                val largest = result.targets.maxByOrNull { it.area } ?: return@onConeResult
                cone = largest
                timeSinceConeSeen = timer.get()
            }
            rotationPID.reset()
            cone = null
            alignedFrames = 0
        })
        .until {
            val target = cone
            val aligned =
                target != null && target.yaw.degrees.asRadians.absoluteValue < 2.degrees.asRadians
            if (aligned) alignedFrames += 1 else alignedFrames = 0
            alignedFrames > 3
        }
        .finallyDo(
            Runnable {
                subscription?.close()
                Vision.pipeline = Vision.ConePipeline.DRIVER
                Drivetrain.stop()
            }
        )
}
