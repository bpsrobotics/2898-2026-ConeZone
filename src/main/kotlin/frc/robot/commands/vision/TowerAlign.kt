package frc.robot.commands.vision

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import frc.robot.utils.DashboardMeasure
import frc.robot.utils.DashboardNumber
import frc.robot.utils.asMeters
import frc.robot.utils.controls.toPID
import frc.robot.utils.convert
import frc.robot.utils.feetPerSecond
import frc.robot.utils.fieldmap.FieldMapConeZone
import frc.robot.utils.meters
import frc.robot.utils.radiansPerSecond
import frc.robot.utils.sec
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.pow

private object TowerAlignTuning {
    var standoffDistance: Distance by DashboardMeasure(Units.Meters, 1.0.meters, "TowerAlign")
    var distanceRange: Distance by DashboardMeasure(Units.Meters, 1.5.meters, "TowerAlign")
    var maxApproachSpeed: LinearVelocity by
        DashboardMeasure(Units.MetersPerSecond, 6.0.feetPerSecond, "TowerAlign")
    var maxOrbitSpeed: LinearVelocity by
        DashboardMeasure(Units.MetersPerSecond, 4.0.feetPerSecond, "TowerAlign")
    var maxTurnSpeed: AngularVelocity by
        DashboardMeasure(Units.RadiansPerSecond, 1.0.radiansPerSecond, "TowerAlign")
    var tagStaleTime: Time by DashboardMeasure(Units.Seconds, 0.15.sec, "TowerAlign")
    var selectionSettleTime: Time by DashboardMeasure(Units.Seconds, 0.1.sec, "TowerAlign")
    var facingWeight: Double by DashboardNumber(1.5, "TowerAlign")
    var tagAmbiguityLimit: Double by DashboardNumber(0.5, "TowerAlign")
}

private data class TagObservation(
    val id: Int,
    val robotToCenter: Translation2d,
    val fieldCenter: Translation2d,
    val seenAt: Double,
)

private fun towerScore(distance: Double, bearing: Double): Double =
    distance * (1.0 + TowerAlignTuning.facingWeight * abs(bearing))

private fun tagToTowerCenter(camera: Vision.Camera, target: PhotonTrackedTarget): Translation2d {
    val robotToTag = camera.robotToCamera.plus(target.bestCameraToTarget)
    val tagToCenter = Translation3d(-FieldMapConeZone.towerHalfWidth, 0.0.meters, 0.0.meters)
    return robotToTag.translation.plus(tagToCenter.rotateBy(robotToTag.rotation)).toTranslation2d()
}

/**
 * Drives to and orbits a tower station. The tower is latched on the first cycle, preferring a
 * visible tower tag in front of the robot and falling back to the nearest tower in the field map.
 * The left stick's horizontal axis orbits and its vertical axis adjusts the standoff distance; the
 * robot keeps facing the tower. Intended to be held while scoring.
 */
fun towerAlign(
    orbitInput: () -> Double,
    distanceInput: () -> Double,
    slowMode: () -> Double = { 0.0 },
): Command {
    val distancePID = Drivetrain.Constants.TRANSLATION_PID.toPID()
    val rotationPID = Drivetrain.Constants.ROTATION_PID.toPID()
    rotationPID.enableContinuousInput(-PI, PI)

    val timer = Timer()
    var subscription: Vision.Subscription? = null
    val observations = mutableMapOf<Int, TagObservation>()
    var selectedTagId: Int? = null
    var selected = false
    var lastMeasuredCenter: Translation2d? = null

    fun selectTower(now: Double) {
        observations.values.removeAll {
            now - it.seenAt > TowerAlignTuning.tagStaleTime.convert(Units.Seconds)
        }

        val visible =
            observations.values.minByOrNull { observation ->
                val distance = observation.robotToCenter.norm
                val bearing = observation.robotToCenter.angle.radians
                towerScore(distance, bearing)
            }
        if (visible != null) {
            selectedTagId = visible.id
            return
        }

        val pose = Drivetrain.pose
        val nearest =
            FieldMapConeZone.towerCentersByTag.entries.minByOrNull { (_, center) ->
                val delta = center - pose.translation
                val bearing = MathUtil.angleModulus(delta.angle.radians - pose.rotation.radians)
                towerScore(delta.norm, bearing)
            }
        selectedTagId = nearest?.key
    }

    fun targetCenter(now: Double): Translation2d? {
        val id = selectedTagId ?: return null
        val observation = observations[id]
        if (
            observation != null &&
                now - observation.seenAt <= TowerAlignTuning.tagStaleTime.convert(Units.Seconds)
        ) {
            lastMeasuredCenter = observation.fieldCenter
            return lastMeasuredCenter
        }
        return lastMeasuredCenter ?: FieldMapConeZone.towerCentersByTag[id]
    }

    fun computeSpeeds(): ChassisSpeeds {
        val now = timer.get()
        if (
            !selected &&
                (now >= TowerAlignTuning.selectionSettleTime.convert(Units.Seconds) ||
                    observations.isNotEmpty())
        ) {
            selectTower(now)
            selected = true
        }

        val center = targetCenter(now) ?: return ChassisSpeeds()
        val pose = Drivetrain.pose
        val toCenter = center - pose.translation
        val distance = toCenter.norm
        if (distance < 1e-6) return ChassisSpeeds()

        val radial = toCenter.div(distance)
        val tangent = Translation2d(-radial.y, radial.x)
        val desiredDistance =
            (TowerAlignTuning.standoffDistance.asMeters +
                    TowerAlignTuning.distanceRange.asMeters * distanceInput())
                .coerceAtLeast(0.0)
        val slowScale = 1.0 - slowMode().pow(3) * 0.8

        val distanceSpeed =
            distancePID
                .calculate(distance - desiredDistance)
                .coerceIn(
                    -TowerAlignTuning.maxApproachSpeed.convert(Units.MetersPerSecond) * slowScale,
                    TowerAlignTuning.maxApproachSpeed.convert(Units.MetersPerSecond) * slowScale,
                )
        val orbitSpeed =
            -orbitInput().coerceIn(-1.0, 1.0) *
                TowerAlignTuning.maxOrbitSpeed.convert(Units.MetersPerSecond) *
                slowScale
        val rotationSpeed =
            rotationPID
                .calculate(pose.rotation.radians, radial.angle.radians)
                .coerceIn(
                    -TowerAlignTuning.maxTurnSpeed.convert(Units.RadiansPerSecond) * slowScale,
                    TowerAlignTuning.maxTurnSpeed.convert(Units.RadiansPerSecond) * slowScale,
                )

        return ChassisSpeeds(
            radial.x * distanceSpeed + tangent.x * orbitSpeed,
            radial.y * distanceSpeed + tangent.y * orbitSpeed,
            rotationSpeed,
        )
    }

    return Drivetrain.driveLive(::computeSpeeds)
        .beforeStarting({
            subscription = Vision.onResult { result, camera ->
                val now = Timer.getFPGATimestamp()
                for (target in result.targets) {
                    if (target.fiducialId !in FieldMapConeZone.towerTagIds) continue
                    if (target.poseAmbiguity > TowerAlignTuning.tagAmbiguityLimit) continue
                    val robotToCenter = tagToTowerCenter(camera, target)
                    observations[target.fiducialId] =
                        TagObservation(
                            target.fiducialId,
                            robotToCenter,
                            Drivetrain.pose.translation +
                                robotToCenter.rotateBy(Drivetrain.pose.rotation),
                            now,
                        )
                }
            }
            timer.restart()
            observations.clear()
            selectedTagId = null
            selected = false
            lastMeasuredCenter = null
            distancePID.reset()
            rotationPID.reset()
        })
        .finallyDo(
            Runnable {
                subscription?.close()
                subscription = null
                Drivetrain.stop()
            }
        )
}
