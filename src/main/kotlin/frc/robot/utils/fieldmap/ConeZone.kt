package frc.robot.utils.fieldmap

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import frc.robot.utils.asMeters
import frc.robot.utils.feet
import frc.robot.utils.inches

/**
 * Field map for BunnyBots 2026: Cone Zone.
 *
 * Derived from the official "BB26 Cone Zone Field Drawings v1.0" (Team 1540). All positions use the
 * WPILib field frame shared with `conezone.json`: origin at the blue-end wall on the B-side corner,
 * +X toward the red end, +Y toward the D side, in meters.
 */
object FieldMapConeZone {
    val fieldLength: Distance = 54.feet + 8.75.inches
    val fieldWidth: Distance = 26.feet + 4.inches

    /** Physical size of the AprilTags on the field (6.5" black square, 36h11 family). */
    val tagSize: Distance = 6.5.inches

    /** Tag IDs on the tower stations (full-size robot scoring locations). */
    val towerTagIds = 1..8

    /** Tag IDs on the inside walls of the minibot arena. */
    val arenaTagIds = 9..12

    /** Center of each tower station, keyed by its AprilTag ID. */
    val towerCentersByTag: Map<Int, Translation2d> =
        mapOf(
                1 to Translation2d((-280.5).inches, 0.0.inches),
                2 to Translation2d((-172.5).inches, 140.0.inches),
                3 to Translation2d(0.0.inches, 140.0.inches),
                4 to Translation2d(172.5.inches, 140.0.inches),
                5 to Translation2d(280.5.inches, 0.0.inches),
                6 to Translation2d(172.5.inches, (-140.0).inches),
                7 to Translation2d(0.0.inches, (-140.0).inches),
                8 to Translation2d((-172.5).inches, (-140.0).inches),
            )
            .mapValues { (_, fromCenter) ->
                fromCenter + Translation2d(fieldLength.asMeters / 2.0, fieldWidth.asMeters / 2.0)
            }

    /**
     * The 15"x15" footprint of a tower station, centered on [center].
     *
     * @return the (min corner, max corner) of the footprint.
     */
    fun towerFootprint(center: Translation2d): Pair<Translation2d, Translation2d> {
        val half = 15.0.inches.asMeters / 2.0
        val offset = Translation2d(half, half)
        return Pair(center - offset, center + offset)
    }

    /**
     * Outer boundary of the 10ft x 8ft minibot arena, centered on the field, as (min corner, max
     * corner).
     */
    val minibotArena: Pair<Translation2d, Translation2d> =
        Pair(
            Translation2d(
                (fieldLength.asMeters - 120.0.inches.asMeters) / 2.0,
                (fieldWidth.asMeters - 96.0.inches.asMeters) / 2.0,
            ),
            Translation2d(
                (fieldLength.asMeters + 120.0.inches.asMeters) / 2.0,
                (fieldWidth.asMeters + 96.0.inches.asMeters) / 2.0,
            ),
        )

    /** X of the field centerline (robots may not fully cross it in auto). */
    val centerlineX: Double = fieldLength.asMeters / 2.0

    /** X of the blue starting line (blue robots start between it and the centerline). */
    val blueStartingLineX: Double = centerlineX - 180.0.inches.asMeters

    /** X of the red starting line (red robots start between it and the centerline). */
    val redStartingLineX: Double = centerlineX + 180.0.inches.asMeters

    /** Returns the center of the tower station nearest to [pos]. */
    fun nearestTower(pos: Translation2d): Translation2d = pos.nearest(towerCentersByTag.values)
}
