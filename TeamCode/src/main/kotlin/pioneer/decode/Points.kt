package pioneer.decode

import pioneer.general.AllianceColor
import pioneer.helpers.Pose
import kotlin.math.PI

/*
                            GOAL SIDE
                |-------------------------------|
                |               +Y              |
                |               ^               |
                |               |               |
                |               |               |
BLUE ALLIANCE   |               0----> +X       |    RED ALLIANCE
                |                               |
                |                               |
                |                               |
                |                               |
                |-------------------------------|

                            AUDIENCE SIDE

    THETA = 0 FACING FORWARD
*/

// All points are defined from the RED ALLIANCE perspective
// Desmos Link: https://www.desmos.com/calculator/ojosiylesc
class Points(
    private val color: AllianceColor,
) {
    private fun Pose.transform(): Pose =
        when (color) {
            AllianceColor.RED -> this
            AllianceColor.BLUE -> Pose(-this.x, this.y, -this.theta)
            AllianceColor.NEUTRAL -> this
        }

    private fun fieldPose(x: Double, y: Double, theta: Double): Pose =
        Pose(x = x, y = y, theta = theta).transform()

    // Key positions on the field
    // Written in ACTION_POSITION format
    val START_GOAL: Pose = fieldPose(130.0, 137.0, 3 * PI / 4)
    val START_FAR: Pose = fieldPose(43.0, -157.0, 0.0)

    val SHOOT_GOAL_CLOSE: Pose = fieldPose(60.0, 60.0, 0.0)
    val SHOOT_GOAL_FAR: Pose = fieldPose(43.0, -145.0, 0.0)
    private fun collectY(i: Int) = 30 - (i * 60.0) // 30, -30, -90
    
    // Collection
    private val prepCollectX = 70.0
    private val collectX = 130.0
    private val collectTheta = -PI / 2 // Point to right

    val PREP_COLLECT_GOAL: Pose = fieldPose(prepCollectX, collectY(0), collectTheta)
    val PREP_COLLECT_MID: Pose = fieldPose(prepCollectX, collectY(1), collectTheta)
    val PREP_COLLECT_AUDIENCE: Pose = fieldPose(prepCollectX, collectY(2), collectTheta)
    
    val COLLECT_GOAL: Pose = fieldPose(collectX, collectY(0), collectTheta)
    val COLLECT_MID: Pose = fieldPose(collectX, collectY(1), collectTheta)
    val COLLECT_AUDIENCE: Pose = fieldPose(collectX, collectY(2), collectTheta)

    val PARK_BASE: Pose = fieldPose(85.0, -100.0, -PI / 2)
}
