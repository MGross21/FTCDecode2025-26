package pioneer.helpers

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import pioneer.pathing.paths.Path
import kotlin.math.cos
import kotlin.math.sin

/**
 * A utility class for plotting data on the FTC Dashboard.
 * Handles coordinate transformation from field coordinates to dashboard coordinates.
 */
object DashboardPlotter {
    // Configuration constants
    const val MAX_PREVIOUS_POSITIONS = 500
    private const val FIELD_WIDTH_CM = 366.0
    private const val DASHBOARD_SIZE = 144.0
    private const val FIELD_CENTER_OFFSET = DASHBOARD_SIZE / 2.0
    private const val ROBOT_SIZE = 8.0 // Robot is 8cm x 8cm
    const val DEFAULT_PATH_SAMPLES = 100

    private val previousPositions = mutableListOf<Pose>()
    var scale = 1.0

    /**
     * Convert from robot field coordinates to dashboard coordinates.
     * Field coordinates: +X right, +Y forward
     * Dashboard coordinates: +X forward, +Y left (with origin at field center)
     */
    private fun Pose.toDashboardCoordinates(): Pose {
        val scaleFactor = (DASHBOARD_SIZE / FIELD_WIDTH_CM) * scale
        return Pose(
            x = (this.y * scaleFactor),
            y = (-this.x * scaleFactor),
            theta = this.theta,
        )
    }

    /**
     * Plot the robot's current position on the dashboard.
     * @param packet The telemetry packet to draw on
     * @param position The robot's current pose in field coordinates
     * @param showPathTaken Whether to show the historical path
     * @param color The color to draw the robot (hex format)
     */
    fun plotBotPosition(
        packet: TelemetryPacket,
        position: Pose?,
        showPathTaken: Boolean = true,
        color: String = "#000000",
    ) {
        val pose = position?.toDashboardCoordinates() ?: return
        previousPositions.add(pose)
        if (previousPositions.size > MAX_PREVIOUS_POSITIONS) {
            previousPositions.removeFirst()
        }

        // Create robot corners relative to center
        val corners = arrayOf(
            Pose(-ROBOT_SIZE, -ROBOT_SIZE),
            Pose(ROBOT_SIZE, -ROBOT_SIZE),
            Pose(ROBOT_SIZE, ROBOT_SIZE),
            Pose(-ROBOT_SIZE, ROBOT_SIZE),
        )

        // Apply rotation and translation to corners
        val cosTheta = cos(pose.theta)
        val sinTheta = sin(pose.theta)
        val rotatedCorners = corners.map { corner ->
            Pose(
                x = pose.x + (corner.x * cosTheta - corner.y * sinTheta),
                y = pose.y + (corner.x * sinTheta + corner.y * cosTheta),
            )
        }

        // Draw robot as polygon (closing the shape by adding first point again)
        val xPoints = rotatedCorners.map { it.x }.toDoubleArray() + rotatedCorners.first().x
        val yPoints = rotatedCorners.map { it.y }.toDoubleArray() + rotatedCorners.first().y
        packet.fieldOverlay().setStroke(color).strokePolygon(xPoints, yPoints)

        if (showPathTaken && previousPositions.isNotEmpty()) {
            val xPath = previousPositions.map { it.x }.toDoubleArray()
            val yPath = previousPositions.map { it.y }.toDoubleArray()
            packet.fieldOverlay().strokePolyline(xPath, yPath)
        }
    }

    /**
     * Plot a path on the dashboard.
     * @param packet The telemetry packet to draw on
     * @param path The path to plot
     * @param color The color to draw the path (hex format)
     * @param samples Number of points to sample along the path
     */
    fun plotPath(
        packet: TelemetryPacket,
        path: Path,
        color: String = "#0000FF",
        samples: Int = DEFAULT_PATH_SAMPLES,
    ) {
        val points = (0 until samples).map { i ->
            val t = i / (samples - 1).toDouble()
            path.getPoint(t).toDashboardCoordinates()
        }

        val xPoints = points.map { it.x }.toDoubleArray()
        val yPoints = points.map { it.y }.toDoubleArray()
        packet.fieldOverlay().setStroke(color).strokePolyline(xPoints, yPoints)
    }

    /**
     * Plot a single point on the dashboard.
     * @param packet The telemetry packet to draw on
     * @param point The point to plot in field coordinates
     * @param color The color to draw the point (hex format)
     * @param radius The radius of the point in pixels
     */
    fun plotPoint(
        packet: TelemetryPacket,
        point: Pose,
        color: String = "#FF0000",
        radius: Double = 2.0,
    ) {
        val dashboardPoint = point.toDashboardCoordinates()
        packet.fieldOverlay().setFill(color).fillCircle(dashboardPoint.x, dashboardPoint.y, radius)
    }

    /**
     * Plot a circle on the dashboard.
     * @param packet The telemetry packet to draw on
     * @param center The center of the circle in field coordinates
     * @param radius The radius in cm (will be scaled to dashboard coordinates)
     * @param color The color to draw the circle (hex format)
     */
    fun plotCircle(
        packet: TelemetryPacket,
        center: Pose,
        radius: Double,
        color: String = "#FF0000",
    ) {
        val dashboardCenter = center.toDashboardCoordinates()
        val scaledRadius = radius * (DASHBOARD_SIZE / FIELD_WIDTH_CM) * scale
        packet.fieldOverlay().setStroke(color).strokeCircle(dashboardCenter.x, dashboardCenter.y, scaledRadius)
    }

    /**
     * Plot a grid overlay on the dashboard for reference.
     * Draws a fine grid with 7x7 divisions and a coarse grid with 2x2 divisions.
     */
    fun plotGrid(packet: TelemetryPacket) {
        val halfSize = DASHBOARD_SIZE / 2.0
        packet.fieldOverlay()
            .setStroke("#888888")
            .drawGrid(-halfSize, -halfSize, DASHBOARD_SIZE, DASHBOARD_SIZE, 7, 7)
            .setStrokeWidth(2)
            .setStroke("#222222")
            .drawGrid(-halfSize, -halfSize, DASHBOARD_SIZE, DASHBOARD_SIZE, 2, 2)
    }

    fun clearPreviousPositions() {
        previousPositions.clear()
    }
}
