package org.firstinspires.ftc.teamcode.pathing.paths

import org.firstinspires.ftc.teamcode.localization.Pose

/**
 * Represents a compound path made up of multiple paths.
 * The paths are assumed to be connected end-to-end, meaning the end pose of one path
 * should match the start pose of the next path in the list.
 * @param paths A list of paths that make up the compound path.
 */
class CompoundPath(val paths: List<Path>) : Path {

    override var startPose: Pose = paths.first().startPose
    override var endPose: Pose = paths.last().endPose
    override var headingInterpolationMode: Path.HeadingInterpolationMode = Path.HeadingInterpolationMode.LINEAR

    init {
        // Validate that the paths are connected end-to-end
        for (i in 0 until paths.size - 1) {
            if (paths[i].endPose != paths[i + 1].startPose) {
                throw IllegalArgumentException("Paths are not connected end-to-end")
            }
        }
    }

    override fun getLength(): Double {
        return paths.sumOf { it.getLength() }
    }

    override fun getHeading(t: Double): Double {
        val pathIndex = (t * paths.size).toInt().coerceIn(0, paths.size - 1)
        val localT = t * paths.size - pathIndex
        return paths[pathIndex].getHeading(localT)
    }

    override fun getPoint(t: Double): Pose {
        val pathIndex = (t * paths.size).toInt().coerceIn(0, paths.size - 1)
        val localT = t * paths.size - pathIndex
        return paths[pathIndex].getPoint(localT)
    }

    override fun getTangent(t: Double): Pose {
        val pathIndex = (t * paths.size).toInt().coerceIn(0, paths.size - 1)
        val localT = t * paths.size - pathIndex
        return paths[pathIndex].getTangent(localT)
    }

    override fun getNormal(t: Double): Pose {
        val pathIndex = (t * paths.size).toInt().coerceIn(0, paths.size - 1)
        val localT = t * paths.size - pathIndex
        return paths[pathIndex].getNormal(localT)
    }

    override fun getSecondDerivative(t: Double): Pose {
        val pathIndex = (t * paths.size).toInt().coerceIn(0, paths.size - 1)
        val localT = t * paths.size - pathIndex
        return paths[pathIndex].getSecondDerivative(localT)
    }

    override fun getCurvature(t: Double): Double {
        val pathIndex = (t * paths.size).toInt().coerceIn(0, paths.size - 1)
        val localT = t * paths.size - pathIndex
        return paths[pathIndex].getCurvature(localT)
    }

    override fun getClosestPointT(position: Pose): Double {
        // Find the closest point on each path and return the closest one
        var closestT = -1.0
        var minDistance = Double.MAX_VALUE

        for (path in paths) {
            val t = path.getClosestPointT(position)
            val point = path.getPoint(t)
            val distance = position.distanceTo(point)

            if (distance < minDistance) {
                minDistance = distance
                closestT = (t + paths.indexOf(path)) / paths.size
            }
        }

        return closestT
    }

    override fun getLengthSoFar(t: Double): Double {
        // Calculate the total length up to the given t value
        val pathIndex = (t * paths.size).toInt().coerceIn(0, paths.size - 1)
        val localT = t * paths.size - pathIndex

        var length = 0.0
        for (i in 0 until pathIndex) {
            length += paths[i].getLength()
        }
        length += paths[pathIndex].getLengthSoFar(localT)

        return length
    }

    override fun getTFromLength(length: Double): Double {
        var accumulatedLength = 0.0

        for (i in paths.indices) {
            val pathLength = paths[i].getLength()
            if (accumulatedLength + pathLength >= length) {
                val remainingLength = length - accumulatedLength
                val localT = paths[i].getTFromLength(remainingLength)
                return (i + localT) / paths.size
            }
            accumulatedLength += pathLength
        }

        return 1.0 // If length exceeds total, return end of the compound path
    }

    class Builder {
        private val paths = mutableListOf<Path>()

        fun addPath(path: Path): Builder {
            paths.add(path)
            return this
        }

        fun addPaths(vararg newPaths: Path): Builder {
            paths.addAll(newPaths)
            return this
        }

        fun build(): CompoundPath {
            return CompoundPath(paths)
        }
    }
}
