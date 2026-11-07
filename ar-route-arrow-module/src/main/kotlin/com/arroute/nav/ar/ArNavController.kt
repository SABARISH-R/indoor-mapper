package com.arroute.nav.ar

// UNTESTED: this file needs the ARCore SDK and was NOT compiled or run when written.
// The pure logic it calls (package com.arroute.nav.core) IS covered by JVM tests.

import android.graphics.Bitmap
import com.arroute.nav.core.ArrowGeometry
import com.arroute.nav.core.Guidance
import com.arroute.nav.core.LocState
import com.arroute.nav.core.LocalizationManager
import com.arroute.nav.core.MarkerRecord
import com.arroute.nav.core.Mount
import com.arroute.nav.core.PlanPose
import com.arroute.nav.core.RouteFollower
import com.arroute.nav.core.ScanResult
import com.arroute.nav.core.SessionObservation
import com.arroute.nav.core.Vec3
import com.google.ar.core.AugmentedImage
import com.google.ar.core.AugmentedImageDatabase
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Pose
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import kotlin.math.atan2

/** Marker images must be added with their MEASURED printed width in meters, not the width in the file. */
fun configureSession(session: Session, markerImages: List<Triple<String, Bitmap, Float>>) {
    val db = AugmentedImageDatabase(session)
    for ((id, bmp, widthM) in markerImages) db.addImage(id, bmp, widthM)
    val cfg = Config(session).apply {
        augmentedImageDatabase = db
        depthMode = Config.DepthMode.DISABLED          // depth is not on the critical path
        focusMode = Config.FocusMode.AUTO
        updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
    }
    session.configure(cfg)
}

class ArNavController(
    private val manager: LocalizationManager,
    private val follower: RouteFollower,
    private val markers: List<MarkerRecord>,
) {
    data class Output(
        val state: LocState,
        val lastScan: ScanResult?,
        val pose: PlanPose?,
        val guidance: Guidance?,
        /** Arrow vertices in the AR session frame, only while localization is valid and the camera is allowed. */
        val arrow: List<Vec3>?,
    )

    private val mountOf = markers.associate { it.id to it.mount }

    /** Call when the activity pauses/resumes or the Session is recreated: old poses are never kept. */
    fun onSessionInterrupted() = manager.onSessionRestart()

    fun onFrame(frame: Frame, nowSec: Double): Output {
        val camera = frame.camera
        val normal = camera.trackingState == TrackingState.TRACKING

        val observed = frame.getUpdatedTrackables(AugmentedImage::class.java)
            .filter { it.trackingState == TrackingState.TRACKING }
            .map { toObservation(it) }
        val scan = if (observed.isNotEmpty()) manager.onMarkerObserved(observed, normal, nowSec) else null

        val cam: Pose = camera.displayOrientedPose
        val z = cam.getTransformedAxis(2, 1f)           // local +Z in the session frame; the camera looks along -Z
        val fx = -z[0].toDouble(); val fz = -z[2].toDouble()
        val pose = manager.onFrame(normal, cam.tx().toDouble(), cam.tz().toDouble(), fx, fz, nowSec)

        val g = pose?.let { follower.guide(it) }
        val floorY = manager.floorYSession
        val arrow = if (g is Guidance.Arrow && floorY != null && manager.cameraAllowed)
            runCatching { ArrowGeometry.vertices(cam.tx().toDouble(), cam.tz().toDouble(), fx, fz, floorY, g.relativeBearing) }.getOrNull()
        else null
        return Output(manager.state, scan, pose, g, arrow)
    }

    private fun toObservation(img: AugmentedImage): SessionObservation {
        val p = img.centerPose
        // UNVERIFIED axis choice, see README check 2. WALL: outward normal. FLOOR: image +X.
        val axis = if (mountOf[img.name] == Mount.FLOOR) p.getTransformedAxis(0, 1f) else p.getTransformedAxis(1, 1f)
        return SessionObservation(
            markerId = img.name,
            x = p.tx().toDouble(), y = p.ty().toDouble(), z = p.tz().toDouble(),
            refYawSession = atan2(axis[2].toDouble(), axis[0].toDouble()),
            fullTracking = img.trackingMethod == AugmentedImage.TrackingMethod.FULL_TRACKING,
        )
    }
}
