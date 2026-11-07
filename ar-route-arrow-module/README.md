# AR Route: arrow navigation module (Android, Kotlin)

Package `com.arroute.nav`.

| Part | Status |
|---|---|
| `core/` (geometry, marker registration, localization state machine, route follower, arrow geometry) | Implemented. 15 JVM unit tests pass (compiled with Kotlin 2.1.20 / JDK 21 in a sandbox, run with a small local JUnit shim; they use only `org.junit.Test` and `org.junit.Assert`). |
| `ar/ArNavController.kt` (ARCore frame to core) | Written, **never compiled or run**. Needs your ARCore 1.48.0 build. |
| OpenGL arrow drawing | Not included. Feed `Output.arrow` (session-frame vertices, y = finished floor) into your existing renderer. |
| Physical accuracy, marker recognition, drift | **No evidence exists.** Nothing here has run on a phone. |

## How it works
1. A surveyed marker is seen: `Rigid2D.fromMarker` gives the session-to-plan transform from the marker's pose in both frames. No scaling.
2. Each frame: camera pose to plan pose, then `RouteFollower` picks a target 4 m ahead and returns `Arrow(relativeBearing)`, `TurnAround`, `OffRoute` or `Arrive`.
3. The arrow is drawn flat on the floor 2.4 m ahead. Floor height = marker session y minus its surveyed `heightAboveFloor`; no plane detection.
4. ANY of these deletes the transform, so no stale pose is ever reused: tracking not normal, session restart/pause, floor change, map version change, camera not allowed, pose jump over 3 m, distance or time budget exceeded, conflicting floor evidence. Only a newly accepted scan of a SURVEYED marker restores FINE.

## Integrate
- Add `src/main/kotlin/com/arroute/nav/core` to a JVM-only module (or your app) and the tests to `src/test`.
- Load `MarkerRecord`s from the released map. A marker whose status is not `SURVEYED` is rejected by design.
- Call `configureSession(...)` with MEASURED printed widths, `ArNavController.onFrame(frame, nowSec)` per frame, and `onSessionInterrupted()` from `onPause` and whenever the Session is recreated. Call `manager.onFloorChange(...)`, `setCameraAllowed(...)` and `onMapVersionChange(...)` from your own logic.
- Starting budgets (placeholders to set from measured drift): 20 m and 90 s.

## Must verify on a phone before trusting anything (UNVERIFIED assumptions)
1. **Frame handedness.** The editor plan is assumed right-handed, y up, (x, z) with the same axis order as the ARCore session. A mirrored plan flips left/right. Test: stand facing a known direction after a scan and compare the reported yaw with the real direction.
2. **Marker axes.** The code assumes the Augmented Images `centerPose` Y axis is the image normal and X is the image's right. Confirm in a log on the device for one wall marker and one floor marker, and fix `toObservation` if not.
3. **Camera forward.** The code uses the negative Z axis of `camera.displayOrientedPose`. Confirm by walking straight and checking the reported yaw is stable.
4. **Marker size.** A wrong printed width scales the marker distance and so the registration. Measure the printed width.
5. **Marker recognition** on your tiles, walls and lighting, and from what distance and angle. Unknown.
6. **Drift and budgets.** Measure error against taped crosses (see survey section 15.4 of the build prompt) and set the budgets from data.
7. **Lifts and floor change.** `onFloorChange` must be called by your floor detection; none is included here.

## Not included
Floor-change detection, no-camera zone data, rerouting after `OffRoute`, marker image-quality scoring, GL rendering.
