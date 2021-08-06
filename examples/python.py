import motioncapture

mc = motioncapture.MotionCaptureOptitrack("130.149.82.29")
# mc = motioncapture.connect("optitrack", "130.149.82.29")
while True:
    mc.waitForNextFrame()
    for name, obj in mc.rigidBodies.items():
        print(name, obj.position, obj.rotation.z)