import omni.replicator.core as rep


"""
This snippet creates a camera and lights that move to random positions each frame, looking at the forklift model.
Expected to be used with synthetic data recorder plugin
By running this script -> specify the replicator camera in the synthetic data recorder -> start recording
"""

camera = rep.create.camera()
lights = rep.create.light(light_type="sphere", count=3)

with rep.trigger.on_frame():
	with camera:
		rep.modify.pose(
			position=rep.distribution.uniform((-30, -30, 1), (30, 30, 8)),look_at="/Root/forklift_c")
	with lights:
		rep.modify.pose(position=rep.distribution.uniform((-5, -5, 3), (5, 5, 8)))
		rep.modify.attribute("intensity", rep.distribution.uniform(1000, 500000))
		rep.modify.attribute("color", rep.distribution.normal((1.0, 1.0, 1.0), (0.5, 0.5, 0.5), 1000))

