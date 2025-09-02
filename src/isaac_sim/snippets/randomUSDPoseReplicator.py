import omni.replicator.core as rep
usd_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Forklift/forklift_c.usd"
asset = rep.create.from_usd(usd_path, semantics=[("class", "forklift_c")])

with rep.trigger.on_frame():
	with asset:
		rep.modify.pose(
			position=rep.distribution.uniform((-24, -9, 0), (-2.1, 7, 0)),
			rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360))
		)
