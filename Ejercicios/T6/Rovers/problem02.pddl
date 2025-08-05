(define (problem roverprob10) (:domain Rover)
(:objects
	general - Lander
	high_res - Mode
	rover0 - Rover
	rover0store - Store
	waypoint0 waypoint1 waypoint2 - Waypoint
	camera0 - Camera
	objective0 - Objective
	)
(:init
	(visible waypoint0 waypoint1)
	(visible waypoint1 waypoint0)
	(visible waypoint2 waypoint0)
	(visible waypoint0 waypoint2)
	(visible waypoint2 waypoint1)
	(visible waypoint1 waypoint2)
    (visible waypoint0 waypoint2)
	
	(at_lander general waypoint1)
	(channel_free general)
	(at rover0 waypoint1)
	(available rover0)
	(equipped_for_imaging rover0)
	(can_traverse rover0 waypoint1 waypoint0)
	(can_traverse rover0 waypoint0 waypoint1)
	(can_traverse rover0 waypoint1 waypoint2)
	(can_traverse rover0 waypoint2 waypoint1)
	(can_traverse rover0 waypoint2 waypoint0)
    (can_traverse rover0 waypoint0 waypoint2)
	(on_board camera0 rover0)
	(calibration_target camera0 objective0)
	(supports camera0 high_res)
	(visible_from objective0 waypoint0)
	(visible_from objective0 waypoint1)
	(visible_from objective0 waypoint2)

	(= (distance-travelled) 0)
	(= (distance waypoint1 waypoint0) 1)
	(= (distance waypoint0 waypoint1) 10)
	(= (distance waypoint1 waypoint2) 20)
	(= (distance waypoint2 waypoint1) 5)
	(= (distance waypoint2 waypoint0) 1)
    (= (distance waypoint0 waypoint2) 1)
)

(:goal (and
(communicated_image_data objective0 high_res)
	)
)

(:metric minimize (distance-travelled))
)
