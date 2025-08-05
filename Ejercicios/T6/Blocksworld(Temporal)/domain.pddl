(define (domain blocksworld)
  (:requirements :strips :typing :durative-actions)
  (:types block)
  (:predicates (on ?x - block ?y - block)
	       (ontable ?x - block)
	       (clear ?x - block)
	       (handempty)
	       (holding ?x - block)
	       )

  (:durative-action pick-up
	     :parameters (?x - block)
		 :duration ( = ?duration 5)
	     :condition (and (at start (clear ?x)) (at start (ontable ?x)) (at start (handempty)))
	     :effect
	     (and (at start (not (ontable ?x)))
		   (at start (not (clear ?x)))
		   (at start (not (handempty)))
		   (at end (holding ?x))))

  (:durative-action put-down
	     :parameters (?x - block)
		 :duration ( = ?duration 6)
	     :condition (and (over all (holding ?x)))
	     :effect
	     (and (at end (not (holding ?x)))
		   (at end (clear ?x))
		   (at end (handempty))
		   (at end (ontable ?x))))

  (:durative-action stack
	     :parameters (?x - block ?y - block)
		 :duration ( = ?duration 5)
	     :condition (and (at start (holding ?x)) (at start (clear ?y)))
	     :effect
	     (and (at end (not (holding ?x)))
		   (at end (not (clear ?y)))
		   (at end (clear ?x))
		   (at end (handempty))
		   (at end (on ?x ?y))))
		   
  (:durative-action unstack
	     :parameters (?x - block ?y - block)
		 :duration ( = ?duration 10)
	     :condition (and (at start (on ?x ?y)) (at start (clear ?x)) (at start (handempty)))
	     :effect
	     (and (at end (holding ?x))
		   (at end (clear ?y))
		   (at start (not (clear ?x)))
		   (at start (not (handempty)))
		   (at start (not (on ?x ?y)))))

)