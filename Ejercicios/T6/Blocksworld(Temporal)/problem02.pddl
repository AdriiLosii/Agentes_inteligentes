

(define (problem BW-rand-10)
(:domain blocksworld)
(:objects b1 b2 b3 b4 b5 b6 b7 b8 b9 b10  - block)
(:init
(handempty)
(ontable b1)
(on b2 b8)
(on b3 b10)
(on b4 b3)
(on b5 b7)
(ontable b6)
(ontable b7)
(on b8 b5)
(ontable b9)
(ontable b10)
(clear b1)
(clear b2)
(clear b4)
(clear b6)
(clear b9)
)
(:goal
(and (on b7 b4))
)
)

