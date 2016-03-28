# Server Robot Commands

List of available jason commands that can be published on topic "robot_command_json".

- ## vel
#### {"vel":30}
Sets robot TCP linear speed in mm/s.

- ## pose
#### {"pose":[[800, 0, 800], [0, 0, 1, 0]]}
Appends single cartesian pose to the robot buffer.
#### {"pose":[[800, 0, 1200], [0, 0, 1, 0],true]}
Appends single cartesian pose with signal trigger to the robot buffer.

- ## path_move
#### {"path_move":1}
Immediately execute linear moves to every pose in the remote buffer.

- ## path_clear
#### {"path_move":1}
Immediately execute linear moves to every pose in the remote buffer.


- ## workobject
#### {"workobject":[[0, 0, 0], [1, 0, 0, 0]]}
Immediately execute linear moves to every pose in the remote buffer.

- ## tool
#### {"tool":[[0, 0, 0], [1, 0, 0, 0]]}
Immediately execute linear moves to every pose in the remote buffer.

- ## move
#### {"move":[[800, 0, 800], [0, 0, 1, 0]]}
Moves to a cartesian pose linearly.
#### {"move":[[800, 0, 1200], [0, 0, 1, 0],true]}
Moves to a cartesian pose linearly with signal trigger.

- ## movej
#### {"movej":[[800, 0, 800], [0, 0, 1, 0]]}
Moves to a cartesian pose.

- ## set_dio
#### {"set_dio":[1, 0]}
Set or reset(first element) a digital output(second element).

- ## set_ao
#### {"set_ao":[3.5, 0]}
Set a float value(first element) to an analog output(second element).
