# Server Robot Commands

List of available jason commands that can be published on topic "robot_command_json":

- *tool*: sets the tool pose (position and orientation).

```json
{"tool":[[0, 0, 0], [1, 0, 0, 0]]}
```

- *workobject*: sets the workobject pose (position and orientation).

```json
{"workobject":[[0, 0, 0], [1, 0, 0, 0]]}
```

- *speed*: sets the TCP linear speed in mm/s.

```json
{"speed":30}
```

- *power*: sets the laser power in W.

```json
{"power":1000}
```

- *stirrer*: sets the stirrer value in %.

```json
{"stirrer":40}
```

- *turntable*: sets the turntable value in rpm.

```json
{"turntable":3}
```

- *powder_start*: starts and stops the powder flow from the feeder.

```json
{"powder_start":1}
```

- *pose*: appends a single cartesian pose to the buffer, and activates or
deactivates the laser when the boolean option appears.

```json
{"pose":[[800, 0, 800], [0, 0, 1, 0]]}
```

```json
{"pose":[[800, 0, 1200], [0, 0, 1, 0],true]}
```

- *path_move*: immediately executes linear moves to every pose in the buffer.

```json
{"path_move":1}
```

- *path_clear*: immediately executes linear moves to every pose in the buffer.

```json
{"path_move":1}
```

- *move*: moves to a cartesian pose linearly, and process the signal when is present.

```json
{"move":[[800, 0, 800], [0, 0, 1, 0]]}
```

```json
{"move":[[800, 0, 1200], [0, 0, 1, 0],true]}
```

- *movej*: moves the system to the specified cartesian pose.

```json
{"movej":[[800, 0, 800], [0, 0, 1, 0]]}
```
