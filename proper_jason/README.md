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

- *laser*: starts (true) and stops (false) the laser equipment to be ready for processing.

```json
{"laser":true}
```

- *power*: sets the laser power in W.

```json
{"power":1000}
```

- *carrier*: sets the carrier gas value in l/min.

```json
{"carrier":5}
```

- *turntable*: sets the turntable value in rpm.

```json
{"turntable":4}
```

- *stirrer*: sets the stirrer value in %.

```json
{"stirrer":20}
```

- *powder*: starts (true) and stops (false) the powder flow from the feeder.

```json
{"powder":true}
```

- *wire*: starts (true) and stops (false) the wire feeder equipment to be ready for processing.

```json
{"wire":true}
```

- *wait*: programs a wait time interruption in seconds.

```json
{"wait":0.5}
```

- *speed*: sets the TCP linear speed in mm/s.

```json
{"speed":30}
```

- *move*: moves to a cartesian pose linearly, and process the signal when is present.

```json
{"move":[[800, 0, 800], [0, 0, 1, 0]]}
```

```json
{"move":[[800, 0, 1200], [0, 0, 1, 0],true]}
```
