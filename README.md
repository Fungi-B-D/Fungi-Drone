# Fungi Drone

## Usage

The fungi drone implements the run and new methods as all other drones,
however for edge cases which aren't covered by the protocol there is a simple debug system.

You can choose whether or not to print the edge case error.
The same applies for sending the error to your simulation controller.


``` rust
    let fungi = FungiDrone::new(droneparams);

    fungi.set_debug_send(true);  // To send the error packet to the simulation controller

    fungi.set_debug_print(true) // To print out the error

```


The output of the prints is formatted as so:

``` terminal
  [DroneId] : <Error Message>
```

