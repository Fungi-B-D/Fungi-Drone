# Fungi Drone

## Usage

The fungi drone implements the run and new methods as all other drones,
however for edge cases which aren't covered by the protocol there is a simple debug system.

You can choose whether or not to print the edge case error.
The same applies for sending the error to your simulation controller.

You can also disable the logging of Flood Requests to the simulation controller.

``` rust
    let mut fungi = FungiDrone::new(droneparams);

    fungi.set_debug_send();  // To send the error packet to the simulation controller

    fungi.set_debug_print(); // To print out the error

    fungi.disable_request_log();

```


The output of the prints is formatted as so:

``` terminal

  [DroneId] : <Error Message>

```

To use this drone as a dependency just add it to your Cargo.toml


``` toml
[dependencies]
fungi_drone = { git = "https://github.com/Fungi-B-D/Fungi-Drone.git" }


```