# Drone
Repository containing the drone for the Advanced Programming course in Universitá di Trento for the year 2024/2025.

## Using the drone
To include the drone in your project just add this line to your `[dependencies]` in the Cargo.toml: 
```toml
rustafarian-drone = { git = "https://github.com/Rustafarian-Unitn/rustafarian-drone"}
```

## Setting the drone log level
The drone offers different log levels. To avoid cluttering the output while testing, the logs can be disabled, by setting
the environment variable `RUSTAFARIAN_LOG_LEVEL` to `NONE`. There are three additional log levels, to fine tune the 
information displayed by the drone. 
These are:
- `ERROR` - While using this level, only errors will be displayed to the standard error
- `INFO` - While using this level, other information will be displayed, as well as errors, still keeping the output to
what is strictly necessary
- `DEBUG` - While using this level, the drone will enter a more verbose behaviour, displaying more information to help 
the debug phase