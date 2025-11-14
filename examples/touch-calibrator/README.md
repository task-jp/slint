# Touch Calibrator

A touchscreen calibration tool for calculating calibration matrices, similar to weston-calibrator.

## Features

- **Multi-Point Calibration**: Supports 3, 4, or 5 point calibration modes
- **Aspect-Ratio Aware**: Target positions automatically adjust for screen dimensions
- **Smooth Animations**: Crosshair targets move with ease-out-back animation
- **Progress Tracking**: Visual progress bar and point counter
- **Least-Squares Fitting**: Accurate calibration matrix calculation using nalgebra
- **Restart Capability**: Easy restart of calibration process
- **Internationalization**: UI text ready for translation with @tr()

## How It Works

1. The application displays an animated crosshair target at predefined positions
2. The user touches each target as it appears on the screen
3. The application records both the raw touch position and expected target position
4. After all points are collected, a 6-parameter affine transformation matrix is calculated using least-squares fitting
5. The calibration matrix is displayed on screen and printed to the console

## Usage

Run with default 5-point calibration:
```bash
cargo run
```

Specify the number of calibration points (3, 4, or 5):
```bash
cargo run -- 3  # 3-point calibration (equilateral triangle)
cargo run -- 4  # 4-point calibration (four corners)
cargo run -- 5  # 5-point calibration (four corners + center)
```

Touch each target as it appears. After completing all points, the calibration matrix will be displayed.

## Calibration Modes

### 3-Point Mode
Uses an equilateral triangle centered on the screen, rotated 15 degrees. This is the minimum number of points needed for affine transformation calibration.

### 4-Point Mode
Uses the four corners of the screen with aspect-ratio aware margins (40px from edges).

### 5-Point Mode (Default)
Uses the four corners plus the center point for improved accuracy.

## Calibration Algorithm

The tool calculates a 6-parameter affine transformation matrix using least-squares fitting:

```
calibrated_x = a * raw_x + b * raw_y + c
calibrated_y = d * raw_x + e * raw_y + f
```

The six parameters (a, b, c, d, e, f) are calculated by solving the overdetermined system of equations formed by the calibration points.

## Output Format

The calibration values are displayed in weston-calibrator compatible format:

```
a b c
d e f
```

Example output:
```
0.996256 0.012633 -0.001569
-0.010563 0.984978 0.003896
```

These values can be used with libinput's [LIBINPUT_CALIBRATION_MATRIX](https://wayland.freedesktop.org/libinput/doc/latest/device-configuration-via-udev.html#libinput_calibration_matrix) property or other touch input systems that support affine transformation.
