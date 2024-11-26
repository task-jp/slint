---
<!-- Copyright © SixtyFPS GmbH <info@slint.dev> ; SPDX-License-Identifier: MIT -->
title: Colors & Brushes
description: Colors Namespaces
---



Color literals follow the syntax of CSS:

```slint
export component Example inherits Window {
    background: blue;
    property<color> c1: #ffaaff;
    property<brush> b2: Colors.red;
}
```

In addition to plain colors, many elements have properties that are of type `brush` instead of `color`.
A brush is a type that can be either a color or gradient. The brush is then used to fill an element or
draw the outline.

CSS Color names are only in scope in expressions of type `color` or `brush`. Otherwise, access
colors from the `Colors` namespace.

## Color Properties

The following properties are exposed:

### red
### green
### blue
### alpha

These properties are in the range 0-255.

Use the colors namespace to select colors by their name. For example you can use `Colors.aquamarine` or `Colors.bisque`.
The entire list of names is very long. You can find a complete list in the [CSS Specification](https://www.w3.org/TR/css-color-3/#svg-color).

These color names are available in scope of `color` and `brush` expressions, as well as in the `Colors` namespace.

```slint no-test
// Using the Colors namespace
background: Colors.aquamarine;

// Using the functions via global scope.
background: aquamarine;
```

## Global Color Functions

### rgb(int, int, int) -> color 
### rgba(int, int, int, float) -> color

Return the color as in CSS. Like in CSS, these two functions are actually aliases that can take
three or four parameters.

The first 3 parameters can be either number between 0 and 255, or a percentage with a `%` unit.
The fourth value, if present, is an alpha value between 0 and 1.

Unlike in CSS, the commas are mandatory.

### hsv(h: float, s: float, v: float) -> color
### hsv(h: float, s: float, v: float, a: float) -> color

Return a color computed from the HSV color space. The hue is between 0 and 360.
The saturation, value, and optional alpha parameter are expected to be within the range of 0 and 1.

## Color Methods
All colors and brushes define the following methods:

### brighter(factor: float) -> brush

Returns a new color derived from this color but has its brightness increased by the specified factor.
This is done by converting the color to the HSV color space and multiplying the brightness (value) with (1 + factor).
For example if the factor is 0.5 (or for example 50%) the returned color is 50% brighter. Negative factors
decrease the brightness.

### darker(factor: float) -> brush

Returns a new color derived from this color but has its brightness decreased by the specified factor.
This is done by converting the color to the HSV color space and dividing the brightness (value) by (1 + factor).
For example if the factor is .5 (or for example 50%) the returned color is 50% darker. Negative factors
increase the brightness.

### mix(other: brush, factor: float) -> brush

Returns a new color that is a mix of this color and `other`. The specified factor is
clamped to be between `0.0` and `1.0` and then applied to this color, while `1.0 - factor`
is applied to `other`. For example `red.mix(green, 70%)` will have a stronger tone of red, while
`red.mix(green, 30%)` will have a stronger tone of green.

### transparentize(factor: float) -> brush

Returns a new color with the opacity decreased by `factor`.
The transparency is obtained by multiplying the alpha channel by `(1 - factor)`.

### with-alpha(alpha: float) -> brush

Returns a new color with the alpha value set to `alpha` (between 0 and 1)

### to-hsv()->{hue: float, saturation: float, value: float, alpha: float}

Converts this color to the HSV color space and returns a struct with the `hue`, `saturation`, `value`,
and `alpha` fields. `hue` is between 0 and 360 while `saturation`, `value`, and `alpha` are between 0 and 1.

### Linear Gradients

Linear gradients describe smooth, colorful surfaces. They're specified using an angle and a series of
color stops. The colors will be linearly interpolated between the stops, aligned to an imaginary line
that is rotated by the specified angle. This is called a linear gradient and is specified using the
`@linear-gradient` macro with the following signature:

### @linear-gradient(angle, color percentage, color percentage, ...)

The first parameter to the macro is an angle (see [Types](types.md)). The gradient line's starting point
will be rotated by the specified value.

Following the initial angle is one or multiple color stops, describe as a space separated pair of a
`color` value and a `percentage`. The color specifies which value the linear color interpolation should
reach at the specified percentage along the axis of the gradient.

The following example shows a rectangle that's filled with a linear gradient that starts with a light blue
color, interpolates to a very light shade in the center and finishes with an orange tone:

```slint
export component Example inherits Window {
    preferred-width: 100px;
    preferred-height: 100px;

    Rectangle {
        background: @linear-gradient(90deg, #3f87a6 0%, #ebf8e1 50%, #f69d3c 100%);
    }
}
```

## Radial Gradients

Radial gradients are like linear gradients but the colors are interpolated circularly instead of
along a line. To describe a radial gradient, use the `@radial-gradient` macro with the following signature:

### @radial-gradient(circle, color percentage, color percentage, ...)

The first parameter to the macro is always `circle` because only circular gradients are supported.
The syntax is otherwise based on the CSS `radial-gradient` function.

Example:

```slint
export component Example inherits Window {
    preferred-width: 100px;
    preferred-height: 100px;
    Rectangle {
        background: @radial-gradient(circle, #f00 0%, #0f0 50%, #00f 100%);
    }
}
```