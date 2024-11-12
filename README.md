# ERCTimsav-GRBL-Servo
This is a niche Inkscape plugin that aims to provide simple, direct conversion of selected paths to gcode for use with GRBL-Servo

## Why?
There is a "servo.py" extension designed for GRBL-Servo, and there are the "gcodetools" plugins that are default in inkscape. Why do we need this?

GRBL-Servo is about 3,000 lines of code, and creates additional paths on the document, and uses the concept of "origin points" which are all inherited from the origonal gcodetools.

Gcodetools is designed to be used for CNC routers which have large cutting heads that need to be offset from the cut line, thus there is additional complexity and confusion when creating simple gcode for a needle cutter. The "origion points" often do not scale properly.

We skip all that. Gcode uses the top-right cartesian plain, but inkscape (presumably inheriting from CRT computer monitors) uses the bottom-right cartesian quadrant. Thus we will offer the ability to invert the Y axis, and potentially use the bottom-left corner of the document as the origin point. We always scale 1:1 from document units to mm, but will offer the option of manual scaling.

## TODO:
 - Enable mirroring by inverting Y axis.
 - Incorperate servo parameters from GUI
 - Add GUI elements for different feed rates for move and cut/score
 - Keep track of cutter up/down for fewer gcode commands
 - Enable Y offset - GUI adjustable, but default to document height.
