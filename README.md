# ERCTimsav-GRBL-Servo
This is a niche Inkscape plugin that aims to provide simple, direct conversion of selected paths to gcode for use with GRBL-Servo

## Why create this plug-in when...
 - there is a [MI-GRBL-Z-AXIS-Servo-Controller](https://github.com/ikae/MI-GRBL-Z-AXIS-Servo-Controller) extension designed for GRBL-Servo?
 - and there are the "gcodetools" plugins that are default in inkscape?

## Why do we need this?
### MI-GRBL-Servo
 - MI-GRBL-Servo is about 3,000 lines of confusing code.
 - It creates additional paths on the document which must be deleted after using the plug-in.
 - It uses the concept of "origin points" which are all inherited from the origonal gcodetools. "Origin points" are frequently scaled incorrectly, and are diffocult to troubleshoot because of the messy code.
 - The code is not easy to maintain with later versions of inkscape.

### Gcodetools
 - Gcodetools (which is now included in inkscape) is designed to be used for CNC routers which have large cutting heads that need to be offset from the cut line, thus there is additional complexity and confusion when creating simple gcode for a needle cutter.
 - Again, the "origion points" often do not scale properly.

## We skip all that.

 - This plugin is nearly feature-complete at ~300 lines of code.
 - This plugin directly converts paths to gcode, with no offset for cutting radius. (If you need that feature, use gcodetools.)
 - This plugin does not use "origin points". If you choose, it simply inverts the Y axis, and offsets the Y axis by a given amount. (You probably want to choose the document height.)

## Why converting SVG to gcode is not a trivial problem

 - Gcode basically has two kinds of shapes it can cut. Straight lines, and circular arcs. SVG path commands can describe straight lines, **eliptical arcs**, cubic bezier curves, and quadratic bezier curves. There is no direct mathematical conversion from these complex shapes to circular arcs. Thus a recursive approximation approach is needed. This plugin cuts the curve or arc in half, approximates that with a circular arc, and checks how much error there is. If the error is too great, it subdivides again, and re-computes. This repeats until the tolerance is met.
 - SVG, inheriting from rastering computer graphics systems, defines 0,0 at the top-left of the design. Positive x moves right, and positive y moves **down**. Gcode defines 0,0 at the bottom left corner of the design. Positive x moves right, but positive y moves **up**. Thus to convert between the two, **we simply invert the y axis (y_inv = -y), and offset by a specified amount (y_offset = y_inv + offset).**


## TODO:
 - ~~Enable mirroring by inverting Y axis.~~
 - ~~Incorperate servo parameters from GUI~~
 - ~~Add GUI elements for different feed rates for move and cut/score~~ (will not implement)
 - ~~Keep track of cutter up/down for fewer gcode commands~~
 - ~~Enable Y offset - GUI adjustable, but default to document height.~~
 - ~~Fix that curves don't use unit conversions (right now, they assume everyone is in mm)~~
 - Implement an algorithm that optimizes cutting route.
 - Add error catching for maximum recursion depth exceeded. This typically ocurrs on curves which are so subtle that they could easily be replaced by a line. Consider replacing segment with line, and issuing warning.

## Project Objectives:
 - Primary focus on CNC needle cutters for foam board. May also apply to laser cutters. Zero cut width assumed.
 - Deals well with confused document units, and large transformed paths.
 - Robust code that handles errors well and fails gracefully.
 - Only support latest versions of Inkscape. This is in line with a "rolling release" mentality that I like in Arch linux.
 - KISS (Keep It Simple, Stupid) code that is easily debugged and maintained.

 ---

 This extension was primarily written in [English](https://en.wikipedia.org/wiki/English_language) and compiled into [Python](https://en.wikipedia.org/wiki/Python_(programming_language)) with [GPT o1-preview](https://openai.com/o1/). Some amount of post-compilation linking was required.
