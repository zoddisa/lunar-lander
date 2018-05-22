# lunar-lander
Lunar Lander v. 1.0

CONTEXT:

This program was my final project for my Intro to Programming class. I utilized my experience with Linear Algebra and Physics to build this. I could have used a more developed graphics package, but wanted to figure out the technical questions of displaying rotation and translation with the limitations this imposed.

LICENSE and CREDITS:

lunar-lander.pyw is open-source software released under the terms of the
GPL (http://www.gnu.org/licenses/gpl.html).

graphics.py is written by John Zelle, a simple graphics library written for use with his book "Python Programming: An Introduction to Computer Science." It is also licensed under the GPL.

HOW TO MODIFY:

Starting at line 629, several constants are defined which can be changed to set difficulty and realism. A future version, if developed, will use 

HOW TO PLAY:

When you start lunar-lander.pyw, two windows will be created. One is a map view, and one is a display of the lander as well as the instructions and a control panel.

The map view does not rotate and uses an absolute coordinate system. Your position is indicated by a yellow dot, and your projected trajectory is indicated by a blue arc.

The lander view rotates such that 'down' is always towards the surface of the moon. The triangle displayed over the lander indicates the direction the lander is moving (the prograde vector).

In the lander view, as you approach the surface, the horizon, land closer than the horizon, and surface will appear. The lightest of the three is the true land.

You land at 2.5 meters altitude, and if your relative velocity is at or below 3.0 meters per second, you land. There is no check for the orientation of the craft in this version.

EPILEPSY WARNING:

If you are epileptic or are otherwise sensitive to flickering, please be aware that unavoidable flickering occurs. This is a result of the way the graphics are displayed. This flickering is more noticable at lower altitudes but is always present.
