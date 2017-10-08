TaiChiViz.py using VTK

The program can be ran in two modes, in one it renders as fast as it can
as an animation and in the other it renders a frame that can be interacted with, below
are examples of command lines for both modes:

"Animation Mode"

python taiChiViz.py JWBG_session_09-29-2017_001.json 8 150 -9.8378703748 2.419864159 -6.00797144214 -3.94280904618 1.57224505345 -3.52672339144 0.123083989085 0.991319524741 0.0462161389378 2.25813430535 11.7407832555 1 0 0.035

"Interactive Mode"

python taiChiViz.py JWBG_session_09-29-2017_001.json 8 2500 -9.8378703748 2.419864159 -6.00797144214 -3.94280904618 1.57224505345 -3.52672339144 0.123083989085 0.991319524741 0.0462161389378 2.25813430535 11.7407832555 1 1 0.035

The first argument after the filename is the stride through the dataset, the second number is the size of the trail in steps
The next 11 numbers describe the xyz position, focal point, up vectors and clipping planes of the 3d camera.

The next arguement turns scaling by value on and off (0 for on 1 for off) 
The next arguement toggles animation and interactive mode
The last argument sets the size for the 3d glyph (sphere)


