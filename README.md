# Honours Thesis
This is the project I did to fulfill my requirements for my undergraduate honours thesis.

This project requires PyBullet which can be installed in the following ways according to which Python version you are running.

For Python 2.x
```
(sudo) pip install PyBullet
```

For Python 3.x
```
pip3 install PyBullet
```

PyBullet's quickstart guide can be found [here](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3). 
Note that since PyBullet is a python wrapper for a C++ library some windows users might find that their compilers need to be updated before PyBullet can be installed.

To run the project:
```
python -i 2D_biped_using_SIMBICON.py
```
A GUI should pop up where you will see the legs.xml specified body fall into the world and begin walking. The `-i` allows you to interact with 
the legs and the GUI once the script starts running. If you would like to play around with the simulation try changing the target angles and PD
controller constants to see what they do! A full disscussion of the theory behind the project can be found [here](https://github.com/kaitlinthachuk/honours_thesis/blob/master/KaitlinThachuk_CPSC449Thesis.pdf).
