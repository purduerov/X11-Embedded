# X11-Embedded
Expands on V1 by fixing the way we intearct with the ESC's. Namely, it is meant to initialize the ESC's correctly which was not even being attempted in V1. Also, fixes some issues with CAN not being setup on the Solenoid the same way it is being setup on the ESC CAN version.

-CAN tested on both solenoid and ESC's both work fine

-solenoid runs only off of GPIO's

-ESC's only run from stall to full forward i.e. 0-127 with 0 being stall and 127 being full forward
