#Document to help track the debugging process with the GPS and Compass

# Introduction #

Please document your debugging steps and solutions to problems here. Also list the current problems that need to be fixed and which ones are completed.


# Details #

---

## GPS: ##
  1. `done` **Why does the GPS get bad data?**
    * It never saw gga datums I had it working yesterday, I don't remember if I had to hack something or if the changes I made to the `YclopsReactiveNav` class did it. _--Eldon_
  1. **(enter more problems here)**
    * (enter solutions here)


---


## Compass: ##

  1. `done` **Why does the compass->getYaw() return 0?**
    * The compass had to have called doProcess first before any valid data will be retrieved.  I will change it so doProcess is called once in the constructor. _--Eldon_
  1. **Compass was returning degrees and MRPT wants radians.**
    * The Compass was given a default parameter of true if the value was to be returned in degrees
  1. `done` **MRPT is trying to do a lot of math to guess at the yaw trying to make up for not having a compass which Always output a 0.**
    * Bypass MRPT on the yaw and store it ourselves, with a getter method for it.
  1. **(enter more problems here)**
    * (enter solutions here)

---
