# cwru_opencv_common

These libraries are meant to fill in some of the functionality that is missing from the standard opencv libraries.
In particular projective geometry algorithms and transformations are included.

'''Note that some features may be common with the ViSP visual servoing library. If this is the case, ViSP may be included instead but ViSP already reinvents many parts of opencv so it is very bloated.'''

## Example usage

The file projective geometry.h includes camera projection and camera deprojection that also generates the appropriate jacobians of the object points in image space.


## Running tests/demos
    
