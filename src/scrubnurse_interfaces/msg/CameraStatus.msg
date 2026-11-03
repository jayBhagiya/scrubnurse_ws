# Which Azure Kinect the camera node streams, published (latched) on every change.

builtin_interfaces/Time stamp
string serial              # streaming now
string configured_serial   # from the parameter file: the calibrated overhead camera
string[] serials           # connected Azure Kinects; switch with the parameter `serial`
