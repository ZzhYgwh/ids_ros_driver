# ids_ros_driver
This is a driver program to enable IDS camera, IDS U3-3041LE Rev.1.2.

#### Camera driver library download

https://cn.ids-imaging.com/store/products/cameras/u3-3041le-rev-1-2.html

```
sudo dpkg -i thirdparty/ids-peak_2.11.0.0-178_amd64.deb
```



#### build

```
catkin build ids_ros_driver

or 

catkin_make
```

#### Fix issue that include file error
Fix IDS_Camera.h line13-17
```
// Add IDS peak library
#include <peak/peak.hpp>
#include <peak_ipl/peak_ipl.hpp>
#include <peak/peak_buffer_converter.hpp>
#include <peak/converters/peak_buffer_converter_ipl.hpp>
```
For the problem that it cannot be found, enter the dpkg installation path and redirect:
```
cd /usr/include/
sudo cp ids_peak-1.8.0/ ./
sudo cp ids_peak_afl-1.6.0/ ./
sudo cp ids_peak_ipl-1.12.1/ ./
```
Then
```
catkin build ids_ros_driver
or
catkin_make
```
