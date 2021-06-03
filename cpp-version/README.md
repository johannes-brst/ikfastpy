needed:

sudo apt-get install libblas-dev liblapack-dev

modify CMakeLists.txt to match installtion path of library:

use  dpkg -L <package> to look up path