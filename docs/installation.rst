Installation Guide
==================


Download
********
To use XICRO, you must create a meta-package for the code generation called "Xicro" and this repository to that meta-package. Build the workspace to finish the installtion. Assume that the workspace is named [xxx_ws] and located in the Home directory. 


.. code-block:: sh

  cd ~/xxx_ws/src      # cd to your workspace
  mkdir Xicro          # create metapackage 
  cd Xicro             # cd to metapackage
  git clone https://github.com/imchin/Xicro .
  cd ~/xxx_ws
  colcon build
  source ~/xxx_ws/install/setup.bash

.. _metapackage: https://github.com/imchin/Xicro/

Python Library Requirements
***************************

The code generation feature of XICRO relies on certain Python libraries. One of them being "pyserial", which often get mistaken with "serial". If you use "serial" library in your other project, you must change the name of the library to something else. Another option is to uninstall "serial" library and install "pyserial" instead. 

  - Pyserial_
  - Numpy_

.. _Pyserial: https://pythonhosted.org/pyserial/
.. _Numpy: https://numpy.org/devdocs/reference/index.html#reference

Now you are ready to generate some codes.
