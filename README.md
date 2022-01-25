# Curso Robots Móviles 2022-2 FI, UNAM

Material para el curso de Robots Móviles de la Facultad de Ingeniería, UNAM, Semestre 2022-2

## Requerimientos

* Ubuntu 18.04 https://releases.ubuntu.com/18.04/
* ROS Melodic http://wiki.ros.org/melodic/Installation/Ubuntu
* Mucha paciencia

## Instalación

Nota: se asume que ya se tiene instalado Ubuntu y ROS.

* $ cd
* $ git clone https://github.com/mnegretev/Mobile-Robots-2022-2
* $ cd Mobile-Robots-2022-2
* $ ./Setup.sh
* $ cd catkin_ws
* $ catkin_make -j2 -l2

Para probar que todo se instaló y compiló correctamente:

* $ cd 
* $ source Mobile-Robots-2022-2/catkin_ws/devel/setup.bash
* $ roslaunch bring_up path_planning.launch

Si todo se instaló y compiló correctamente, se debería ver un RViz como el siguiente:

<img src="https://github.com/mnegretev/Mobile-Robots-2022-2/blob/master/Media/rviz.png" alt="RViz" width="639"/>

Un Gazebo como el siguiente:

<img src="https://github.com/mnegretev/Mobile-Robots-2022-2/blob/master/Media/gazebo.png" alt="Gazebo" width="631"/>

Y una GUI como la siguiente:

<img src="https://github.com/mnegretev/Mobile-Robots-2022-2/blob/master/Media/gui.png" alt="GUI" width="319"/>

## Contacto
Dr. Marco Negrete<br>
Profesor Asociado C<br>
Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
[mnegretev.info](http://mnegretev.info)<br>
marco.negrete@ingenieria.unam.edu<br>
