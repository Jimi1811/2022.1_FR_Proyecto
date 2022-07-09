# FR_Proyecto_2022-1
Proyecto de curso Fundamentos de Robótica 2022-1

* Robot manipulador en URDF: https://github.com/cian777/VL

* Para poder usar el proyecto en RDS: 

  1. Clonar este repositorio en ~/
      ```
      $ git clone https://github.com/Jimi1811/FR_Proyecto_2022-1.git

      $ source FR_Proyecto_2022-1/project_ws/devel/setup.bash
      ```
  3. En ~/FR_Proyecto_2022-1/project_ws/
  
      `$ catkin_make`

  4. Para correr en RViz

      `$ roslaunch VL_description display.launch`
  
  5. Para correr control dinámico usando RBDL, al igual que source:
      ```
      $ export LD_LIBRARY_PATH=/home/user/lab_ws/install/lib:$LD_LIBRARY_PATH
      
      $ export PYTHONPATH=/home/user/lab_ws/install/lib/python2.7/site-packages:$PYTHONPATH

* Para iniciar sesión y subir sus avances:
    ```
     $ git config --global user.name "Your name here"

     $ git config --global user.email "your_email@example.com" 
    ```
* Para entrar cada uno a su rama: https://www.youtube.com/watch?v=tFr0Vg1q9Eg
