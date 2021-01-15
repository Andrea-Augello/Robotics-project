# Compilazione
Per compilare usare i seguenti comandi, sostituendo a `path_to_project_folder` il path della cartella del progetto:

```sh
cd ~/path_to_project_folder/catkin_ws
source devel/setup.bash
catkin_make
```

# Esecuzione
Per eseguire usare il seguente comando:

```sh
roslaunch webots_ros change.launch
```