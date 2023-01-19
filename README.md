AutomaticShading

-----------Installation------------

This project requires 
-OpenMesh 8.0
-QtCreator
-PointCloudLibrary (pcl)
RenderMan 23.3 (only for render the final scene).

This software has been developed in Ubuntu 18.04.2 LTS and hasn't been tested in any other platform.
Since having RenderMan working on this version of Ubuntu is quite a tedious and tricky task we also provide here the headers and the libraries needed to create a scene that could be rendered using RenderMan.
To do the user have to  simply copy the folder named "Data" and then use Renderman.
It's possible to also use the modified simulation to create new landscaped that can be passed as input to our framework.
The modified simulation has been developed using this [terrain-errosion](https://github.com/karhu/terrain-erosion) project.

| Keys       | Action                       |
| -----------|------------------------------|
| W/S        | move camera forward/backward |
| A/D        | move camera left/right       |
| R/F        | move camera up/down          |
| Q/E        | rotate camera left/right     |
| T/G        | rotate camera up/down        |
| O/P        | start/stop rain              |
| K/L        | start/stop flood             |
| arrow keys | move flood position          |
| U/J	     | pause/unpause the simulation |
| V	     | save the mesh and the data   |
| B/N        | activate the wind simulation |
| 1/2	     | Wind debug mode 		    |
| 3/4 	     | Multiple sediments transport |


In order to save, the scene must be paused, by  pressing the key "u" , and then saved by pressing the key "v" . The scene can be unpaused by pressing "j" .
The two debug modes that we have created can be toggled on/off by pressing the keys 1-2 for the wind debug mode and 3-4 for the multiple sediments transport.
By pressing "b" and "n" it's possible to turn on/off the wind simulation that is visualized as a simple 
 
