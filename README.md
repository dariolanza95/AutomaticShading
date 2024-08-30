This project is part of my Master's Thesis Automatic Shading , the full thesis can be downloaded here. 
The thesis has been supervised by [Prof. Alexander Wilkie](https://cgg.mff.cuni.cz/~wilkie/Website/Home.html) when I was a master student at [Charles University Computer Graphics Group](https://cgg.mff.cuni.cz/). The goal of the thesis is to provide


AutomaticShading

-----------Installation------------

This project requires 
-OpenMesh 8.0
-QtCreator
-PointCloudLibrary (pcl)
RenderMan 23.3 (only for render the final scene).

This software has been developed in Ubuntu 18.04.2 LTS and hasn't been tested in any other platform.
Since having RenderMan working on this version of Ubuntu is quite a tedious and tricky task we also provide here the headers and the libraries needed to create a scene that could be rendered using RenderMan.
To do so the user have to  simply copy the folder named "Data" and then use Renderman.
# H2 Simulation Hydraulic and Wind Erosion 
The framework proposed in the thesis begins by running a simulation and then use simulation data to augment with finer details the final mesh. In this step we will take care of generating the initial mesh and the simulation data that will be later processed. To do so, we edit the simulation [terrain-errosion](https://github.com/karhu/terrain-erosion) project to our needs.
You can run the simulation by running the command: 'xxx' (TODO)
Belowe we report some of the key-bindings that can be used to generate various terrains through our edited simulation. The main ways a user can edit the terrain is by making rain, generate water in a specific point (which can be moved through the arrow keys) and by wind.

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

Once the user is satisfied with the results it is possible to save the mesh and the simulation data. In order to save, the scene must be paused, by  pressing the key "u" , and then saved by pressing the key "v" . The scene can be unpaused by pressing "j". The two debug modes that we have created can be toggled on/off by pressing the keys 1-2 for the wind debug mode and 3-4 for the multiple sediments transport. By pressing "b" and "n" it's possible to turn on/off the wind simulation that is visualized with a simple blue hue.
 
