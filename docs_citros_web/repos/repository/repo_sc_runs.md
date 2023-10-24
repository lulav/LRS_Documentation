 <!-- TODO Gleb is working on simulation documentation, he might have something that will fit this screen  -->



# Repository Runs
When you navigate to the `Runs` tab within a repository, you will see a list of all previous runs for that repository.

![Alt text](img/Runs_page.png)  

The default order of the list is from newest to oldest, with the newest one appearing first. This order can be changed by clicking on the green `Filter` button.  
To upload a new simulation, click on the green `Run simulation` button.  
<!-- For instructions on how to upload a new simulation, please refer to [source].(../docs_citros_web/simulations/sim_run_simulation.md). -->

Let's examine a particular run in more detail.  

![Alt text](img/run_closer_look.png)  

The syntax for the red box is the repository name, followed by the launch file and the run name.  
The blue box provides a description of the run provided.  
The green box displays the total number of runs and the number of runs left to complete. The purple box shows the team member who performed the run, while the orange box displays the batch ID. Next to the ID number is a copy icon, which enables you to easily copy the batch ID for analyzing the simulation data.  
The gray box displays the date and time when the simulation was first run while the pink box helps you keep track of how many days have passed since the first simulation run.  
The yellow box on the CITROS server displays the simulation status, which can be `LOADING`, `LOADED` ,`UNLOADED`or `UNKNOWN`.

When the term `LOADING` is displayed, it means that the data is currently being loaded into the database. Please wait for the data to be fully `LOADED` before using it for data analysis purposes. Once `LOADED` is displayed, it indicates that the data is now available for analysis. 

On the other hand, `UNLOADED` refers to the state of the data when it has been removed from the database. To load the data again, use the data analysis package. For more information on how to use the package, see <!-- [here]().-->

If the status of the data is `UNKNOWN`, it means that it is undetermined. In this case, please use the data analysis package to initiate state fixes and load the data into the database.