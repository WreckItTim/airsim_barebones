see RUN section to run a demo to ensure the environment is set up properly

to set up, see INSTALLATION section below


RUN:

executing below should do the following:

	launch an instance of AirSim that you can see in a window
	
	auto move the drone to a given point
	
	collect data over several frames then write to file

open terminal in this directory and type:

	conda activate airsim
    
	python data_collector.py

watch the drone move and collect data

after, view all of the returned data by running the view_data.ipynb jupyter notebook

INSTALLATION:

download needed scripts from: https://github.com/WreckItTim/airsim_barebones

install conda

launch terminal in installation directory

to fresh install airsim conda environment, execute in terminal: bash -i ./conda_env.sh

download AirSim map of your choice, which is a .zip precompiled binary that you can get from microsoft here: https://github.com/microsoft/airsim/releases

unzip .zip file from above

note the path to the desired .sh file within the unzipped directory, example from the Blocks map: ./Blocks/LinuxBlocks1.8.1/LinuxNoEditor/Blocks.sh

change the appropriate release paths if needed in .py files to desired AirSim map .sh file


	

