# ROB535_FinalProject_Team9
Self Driving Cars Final Project - Team 9

## Classification Code

To run the Classification Code, you need to have:
 - Deep Learning Toolbox
 - NasNet-Large downloaded from MathWorks Add-On Center

### Execution Steps
1. Change MATLAB current directory to `ROB535_FinalProject_Team9/Perception/`
2. Run `classifyImages_Master.m`
3. A folder dialog will appear. Locate the folder with the training data. 
4. Another folder dialog will appear. Locate the folder with the testing data. 
5. When finished running, MATLAB will save a `testFiles.csv` file to `ROB535_FinalProject_Team9/Perception/` containing the predicted labels. 


## Localization Code

To run the Localization Code, you need to have: 
- Deep Learning Toolbox

### Execution Steps
1. Change MATLAB current directory to `ROB535_FinalProject_Team9/Perception/`
2. Run `localize_master.m` to retrain the model. This script will train the network for our data and may take a long time (3+ days) depending on hardware. We attempted to included the results of our training in the repo, but the files were too large for GitHub. ~~You can skip to step 5 if you do not want to retrain~~.
3. A folder dialog will appear. Locate the folder with the training data. Note that it will skip this step if the folder is already in memory. 
4. Another folder dialog will appear. Locate the folder with the testing data. Again, note that it will skip this step if the folder is already in memory. 
5. Once the script is done running, run `predictCentroids.m` to use the trained network and localize the vehicles. 
6. When finished running, MATLAB will save a `testCentroids.csv` file to `ROB535_FinalProject_Team9/Perception/` containing the predicted centroid locations.