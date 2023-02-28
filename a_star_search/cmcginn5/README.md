
---

**EN.605.645** - Module 4 Programming Assignment  
**Author:** Christofer McGinnis  
**Email:** cmcginn5@jh.edu  
**Due:** February 19, 2023  

---  

All required files are saved in the folder: cmcginn5

### LIST OF FILES 

***README.md***: this readme file  
***cmcginn5.py***: the main Streamlit application  
***a_star_serach.py***: the A* search module code  
***jhu-logo.png***: an image used for display in the app  

### INSTRUCTIONS FOR RUNNING:

Use the en605645 environment as provided by the course materials for
EN.605.645.

1. Unzip the cmcginn5.zip file to a location you can access with the command line.
2. On the command line, navigate to the root project folder (cmcginn5).
3. Run the command: streamlit run cmcginn5.py

### USING THE APP:

The Streamlit app will launch in your default web browser. The app
provides instructions and explainanations for use, but the following
is a brief overview of some of the major features.

***Tabs***:
The main app provides 3 tabs:  
* *Grid World*: A visual display of the grid world. The `Find Path` button runs the A\* search algorithm. The `Toggle Map Markers` button will show or hide the path, start, and goal markers.
* *Path Details*: A list of details on the solved path. An overview of the path movements, terrain types moved through, and a breakdown of the path cost calculations are provided.
* *Terrain Information*: This is an informational tab that provides background and cost information for terrain types.  

 
***Options (sidebar)***:
The sidebar on the left provides various settings for generating a random grid world. This inclues the ability to specify the width and height, the start and goal locations, and whether or not a valid path must exist. Clicking the `Generate World` button will create the random world according to the parameters specified. The original default world can also be loaded using the button a the bottom of the sidebar. The sidebar also provides information about the A\* search algorithm and the app.  At the top of the options menu, there is `Reset the Options?` button to reset the settings to the defaults at any time.  