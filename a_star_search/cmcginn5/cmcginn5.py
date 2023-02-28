import streamlit as st
import a_start_search as a_star
import time

#Helper functions to synchronize options
def update_rows():
    """
    Helper function to set the row options based on the map dimensions
  
    Returns
    -------
    None: No values are returned. The session_state is updated.
    """
    if(st.session_state.start_row > st.session_state.height):
        st.session_state.start_row = st.session_state.height
    else:
        st.session_state.start_row = st.session_state.start_row
    
    if(st.session_state.goal_row > st.session_state.height):
        st.session_state.goal_row = st.session_state.height
    else:
        st.session_state.goal_row = st.session_state.goal_row

def update_cols():
    """
    Helper function to set the column options based on the map dimensions
  
    Returns
    -------
    None: No values are returned. The session_state is updated.
    """
    if(st.session_state.start_col > st.session_state.width):
        st.session_state.start_col = st.session_state.width
    else:
        st.session_state.start_col = st.session_state.start_col
        
    if(st.session_state.goal_col > st.session_state.width):
        st.session_state.goal_col = st.session_state.width
    else:
        st.session_state.goal_col = st.session_state.goal_col

#About the App
about_exp = st.sidebar.expander("About this app")
with about_exp:
    about_exp.markdown('**Author:** C.J. McGinnis<br/>'
                       +'**Last Update:** 19 February 2023', True)
   
    about_exp.image("https://github.com/cjmjhu/streamlit/blob/main/a_star_search/cmcginn5/jhu-logo.png?raw=true", width=200)
    about_exp.markdown('Whiting School of Engineering<br/>'
                   + 'Johns Hopkins University<br/> Baltimore, MD, USA<br/>', 
                   True)

astar_exp = st.sidebar.expander("About A* Search")
with astar_exp:
    astar_exp.markdown('**A\* Search** is a state-space search algorithm. It '
                       + 'uses a cost estimate to the goal (***heuristic***) '
                       + 'and the cumulative path cost to prioritize where '
                       + 'to explore next from a list of nodes reached but not '
                       + 'explored (the ***frontier***). '
                       + 'This app implements an admississible heuristic, so '
                       + 'that it never overestimates the cost to the goal. As '
                       + 'a result, the search is cost-optimal and '
                       + 'complete for all spaces with a finite solution.'
                       + ' The illustration below shows the nodes on the '
                       + 'frontier as open circles. The explored node color '
                       + 'indicates the heuristic, with green colors being '
                       + 'closer to the goal.')
    astar_exp.markdown("![Example A* Illustration](https://upload.wikimedia.org"
                       +"/wikipedia/commons/5/5d/Astar_progress_animation.gif)")
    astar_exp.markdown("*Image Source:* <a href=" 
                      +"'https://en.wikipedia.org/wiki/A*_search_algorithm#/"
                      +"media/File:Astar_progress_animation.gif'"
                      +">Wikimedia Commons</a>",True)
   
#Options
st.sidebar.header('Options')
st.sidebar.markdown('Use these settings to refine the type of random grid world'
                +' that will be generated.')

#Options: Default values
MAX_HW = 27
MIN_HW = 2
DEFAULT_HW = 27
DEFAULT_START = 1
DEFAULT_PATH = True
DEFAULT_IMPASS_PER = 20

#reset the options to default values
if st.sidebar.button("Reset the options?", key="reset"):
    st.session_state.height = DEFAULT_HW
    st.session_state.width = DEFAULT_HW
    st.session_state.start_row = DEFAULT_START
    st.session_state.start_col = DEFAULT_START
    st.session_state.goal_row = DEFAULT_HW
    st.session_state.goal_col = DEFAULT_HW
    st.session_state.ensure_path = DEFAULT_PATH
    st.session_state.perc_impass = DEFAULT_IMPASS_PER

st.sidebar.caption('This will reset the options to default values.')

#Options: World Dimensions
st.sidebar.subheader('World Dimensions')
world_height = st.sidebar.slider('**World Height**', MIN_HW, MAX_HW, DEFAULT_HW, 
                                 key='height', on_change=update_rows)
st.sidebar.caption('This is the number of *rows* in the grid world.')
world_width = st.sidebar.slider('**World Width**', MIN_HW, MAX_HW, DEFAULT_HW, 
                                key='width', on_change=update_cols)
st.sidebar.caption('This is the number of *columns* in the grid world.')

#Options: Start Location
st.sidebar.subheader('Start and Goal')
row_start = st.sidebar.number_input('**Start Row**', min_value = DEFAULT_START,
                                    max_value = world_height,  
                                    key='start_row',
                                    value=DEFAULT_START)
col_start = st.sidebar.number_input('**Start Column**', min_value = DEFAULT_START,
                                    max_value = world_width,  
                                    key='start_col',
                                    value=DEFAULT_START)
st.sidebar.caption('These set your starting location in the world.')    

#Options: Goal Location
row_goal = st.sidebar.number_input('**Goal Row**', min_value = DEFAULT_START,
                                   max_value = world_height,  key='goal_row',
                                   value=world_height)
col_goal = st.sidebar.number_input('**Goal Column**', min_value = DEFAULT_START,
                                   max_value = world_width,  key='goal_col',
                                   value=world_width)
st.sidebar.caption('These set the goal location in the world.')  

#Options: Path Settings
st.sidebar.subheader('Path Settings')
impass_perc = st.sidebar.number_input('**Percent Impassible**', min_value = 0,
                                   max_value = 100,  key='perc_impass',
                                   value=DEFAULT_IMPASS_PER)
st.sidebar.caption('This is the approximate percentage of the map that will be '
                   +' impassible, before any necessary adjustents to make a '
                   + 'valid world.') 
                   
path_ensure = st.sidebar.checkbox('Guarantee that a path exists?', key='ensure_path',
                    value=DEFAULT_PATH)  

st.sidebar.caption('This will ensure that a valid path exists from the start'
                   +' to the goal.') 


#Main App:Default values
START_ICON = "🏁"
GOAL_ICON = "🎁"
SUC_ICON = "✅"
FAIL_ICON = "🤖"
MOVES = {(1,0): "⏩", (-1,0): "⏪", (0,1): "⏬", (0,-1): "⏫"}
NEW_LINE = '<br/>'
FIND_TXT = "Find Path"
TOG_TXT = "Toggle Map Markers"
TAB_GW = "Grid World"
TAB_INFO = "Terrain Information"
TAB_PATH = "Path Details"
MSG_START_GOAL = ('The start and goal are the same location, so no movement is '
                  +' required to obtain the goal.')

#Main App: Introduction
st.title("A* Search for Grid Worlds")
st.markdown('A ***grid world*** is shown below. It is made of emojis that '
            + 'represent various types of terrain on a map. Each terrain type '
            + 'has an associated movement cost as described on the `'+TAB_INFO
            + '` tab below. The `'+FIND_TXT+'` button will run the ***A\* '
            + 'search*** algorithm to find the lowest cost path (if one exists)'
            + ' from the start (' + START_ICON + ') to the goal (' + GOAL_ICON 
            + '). The `'+TOG_TXT+'` button will show or hide the start, '
            + 'goal, and path icons.',True)
st.caption('See the ***sidebar*** on the left to set options, generate a new '
            + 'grid world map, and learn more about A* Search.')

tab_gw, tab_p, tab_i = st.tabs(["Grid World", TAB_PATH, TAB_INFO])


#Main App: Explain Terrain Information
with tab_i:
    st.markdown(
        'The various ***terrain*** types in the grid world are represented '
        +'by emojis. Each has an associated cost that represents the amount of '
        +'effort it would require to traverse this terrain type. The total cost'
        +' of a path is sum of the costs for each terrain type moved through. '
        +'Costs are only accumulated when taking an action to move from one '
        +'location to another (including the goal).')
    
    st.markdown('***Start:*** '+START_ICON+'<br/>This represents the starting '
                +'location (i.e., the entry point).',True)
    
    st.markdown('***Goal:*** '+GOAL_ICON+'<br/>This represents the goal '
                +'location (i.e, the desired destination).',True)
    
    st.markdown('***Terrain Costs:***')
    t_costs = ', '.join([f'{c[0]}: {c[1]}' for c in a_star.COSTS.items()])
    st.markdown(f'{t_costs}')
 
    st.markdown('***Impassible Terrain:*** The following terrain types can '
                +'never be traversed and can be considered to have an infinite '
                +'terrain cost.')
    t_imp = ', '.join(a_star.IMPASS_LIST)
    st.markdown(f'{t_imp}')
    
    st.markdown('Note that the only type of impassible terrain that appears in '
                +'the default world is 🗻.')

#helper function to get a string of the map that will display in the app
def get_map_str():
    """
    Helper function to get a string of the map that will display in the app
    properly.
  
    Returns
    -------
    str: a string representation of a map
    """
    if(st.session_state.marks_on):
        markers = [(st.session_state.the_start, START_ICON), 
                   (st.session_state.the_goal, GOAL_ICON)]
    else: 
        markers = None 
    return a_star.print_map(
        world = st.session_state.unsolved_world,
        marks = markers,
        line_char = NEW_LINE,
        print_out = False)

def generate_map():
    """
    Generates the map of the grid world by calling create_random_world() in the
    a_star_search module. The parameters of the grid world to be created are 
    taken from the session_state variables that are user-defined from the 
    options sidebar.
  
    Returns
    -------
    None: No values are returned. Instead, the session_state of the app is set
    for the start, goal, unsolved_world, and unsolved_world_str. 
    
    """
    st.session_state.the_start = (col_start-1, row_start-1)
    st.session_state.the_goal = (col_goal-1, row_goal-1)
    st.session_state.unsolved_world = a_star.create_random_world(
        height = world_height, 
        width = world_width, 
        costs = a_star.COSTS,
        impassible = a_star.IMPASS_LIST, 
        impass_freq = impass_perc/100, 
        start = st.session_state.the_start,
        goal = st.session_state.the_goal,
        make_path = path_ensure)
    st.session_state.unsolved_world_str = get_map_str()

def set_default_map():
    """
    Loads the default map from tha a_star_search module and sest the apropriate 
    session state variables based on its properties.
  
    Returns
    -------
    None: No values are returned. Instead, the session_state of the app is set
    for the start, goal, unsolved_world, and unsolved_world_str. 
    
    """
    st.session_state.unsolved_world = a_star.full_world
    width_height = len(st.session_state.unsolved_world)
    st.session_state.the_start = (0, 0)
    st.session_state.the_goal = (width_height-1, width_height-1)
    st.session_state.unsolved_world_str = get_map_str()

def perform_search():
    """
    Attempts to find a valid path through the current grid world using the 
    a_star_search module. 
  
    Returns
    -------
    None: No values are returned. Instead, the session_state of the app is set
    for the start, goal, unsolved_world, and unsolved_world_str. The discovered
    path is set to session_state variables path_found, path_terrain, 
    solved_world_str, and sovled_cost. If no valid path is found, the variables
    are set to None.
    
    """
    results = a_star.a_star_search(
        world = st.session_state.unsolved_world, 
        start = st.session_state.the_start, 
        goal = st.session_state.the_goal, 
        costs = a_star.COSTS, 
        moves = a_star.MOVES, 
        heuristic = a_star.heuristic,
        terrain=True)
    if(results is None):
        st.session_state.path_found = None
        st.session_state.path_terrain = None
    else:
        st.session_state.path_found = results[0]
        st.session_state.path_terrain = results[1]
    cost, solution = a_star.pretty_print_path(
    world = st.session_state.unsolved_world,
    path = st.session_state.path_found,
    start = st.session_state.the_start, 
    goal = st.session_state.the_goal, 
    costs = a_star.COSTS,
    line_char= NEW_LINE,
    print_out = False)    
    st.session_state.solved_world_str = solution
    st.session_state.solved_cost = cost

#Main App: Load defaults 
if "marks_on" not in st.session_state:
    st.session_state.marks_on = True

if "solved" not in st.session_state:
    st.session_state.solved = False
    
if "unsolved_world" not in st.session_state:
    set_default_map()
    

#Main App: Set Grid World Buttons
with tab_gw:
    col1,col2 = st.columns([1,4])
    with col2:
        toggle_btn = st.button(TOG_TXT)
    with col1:
        path_bth = st.button(FIND_TXT)
    if(toggle_btn):
        st.session_state.marks_on = not st.session_state.marks_on
        st.session_state.unsolved_world_str = get_map_str()
    if(path_bth):
        st.session_state.marks_on = True
        if(not st.session_state.solved):
            with st.spinner('Performing A* Search...'):
                start_time = time.time()
                perform_search()
                end_time = time.time()
                st.session_state.time = end_time-start_time
            st.session_state.solved = True
        
#Main App: Set Grid World Container
with tab_gw:
    gw_container = st.empty();

gen_btn = st.sidebar.button("Generate World", type="primary")
st.sidebar.caption("Looking for the original grid world? You can reload the "
                    +"default world by clicking the button below.")
default_btn = st.sidebar.button("Load Default World")

if gen_btn:
    st.session_state.solved = False
    generate_map()

    with tab_gw:
        with gw_container.container():
            st.markdown(st.session_state.unsolved_world_str, True)
if default_btn:
    st.session_state.solved = False
    set_default_map()

    with tab_gw:
        with gw_container.container():
            st.markdown(st.session_state.unsolved_world_str, True)

with tab_gw:
    with gw_container.container():
        st.markdown(st.session_state.unsolved_world_str, True)
     
    if(st.session_state.solved):
        with gw_container.container():
            if(st.session_state.path_found == None):
                st.warning('No valid path exists', icon=FAIL_ICON)
            elif(len(st.session_state.path_found)==0):
                st.success(MSG_START_GOAL, icon=SUC_ICON)
            else:
                st.success(f'Found optimal path with a cost of ' 
                           +f'{st.session_state.solved_cost} and a length'
                           +f' of {len(st.session_state.path_found)}'
                           +f' in {st.session_state.time:.2f} seconds. ', 
                           icon=SUC_ICON)
            if(st.session_state.marks_on):
                st.markdown(st.session_state.solved_world_str, True)
            else:
                st.markdown(st.session_state.unsolved_world_str, True)

#Main App: Report Path Details    
with tab_p:
    st.markdown('This tab displays information about the optimal path.') 
    path_container = st.empty();    
    #if "path_terrain" not in st.session_state:
    if(not st.session_state.solved):
        with path_container.container():
            st.markdown('No path has been generated yet. Click the `'+FIND_TXT
                        +'` button on the `'+TAB_GW +'` tab to perform a search'
                        +' for the optimal path.')
    elif(st.session_state.path_terrain is None):
        with path_container.container():
            st.warning('No valid path exists', icon=FAIL_ICON)
    elif(len(st.session_state.path_found)==0):
        with path_container.container():
            st.success(MSG_START_GOAL, icon=SUC_ICON)
    else:
        with path_container.container():
            
            locs = []
            for loc in st.session_state.path_terrain:
                locs.append(st.session_state.unsolved_world[loc[1]][loc[0]])
            
            st.markdown('***Terrain order:***') 
            st.markdown("".join(locs))
           
            st.markdown('***Movement directions:***') 
            mvs = []
            for mov in st.session_state.path_found:
                mvs.append(MOVES[mov])
            st.markdown("".join(mvs))
            
            st.markdown('***Combined path info:***')
            st.markdown('This shows the movement direction immediately followed'
                        +' by the resulting terrain. (e.g., ⏩🌲 means moved '
                        +'right to a 🌲).')            
            st.markdown(START_ICON + 
                        "".join([c[0]+c[1] for c in zip(mvs,locs)]) 
                        + f'( {GOAL_ICON})')
                        
            st.markdown('***Cost summary:***') 
            st.markdown('This shows the terrain type followed by the number '
                        + 'traversed x the cost = total cost')
            cost_sum = []
            tot_cost = 0
            for c in a_star.COSTS.items():
                tot = locs.count(c[0])
                tot_cost += tot*c[1]
                cost_sum.append(f'{c[0]}: {tot} x {c[1]} = {tot*c[1]}')
            st.markdown('<br/>'.join(cost_sum),True)
            st.markdown(f'***Total moves***: {len(locs)}'
                        +f'<br/>***Total Cost***: {tot_cost}' 
                        +f'<br/>***Avg. cost/move***: {tot_cost/len(locs):.2f}' 
                        , True)
