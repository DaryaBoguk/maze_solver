import cv2
import numpy as np


draw_intrstpts = True
debug_mapping = False

class Graph():
    def __init__(self):
        self.graph = {}

        self.start = 0
        self.end = 0

    def add_vertex(self,vertex,neighbor=None, case=None, cost=None):
        if vertex in self.graph.keys():
            self.graph[vertex][neighbor] = {}
            self.graph[vertex][neighbor]["case"] = case
            self.graph[vertex][neighbor]["cost"] = cost
        else:
            self.graph[vertex] = {}
            self.graph[vertex]["case"] = case

    def displaygraph(self):
        for key, value in self.graph.items():
            print("key {} has value {}".format(key,value))

class bot_mapper():

    def __init__(self):

        self.graphified = False

        self.crm_amt = 5

        self.Graph = Graph()

        self.connected_left = False
        self.connected_upleft = False
        self.connected_up = False
        self.connected_upright = False

        self.maze_connect = []

    def display_connected_nodes(self,curr_node,neighbor_node,case="Unkown",color=(0,0,255)):
        curr_pixel = (curr_node[1],curr_node[0])
        neighbor_pixel = (neighbor_node[1],neighbor_node[0])
        #self.maze_connect= cv2.circle(self.maze_connect, curr_pixel, 5, (255,0,0))
        #self.maze_connect= cv2.circle(self.maze_connect, neighbor_pixel, 5, (255,0,0))
        print("----------------------) CONNECTED >> {} << ".format(case))
        self.maze_connect = cv2.line(self.maze_connect,curr_pixel,neighbor_pixel,color,1)
        cv2.imshow("Nodes Conected", self.maze_connect)
        if debug_mapping:
            cv2.waitKey(0)                    
            self.maze_connect = cv2.line(self.maze_connect,curr_pixel,neighbor_pixel,(255,255,255),1)

    def connect_neighbors(self, maze, node_row, node_col, case, step_l = 1, step_up = 0, totl_cnncted=0):
        curr_node = (node_row, node_col)
        if (maze[node_row-step_up][node_col-step_l]>0):
            neighbor_node = (node_row-step_up, node_col - step_l)
            if neighbor_node in self.Graph.graph.keys():
            
                neighbors_case = self.Graph.graph[neighbor_node]["case"]
                cost = max(abs(step_l),abs(step_up))
                totl_cnncted += 1

                self.Graph.add_vertex(curr_node, neighbor_node, neighbors_case, cost)
                self.Graph.add_vertex(neighbor_node,curr_node,case,cost)
                print("\nConnected {} to {} with Case [step_l,step_up] = [ {} , {} ] & Cost -> {}".format(curr_node,neighbor_node,step_l,step_up,cost))

                if not self.connected_left:
                    self.display_connected_nodes(curr_node, neighbor_node,"LEFT",(0,0,255))
                    # Vertex has connected to its left neighbor.                    
                    self.connected_left = True
                    # Check up-Left route now
                    step_l = 1
                    step_up = 1
                    self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
                if not self.connected_upleft:
                    self.display_connected_nodes(curr_node, neighbor_node,"UPLEFT",(0,128,255))
                    # Vertex has connected to its up-left neighbor.
                    self.connected_upleft = True
                    # Check top route now
                    step_l  = 0
                    step_up = 1
                    self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
                if not self.connected_up:
                    self.display_connected_nodes(curr_node, neighbor_node,"UP",(0,255,0))
                    # Vertex has connected to its up neighbor.
                    self.connected_up = True
                    # Check top-right route now
                    step_l  = -1
                    step_up = 1
                    self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
                if not self.connected_upright:
                    self.display_connected_nodes(curr_node, neighbor_node,"UPRIGHT",(255,0,0))
                    # Vertex has connected to its up-right neighbor.
                    self.connected_upright = True

            if not self.connected_upright:
                if not self.connected_left:
                    # Look a little more left, You'll find it ;)
                    step_l +=1
                elif not self.connected_upleft:
                    # Look a little more (diagnolly) upleft, You'll find it ;)
                    step_l+=1
                    step_up+=1
                elif not self.connected_up:
                    # Look a little more up, You'll find it ;)
                    step_up+=1
                elif not self.connected_upright:
                    # Look a little more upright, You'll find it ;)
                    step_l-=1
                    step_up+=1
                self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
        else:
            # No path in the direction you are looking, Cycle to next direction
            if not self.connected_left:
                # Basically there is a wall on left so just start looking up lft:)
                self.connected_left = True
                # Looking upleft now
                step_l = 1
                step_up = 1
                self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
            elif not self.connected_upleft:
                # Basically there is a wall up lft so just start looking up :)
                self.connected_upleft = True
                step_l = 0
                step_up = 1
                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, totl_cnncted)
                

            elif not self.connected_up:
                # Basically there is a wall above so just start looking up-right :)
                self.connected_up = True
                step_l = -1
                step_up = 1
                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, totl_cnncted)

            elif not self.connected_upright:
                # Basically there is a wall above so just start looking up-right :)
                self.connected_upright = True
                step_l = 0
                step_up = 0                
                return     

    @staticmethod
    def triangle(image,ctr_pt,radius,colour=(0,255,255),thickness=2):
        # Polygon corner points coordinates
        pts = np.array( [ [ctr_pt[0]        , ctr_pt[1]-radius]  , 
                          [ctr_pt[0]-radius , ctr_pt[1]+radius]  ,
                          [ctr_pt[0]+radius , ctr_pt[1]+radius]   
                        ] 
                        ,np.int32
                      )
        
        pts = pts.reshape((-1, 1, 2))
        
        image = cv2.polylines(image,[pts],True,colour,thickness)
        return image

    @staticmethod
    def get_surround_pixel_intensities(maze, curr_row, curr_col):

        maze = cv2.threshold(maze, 1, 1, cv2.THRESH_BINARY)[1]

        rows = maze.shape[0]
        cols = maze.shape[1]

        top_row = False
        btm_row = False 
        lft_col = False 
        rgt_col = False

        if (curr_row==0):
            # Top row => Row above not accesible
            top_row = True
        if (curr_row == (rows-1)):
            # Bottom row ==> Row below not accesible
            btm_row = True
        if (curr_col == 0):
            # Left col ==> Col to the left not accesible
            lft_col = True
        if (curr_col == (cols-1)):
            # Right col ==> Col to the right not accesible
            rgt_col = True

        if (top_row or lft_col):
            top_left = 0
        else:
            top_left = maze[curr_row-1][curr_col-1]
        if( top_row or rgt_col ):
            top_rgt = 0
        else:
            top_rgt = maze[curr_row-1][curr_col+1]

        if( btm_row or lft_col ):
            btm_left = 0
        else:
            btm_left = maze[curr_row+1][curr_col-1]

        if( btm_row or rgt_col ):
            btm_rgt = 0
        else:
            btm_rgt = maze[curr_row+1][curr_col+1]

        if (top_row):
            top = 0
        else:
            top = maze[curr_row-1][curr_col]
        if (rgt_col):
            rgt = 0
        else:
            rgt = maze[curr_row][curr_col+1]
        
        if (btm_row):
            btm = 0
        else:
            btm = maze[curr_row+1][curr_col]

        if (lft_col):
            lft = 0
        else:
            lft = maze[curr_row][curr_col-1]

        no_of_pathways = ( top_left + top      + top_rgt  +
                           lft      + 0        + rgt      + 
                           btm_left + btm      + btm_rgt        
                         )

        if no_of_pathways>2:  
            print("  [ top_left , top      , top_rgt  ,lft    , rgt      , btm_left , btm      , btm_rgt   ] \n [ ",str(top_left)," , ",str(top)," , ",str(top_rgt)," ,\n   ",str(lft)," , ","-"," , ",str(rgt)," ,\n   ",str(btm_left)," , ",str(btm)," , ",str(btm_rgt)," ] ")
            print("\nno_of_pathways [row,col]= [ ",curr_row," , ",curr_col," ] ",no_of_pathways) 

        return top_left,top,top_rgt,rgt,btm_rgt,btm,btm_left,lft,no_of_pathways

    def reset_connct_paramtrs(self):
        # Reseting member variables to False initially when looking for nodes to connect
        self.connected_left = False
        self.connected_upleft = False
        self.connected_up = False
        self.connected_upright = False

    def one_pass(self, maze):

        self.Graph.graph.clear()

         # Initalizing Maze_connect with Colored Maze
        self.maze_connect = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        cv2.namedWindow("Nodes Conected",cv2.WINDOW_FREERATIO)

        turns = 0
        junc_3 = 0
        junc_4 = 0

        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)

        cv2.namedWindow("Maze (Interest Points)", cv2.WINDOW_FREERATIO)
        rows = maze.shape[0]
        cols = maze.shape[1]

        for row in range(rows):
            for col in range(cols):
                if (maze[row][col]==255):
                    if debug_mapping:
                        self.maze_connect = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
                    top_left,top,top_rgt,rgt,btm_rgt,btm,btm_left,lft, paths = self.get_surround_pixel_intensities(maze.copy(),row,col)

                    if ((row==0) or (row==(rows-1)) or (col==0) or (col==(cols-1))):
                        if (row == 0):
                            #Start
                            maze_bgr[row][col] = (0, 128, 255)
                            cv2.imshow("Maze (Interest Points)", maze_bgr)
                            self.Graph.add_vertex((row,col),"_Start_")
                            self.Graph.start = (row, col)
                        else:
                            #End (Maze Exit)
                            maze_bgr[row][col] = (0, 255, 0)
                            cv2.imshow("Maze (Interest Points)", maze_bgr)
                            self.Graph.add_vertex((row, col), case="_End_")
                            self.Graph.end = (row, col)
                            self.reset_connct_paramtrs()
                            self.connect_neighbors(maze, row, col, "_End_")
                    elif (paths==1):
                        crop = maze[row-1:row+2, col-1:col+2]
                        print(" ** [Dead End] ** \n", crop)
                        maze_bgr[row][col] = (0,0,255)# Red color
                        if draw_intrstpts:
                            maze_bgr= cv2.circle(maze_bgr, (col,row), 10, (0,0,255),4)
                        cv2.imshow("Maze (Interest Points)", maze_bgr)
                        self.Graph.add_vertex((row, col), case="_DeadEnd_")
                        self.reset_connct_paramtrs()
                        self.connect_neighbors(maze, row, col, "_DeadEnd_")

                    elif (paths==2):
                        crop = maze[row-1:row+2,col-1:col+2]
                        nzero_loc = np.nonzero(crop > 0)
                        nzero_ptA = (nzero_loc[0][0],nzero_loc[1][0])
                        nzero_ptB = (nzero_loc[0][2],nzero_loc[1][2])
                        if not ( ( (2 - nzero_ptA[0])==nzero_ptB[0] ) and 
                                    ( (2 - nzero_ptA[1])==nzero_ptB[1] )     ):
                            maze_bgr[row][col] = (255,0,0)
                            #if draw_intrstpts:
                                #maze_bgr= cv2.circle(maze_bgr, (col,row), 10, (255,0,0),2)
                            self.Graph.add_vertex((row,col),case = "_Turn_")
                            # Connecting vertex to its neighbor (if-any)
                            self.reset_connct_paramtrs()
                            self.connect_neighbors(maze, row, col, "_Turn_")
                            turns+=1
                        cv2.imshow("Maze (Interest Points)",maze_bgr)
                    elif (paths>2):
                        if (paths ==3):
                            maze_bgr[row][col] = (255,244,128)
                            if draw_intrstpts:
                                maze_bgr = self.triangle(maze_bgr, (col,row), 10,(144,140,255),4)
                            cv2.imshow("Maze (Interest Points)",maze_bgr) 
                            self.Graph.add_vertex((row,col),case = "_3-Junc_")
                            # Connecting vertex to its neighbor (if-any)
                            self.reset_connct_paramtrs()
                            self.connect_neighbors(maze, row, col, "_3-Junc_")
                            junc_3+=1                                          
                        else:
                            maze_bgr[row][col] = (128,0,128)
                            if draw_intrstpts:
                                cv2.rectangle(maze_bgr,(col-10,row-10) , (col+10,row+10), (255,215,0),4)
                            cv2.imshow("Maze (Interest Points)",maze_bgr)
                            self.Graph.add_vertex((row,col),case = "_4-Junc_")
                            # Connecting vertex to its neighbor (if-any)
                            self.reset_connct_paramtrs()
                            self.connect_neighbors(maze, row, col, "_4-Junc_")
                            junc_4+=1
        print("Графааааа", self.Graph.graph)
        print("\nInterest Points !!! \n[ Turns , 3_Junc , 4_Junc ] [ ",turns," , ",junc_3," , ",junc_4," ] \n")

    def graphify(self, extracted_maze):

        if not self.graphified:
            cv2.imshow("Extracted_Maze [MazeConverter]", extracted_maze)

            thinned = cv2.ximgproc.thinning(extracted_maze)
            cv2.imshow('Maze (thinned)', thinned)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
            thinned_dilated = cv2.morphologyEx(thinned, cv2.MORPH_DILATE, kernel)
            _, bw2 = cv2.threshold(thinned_dilated, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)        
            thinned = cv2.ximgproc.thinning(bw2)
            cv2.imshow('Maze (thinned*2)', thinned)


            thinned_cropped = thinned[self.crm_amt:thinned.shape[0]-self.crm_amt,
                                      self.crm_amt:thinned.shape[1]-self.crm_amt]
            
            cv2.imshow('Maze (thinned*2)(Cropped)', thinned_cropped)

            extracted_maze_cropped = extracted_maze[self.crm_amt:extracted_maze.shape[0]-self.crm_amt,
                                                    self.crm_amt:extracted_maze.shape[1]-self.crm_amt]
            extracted_maze_cropped = cv2.cvtColor(extracted_maze_cropped, cv2.COLOR_GRAY2BGR)
            extracted_maze_cropped[thinned_cropped>0] = (0,255,255)
            cv2.imshow('Maze (thinned*2)(Cropped)(Path_Overlayed)', extracted_maze_cropped)


            self.one_pass(thinned_cropped)

            cv2.waitKey(0)

            self.maze = thinned_cropped
            self.graphified = True


            


