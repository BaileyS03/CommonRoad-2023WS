# CommonRoad-2023WS
This is a programming exercise of the lecture **Fundamentals of Artificial Intelligence (IN2406)** delivered at the Department of Informatics, TUM. The task is to implement a heuristic function and/or a search algorithm with motion primitives to solve motion planning problems in [CommonRoad](https://commonroad.in.tum.de/). 

# Outcome 

| Place              | 1                   |
| ----------------- | ---------------------------------|
| Number of Best Solutions         | 1140  |
| Solutions Solved  | 1622/2077 |

After further testing and revision of the algorithm, it performs more than 10% better when using GBFS over A*. This is likely because admissibility cannot be guaranteed. 

# Code Sections
Student.py is where I implemented the heuristic for the A* search algorithm to rank the search paths. The motion primitives and automata (automata is a conversation of motion primitive files required for batch processing/tutorial 2) are in their own files. The batch processor was run with each automata file to achieve the lowest cost, however the most flexible automata were V_0.0_20.0_Vstep_2.22_SA_-1.066_1.066_SAstep_0.18_T_0.5_Model_BMW_320i.xml which solves the most scenarios in one run. 

# In action solutions 
Motion Primitive Used: **V_0.0_20.0_Vstep_2.22_SA_-1.066_1.066_SAstep_0.18_T_0.5_Model_BMW_320i.xml**

![Alt text](Solution%20GIFs/DEU_BadEssen-2_2_T-1.gif)
DEU_BadEssen-2_2_T-1

![Alt text](Solution%20GIFs/DEU_Flensburg-68_1_T-1.gif)
DEU_Flensburg-68_1_T-1

![Alt text](Solution%20GIFs/FRA_Miramas-1_4_T-1.gif)
FRA_Miramas-1_4_T-1

![Alt text](Solution%20GIFs/USA_Lanker-2_20_T-1.gif)
USA_Lanker-2_20_T-1

![Alt text](Solution%20GIFs/USA_Peach-3_3_T-1.gif)
USA_Peach-3_3_T-1

![Alt text](Solution%20GIFs/USA_US101-20_2_T-1.gif)
USA_US101-20_2_T-1

![Alt text](Solution%20GIFs/ZAM_Tjunction-1_4_T-1.gif)
ZAM_Tjunction-1_4_T-1

# Approach 
To achieve lower costs based on the SM1 cost function in this challenge, the crucial strategy involves minimising acceleration (velocity change) and steering changes. This was successfully implemented by constraining search options using motion primitives. Furthermore, the route planner was employed alongside a KD tree, allowing a comparison of the current position with the closest point in the KD data structure. To enhance the suitability for the heuristic function, the distance metric obtained from this comparison was then scaled through an additional function.

