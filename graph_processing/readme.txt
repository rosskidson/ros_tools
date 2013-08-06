g2o_add_edges -
 takes a g2o file and adds identity edges between vertices that have a connecting edge with a weak edge (information matrix is small)
This was to test if adding edges where edges were weak would help global optimization propagate error throughout the graph

g2o_pcd_converter
 Take a g2o file and converts it to a pointcloud and save as a pcd file.  Only considers vertices, not edges.  Useful to compare 
trajectories using pcd_viewer

g2o_replace_weak_edges
 similar to g2o_add_edges but replaces the information matrix of weak edges with a stronger info matrix

icp
 Takes 2 pcd files and performs icp to align them.  Useful for aligning trajectories for comparison.  Outputs an aligned pointcloud.
(the first input file is aligned to the second one.  Therefore, visualize icp_output.icp and the 2nd input pcd)
