\page sparsemapping Sparse mapping

# Map Creation Overview 
Starting with a bagfile, an image feature map (sparse map) can be created.
The general pipeline involves splicing the bagfile into smaller bags if desired, 
creating individual maps for each bag, then merging these maps together.
If a map already exists and should be used as a starting point, the new maps can additionally be merged with this existing map. 

Splicing a bag into multiple bags before mapping adds some robustness to 
the mapping pipeline along with making the map creation pipeline more computationally efficient as the map creation for 
individual bags is parallelized.
The individual bags can later be introspected with the maps_visualizer tool mentioned in \ref maps_visualizer and errors
can be mitigated without affecting the other generated maps before merging these together. 
(TODO: add section on error mitigation)

# SURF Map vs. BRISK Localization Map
For map creation SURF image features are used as these are more robust and accurate for matching. 
However, SURF features are too expensive to compute and match during online localization. 
Therefore, BRISK features are used for localization.
Maps for map creation contain SURF features and these features should never be reduced or pruned to ensure 
the highest accuracy during map creation.
For localization, an existing SURF map is converted to a BRISK map (see section Maps for Localization for more details).
BRISK features are estimated and matched and triangulated, but the image poses generated during the SURF map
creation pipeline are not modified since BRISK feature matching is less accurate and robust. 

## Tool and Script Usage Instructions 
For each tool and script mentioned below, run `rosrun sparse_mapping tool_or_script_name -h` for further details and 
usage instructions.
Additionally, see readme.md for an overview of these.

## SURF Map Creation Pipeline

### Splicing Bag 
A bagfile can be spliced into smaller bags using 
  `rosrun bag_processing splice_bags.py bagfile`

### Generating Individual Maps
Maps can be generated for each of the resulting bags using
  `rosrun sparse_mapping make_surf_maps.py`

At this stage a unique surf map will exist for each bagfile.  
### Visualization
At anytime visualization of a map can be performed.
TODO: reference map_visualizer

### Merge maps
TODO: reference overlap calcular, merge tool

## BRISK Localization Map Creation Pipeline

