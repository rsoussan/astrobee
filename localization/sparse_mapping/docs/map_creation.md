\page sparsemapping Sparse mapping

# Map Creation 
Starting with a bagfile, an image feature map (sparse map) can be created.
The general pipeline includes splicing the bagfile into smaller bags if desired, 
creating individual maps for each bag, then merging the maps together.
If a map already exists and should be used as the starting point, the new maps can additionally be merged with this existing map. 

# Maps for map creation vs. Localization
For map creation SURF image features are used as these are more robust and accurate for matching. 
However, SURF features are too expensive to compute and match during online localization. 
Therefore, BRISK features are used for localization.
Maps for map creation contain SURF features and these features should never be reduced or pruned to ensure 
the highest accuracy during map creation.
For localization, an existing SURF map is converted to a BRISK map (see section Maps for Localization for more details).
BRISK features are estimated and matched and triangulated, but the image poses generated during the SURF map
creation pipeline are not modified since BRISK feature matching is less accurate. 

## SURF Map Pipeline

### Splicing Bag 
A bagfile can be spliced into smaller bags using 
  `rosrun bag_processing splice_bags.py bagfile`

### Generated Individual Maps
Maps can be generated from the spliced bags using
  `rosrun sparse_mapping make_maps.py`

This script extracts images for each bag, detects and matches SURF features, and
performs incremental bundle adjustment to generate each map. The mapping is performed in parallel
for each bagfile.

### Visualization
At anytime visualization of a map can be performed.
TODO: reference map_visualizer

### Merge maps
TODO: reference overlap calcular, merge tool

## Localization Map Pipeline

