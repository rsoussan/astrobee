\page colmapmapping Colmap mapping

# Package Overview
## View feature extraction/matching results
`colmap gui --database_path database_name --image_path image_directory_name`
  where `database_name` and `image_directory_name` are filled in with the correct values.
This opens the gui with the database and images directory.
The user should then navigate to the `Processing` tab in the gui and select `Database management`.
Extracted features can be seen per image by clicked `Show Image` for the respective image.
Matches can be seen by selecting `Overlapping Images` for an image then `Show matches` for the desired
matched image.  

## View sparse mapping results
`rosrun colmap_mapping view_map.py name_of_mapper.ini`
  where `name_of_mapper.ini` is the *_mapper.ini file creating during map creation.
The sparse map should load in the visualizer, showing camera poses and bundle-adjusted feature points.



