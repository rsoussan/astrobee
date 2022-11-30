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
`colmap gui --import_path import_path_name --database_path database_name --image_path image_directory_name`
  where `database_name` and `image_directory_name` are filled in with the correct values and `import_path_name` is the location of the cameras, images, and point3d files created during map creation.
The sparse map should load in the visualizer, showing camera poses and bundle-adjusted feature points.



