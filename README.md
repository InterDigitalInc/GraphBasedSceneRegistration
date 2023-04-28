# Graph-based Scene Registration

This methods converts two image sequences (label maps, depth maps and camera poses) into semantic scene graphs, creates node descriptors for those graphs in an attempts to semantically match them. The results of the semantic matching are then use to calculate the relative transformation matrix between the two sequences world coordinate systems.

Our adjacency-based descriptors are compared against:
* the random walk descriptors from *X-View: Graph-Based Semantic Multi-View Localization* (Gawel et al. 2018)
* the walk histogram descriptors from *Semantic Histogram Based Graph Matching for Real-Time Multi-Robot Global Localization in Large Scale Environment* (Guo et al. 2021)

We re-implemented the relevant parts of both works to fit our datasets and to fairly compare their performances against ours.

We also provide an alternative graph extraction method that produces nodes based on object instances rather than spatial regions, for a more accurate object-level understanding of the scene.

# 1. Prerequisites #
* Ubuntu (20.04)
* CMake (3.22.1)
* Eigen (3.3.7)
* OpenCV (4.2.0)
* PCL (1.10)

You must set the correct `DATA` and `SOURCES` paths in the scripts and python utilities to match your system configuration.

# 2. Running #
Clone the repository and run:
```
    mkdir build
    cd build
    cmake ..
    make
    ln -f -s ../scripts/make_scripts.sh . && ./make_scripts.sh
    && ./make_python.sh
```

To run the registration:
```
./sgRegistration instance1 instance2 [descriptor_type descriptor_depth graph_type task registration_type saving start1 count1 start2 count2 suffix]
```
`instance1` and `instance2` are folders in the `Dataset` folder.

# 3. Compatible datasets #
They all require processing before being used:
* ScanNet
* ChangeSim
* Prototypical: 3RScan, as we have yet to create the registration ground truth.

To speed up the process, we can pre-compute the "blobs" (no need to load the images) using:
* `preprocess_scannet_blobs`
* `preprocess_changesim_blobs`

We can also pre-compute the super-nodes (no need to load the blobs) using:
* `preprocess_scannet_snodes`
* `preprocess_changesim_snodes`

# 4. Python utilities #
Most of the important python utilities can be launched using the scripts.

* `3rscan_link`: links the relevant 3RScan files into the `Dataset` folder using the dataset's `json` files
* `changesim_poses`: **changeSim processing** create the `trajectory.txt` file by matching raw and processed depth maps and interpolating quaternions
* `fusion_error`: gives summary of the error (error against depth curves) over the whole run
* `fusion_graph`: gives summary of the graph extraction metrics (to use on a `cmd-snodes` folder)
* `fusion_post`: filters and merges results from the `fusion_error` and `fusion_time` utilities
* `fusion_success`: gives the overall the error results (error against instance/success against threshold) over whole run
* `fusion_time`: gives summary of the run-time (time against depth curves) over the whole run
* `metrics_dataset`: gives information on the instances in the datasets (ie camera speed)
* `metrics_graph`: gives summary of the distances between objects, their radii etc... over the generated graphs
* `orientation_checker`: **ScanNet processing** find the transformation required to align different instances of a same scene in ScanNet

# 5. Scripts #
* `changesim_link`: links the relevant ScanNet files into the `Dataset` folder and creates the `trajectory.txt` files
* `make_python`: creates links for the python scripts in the `build` folder
* `make_scripts`: creates links for the `sh` scripts in the `build` folder
* `metrics`: runs the `fusion_error`, `fusion_graph`, `fusion_post`, `fusion_success` and `fusion_graph` python utilities over the selected configuration
* `preprocess_changesim_blobs`: convert the images of ChangeSim into a list of blobs
* `preprocess_changesim_snodes`: convert the images (or blobs if available) of ChangeSim into a list of super-nodes
* `preprocess_scannet_blobs`: convert the images of ScanNet into a list of blobs
* `preprocess_scannet_snodes`: convert the images (or blobs if available) of ScanNet into a list of super-nodes
* `process_changesim`: run the registration pipeline on ChangeSim (argument to select a subset of scenes)
* `process_scannet`: run the registration pipeline on ScanNet (argument to select a subset of scenes)
* `process`: **compiles** and run the registration on a specific scene
* `result_merge`: merges results files in `build` if `process_DATASET` was run with an argument
* `result_process`: moves a run from `build` to `runs` in a named folder (default: `last`) and generates the result summaries
* `result_redo`: clears the run files in `build`, and moves back a run from `runs` if its name is provided
* `scannet_extract`: extracts the relevant files from the `sens` files
* `scannet_link_esalab`: links ESANet labels for ScanNet if they have been created on Windows
* `scannet_link_instance`: links instance images for ScanNet for graph ground truth
* `scannet_link`: links the relevant ScanNet files into the `Dataset` folder
* `scannet_orientation`: create the `orientation.txt` files for ScanNet (for ground truth alignment)

# 6. Known issues #
* PCL may not be compatible with the double precision coordinates used in the `cloud_map.ply` models of ChangeSim. They should be converted to float precision and renamed `cloud_map_float.ply`
