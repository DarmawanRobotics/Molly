# molly_mapping

Occupancy maps recorded from the LiDAR-inertial odometry, and **sites**: one directory per mapped
place with its map, the AprilTags that anchor it, its POIs, tours and routes.

```
map/<site>/
├── map.yaml, map.pgm   occupancy map (map_server format: 0 occupied, 254 free, 205 unknown)
├── tags.json           AprilTag map poses (localization/record_tags)
├── pois.json           {"pois": [...]}   molly_interfaces/Poi fields
├── tours.json          {"tours": [...]}  molly_interfaces/Tour fields
└── routes/             recorded routes
```

Localization stays tag based: a site's map frame is the frame its tags were recorded in. Mapping
a **new** site takes the current map frame (the current pose when the robot is not localized);
**continuing** an existing site needs the robot localized on one of its tags first, or `reset` to
start it over (the old files move to `backup-<time>/`).

## Nodes

### grid_mapper_node

Each processed scan (`rate_hz`) is voxel-downsampled, cut to the obstacle band around the robot's
height (in the gravity-aligned map frame, so a pitching body never turns the floor into a wall)
and the robot's own footprint, then integrated into a log-odds grid in `map`: hits occupy their
cells, a 2D ray from the sensor to the nearest hit of each angular bin carves free space (bins
without a hit carve nothing). The grid grows in whole cells. The scans are only subscribed while
mapping.

| | Name | Type |
|---|---|---|
| sub | `cloud_topic` (`/cloud_registered_body`) | `sensor_msgs/PointCloud2`, while mapping |
| pub | `mapping/grid` | `nav_msgs/OccupancyGrid`, latched, `publish_hz` while mapping |
| pub | `mapping/status` | `molly_interfaces/MappingStatus`, latched |
| srv | `grid_mapper/start` | `std_srvs/SetBool`: `data` = start from an empty grid |
| srv | `grid_mapper/stop` | `std_srvs/Trigger` |
| srv | `grid_mapper/save` | `molly_interfaces/LoadFile`: `path` = site directory, writes map.pgm + map.yaml |
| srv | `grid_mapper/load` | `molly_interfaces/LoadFile`: `path` = site directory or map.yaml, the map to continue |

### site_manager_node

Owns the active site and its files; the only publisher of `/map`.

| | Name | Type |
|---|---|---|
| pub | `/map` | `nav_msgs/OccupancyGrid`, latched: the site map, the live grid while mapping, empty when the site has none |
| pub | `site/status` | `molly_interfaces/SiteStatus`, latched |
| pub | `site/pois`, `site/tours` | `molly_interfaces/PoiArray`, `TourArray`, latched |
| srv | `site/list`, `site/load` | `ListSites`, `LoadSite` (another site resets localization) |
| srv | `site/save_poi`, `site/delete_poi` | `SavePoi` (`at_robot_pose`), `DeleteById` (also from the tours) |
| srv | `site/save_tour`, `site/delete_tour` | `SaveTour` (known POIs only), `DeleteById` |
| srv | `mapping/start`, `mapping/stop`, `mapping/save` | `StartMapping`, `std_srvs/Trigger` |

`mapping/start {site, reset}` creates the site when new, hands its `tags.json` to localization
(`localization/load_tags`, created empty), takes the current pose as the map origin when nothing
is localized (`localization/assume_origin`), loads the saved map into the grid mapper to continue
it (`grid_mapper/load`) unless `reset`, and starts it. `mapping/save` writes the grid into the
site and makes it the site map. Names (sites, POI and tour ids) are `^[a-z0-9][a-z0-9_-]{0,47}$`;
ids are made from the names when empty. JSON files are written atomically with a `.bak`.

## Parameters

[config/mapping_params.yaml](config/mapping_params.yaml):

| Parameter | Default | |
|---|---|---|
| `grid_mapper_node.resolution` | `0.05` | m per cell |
| `grid_mapper_node.rate_hz` | `2.0` | scans integrated per second |
| `grid_mapper_node.obstacle_z_min`, `obstacle_z_max` | `-0.25`, `1.2` | m around base_link's height: what counts as an obstacle |
| `grid_mapper_node.self_filter_*` | `±0.40`, `±0.28` | the robot's own body in base_link |
| `grid_mapper_node.max_range` | `15.0` | m |
| `grid_mapper_node.log_odds_hit`, `log_odds_miss` | `0.85`, `-0.4` | per scan |
| `grid_mapper_node.occupied_threshold`, `free_threshold` | `0.65`, `0.35` | probabilities |
| `site_manager_node.sites_dir` | `~/dev/Molly/map` | |
| `site_manager_node.site` | `""` | site loaded at start; empty: the one of the last run (`state_file`) |
| `site_manager_node.start_mapping` | `false` | map `site` right after start when it is a new site (`mode:=mapping`) |

## Launch

```bash
ros2 launch molly_mapping mapping.launch.py
ros2 service call /mapping/start molly_interfaces/srv/StartMapping "{site: lobby, reset: true}"
ros2 service call /mapping/save std_srvs/srv/Trigger
ros2 service call /site/save_poi molly_interfaces/srv/SavePoi "{poi: {name: Fountain}, at_robot_pose: true}"
```
