# site_content_node -> site/pois (latched): every POI of the active site.

std_msgs/Header header    # frame_id: map
string site               # empty: no site loaded
Poi[] pois
