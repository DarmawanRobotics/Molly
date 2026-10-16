# An ordered list of POIs the guide walks through.

string id                 # stable id: a-z, 0-9, _ and -, unique per site
string name
string description
string[] poi_ids          # stops in visiting order
string return_poi_id      # walk here after the last stop; empty: stay at the last stop
