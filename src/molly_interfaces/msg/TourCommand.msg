# Dashboard / handheld -> guide_node, on tour/cmd or through tour/control.

uint8 START = 0
uint8 PAUSE = 1
uint8 RESUME = 2
uint8 STOP = 3
uint8 SKIP = 4

uint8 kind
string tour_id            # START: a saved tour of the active site, or
string[] poi_ids          # START: these POIs in this order (used when tour_id is empty)
