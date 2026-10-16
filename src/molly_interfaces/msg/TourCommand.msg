# Dashboard / handheld -> guide_node, on /tour/cmd or through /tour/control.

uint8 KIND_START=0
uint8 KIND_PAUSE=1
uint8 KIND_RESUME=2
uint8 KIND_STOP=3
uint8 KIND_SKIP=4

uint8 kind
string tour_id            # START: a saved tour of the active site, or
string[] poi_ids          # START: these POIs in this order (used when tour_id is empty)
