"""Generate the warehouse occupancy grid (PGM + YAML) and the layout file (nodes, racks, crossing sites).

Everything is in metres, map frame, origin at the lower left corner. Output is deterministic.
"""
import json
from pathlib import Path

RES = 0.05
W, H = 32.0, 22.0
OUT = Path(__file__).resolve().parent.parent / "ros2_ws/src/tugger_nav/maps"

# Rectangles (x0, y0, x1, y1) that are occupied.
WALL = 0.2
walls = [(0, 0, W, WALL), (0, H - WALL, W, H), (0, 0, WALL, H), (W - WALL, 0, W, H)]
rack_x = [(8.0, 11.5), (13.5, 18.5), (20.5, 24.0)]
racks = []
for y0, y1 in [(WALL, 4.0), (8.0, 14.0), (18.0, H - WALL)]:
    racks += [(x0, y0, x1, y1) for x0, x1 in rack_x]
# Receiving area pallets (west) and production machines (east).
pallets = [(1.0, 7.0, 3.0, 9.5), (1.0, 12.5, 3.0, 15.0)]
machines = [(29.0, 8.0, 31.0, 10.5), (29.0, 11.5, 31.0, 14.0)]
blocks = walls + racks + pallets + machines

# Tugger loop nodes on the corridor centre lines (y = 6 and 16, x = 6 and 26).
nodes = {
    "A": (6.0, 6.0), "B": (12.5, 6.0), "C": (19.5, 6.0), "D": (26.0, 6.0), "E": (26.0, 11.0),
    "F": (26.0, 16.0), "G": (19.5, 16.0), "H": (12.5, 16.0), "I": (6.0, 16.0), "J": (6.0, 11.0),
}
# People step out of the 2 m cross aisles between racks and cross the loop corridor.
# hidden = start point inside the rack gap (occluded from the corridor), far = end point in the opposite gap.
sites = {
    "S1": {"hidden": (12.5, 2.0), "path": (12.5, 6.0), "far": (12.5, 10.5)},
    "S2": {"hidden": (19.5, 2.0), "path": (19.5, 6.0), "far": (19.5, 10.5)},
    "N1": {"hidden": (12.5, 20.0), "path": (12.5, 16.0), "far": (12.5, 11.5)},
    "N2": {"hidden": (19.5, 20.0), "path": (19.5, 16.0), "far": (19.5, 11.5)},
}


def main() -> None:
    nx, ny = round(W / RES), round(H / RES)
    grid = bytearray([254] * (nx * ny))  # free
    for x0, y0, x1, y1 in blocks:
        for iy in range(max(0, round(y0 / RES)), min(ny, round(y1 / RES))):
            row = (ny - 1 - iy) * nx  # PGM rows go top to bottom
            for ix in range(max(0, round(x0 / RES)), min(nx, round(x1 / RES))):
                grid[row + ix] = 0  # occupied
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "warehouse.pgm").write_bytes(f"P5\n{nx} {ny}\n255\n".encode() + bytes(grid))
    (OUT / "warehouse.yaml").write_text(
        "image: warehouse.pgm\nmode: trinary\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    layout = {
        "size": [W, H], "resolution": RES, "blocks": blocks, "nodes": nodes, "sites": sites,
        "loop": ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "A"],
    }
    (OUT / "layout.json").write_text(json.dumps(layout, indent=1))
    print(f"map {nx}x{ny} cells, {len(blocks)} blocks, {len(nodes)} nodes")


if __name__ == "__main__":
    main()
