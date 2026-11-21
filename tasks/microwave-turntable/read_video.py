"""Read the loaded-turn marks from a 20 fps plan recording. Coordinates are image pixels."""

import argparse
import json
from itertools import islice
from pathlib import Path

import imageio.v2 as imageio

from markers import PlanMarks, lamp_lit
from scoring import HZ, LoadedTurn


def read(path, ticks, lamp, first, last):
    with imageio.get_reader(str(path), format="FFMPEG") as recording:
        metadata = recording.get_meta_data()
        if abs(metadata["fps"] - HZ) > .05:
            raise ValueError("Use a readable, constant 20 fps recording.")
        width, height = metadata["size"]
        if not (2 <= lamp[0] < width-2 and 2 <= lamp[1] < height-2):
            raise ValueError("Lamp patch falls outside the recording.")
        marks = PlanMarks(width, height, ticks)
        turn = LoadedTurn()
        crossings = [None]*4
        best = None
        first_full = first_lit = last_lit = None
        stop = last + 1 if last is not None else None
        frame = None
        for frame, im in enumerate(islice(recording, first, stop), start=first):
            bearings = None
            if lamp_lit(im, lamp):
                if first_lit is None:
                    first_lit = frame
                last_lit = frame
                bearings = marks.bearings(im)
            visible = bearings is not None
            resets_before = turn.resets
            starting = turn.previous is None
            complete = turn.sample(*(bearings if visible else (0, 0)), eligible=visible)
            if visible:
                if starting or turn.resets != resets_before:
                    start = frame
                    crossings = [None]*4
                for i, angle in enumerate((90, 180, 270, 360)):
                    if crossings[i] is None and min(turn.travel) >= angle:
                        crossings[i] = frame
                if best is None or min(turn.travel) > min(best["travel_deg"]):
                    best = dict(frames=[start, frame], travel_deg=turn.travel.tolist(),
                                both_past_quarter_frame=crossings.copy())
                if complete and first_full is None:
                    first_full = frame
        if frame is None:
            raise ValueError("No frames in the selected interval.")
        if best:
            best["travel_deg"] = [round(a, 2) for a in best["travel_deg"]]
        return dict(first_lit_frame=first_lit, last_lit_frame=last_lit, first_complete_turn_frame=first_full,
                    longest_visible_turn=best)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--ticks", type=float, nargs=4, default=(61.5, 287.5, 483.5, 287.5),
                        metavar=("X1", "Y1", "X2", "Y2"), help="Centers of opposite 139 mm cardinal ticks")
    parser.add_argument("--lamp", type=int, nargs=2, default=(562, 487), metavar=("X", "Y"))
    parser.add_argument("--first", type=int, default=0, help="First manually eligible frame (inclusive)")
    parser.add_argument("--last", type=int, help="Last manually eligible frame (inclusive)")
    args = parser.parse_args()
    if args.first < 0 or (args.last is not None and args.last < args.first):
        parser.error("Frame numbers start at zero. The last frame must follow the first.")
    print(json.dumps(read(args.video, args.ticks, args.lamp, args.first, args.last), indent=2))
