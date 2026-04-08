import json
import math
from pathlib import Path

import cv2
import fire
import numpy as np


BG_COLOR = (245, 243, 238)
FREE_COLOR = (236, 234, 229)
OBSTACLE_COLOR = (58, 62, 67)
ENDPOINT_COLOR = (214, 229, 255)
WORKSTATION_COLOR = (255, 225, 166)
GRID_COLOR = (222, 218, 210)
TEXT_COLOR = (45, 48, 53)
TASK_COLOR = (255, 106, 74)

AGENT_COLORS = [
    (231, 76, 60),
    (46, 204, 113),
    (52, 152, 219),
    (241, 196, 15),
    (155, 89, 182),
    (26, 188, 156),
    (230, 126, 34),
    (149, 165, 166),
    (127, 140, 141),
    (192, 57, 43),
    (39, 174, 96),
    (41, 128, 185),
]


def _load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _agent_color(agent_id):
    return AGENT_COLORS[agent_id % len(AGENT_COLORS)]


def _draw_map(layout, cell_size, margin, header_h):
    rows = len(layout)
    cols = len(layout[0])
    width = cols * cell_size + margin * 2
    height = rows * cell_size + margin * 2 + header_h

    canvas = np.full((height, width, 3), BG_COLOR, dtype=np.uint8)

    for r, row in enumerate(layout):
        for c, ch in enumerate(row):
            x0 = margin + c * cell_size
            y0 = margin + header_h + r * cell_size
            x1 = x0 + cell_size
            y1 = y0 + cell_size

            if ch in {"@", "T"}:
                color = OBSTACLE_COLOR
            elif ch == "e":
                color = ENDPOINT_COLOR
            elif ch == "w":
                color = WORKSTATION_COLOR
            else:
                color = FREE_COLOR

            cv2.rectangle(canvas, (x0, y0), (x1, y1), color, thickness=-1)
            cv2.rectangle(canvas, (x0, y0), (x1, y1), GRID_COLOR, thickness=1)

    return canvas


def _cell_to_px(x, y, cell_size, margin, header_h):
    px = int(round(margin + x * cell_size + cell_size / 2))
    py = int(round(margin + header_h + y * cell_size + cell_size / 2))
    return px, py


def _orientation_endpoint(px, py, orient, radius):
    if orient == 0:
        return px + radius, py
    if orient == 1:
        return px, py + radius
    if orient == 2:
        return px - radius, py
    if orient == 3:
        return px, py - radius
    return px, py


def _draw_agent(frame, agent_id, state, prev_state, cell_size, margin, header_h):
    row, col, orient, task_id = state
    px, py = _cell_to_px(col, row, cell_size, margin, header_h)
    color = _agent_color(agent_id)
    radius = max(5, int(cell_size * 0.3))

    if prev_state is not None:
        prev_row, prev_col, _, _ = prev_state
        ppx, ppy = _cell_to_px(prev_col, prev_row, cell_size, margin, header_h)
        cv2.line(frame, (ppx, ppy), (px, py), color, thickness=max(1, cell_size // 10))

    cv2.circle(frame, (px, py), radius, color, thickness=-1)
    cv2.circle(frame, (px, py), radius, (255, 255, 255), thickness=2)

    tip_x, tip_y = _orientation_endpoint(px, py, int(round(orient)), radius)
    cv2.arrowedLine(
        frame,
        (px, py),
        (tip_x, tip_y),
        (255, 255, 255),
        thickness=2,
        tipLength=0.35,
    )

    cv2.putText(
        frame,
        str(agent_id),
        (px - radius // 2, py + radius // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        TEXT_COLOR,
        1,
        cv2.LINE_AA,
    )

    if int(task_id) >= 0:
        cv2.circle(frame, (px, py), radius + 6, TASK_COLOR, thickness=2)


def _draw_goal_marker(frame, x, y, task_id, cell_size, margin, header_h):
    row, col = x, y
    px, py = _cell_to_px(col, row, cell_size, margin, header_h)
    outer = max(6, int(cell_size * 0.22))
    inner = max(2, int(cell_size * 0.1))

    cv2.circle(frame, (px, py), outer, TASK_COLOR, thickness=2)
    cv2.circle(frame, (px, py), inner, TASK_COLOR, thickness=-1)
    cv2.putText(
        frame,
        str(task_id),
        (px + outer + 2, py - outer - 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.35,
        TASK_COLOR,
        1,
        cv2.LINE_AA,
    )


def _draw_live_goal_marker(
    frame, x, y, task_id, agent_id, cell_size, margin, header_h
):
    row, col = x, y
    px, py = _cell_to_px(col, row, cell_size, margin, header_h)
    color = _agent_color(agent_id)
    outer = max(8, int(cell_size * 0.28))

    cv2.circle(frame, (px, py), outer, color, thickness=2)
    cv2.line(frame, (px - outer, py), (px + outer, py), color, thickness=2)
    cv2.line(frame, (px, py - outer), (px, py + outer), color, thickness=2)
    cv2.putText(
        frame,
        f"A{agent_id}->T{task_id}",
        (px + outer + 3, py + outer + 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.32,
        color,
        1,
        cv2.LINE_AA,
    )


def _find_next_goal(path, start_idx):
    for state in path[start_idx:]:
        if int(state[3]) >= 0:
            return state
    return None


def _normalize_tick_state(state):
    if len(state) >= 4:
        return state[0], state[1], state[2], state[3]
    return state[0], state[1], state[2], -1


def _normalize_tick_goal(goal):
    if len(goal) >= 3:
        return goal[0], goal[1], int(goal[2])
    return -1, -1, -1


def render_vis(
    vis_json="sam_test_vis.json",
    map_json="maps/kiva_large_w_mode.json",
    output_mp4="sam_test.mp4",
    fps=8,
    cell_size=28,
    margin=28,
    header_h=64,
):
    vis_path = Path(vis_json)
    map_path = Path(map_json)
    output_path = Path(output_mp4)

    vis_data = _load_json(vis_path)
    map_data = _load_json(map_path)
    throughput = vis_data.get("throughput")
    sum_of_cost = vis_data.get("sum_of_cost")

    layout = map_data["layout"]
    robot_paths = vis_data["robot_paths"]
    tick_robot_states = vis_data.get("tick_robot_states", [])
    tick_robot_goals = vis_data.get("tick_robot_goals", [])
    if tick_robot_states:
        num_frames = len(tick_robot_states)
    else:
        num_frames = max(len(path) for path in robot_paths)

    base = _draw_map(layout, cell_size, margin, header_h)
    height, width = base.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))
    if not video.isOpened():
        raise RuntimeError(f"Failed to open video writer for {output_path}")

    for frame_idx in range(num_frames):
        frame = base.copy()
        cv2.putText(
            frame,
            f"Frame {frame_idx + 1}/{num_frames}",
            (margin, margin + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            TEXT_COLOR,
            2,
            cv2.LINE_AA,
        )

        finished_tasks = 0
        completed_goals = {}
        for path in robot_paths:
            state = path[min(frame_idx, len(path) - 1)]
            if int(state[3]) >= 0:
                completed_goals[int(state[3])] = (state[0], state[1])
                finished_tasks += 1

        for task_id, (goal_x, goal_y) in completed_goals.items():
            _draw_goal_marker(
                frame,
                goal_x,
                goal_y,
                task_id,
                cell_size,
                margin,
                header_h,
            )

        cv2.putText(
            frame,
            f"Tasks completed this frame: {len(completed_goals)}",
            (margin, margin + 44),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            TEXT_COLOR,
            1,
            cv2.LINE_AA,
        )

        metric_parts = []
        if throughput is not None:
            metric_parts.append(f"Throughput: {float(throughput):.3f}")
        if sum_of_cost is not None:
            metric_parts.append(f"Sum of cost: {sum_of_cost}")
        if metric_parts:
            cv2.putText(
                frame,
                "   ".join(metric_parts),
                (margin + 260, margin + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                TEXT_COLOR,
                1,
                cv2.LINE_AA,
            )

        for agent_id, path in enumerate(robot_paths):
            if tick_robot_states:
                state = _normalize_tick_state(tick_robot_states[frame_idx][agent_id])
                prev_state = (
                    _normalize_tick_state(tick_robot_states[frame_idx - 1][agent_id])
                    if frame_idx > 0
                    else None
                )
                path_idx = min(frame_idx, len(path) - 1)
            else:
                path_idx = min(frame_idx, len(path) - 1)
                state = path[path_idx]
                prev_state = path[path_idx - 1] if path_idx > 0 else None

            live_goal = None
            if tick_robot_goals and frame_idx < len(tick_robot_goals):
                goal_row, goal_col, goal_task_id = _normalize_tick_goal(
                    tick_robot_goals[frame_idx][agent_id]
                )
                if goal_task_id >= 0:
                    live_goal = (goal_row, goal_col, goal_task_id)
            else:
                next_goal_state = _find_next_goal(path, path_idx)
                if next_goal_state is not None:
                    live_goal = (
                        next_goal_state[0],
                        next_goal_state[1],
                        int(next_goal_state[3]),
                    )

            if live_goal is not None:
                _draw_live_goal_marker(
                    frame,
                    live_goal[0],
                    live_goal[1],
                    live_goal[2],
                    agent_id,
                    cell_size,
                    margin,
                    header_h,
                )

            _draw_agent(frame, agent_id, state, prev_state, cell_size, margin, header_h)

        video.write(frame)

    video.release()
    print(f"Saved visualization to {output_path}")


if __name__ == "__main__":
    fire.Fire(render_vis)
